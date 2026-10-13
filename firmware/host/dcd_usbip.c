/* TinyUSB device controller driver whose "bus" is a USB/IP connection (Linux
 * Documentation/usb/usbip_protocol.rst): the virtual board is a USB/IP server that `usbip attach`
 * (vhci_hcd) or a user-space client imports. One device, full speed, bus id "1-1".
 *
 * Host requests arrive as CMD_SUBMIT URBs and are held until the stack has data or buffer space
 * for them, as a real controller NAKs: control transfers are played through the stack's setup,
 * data and status stages; bulk and interrupt URBs are matched with the stack's endpoint
 * transfers; isochronous IN URBs get one packet per 1 ms frame (usbip_frame), so audio flows at
 * the real sample rate. vhci_hcd handles SET_ADDRESS itself and never forwards it, so the driver
 * gives the stack a SET_ADDRESS of its own when a client imports the device. */
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "tusb.h"
#include "device/dcd.h"
#include "usbip_server.h"

enum {
    OP_REQ_DEVLIST = 0x8005, OP_REP_DEVLIST = 0x0005, OP_REQ_IMPORT = 0x8003, OP_REP_IMPORT = 0x0003,
    USBIP_VERSION = 0x0111,
    CMD_SUBMIT = 1, CMD_UNLINK = 2, RET_SUBMIT = 3, RET_UNLINK = 4,
    USBIP_DIR_OUT = 0, USBIP_DIR_IN = 1,
    USB_SPEED_FULL_LINUX = 2,
};
#define BUSID "1-1"
#define MAX_ISO 128

typedef struct urb {
    struct urb *next;
    uint32_t seqnum;
    uint8_t ep;                         /* number, without direction */
    uint8_t dir;                        /* USBIP_DIR_IN / OUT */
    uint8_t *buf;                       /* OUT: data from the host; IN: data for the host */
    uint32_t len, done;
    int32_t n_iso, iso_next;
    struct { uint32_t offset, length, actual; } iso[MAX_ISO];
    bool synthetic;                     /* driver-made (SET_ADDRESS): no reply */
} urb_t;

typedef struct {
    bool open, stalled, busy;
    uint8_t type;
    uint16_t mps;
    uint8_t *buf;
    tu_fifo_t *ff;
    uint16_t total, done;
} ep_t;

static ep_t eps[16][2];
static urb_t *held;                     /* URBs waiting for the stack, in arrival order */
static urb_t *ctrl;                     /* the control URB being played */
static int listen_fd = -1, conn_fd = -1;
static bool imported, sof_on;
static uint32_t frame;
static usbip_stats_t stats;
static bool trace;                      /* ELF_USBIP_TRACE=1: one line per URB on stderr */

/* ---------- wire helpers (USB/IP headers are big-endian) ---------- */

static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }

static bool read_full(int fd, void *buf, size_t n)
{
    uint8_t *p = buf;
    while (n) {
        ssize_t r = read(fd, p, n);
        if (r <= 0) {
            if (r < 0 && errno == EINTR) continue;
            return false;
        }
        p += r;
        n -= (size_t)r;
    }
    return true;
}

static bool write_full(int fd, const void *buf, size_t n)
{
    const uint8_t *p = buf;
    while (n) {
        ssize_t r = write(fd, p, n);
        if (r <= 0) {
            if (r < 0 && errno == EINTR) continue;
            return false;
        }
        p += r;
        n -= (size_t)r;
    }
    return true;
}

/* ---------- device description for OP_REP_DEVLIST / OP_REP_IMPORT ---------- */

static size_t device_record(uint8_t *out, bool with_interfaces)
{
    const tusb_desc_device_t *d = (const tusb_desc_device_t *)tud_descriptor_device_cb();
    const uint8_t *cfg = tud_descriptor_configuration_cb(0);
    uint16_t total = (uint16_t)(cfg[2] | cfg[3] << 8);
    memset(out, 0, 312);
    snprintf((char *)out, 256, "/sys/devices/platform/elf-virtual/usb1/" BUSID);
    snprintf((char *)out + 256, 32, BUSID);
    put32(out + 288, 1);                               /* busnum */
    put32(out + 292, 2);                               /* devnum */
    put32(out + 296, USB_SPEED_FULL_LINUX);
    put16(out + 300, d->idVendor);
    put16(out + 302, d->idProduct);
    put16(out + 304, d->bcdDevice);
    out[306] = d->bDeviceClass;
    out[307] = d->bDeviceSubClass;
    out[308] = d->bDeviceProtocol;
    out[309] = 0;                                      /* bConfigurationValue: not configured yet */
    out[310] = d->bNumConfigurations;
    out[311] = cfg[4];                                 /* bNumInterfaces */
    size_t n = 312;
    if (with_interfaces) {
        for (uint16_t k = 0; k < total; k += cfg[k]) {
            if (cfg[k + 1] == TUSB_DESC_INTERFACE && cfg[k + 3] == 0) {
                out[n++] = cfg[k + 5];
                out[n++] = cfg[k + 6];
                out[n++] = cfg[k + 7];
                out[n++] = 0;
            }
        }
    }
    return n;
}

/* ---------- URB replies ---------- */

static void urb_free(urb_t *u)
{
    free(u->buf);
    free(u);
}

static void unlink_held(urb_t *u)
{
    for (urb_t **pp = &held; *pp; pp = &(*pp)->next)
        if (*pp == u) {
            *pp = u->next;
            return;
        }
}

static void reply_submit(urb_t *u, int32_t status)
{
    if (trace) fprintf(stderr, "ret %u ep %u status %d done %u%s\n", u->seqnum, u->ep, status, u->done, u->synthetic ? " (synthetic)" : "");
    if (!u->synthetic && conn_fd >= 0) {
        uint8_t h[48] = {0};
        put32(h, RET_SUBMIT);
        put32(h + 4, u->seqnum);
        put32(h + 20, (uint32_t)status);
        uint32_t actual = 0;
        if (u->n_iso) for (int k = 0; k < u->n_iso; k++) actual += u->iso[k].actual;
        else actual = u->done;
        put32(h + 24, actual);
        put32(h + 28, u->n_iso ? frame : 0);              /* start_frame */
        put32(h + 32, (uint32_t)u->n_iso);                /* 0 for non-ISO, as the kernel stub sends */
        bool ok = write_full(conn_fd, h, sizeof h);
        if (ok && u->dir == USBIP_DIR_IN) {
            if (u->n_iso) {                               /* iso IN data goes packed, packet after packet */
                for (int k = 0; k < u->n_iso && ok; k++)
                    ok = write_full(conn_fd, u->buf + u->iso[k].offset, u->iso[k].actual);
            } else if (actual) {
                ok = write_full(conn_fd, u->buf, actual);
            }
        }
        for (int k = 0; k < u->n_iso && ok; k++) {
            uint8_t d[16] = {0};
            put32(d, u->iso[k].offset);
            put32(d + 4, u->iso[k].length);
            put32(d + 8, u->iso[k].actual);
            ok = write_full(conn_fd, d, sizeof d);
        }
        if (!ok) stats.write_errors++;
        stats.urbs_completed++;
    }
    if (u == ctrl) ctrl = NULL;
    unlink_held(u);
    urb_free(u);
}

/* ---------- matching held URBs with the stack's transfers ---------- */

static void complete_xfer(uint8_t ep_addr, uint16_t len)
{
    ep_t *e = &eps[tu_edpt_number(ep_addr)][tu_edpt_dir(ep_addr)];
    e->busy = false;
    dcd_event_xfer_complete(0, ep_addr, len, XFER_RESULT_SUCCESS, false);
}

static uint16_t ep_copy_in(ep_t *e, uint8_t *dst, uint16_t n)
{
    if (e->ff) return tu_fifo_read_n(e->ff, dst, n);
    memcpy(dst, e->buf + e->done, n);
    return n;
}

static void ep_copy_out(ep_t *e, const uint8_t *src, uint16_t n)
{
    if (e->ff) tu_fifo_write_n(e->ff, src, n);
    else memcpy(e->buf + e->done, src, n);
}

/* Bulk / interrupt: move what both sides allow; returns true when the URB finished. */
static bool service_bulk(urb_t *u)
{
    uint8_t epa = (uint8_t)(u->ep | (u->dir == USBIP_DIR_IN ? TUSB_DIR_IN_MASK : 0));
    ep_t *e = &eps[u->ep][u->dir == USBIP_DIR_IN];
    if (e->stalled) {
        reply_submit(u, -32);                             /* -EPIPE */
        return true;
    }
    if (!e->busy) return false;
    if (u->dir == USBIP_DIR_OUT) {
        uint32_t room = (uint32_t)(e->total - e->done), left = u->len - u->done;
        uint16_t n = (uint16_t)(left < room ? left : room);
        ep_copy_out(e, u->buf + u->done, n);
        e->done = (uint16_t)(e->done + n);
        u->done += n;
        /* the stack's transfer ends when full or when the host's data ends (a short packet) */
        if (e->done == e->total || u->done == u->len) complete_xfer(epa, e->done);
        if (u->done == u->len) {
            reply_submit(u, 0);
            return true;
        }
        return false;
    }
    uint32_t avail = (uint32_t)(e->total - e->done), room = u->len - u->done;
    uint16_t n = (uint16_t)(avail < room ? avail : room);
    n = ep_copy_in(e, u->buf + u->done, n);
    e->done = (uint16_t)(e->done + n);
    u->done += n;
    if (e->done == e->total) {
        complete_xfer(epa, e->total);
        reply_submit(u, 0);
        return true;
    }
    if (u->done == u->len) {
        reply_submit(u, 0);
        return true;
    }
    return false;
}

static void service_all(void)
{
    bool again = true;
    while (again) {
        again = false;
        for (urb_t *u = held; u; u = u->next) {
            if (u->ep == 0 || u->n_iso) continue;
            if (service_bulk(u)) {
                again = true;
                break;
            }
        }
    }
}

/* ---------- control transfers on endpoint 0 ---------- */

static void ctrl_start(urb_t *u, const uint8_t setup[8])
{
    ctrl = u;
    eps[0][0].stalled = eps[0][1].stalled = false;
    dcd_event_setup_received(0, setup, false);
}

/* EP0 stages complete at once: the host side is always ready for them. */
static bool ctrl_xfer(uint8_t ep_addr, uint8_t *buffer, uint16_t total)
{
    if (!ctrl) return false;
    bool in = tu_edpt_dir(ep_addr) == TUSB_DIR_IN;
    bool data_in = ctrl->dir == USBIP_DIR_IN;
    if (total == 0 && (in != data_in || ctrl->len == 0)) {      /* status stage */
        dcd_event_xfer_complete(0, ep_addr, 0, XFER_RESULT_SUCCESS, false);
        reply_submit(ctrl, 0);
        return true;
    }
    if (in) {
        uint32_t room = ctrl->len - ctrl->done;
        uint16_t n = (uint16_t)(total < room ? total : room);
        memcpy(ctrl->buf + ctrl->done, buffer, n);
        ctrl->done += n;
    } else {
        uint32_t left = ctrl->len - ctrl->done;
        uint16_t n = (uint16_t)(total < left ? total : left);
        memcpy(buffer, ctrl->buf + ctrl->done, n);
        ctrl->done += n;
        total = n;
    }
    dcd_event_xfer_complete(0, ep_addr, total, XFER_RESULT_SUCCESS, false);
    return true;
}

/* ---------- the TinyUSB DCD API ---------- */

bool dcd_init(uint8_t rhport, const tusb_rhport_init_t *rh_init)
{
    (void)rhport, (void)rh_init;
    memset(eps, 0, sizeof eps);
    return true;
}

void dcd_int_enable(uint8_t rhport) { (void)rhport; }
void dcd_int_disable(uint8_t rhport) { (void)rhport; }
void dcd_int_handler(uint8_t rhport) { (void)rhport; }
void dcd_remote_wakeup(uint8_t rhport) { (void)rhport; }
void dcd_connect(uint8_t rhport) { (void)rhport; }
void dcd_disconnect(uint8_t rhport) { (void)rhport; }
void dcd_sof_enable(uint8_t rhport, bool en) { (void)rhport; sof_on = en; }

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
    (void)rhport, (void)dev_addr;
    /* the DCD answers the status stage itself (usbd.c); nothing reaches the host */
    if (ctrl) {
        dcd_event_xfer_complete(0, TUSB_DIR_IN_MASK, 0, XFER_RESULT_SUCCESS, false);
        reply_submit(ctrl, 0);
    }
}

bool dcd_edpt_open(uint8_t rhport, const tusb_desc_endpoint_t *desc)
{
    (void)rhport;
    uint8_t n = tu_edpt_number(desc->bEndpointAddress), d = tu_edpt_dir(desc->bEndpointAddress);
    ep_t *e = &eps[n][d];
    memset(e, 0, sizeof *e);
    e->open = true;
    e->type = desc->bmAttributes.xfer;
    e->mps = tu_edpt_packet_size(desc);
    return true;
}

bool dcd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size)
{
    (void)rhport;
    ep_t *e = &eps[tu_edpt_number(ep_addr)][tu_edpt_dir(ep_addr)];
    e->mps = largest_packet_size;
    e->type = TUSB_XFER_ISOCHRONOUS;
    return true;
}

bool dcd_edpt_iso_activate(uint8_t rhport, const tusb_desc_endpoint_t *desc) { return dcd_edpt_open(rhport, desc); }

void dcd_edpt_close_all(uint8_t rhport)
{
    (void)rhport;
    for (int n = 1; n < 16; n++) memset(eps[n], 0, sizeof eps[n]);
}

static bool xfer_common(uint8_t ep_addr, uint8_t *buffer, tu_fifo_t *ff, uint16_t total)
{
    uint8_t n = tu_edpt_number(ep_addr), d = tu_edpt_dir(ep_addr);
    if (n == 0) return ctrl_xfer(ep_addr, buffer, total);
    ep_t *e = &eps[n][d];
    e->buf = buffer;
    e->ff = ff;
    e->total = total;
    e->done = 0;
    e->busy = true;
    if (e->type != TUSB_XFER_ISOCHRONOUS) service_all();
    return true;
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes, bool is_isr)
{
    (void)rhport, (void)is_isr;
    return xfer_common(ep_addr, buffer, NULL, total_bytes);
}

bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t *ff, uint16_t total_bytes, bool is_isr)
{
    (void)rhport, (void)is_isr;
    return xfer_common(ep_addr, NULL, ff, total_bytes);
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
    (void)rhport;
    uint8_t n = tu_edpt_number(ep_addr);
    if (n == 0) {
        if (ctrl) reply_submit(ctrl, -32);                      /* -EPIPE: request not supported */
        return;
    }
    eps[n][tu_edpt_dir(ep_addr)].stalled = true;
    service_all();
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
    (void)rhport;
    eps[tu_edpt_number(ep_addr)][tu_edpt_dir(ep_addr)].stalled = false;
}

/* ---------- server ---------- */

static void drop_connection(void)
{
    if (conn_fd >= 0) close(conn_fd);
    conn_fd = -1;
    while (held) {
        urb_t *u = held;
        held = u->next;
        urb_free(u);
    }
    if (ctrl) {
        urb_free(ctrl);
        ctrl = NULL;
    }
    if (imported) dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, false);
    imported = false;
}

static void import_device(void)
{
    imported = true;
    stats.imports++;
    dcd_event_bus_reset(0, TUSB_SPEED_FULL, false);
    tud_task();
    /* SET_ADDRESS 2, which vhci_hcd keeps to itself */
    static const uint8_t set_address[8] = {0x00, TUSB_REQ_SET_ADDRESS, 2, 0, 0, 0, 0, 0};
    urb_t *u = calloc(1, sizeof *u);
    u->synthetic = true;
    u->dir = USBIP_DIR_OUT;
    ctrl_start(u, set_address);
    tud_task();
}

static bool handle_op(void)
{
    uint8_t h[8];
    if (!read_full(conn_fd, h, sizeof h)) return false;
    uint16_t code = get16(h + 2);
    uint8_t rec[312 + 4 * 32];
    if (code == OP_REQ_DEVLIST) {
        uint8_t r[12];
        put16(r, USBIP_VERSION);
        put16(r + 2, OP_REP_DEVLIST);
        put32(r + 4, 0);
        put32(r + 8, 1);
        size_t n = device_record(rec, true);
        bool ok = write_full(conn_fd, r, sizeof r) && write_full(conn_fd, rec, n);
        return ok && false;                                     /* the client closes after a list */
    }
    if (code == OP_REQ_IMPORT) {
        char busid[32];
        if (!read_full(conn_fd, busid, sizeof busid)) return false;
        busid[31] = 0;
        uint8_t r[8];
        put16(r, USBIP_VERSION);
        put16(r + 2, OP_REP_IMPORT);
        bool ours = strcmp(busid, BUSID) == 0;
        put32(r + 4, ours ? 0 : 1);
        if (!write_full(conn_fd, r, sizeof r)) return false;
        if (!ours) return false;
        size_t n = device_record(rec, false);
        if (!write_full(conn_fd, rec, n)) return false;
        import_device();
        return true;
    }
    return false;
}

static bool handle_urb(void)
{
    uint8_t h[48];
    if (!read_full(conn_fd, h, sizeof h)) return false;
    uint32_t cmd = get32(h), seq = get32(h + 4), dir = get32(h + 12), ep = get32(h + 16);
    if (cmd == CMD_UNLINK) {
        uint32_t victim = get32(h + 20);
        int32_t status = 0;
        for (urb_t *u = held; u; u = u->next)
            if (u->seqnum == victim) {
                if (u == ctrl) ctrl = NULL;
                unlink_held(u);
                urb_free(u);
                status = -104;                                   /* -ECONNRESET: unlinked before completion */
                stats.unlinks++;
                break;
            }
        uint8_t r[48] = {0};
        put32(r, RET_UNLINK);
        put32(r + 4, seq);
        put32(r + 20, (uint32_t)status);
        return write_full(conn_fd, r, sizeof r);
    }
    if (cmd != CMD_SUBMIT || ep > 15 || dir > USBIP_DIR_IN) return false;   /* malformed: drop the client */
    int32_t len = (int32_t)get32(h + 24), n_iso = (int32_t)get32(h + 32);
    if (n_iso == -1) n_iso = 0;                                  /* 0xffffffff: not an isochronous URB */
    if (len < 0 || len > 1 << 20 || n_iso < 0 || n_iso > MAX_ISO) return false;
    urb_t *u = calloc(1, sizeof *u);
    u->seqnum = seq;
    u->ep = (uint8_t)ep;
    u->dir = (uint8_t)dir;
    u->len = (uint32_t)len;
    u->buf = calloc(1, (size_t)len + 1);
    u->n_iso = n_iso;
    if (dir == USBIP_DIR_OUT && len && !read_full(conn_fd, u->buf, (size_t)len)) {
        urb_free(u);
        return false;
    }
    for (int k = 0; k < n_iso; k++) {
        uint8_t d[16];
        if (!read_full(conn_fd, d, sizeof d)) {
            urb_free(u);
            return false;
        }
        u->iso[k].offset = get32(d);
        u->iso[k].length = get32(d + 4);
    }
    stats.urbs_received++;
    if (trace) fprintf(stderr, "urb %u ep %u %s len %d iso %d\n", seq, ep, dir ? "in" : "out", len, n_iso);
    /* keep arrival order: append */
    urb_t **pp = &held;
    while (*pp) pp = &(*pp)->next;
    *pp = u;
    if (u->ep == 0) {
        if (ctrl) {                                              /* one control transfer at a time */
            reply_submit(u, -16);                                /* -EBUSY */
            return true;
        }
        ctrl_start(u, h + 40);
        tud_task();
        return true;
    }
    service_all();
    tud_task();
    return true;
}

int usbip_server_start(uint16_t port)
{
    trace = getenv("ELF_USBIP_TRACE") != NULL;
    listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) return -1;
    int one = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons(port), .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
    if (bind(listen_fd, (struct sockaddr *)&a, sizeof a) || listen(listen_fd, 1)) {
        close(listen_fd);
        listen_fd = -1;
        return -1;
    }
    return 0;
}

void usbip_poll(int timeout_ms)
{
    /* the first message may wait timeout_ms; whatever else is already queued is handled too */
    for (int k = 0; k < 64; k++) {
        struct pollfd p = {.fd = conn_fd >= 0 ? conn_fd : listen_fd, .events = POLLIN};
        if (p.fd < 0 || poll(&p, 1, k ? 0 : timeout_ms) <= 0) return;
        if (conn_fd < 0) {
            conn_fd = accept(listen_fd, NULL, NULL);
            if (conn_fd >= 0) {
                int one = 1;
                setsockopt(conn_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
            }
            return;
        }
        bool ok = imported ? handle_urb() : handle_op();
        if (!ok) {
            drop_connection();
            return;
        }
    }
}

void usbip_frame(void)
{
    frame = (frame + 1) & 0x7FF;
    if (sof_on) dcd_event_sof(0, frame, false);
    /* one packet of the oldest isochronous IN URB per endpoint per frame */
    for (int n = 1; n < 16; n++) {
        urb_t *u = held;
        while (u && !(u->ep == n && u->n_iso && u->dir == USBIP_DIR_IN)) u = u->next;
        if (!u) continue;
        ep_t *e = &eps[n][1];
        uint32_t want = u->iso[u->iso_next].length;
        uint16_t got = 0;
        if (e->busy && !e->stalled) {
            uint16_t take = (uint16_t)(e->total < want ? e->total : want);
            got = ep_copy_in(e, u->buf + u->iso[u->iso_next].offset, take);
            complete_xfer((uint8_t)(n | TUSB_DIR_IN_MASK), got);
        } else {
            stats.iso_empty_packets++;
        }
        u->iso[u->iso_next].actual = got;
        stats.iso_packets++;
        if (++u->iso_next == u->n_iso) reply_submit(u, 0);
    }
    tud_task();
}

bool usbip_connected(void) { return imported; }
const usbip_stats_t *usbip_stats(void) { return &stats; }
