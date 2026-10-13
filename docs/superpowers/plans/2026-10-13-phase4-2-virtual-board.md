# Phase 4.2: The Virtual Board over USB/IP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The firmware with its TinyUSB stack runs as `elf-virtual`, a USB/IP server whose isochronous I/Q comes from the digital twin rendering the stations for exactly the hardware state the firmware set; a user-space USB/IP client proves enumeration, CAT, UAC2 and CW decoding on five bands; the kernel-path tools are ready for the attach run.

**Architecture:** `host/dcd_usbip.c` is a TinyUSB device controller driver that speaks USB/IP: control transfers are played through the stack's stages, bulk and interrupt URBs are held until the stack has data or room, isochronous IN gets one packet per 1 ms frame. `host/twin_link.c` sends the Si5351 register image, relay outputs and path select to `python -m neosim vboard`, which decodes them independently and renders I/Q through the hardware model. `sim/neosim/usbip_client.py` is the test host; `tools/elf_cw.py` is the kernel-path application (ALSA + CDC).

**Tech Stack:** C11, TinyUSB 0.21.0 (git submodule), Linux sockets, Python 3 (numpy, scipy, PyYAML), the shipping C# DSP CLI, usbip/vhci_hcd for the kernel path.

**Spec:** `docs/superpowers/specs/2026-10-13-phase4-firmware-virtual-board-design.md` (§1 items 3–4, §3, §4, §6).

## Global Constraints

- Full-speed device: UAC2 stereo 24-bit (3-byte subslots) 96 kHz, asynchronous isochronous IN (max 582 bytes), CDC-ACM; pid.codes test VID/PID 0x1209/0x0001.
- USB/IP on 127.0.0.1 only; bus id "1-1"; big-endian headers; `ep` is the endpoint number 0–15.
- The twin decodes the state independently of the firmware; a fault in registers, relays or path select blocks the stations.
- Nothing in `make test` needs root; the kernel attach (`tools/vboard-attach.sh`) is run by the user.
- Unix socket paths are relative (107-byte limit).

## Review Focus

- URB lifecycle: held, completed, unlinked; no reply for a freed URB; malformed URBs drop the client but not the server (tests: `test_a_held_urb_can_be_unlinked`, `test_a_malformed_urb_drops_the_client_but_not_the_server`).
- Isochronous pacing and continuity (test: `test_iso_stream_is_continuous_at_the_sample_rate`).
- The independent state decode blocks wrong firmware states (tests: `test_a_wrong_firmware_state_blocks_the_stations`, `test_a_blocked_state_renders_noise_only`).
- Acceptance on five bands, both LO paths (test: `test_cw_station_decodes_through_the_virtual_board`).
- Kernel interop that the user-space client cannot prove: vhci_hcd never forwards SET_ADDRESS (the DCD injects one); RET_SUBMIT for non-ISO carries 0 packets, as the kernel stub sends; iso data is packed. Checked by reading the kernel's usbip code paths; confirmed in the attach run.

---

### Task 1: TinyUSB, the USB/IP device controller and elf-virtual

**Files:**
- Create: submodule `third_party/tinyusb` (tag 0.21.0), `firmware/usb/{tusb_config.h,usb_desc.{c,h},usb_app.{c,h}}`, `firmware/host/{usbip_server.h,dcd_usbip.c,hal_virtual.{c,h},twin_link.{c,h},main.c}`, `sim/neosim/usbip_client.py` (the twin link is exercised in Task 2)
- Test: `firmware/tests/test_usbip.py`
- Modify: `firmware/CMakeLists.txt`

**Interfaces:**
- Consumes: plan 4.1 `elf_core`, `elf_vhw`, `audio_*`, `cat_*`, `radio_*`, `supervisor_*`.
- Produces: `usbip_server_start(port)`, `usbip_poll(timeout_ms)`, `usbip_frame()`, `usbip_stats()`; `hal_virtual_init/i2s_push/now_ms`, global `vboard`; `usb_app_init/frame/streaming`; `twin_link_open(path)`, `twin_link_frame()`; executable `elf-virtual [--port N] [--twin SOCKET | --tone HZ] [--seconds S]`; Python `usbip_client.devlist(host, port)`, `Client(host, port).{control, get_descriptor, string, set_configuration, set_interface, bulk_out, bulk_in, iso_in, submit, wait, unlink}`, `Reply(status, data, iso, actual)`.

- [ ] **Step 1: Add TinyUSB and write the failing tests**

```bash
git submodule add https://github.com/hathach/tinyusb third_party/tinyusb
git -C third_party/tinyusb checkout 0.21.0
git config -f .gitmodules submodule.third_party/tinyusb.shallow true
```

`firmware/tests/test_usbip.py`:

```python
"""The virtual board's USB device over USB/IP, through the user-space client (phase 4 spec §1
item 3): enumeration, CAT over CDC, UAC2 controls and isochronous I/Q, unlink and recovery."""
from __future__ import annotations

import os
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
BUILD = Path(os.environ.get("ELF_FW_BUILD", ROOT / "build" / "firmware"))
sys.path.insert(0, str(ROOT / "sim"))

from neosim import usbip_client as u  # noqa: E402

TONE_HZ = 1000.0


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def board():
    exe = BUILD / "elf-virtual"
    if not exe.exists():
        pytest.skip(f"{exe} not built (make fw)")
    port = _free_port()
    proc = subprocess.Popen([str(exe), "--port", str(port), "--tone", str(TONE_HZ), "--seconds", "30"],
                            stdout=subprocess.PIPE, text=True)
    assert "USB/IP on" in proc.stdout.readline()
    yield port
    proc.terminate()
    proc.wait(5)


def _configured(port: int) -> u.Client:
    c = u.Client("127.0.0.1", port)
    assert c.set_configuration(1) == 0
    return c


def _interfaces(cfg: bytes) -> list[tuple[int, int, int, int]]:
    out, k = [], 0
    while k < len(cfg):
        if cfg[k + 1] == 4:                                   # interface: number, alt, class, subclass
            out.append((cfg[k + 2], cfg[k + 3], cfg[k + 5], cfg[k + 6]))
        k += cfg[k]
    return out


def _endpoints(cfg: bytes) -> dict[int, tuple[int, int]]:
    out, k = {}, 0
    while k < len(cfg):
        if cfg[k + 1] == 5:                                   # endpoint: address -> (type, max packet)
            out[cfg[k + 2]] = (cfg[k + 3] & 3, cfg[k + 4] | cfg[k + 5] << 8)
        k += cfg[k]
    return out


def _samples(packets) -> np.ndarray:
    raw = np.frombuffer(b"".join(packets), dtype=np.uint8).reshape(-1, 3).astype(np.int32)
    v = raw[:, 0] | raw[:, 1] << 8 | raw[:, 2] << 16
    v = np.where(v >= 1 << 23, v - (1 << 24), v)
    return v[0::2] + 1j * v[1::2]


def test_device_list_names_the_board_and_its_interfaces(board):
    (d,) = u.devlist("127.0.0.1", board)
    assert (d.busid, d.vid, d.pid, d.speed) == ("1-1", 0x1209, 0x0001, 2)        # full speed
    assert [i[0] for i in d.interfaces] == [1, 1, 2, 10]                          # audio ×2, CDC, CDC data


def test_descriptors_describe_uac2_iq_and_cdc(board):
    with u.Client("127.0.0.1", board) as c:
        dev = c.get_descriptor(1, 0, 18)
        assert struct.unpack("<HH", dev[8:12]) == (0x1209, 0x0001)
        total = struct.unpack("<H", c.get_descriptor(2, 0, 9)[2:4])[0]
        cfg = c.get_descriptor(2, 0, total)
        assert len(cfg) == total
        assert c.string(2) == "Elf HF Receiver"
    assert _interfaces(cfg) == [(0, 0, 1, 1), (1, 0, 1, 2), (1, 1, 1, 2), (2, 0, 2, 2), (3, 0, 10, 0)]
    eps = _endpoints(cfg)
    assert eps[0x81] == (1, 582)                    # isochronous IN: 97 frames × 6 bytes headroom
    assert eps[0x03] == (2, 64) and eps[0x83] == (2, 64) and eps[0x82][0] == 3
    k = cfg.index(bytes([6, 0x24, 2, 1, 3, 24]))    # UAC2 type I format: 3-byte subslot, 24 bits
    assert k > 0


def test_cat_round_trip_over_cdc(board):
    with _configured(board) as c:
        assert c.control(0x21, 0x22, 1, 2).status == 0                       # DTR
        cmd = b"ID;FA00014050000;FA;XX;"
        assert c.bulk_out(3, cmd).actual == len(cmd)
        got = b""
        while not got.endswith(b"?;"):
            got += c.bulk_in(3, 64).data
        assert got == b"ID019;FA00014050000;?;"


def test_sample_rate_controls_report_96_khz(board):
    with _configured(board) as c:
        cur = c.control(0xA1, 0x01, 0x0100, 0x0400, 4)                    # CUR, SAM_FREQ, clock 4
        assert cur.status == 0 and int.from_bytes(cur.data, "little") == 96000
        rng = c.control(0xA1, 0x02, 0x0100, 0x0400, 14)
        assert struct.unpack("<HIII", rng.data) == (1, 96000, 96000, 0)
        assert c.control(0xA1, 0x01, 0x0200, 0x0400, 1).data == b"\x01"    # clock valid


def test_unsupported_requests_stall(board):
    with _configured(board) as c:
        assert c.control(0x80, 0x55, 0, 0, 4).status == u.EPIPE
        assert c.control(0xA1, 0x01, 0x0300, 0x0400, 4).status == u.EPIPE   # unknown clock control
        assert c.string(2) == "Elf HF Receiver"                              # and the device carries on


def test_iso_stream_is_continuous_at_the_sample_rate(board):
    with _configured(board) as c:
        assert c.set_interface(1, 1) == 0
        t0 = time.monotonic()
        packets = []
        for _ in range(50):
            packets += c.iso_in(1, 8, 582).iso
        elapsed = time.monotonic() - t0
        assert c.set_interface(1, 0) == 0
    iq = _samples(packets[2:])                       # the first packets prime the stream
    assert len(packets) == 400
    assert 0.3 <= elapsed <= 0.6                     # one packet per 1 ms frame: 400 ms of audio
    # 96 kHz: an asynchronous UAC2 source may send one frame more or less per packet
    assert abs(len(iq) - 96 * 398) <= 398
    assert np.allclose(np.abs(iq), 2 ** 22, rtol=1e-6)
    phase = np.unwrap(np.angle(iq))
    slope, icept = np.polyfit(np.arange(len(phase)), phase, 1)
    assert slope * 96000 / (2 * np.pi) == pytest.approx(TONE_HZ, abs=1e-6)
    assert np.max(np.abs(phase - (slope * np.arange(len(phase)) + icept))) < 1e-5   # no lost or repeated sample


def test_a_held_urb_can_be_unlinked(board):
    with _configured(board) as c:
        seq = c.submit(3, u.DIR_IN, 64)               # nothing to read: held like a NAK
        assert c.unlink(seq) == -104                   # -ECONNRESET
        assert c.bulk_out(3, b"ID;").status == 0
        assert c.bulk_in(3, 64).data == b"ID019;"


def test_a_malformed_urb_drops_the_client_but_not_the_server(board):
    with _configured(board) as c:
        hdr = struct.pack(">IIIIIIiiiI", u.CMD_SUBMIT, 99, 0x10002, 1, 0x83, 0, 64, 0, -1, 0) + bytes(8)
        c.sock.sendall(hdr)                            # endpoint field 0x83: not a number 0–15
        with pytest.raises((u.UsbipError, ConnectionError, OSError)):
            c.wait(99)
    with _configured(board) as c:                      # a new client imports the device again
        assert c.string(2) == "Elf HF Receiver"
```

- [ ] **Step 2: Run them to verify they fail**

Run: `cd firmware && python3 -m pytest -q tests/test_usbip.py`
Expected: FAIL — `ImportError: cannot import name 'usbip_client' from 'neosim'`.

- [ ] **Step 3: Implement**

`firmware/usb/tusb_config.h`:

```c
/* TinyUSB configuration for Elf: UAC2 stereo 24-bit 96 kHz capture (I/Q) + CDC-ACM (CAT),
 * full speed (the ESP32-S3's USB). The host build sets CFG_TUSB_MCU = OPT_MCU_NONE and runs
 * the device controller over USB/IP (host/dcd_usbip.c). */
#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined by the build
#endif
#if CFG_TUSB_MCU == OPT_MCU_NONE
#define TUP_DCD_ENDPOINT_MAX 8
#endif

#define CFG_TUSB_OS OPT_OS_NONE
#define CFG_TUSB_DEBUG 0
#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC 1
#define CFG_TUD_AUDIO 1
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256
#define CFG_TUD_CDC_EP_BUFSIZE 64

/* audio: 2 channels (I, Q), 3-byte samples, 96 kHz, isochronous IN */
#define ELF_AUDIO_RATE 96000
#define ELF_AUDIO_CHANNELS 2
#define ELF_AUDIO_BYTES 3
#define ELF_AUDIO_EP_SIZE TUD_AUDIO_EP_SIZE(0, ELF_AUDIO_RATE, ELF_AUDIO_BYTES, ELF_AUDIO_CHANNELS)
#define CFG_TUD_AUDIO_ENABLE_EP_IN 1
#define CFG_TUD_AUDIO_ENABLE_EP_OUT 0
#define CFG_TUD_AUDIO_EP_IN_FLOW_CONTROL 1
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX ELF_AUDIO_EP_SIZE
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ (8 * ELF_AUDIO_EP_SIZE)
#define CFG_TUD_AUDIO_CTRL_BUF_SZ 64

#endif
```

`firmware/usb/usb_desc.h`:

```c
#ifndef ELF_USB_DESC_H
#define ELF_USB_DESC_H

/* pid.codes test VID/PID (https://pid.codes/1209/0001/) until the project has its own */
#define ELF_USB_VID 0x1209
#define ELF_USB_PID 0x0001
#define ELF_USB_BCD 0x0100

enum { ITF_AUDIO_CONTROL = 0, ITF_AUDIO_STREAMING, ITF_CDC, ITF_CDC_DATA, ITF_TOTAL };
enum { EP_AUDIO_IN = 0x81, EP_CDC_NOTIF = 0x82, EP_CDC_OUT = 0x03, EP_CDC_IN = 0x83 };
enum { UAC2_ENTITY_INPUT_TERMINAL = 0x01, UAC2_ENTITY_OUTPUT_TERMINAL = 0x03, UAC2_ENTITY_CLOCK = 0x04 };

#endif
```

`firmware/usb/usb_desc.c`:

```c
/* USB descriptors: composite device with an IAD per function.
 * Audio (UAC2): clock source (internal, fixed 96 kHz) → input terminal (2 channels: I = front
 * left, Q = front right) → output terminal (USB streaming); one streaming interface with a
 * zero-bandwidth alternate 0 and alternate 1 = 24-bit PCM in 3-byte subslots, asynchronous
 * isochronous IN. CDC-ACM for CAT. */
#include <string.h>

#include "tusb.h"
#include "usb_desc.h"

static const tusb_desc_device_t device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = ELF_USB_VID,
    .idProduct = ELF_USB_PID,
    .bcdDevice = ELF_USB_BCD,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

const uint8_t *tud_descriptor_device_cb(void) { return (const uint8_t *)&device; }

#define AUDIO_AC_LEN (TUD_AUDIO20_DESC_CLK_SRC_LEN + TUD_AUDIO20_DESC_INPUT_TERM_LEN + TUD_AUDIO20_DESC_OUTPUT_TERM_LEN)
#define AUDIO_DESC_LEN                                                                                  \
    (TUD_AUDIO20_DESC_IAD_LEN + TUD_AUDIO20_DESC_STD_AC_LEN + TUD_AUDIO20_DESC_CS_AC_LEN + AUDIO_AC_LEN + \
     2 * TUD_AUDIO20_DESC_STD_AS_LEN + TUD_AUDIO20_DESC_CS_AS_INT_LEN + TUD_AUDIO20_DESC_TYPE_I_FORMAT_LEN + \
     TUD_AUDIO20_DESC_STD_AS_ISO_EP_LEN + TUD_AUDIO20_DESC_CS_AS_ISO_EP_LEN)
#define CONFIG_LEN (TUD_CONFIG_DESC_LEN + AUDIO_DESC_LEN + TUD_CDC_DESC_LEN)
#define IQ_CHANNELS (AUDIO20_CHANNEL_CONFIG_FRONT_LEFT | AUDIO20_CHANNEL_CONFIG_FRONT_RIGHT)

static const uint8_t configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_TOTAL, 0, CONFIG_LEN, 0x00, 500),
    TUD_AUDIO20_DESC_IAD(ITF_AUDIO_CONTROL, 2, 0),
    TUD_AUDIO20_DESC_STD_AC(ITF_AUDIO_CONTROL, 0, 4),
    TUD_AUDIO20_DESC_CS_AC(0x0200, AUDIO20_FUNC_MICROPHONE, AUDIO_AC_LEN, 0),
    TUD_AUDIO20_DESC_CLK_SRC(UAC2_ENTITY_CLOCK, AUDIO20_CLOCK_SOURCE_ATT_INT_FIX_CLK,
                             (AUDIO20_CTRL_R << AUDIO20_CLOCK_SOURCE_CTRL_CLK_FRQ_POS), UAC2_ENTITY_INPUT_TERMINAL, 0),
    TUD_AUDIO20_DESC_INPUT_TERM(UAC2_ENTITY_INPUT_TERMINAL, AUDIO_TERM_TYPE_IN_GENERIC_MIC, 0, UAC2_ENTITY_CLOCK,
                                ELF_AUDIO_CHANNELS, IQ_CHANNELS, 0, 0, 0),
    TUD_AUDIO20_DESC_OUTPUT_TERM(UAC2_ENTITY_OUTPUT_TERMINAL, AUDIO_TERM_TYPE_USB_STREAMING, 0,
                                 UAC2_ENTITY_INPUT_TERMINAL, UAC2_ENTITY_CLOCK, 0, 0),
    TUD_AUDIO20_DESC_STD_AS_INT(ITF_AUDIO_STREAMING, 0, 0, 0),
    TUD_AUDIO20_DESC_STD_AS_INT(ITF_AUDIO_STREAMING, 1, 1, 0),
    TUD_AUDIO20_DESC_CS_AS_INT(UAC2_ENTITY_OUTPUT_TERMINAL, AUDIO20_CTRL_NONE, AUDIO20_FORMAT_TYPE_I,
                               AUDIO20_DATA_FORMAT_TYPE_I_PCM, ELF_AUDIO_CHANNELS, IQ_CHANNELS, 0),
    TUD_AUDIO20_DESC_TYPE_I_FORMAT(ELF_AUDIO_BYTES, 24),
    TUD_AUDIO20_DESC_STD_AS_ISO_EP(EP_AUDIO_IN,
                                   (uint8_t)(TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_ASYNCHRONOUS | TUSB_ISO_EP_ATT_DATA),
                                   ELF_AUDIO_EP_SIZE, 1),
    TUD_AUDIO20_DESC_CS_AS_ISO_EP(AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, AUDIO20_CTRL_NONE,
                                  AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, 0),
    TUD_CDC_DESCRIPTOR(ITF_CDC, 5, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 64),
};

_Static_assert(sizeof configuration == CONFIG_LEN, "configuration length");

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return configuration;
}

static const char *const strings[] = {
    NULL,                       /* 0: language */
    "Elf",                      /* 1: manufacturer */
    "Elf HF Receiver",          /* 2: product (the ALSA card name) */
    "ELF0001",                  /* 3: serial */
    "Elf I/Q",                  /* 4: audio function */
    "Elf CAT",                  /* 5: CDC */
};

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    static uint16_t out[32];
    size_t n;
    if (index == 0) {
        out[1] = 0x0409;
        n = 1;
    } else {
        if (index >= sizeof strings / sizeof strings[0]) return NULL;
        const char *s = strings[index];
        n = strlen(s);
        if (n > 31) n = 31;
        for (size_t k = 0; k < n; k++) out[1 + k] = (uint8_t)s[k];
    }
    out[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return out;
}
```

`firmware/usb/usb_app.h`:

```c
/* USB application glue: UAC2 class requests, I/Q streaming and CAT over CDC. Shared by the
 * target and the virtual board. */
#ifndef ELF_USB_APP_H
#define ELF_USB_APP_H

#include <stdbool.h>

void usb_app_init(void);
/* Call every 1 ms: moves one frame's I/Q (96 frames) from the audio ring to the USB FIFO while
 * the host streams; otherwise drains the ring so stale samples never reach a new stream. */
void usb_app_frame(void);
bool usb_app_streaming(void);

#endif
```

`firmware/usb/usb_app.c`:

```c
#include "usb_app.h"

#include "audio.h"
#include "cat.h"
#include "tusb.h"
#include "usb_desc.h"

static bool streaming;

static void cat_out(const char *s, size_t n)
{
    tud_cdc_write(s, (uint32_t)n);
    tud_cdc_write_flush();
}

void usb_app_init(void)
{
    streaming = false;
    cat_init(cat_out);
}

bool usb_app_streaming(void) { return streaming; }

void usb_app_frame(void)
{
    uint8_t pkt[AUDIO_PACKET_BYTES];
    audio_packet(pkt, AUDIO_FRAMES_PER_MS);
    if (streaming) tud_audio_write(pkt, AUDIO_PACKET_BYTES);
}

/* ---------- CDC: CAT ---------- */

void tud_cdc_rx_cb(uint8_t itf)
{
    (void)itf;
    uint8_t buf[64];
    uint32_t n;
    while ((n = tud_cdc_read(buf, sizeof buf)) > 0) cat_input(buf, n);
}

/* ---------- UAC2 ---------- */

bool tud_audio_set_itf_cb(uint8_t rhport, const tusb_control_request_t *req)
{
    (void)rhport;
    if (TU_U16_LOW(req->wIndex) == ITF_AUDIO_STREAMING) streaming = TU_U16_LOW(req->wValue) == 1;
    return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, const tusb_control_request_t *req)
{
    (void)rhport;
    if (TU_U16_LOW(req->wIndex) == ITF_AUDIO_STREAMING) streaming = false;
    return true;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, const tusb_control_request_t *req)
{
    uint8_t entity = TU_U16_HIGH(req->wIndex), sel = TU_U16_HIGH(req->wValue);
    if (entity == UAC2_ENTITY_CLOCK && sel == AUDIO20_CS_CTRL_SAM_FREQ) {
        if (req->bRequest == AUDIO20_CS_REQ_CUR) {
            audio20_control_cur_4_t f = {.bCur = (int32_t)ELF_AUDIO_RATE};
            return tud_audio_buffer_and_schedule_control_xfer(rhport, req, &f, sizeof f);
        }
        if (req->bRequest == AUDIO20_CS_REQ_RANGE) {
            audio20_control_range_4_n_t(1) r = {.wNumSubRanges = 1};
            r.subrange[0].bMin = ELF_AUDIO_RATE;
            r.subrange[0].bMax = ELF_AUDIO_RATE;
            r.subrange[0].bRes = 0;
            return tud_audio_buffer_and_schedule_control_xfer(rhport, req, &r, sizeof r);
        }
    }
    if (entity == UAC2_ENTITY_CLOCK && sel == AUDIO20_CS_CTRL_CLK_VALID && req->bRequest == AUDIO20_CS_REQ_CUR) {
        audio20_control_cur_1_t v = {.bCur = 1};
        return tud_audio_buffer_and_schedule_control_xfer(rhport, req, &v, sizeof v);
    }
    if (entity == UAC2_ENTITY_INPUT_TERMINAL && sel == AUDIO20_TE_CTRL_CONNECTOR) {
        audio20_desc_channel_cluster_t c = {.bNrChannels = ELF_AUDIO_CHANNELS,
                                            .bmChannelConfig = (audio20_channel_config_t)(AUDIO20_CHANNEL_CONFIG_FRONT_LEFT |
                                                                                          AUDIO20_CHANNEL_CONFIG_FRONT_RIGHT),
                                            .iChannelNames = 0};
        return tud_audio_buffer_and_schedule_control_xfer(rhport, req, &c, sizeof c);
    }
    return false;                                   /* stall: not supported */
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, const tusb_control_request_t *req, uint8_t *buf)
{
    (void)rhport;
    /* the clock is fixed: accept a host setting it to the only rate there is */
    uint8_t entity = TU_U16_HIGH(req->wIndex), sel = TU_U16_HIGH(req->wValue);
    if (entity == UAC2_ENTITY_CLOCK && sel == AUDIO20_CS_CTRL_SAM_FREQ && req->bRequest == AUDIO20_CS_REQ_CUR &&
        req->wLength == 4)
        return tu_unaligned_read32(buf) == ELF_AUDIO_RATE;
    return false;
}
```

`firmware/host/usbip_server.h`:

```c
/* USB/IP server side of the virtual board's device controller (dcd_usbip.c). */
#ifndef ELF_USBIP_SERVER_H
#define ELF_USBIP_SERVER_H

#include <stdbool.h>
#include <stdint.h>

#define USBIP_PORT 3240

typedef struct {
    uint32_t imports, urbs_received, urbs_completed, unlinks, write_errors;
    uint32_t iso_packets, iso_empty_packets;
} usbip_stats_t;

/* Listens on 127.0.0.1:port; returns 0 or -1. */
int usbip_server_start(uint16_t port);
/* Waits up to timeout_ms for the client and handles one message. */
void usbip_poll(int timeout_ms);
/* The 1 ms frame tick: SOF and one isochronous packet per streaming endpoint. */
void usbip_frame(void);
bool usbip_connected(void);
const usbip_stats_t *usbip_stats(void);

#endif
```

`firmware/host/dcd_usbip.c`:

```c
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
```

`firmware/host/hal_virtual.h`:

```c
/* hal.h for the virtual board: the parts are vhw.c models; I2S frames come from a source the
 * main loop feeds once per 1 ms frame (a test tone, or the digital twin). */
#ifndef ELF_HAL_VIRTUAL_H
#define ELF_HAL_VIRTUAL_H

#include <stddef.h>
#include <stdint.h>

#include "vhw.h"

extern vhw_t vboard;

void hal_virtual_init(void);
/* Queues frames for hal_i2s_read (interleaved I, Q, signed 24-bit in int32). */
void hal_virtual_i2s_push(const int32_t *iq, size_t frames);
/* Milliseconds since hal_virtual_init. */
uint64_t hal_virtual_now_ms(void);

#endif
```

`firmware/host/hal_virtual.c`:

```c
#include "hal_virtual.h"

#include <time.h>

#include "hal.h"

#define I2S_QUEUE_FRAMES 16384u             /* power of two, ≈ 170 ms */

vhw_t vboard;
static int32_t i2s[I2S_QUEUE_FRAMES][2];
static uint32_t i2s_head, i2s_tail;
static struct timespec t0;

void hal_virtual_init(void)
{
    vhw_reset(&vboard);
    i2s_head = i2s_tail = 0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
}

uint64_t hal_virtual_now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)(t.tv_sec - t0.tv_sec) * 1000u + (uint64_t)((t.tv_nsec - t0.tv_nsec) / 1000000);
}

void hal_virtual_i2s_push(const int32_t *iq, size_t frames)
{
    for (size_t k = 0; k < frames; k++) {
        if (i2s_head - i2s_tail >= I2S_QUEUE_FRAMES) i2s_tail++;   /* the ADC never waits: oldest frame lost */
        i2s[i2s_head % I2S_QUEUE_FRAMES][0] = iq[2 * k];
        i2s[i2s_head % I2S_QUEUE_FRAMES][1] = iq[2 * k + 1];
        i2s_head++;
    }
}

int hal_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len) { return vhw_i2c_write(&vboard, addr, reg, data, len); }
int hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) { return vhw_i2c_read(&vboard, addr, reg, data, len); }
void hal_gpio_set(hal_gpio_t pin, bool level) { vhw_gpio_set(&vboard, pin, level); }
bool hal_gpio_get(hal_gpio_t pin) { return vboard.gpio[pin]; }
uint32_t hal_adc_mv(hal_adc_t ch) { return vboard.adc_mv[ch]; }

size_t hal_i2s_read(int32_t *iq, size_t max_frames)
{
    size_t n = 0;
    while (n < max_frames && i2s_tail != i2s_head) {
        iq[2 * n] = i2s[i2s_tail % I2S_QUEUE_FRAMES][0];
        iq[2 * n + 1] = i2s[i2s_tail % I2S_QUEUE_FRAMES][1];
        i2s_tail++;
        n++;
    }
    return n;
}

uint32_t hal_millis(void) { return (uint32_t)hal_virtual_now_ms(); }

/* TinyUSB's time source (OPT_OS_NONE) */
uint32_t tusb_time_millis_api(void);
uint32_t tusb_time_millis_api(void) { return hal_millis(); }

void hal_delay_ms(uint32_t ms)
{
    struct timespec d = {.tv_sec = ms / 1000, .tv_nsec = (long)(ms % 1000) * 1000000L};
    nanosleep(&d, NULL);
}
```

`firmware/host/twin_link.h`:

```c
/* The virtual board's link to the digital twin (sim/neosim/vboard.py): sends the hardware state
 * the firmware set (Si5351 registers, relay outputs, LO path select) whenever it changes, and
 * feeds the virtual CS5361 with the I/Q the twin renders for it. */
#ifndef ELF_TWIN_LINK_H
#define ELF_TWIN_LINK_H

#include <stdbool.h>
#include <stdint.h>

/* Connects to the twin's Unix socket; returns 0 or -1. */
int twin_link_open(const char *socket_path);
/* One 1 ms frame: state update if changed, read what has arrived, push 96 frames to the ADC
 * (silence when the twin has not delivered yet). */
void twin_link_frame(void);
bool twin_link_up(void);
uint32_t twin_link_short_frames(void);   /* frames filled with silence */

#endif
```

`firmware/host/twin_link.c`:

```c
#include "twin_link.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "audio.h"
#include "hal_virtual.h"

#define FIFO_FRAMES 65536u                /* power of two */
#define REQUEST_FRAMES 4800u              /* 50 ms per request */
#define LOW_WATER 9600u                   /* keep ≈ 100 ms queued */

static int fd = -1;
static int32_t fifo[FIFO_FRAMES][2];
static uint32_t head, tail, requested, short_frames;
static uint8_t rx[16 + 8 * REQUEST_FRAMES];
static size_t rx_len;
static uint32_t last_writes = UINT32_MAX, last_relays = UINT32_MAX;
static int last_path = -1;

int twin_link_open(const char *path)
{
    struct sockaddr_un a = {.sun_family = AF_UNIX};
    if (strlen(path) >= sizeof a.sun_path) return -1;
    strcpy(a.sun_path, path);
    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    if (connect(fd, (struct sockaddr *)&a, sizeof a)) {
        close(fd);
        fd = -1;
        return -1;
    }
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
    return 0;
}

bool twin_link_up(void) { return fd >= 0; }
uint32_t twin_link_short_frames(void) { return short_frames; }

static void put32le(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint32_t get32le(const uint8_t *p) { return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

static void lost(void)
{
    fprintf(stderr, "elf-virtual: twin link lost\n");
    close(fd);
    fd = -1;
}

/* small control messages: block until sent (the socket buffer always has room for them) */
static void send_all(const uint8_t *p, size_t n)
{
    while (n && fd >= 0) {
        ssize_t w = send(fd, p, n, MSG_NOSIGNAL);
        if (w < 0 && (errno == EAGAIN || errno == EINTR)) continue;
        if (w <= 0) {
            lost();
            return;
        }
        p += w;
        n -= (size_t)w;
    }
}

static void send_state(void)
{
    uint8_t m[1 + 256 + 5];
    m[0] = 'S';
    memcpy(m + 1, vboard.si5351, 256);
    put32le(m + 257, vhw_relay_outputs(&vboard));
    m[261] = vboard.gpio[HAL_GPIO_LO_PATH] ? 1 : 0;
    send_all(m, sizeof m);
    /* the new state's I/Q replaces whatever is queued for the old one */
    tail = head;
}

static void receive(void)
{
    for (;;) {
        ssize_t r = recv(fd, rx + rx_len, sizeof rx - rx_len, 0);
        if (r < 0 && (errno == EAGAIN || errno == EINTR)) break;
        if (r <= 0) {
            lost();
            return;
        }
        rx_len += (size_t)r;
        /* complete D messages */
        while (rx_len >= 5 && rx[0] == 'D') {
            uint32_t n = get32le(rx + 1);
            size_t need = 5 + 8 * (size_t)n;
            if (rx_len < need) break;
            for (uint32_t k = 0; k < n; k++) {
                if (head - tail < FIFO_FRAMES) {
                    fifo[head % FIFO_FRAMES][0] = (int32_t)get32le(rx + 5 + 8 * k);
                    fifo[head % FIFO_FRAMES][1] = (int32_t)get32le(rx + 9 + 8 * k);
                    head++;
                }
            }
            requested -= n < requested ? n : requested;
            memmove(rx, rx + need, rx_len - need);
            rx_len -= need;
        }
        if (rx_len >= 1 && rx[0] != 'D') {
            lost();
            return;
        }
    }
}

void twin_link_frame(void)
{
    int32_t iq[2 * AUDIO_FRAMES_PER_MS];
    if (fd >= 0) {
        int path = vboard.gpio[HAL_GPIO_LO_PATH];
        uint32_t relays = vhw_relay_outputs(&vboard);
        if (vboard.si5351_writes != last_writes || relays != last_relays || path != last_path) {
            last_writes = vboard.si5351_writes;
            last_relays = relays;
            last_path = path;
            send_state();
        }
        if (fd >= 0) receive();
        if (fd >= 0 && requested == 0 && head - tail < LOW_WATER) {
            uint8_t m[5] = {'R'};
            put32le(m + 1, REQUEST_FRAMES);
            send_all(m, sizeof m);
            requested = REQUEST_FRAMES;
        }
    }
    for (unsigned k = 0; k < AUDIO_FRAMES_PER_MS; k++) {
        if (head != tail) {
            iq[2 * k] = fifo[tail % FIFO_FRAMES][0];
            iq[2 * k + 1] = fifo[tail % FIFO_FRAMES][1];
            tail++;
        } else {
            iq[2 * k] = iq[2 * k + 1] = 0;
            short_frames++;
        }
    }
    hal_virtual_i2s_push(iq, AUDIO_FRAMES_PER_MS);
}
```

`sim/neosim/usbip_client.py`:

```python
"""User-space USB/IP client (phase 4 spec §4): imports a device from a USB/IP server and
issues URBs the way vhci_hcd does, so the virtual board's USB stack is tested without root or
kernel modules. Linux Documentation/usb/usbip_protocol.rst; headers are big-endian."""
from __future__ import annotations

import socket
import struct
from dataclasses import dataclass

USBIP_VERSION = 0x0111
OP_REQ_DEVLIST, OP_REP_DEVLIST, OP_REQ_IMPORT, OP_REP_IMPORT = 0x8005, 0x0005, 0x8003, 0x0003
CMD_SUBMIT, CMD_UNLINK, RET_SUBMIT, RET_UNLINK = 1, 2, 3, 4
DIR_OUT, DIR_IN = 0, 1
EPIPE = -32


class UsbipError(RuntimeError):
    pass


@dataclass(frozen=True)
class Device:
    path: str
    busid: str
    busnum: int
    devnum: int
    speed: int
    vid: int
    pid: int
    bcd: int
    dev_class: int
    configuration: int
    n_configurations: int
    n_interfaces: int
    interfaces: tuple[tuple[int, int, int], ...] = ()


@dataclass(frozen=True)
class Reply:
    status: int
    data: bytes
    iso: tuple[bytes, ...] = ()      # isochronous IN: each packet's data
    actual: int = 0                  # bytes transferred (OUT: accepted by the device)


def _device(raw: bytes) -> Device:
    path = raw[:256].split(b"\0")[0].decode()
    busid = raw[256:288].split(b"\0")[0].decode()
    busnum, devnum, speed, vid, pid, bcd = struct.unpack(">IIIHHH", raw[288:306])
    dclass, _sub, _proto, conf, nconf, nitf = raw[306:312]
    return Device(path, busid, busnum, devnum, speed, vid, pid, bcd, dclass, conf, nconf, nitf)


def _recv(sock: socket.socket, n: int) -> bytes:
    out = bytearray()
    while len(out) < n:
        chunk = sock.recv(n - len(out))
        if not chunk:
            raise UsbipError("connection closed by the server")
        out += chunk
    return bytes(out)


def devlist(host: str, port: int) -> list[Device]:
    with socket.create_connection((host, port), timeout=5) as s:
        s.sendall(struct.pack(">HHI", USBIP_VERSION, OP_REQ_DEVLIST, 0))
        _ver, code, status, n = struct.unpack(">HHII", _recv(s, 12))
        if code != OP_REP_DEVLIST or status:
            raise UsbipError(f"devlist failed: code {code:#x} status {status}")
        out = []
        for _ in range(n):
            d = _device(_recv(s, 312))
            itfs = tuple(tuple(_recv(s, 4)[:3]) for _ in range(d.n_interfaces))
            out.append(Device(**{**d.__dict__, "interfaces": itfs}))
        return out


class Client:
    """One imported device. Methods block until the URB completes."""

    def __init__(self, host: str, port: int, busid: str = "1-1", timeout: float = 5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.sendall(struct.pack(">HHI", USBIP_VERSION, OP_REQ_IMPORT, 0) + busid.encode().ljust(32, b"\0"))
        _ver, code, status = struct.unpack(">HHI", _recv(self.sock, 8))
        if code != OP_REP_IMPORT or status:
            raise UsbipError(f"import of {busid} refused (status {status})")
        self.device = _device(_recv(self.sock, 312))
        self.seq = 0
        self.pending: dict[int, Reply] = {}
        self.direction: dict[int, int] = {}     # seqnum -> direction: an OUT reply carries no data

    def close(self) -> None:
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- URBs ----

    def submit(self, ep: int, direction: int, length: int = 0, data: bytes = b"", setup: bytes = bytes(8),
               iso: list[int] | None = None) -> int:
        self.seq += 1
        self.direction[self.seq] = direction
        buf_len = len(data) if direction == DIR_OUT else length
        hdr = struct.pack(">IIIII", CMD_SUBMIT, self.seq, 0x00010002, direction, ep & 0x0F)   # number only
        hdr += struct.pack(">IiiiI", 0, buf_len, 0, len(iso) if iso else -1, 1 if iso else 0) + setup
        body = data if direction == DIR_OUT else b""
        if iso:
            off = 0
            for size in iso:
                body += struct.pack(">IIIi", off, size, 0, 0)
                off += size
        self.sock.sendall(hdr + body)
        return self.seq

    def wait(self, seq: int) -> Reply:
        while seq not in self.pending:
            h = _recv(self.sock, 48)
            cmd, rseq = struct.unpack(">II", h[:8])
            if cmd == RET_UNLINK:
                self.pending[rseq] = Reply(struct.unpack(">i", h[20:24])[0], b"")
                continue
            if cmd != RET_SUBMIT:
                raise UsbipError(f"unexpected reply {cmd}")
            status, actual, _start, npk, _err = struct.unpack(">iiiii", h[20:40])
            inbound = self.direction.pop(rseq, DIR_IN) == DIR_IN
            data = _recv(self.sock, actual) if inbound and actual > 0 else b""
            packets: tuple[bytes, ...] = ()
            if npk not in (0, -1):
                descs = [struct.unpack(">IIIi", _recv(self.sock, 16)) for _ in range(npk)]
                pos, out = 0, []
                for _off, _length, act, _st in descs:
                    out.append(data[pos:pos + act])
                    pos += act
                packets = tuple(out)
            self.pending[rseq] = Reply(status, data, packets, actual)
        return self.pending.pop(seq)

    def unlink(self, victim: int) -> int:
        self.seq += 1
        hdr = struct.pack(">IIIIII", CMD_UNLINK, self.seq, 0x00010002, 0, 0, victim) + bytes(24)
        self.sock.sendall(hdr)
        return self.wait(self.seq).status

    # ---- conveniences ----

    def control(self, request_type: int, request: int, value: int, index: int, length: int = 0,
                data: bytes = b"") -> Reply:
        direction = DIR_IN if request_type & 0x80 else DIR_OUT
        setup = struct.pack("<BBHHH", request_type, request, value, index, length if direction == DIR_IN else len(data))
        return self.wait(self.submit(0, direction, length, data, setup))

    def get_descriptor(self, dtype: int, index: int = 0, length: int = 255, lang: int = 0) -> bytes:
        r = self.control(0x80, 6, dtype << 8 | index, lang, length)
        if r.status:
            raise UsbipError(f"GET_DESCRIPTOR {dtype}/{index}: status {r.status}")
        return r.data

    def string(self, index: int) -> str:
        return self.get_descriptor(3, index, 255, 0x0409)[2:].decode("utf-16-le")

    def set_configuration(self, value: int) -> int:
        return self.control(0x00, 9, value, 0).status

    def set_interface(self, interface: int, alt: int) -> int:
        return self.control(0x01, 11, alt, interface).status

    def bulk_out(self, ep: int, data: bytes) -> Reply:
        return self.wait(self.submit(ep, DIR_OUT, data=data))

    def bulk_in(self, ep: int, length: int = 64) -> Reply:
        return self.wait(self.submit(ep, DIR_IN, length))

    def iso_in(self, ep: int, packets: int, packet_size: int) -> Reply:
        return self.wait(self.submit(ep, DIR_IN, packets * packet_size, iso=[packet_size] * packets))
```

`firmware/host/main.c`:

```c
/* elf-virtual: the Elf firmware running as a Linux process, its USB device served over USB/IP.
 *
 *   elf-virtual [--port N] [--twin SOCKET | --tone HZ] [--seconds S]
 *
 * --twin takes the ADC's I/Q from the digital twin (python -m neosim vboard STATIONS --socket
 * SOCKET), rendered for whatever the firmware set the hardware to; --tone feeds a complex tone at
 * HZ instead (I = cos, Q = sin, −6 dBFS; the default, 1 kHz); --seconds exits after S seconds
 * (tests). Attach with
 *   sudo modprobe vhci-hcd && sudo usbip attach -r 127.0.0.1 -b 1-1 */
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "audio.h"
#include "hal_virtual.h"
#include "radio.h"
#include "supervisor.h"
#include "tables_gen.h"
#include "tusb.h"
#include "twin_link.h"
#include "usb_app.h"
#include "usbip_server.h"

static volatile sig_atomic_t stop;

static void on_signal(int sig)
{
    (void)sig;
    stop = 1;
}

/* board state the supervisor reads: a 12 V adapter on the jack, 5V_MAIN in regulation, a USB
 * host connected (the monitors see the node voltages through the dividers in power.json) */
static void set_supplies(void)
{
    vboard.adc_mv[HAL_ADC_SENSE_EXT] = (uint32_t)(11600ull * 1000 / TABLES_SENSE_DIV_EXT_X1000);
    vboard.adc_mv[HAL_ADC_SENSE_MAIN] = (uint32_t)(5480ull * 1000 / TABLES_SENSE_DIV_MAIN_X1000);
    vboard.adc_mv[HAL_ADC_SENSE_VBUS] = (uint32_t)(5000ull * 1000 / TABLES_SENSE_DIV_VBUS_X1000);
    vboard.adc_mv[HAL_ADC_BIAS_ISENSE] = 628;
}

static void tone_frame(double hz, uint64_t frame)
{
    int32_t iq[2 * AUDIO_FRAMES_PER_MS];
    const double a = 4194304.0;                   /* 2^22: −6 dBFS of 24 bits */
    for (unsigned k = 0; k < AUDIO_FRAMES_PER_MS; k++) {
        double t = (double)(frame * AUDIO_FRAMES_PER_MS + k) / AUDIO_RATE_HZ;
        iq[2 * k] = (int32_t)lround(a * cos(2 * M_PI * hz * t));
        iq[2 * k + 1] = (int32_t)lround(a * sin(2 * M_PI * hz * t));
    }
    hal_virtual_i2s_push(iq, AUDIO_FRAMES_PER_MS);
}

int main(int argc, char **argv)
{
    unsigned port = USBIP_PORT;
    double tone_hz = 1000.0, seconds = 0;
    const char *twin = NULL;
    for (int k = 1; k < argc; k++) {
        if (!strcmp(argv[k], "--port") && k + 1 < argc) port = (unsigned)atoi(argv[++k]);
        else if (!strcmp(argv[k], "--tone") && k + 1 < argc) tone_hz = atof(argv[++k]);
        else if (!strcmp(argv[k], "--seconds") && k + 1 < argc) seconds = atof(argv[++k]);
        else if (!strcmp(argv[k], "--twin") && k + 1 < argc) twin = argv[++k];
        else {
            fprintf(stderr, "usage: elf-virtual [--port N] [--twin SOCKET | --tone HZ] [--seconds S]\n");
            return 2;
        }
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    hal_virtual_init();
    set_supplies();
    radio_init();
    supervisor_init();
    audio_init();
    usb_app_init();
    tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
    tusb_init(0, &dev_init);
    if (twin && twin_link_open(twin)) {
        fprintf(stderr, "elf-virtual: cannot reach the twin at %s\n", twin);
        return 1;
    }
    if (usbip_server_start((uint16_t)port)) {
        perror("elf-virtual: listen");
        return 1;
    }
    printf("elf-virtual: USB/IP on 127.0.0.1:%u, bus id 1-1\n", port);
    fflush(stdout);

    uint64_t frames = 0;
    while (!stop) {
        uint64_t now = hal_virtual_now_ms();
        if (seconds > 0 && now >= (uint64_t)(seconds * 1000)) break;
        while (frames < now) {                     /* catch up after a slow iteration */
            if (twin) twin_link_frame();
            else tone_frame(tone_hz, frames);
            audio_pump();
            usb_app_frame();
            usbip_frame();
            if (frames % 10 == 0) supervisor_poll();
            frames++;
        }
        usbip_poll(1);
        tud_task();
    }
    const usbip_stats_t *s = usbip_stats();
    printf("elf-virtual: imports %u, urbs %u/%u, unlinks %u, iso packets %u (%u empty), audio underruns %u, "
           "twin silence frames %u\n", s->imports, s->urbs_completed, s->urbs_received, s->unlinks, s->iso_packets,
           s->iso_empty_packets, audio_stats()->underruns, twin_link_short_frames());
    return 0;
}
```

`firmware/CMakeLists.txt (append)`:

```cmake
# ---- USB: TinyUSB (third_party/tinyusb, release 0.21.0) and the virtual board ----
set(TUSB ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/tinyusb/src)
add_library(tinyusb STATIC
    ${TUSB}/tusb.c ${TUSB}/common/tusb_fifo.c ${TUSB}/device/usbd.c
    ${TUSB}/class/audio/audio_device.c ${TUSB}/class/cdc/cdc_device.c)
target_include_directories(tinyusb SYSTEM PUBLIC ${TUSB})
target_include_directories(tinyusb PUBLIC usb host)
target_compile_definitions(tinyusb PUBLIC CFG_TUSB_MCU=OPT_MCU_NONE)
# third-party code: build it as it is, without the project's strict warnings
target_compile_options(tinyusb PRIVATE -w)

# the device controller driver, descriptors and class callbacks are objects, not an archive:
# TinyUSB calls back into them
add_library(elf_usb OBJECT usb/usb_desc.c usb/usb_app.c host/dcd_usbip.c)
target_link_libraries(elf_usb PUBLIC tinyusb elf_core)

add_executable(elf-virtual host/main.c host/hal_virtual.c host/twin_link.c $<TARGET_OBJECTS:elf_usb>)
target_link_libraries(elf-virtual PRIVATE elf_usb elf_vhw tinyusb elf_core m)
```

- [ ] **Step 4: Run them to verify they pass**

Run: `make fw && cd firmware && python3 -m pytest -q tests/test_usbip.py`
Expected: `8 passed`. Also `usbip list -r 127.0.0.1` against a running `build/firmware/elf-virtual` lists `1-1: pid.codes Test PID (1209:0001)` with interfaces 01/01, 01/02, 02/02, 0a/00.

- [ ] **Step 5: Commit**

```bash
git add .gitmodules third_party/tinyusb firmware sim/neosim/usbip_client.py
git commit -m "feat(fw): TinyUSB device over USB/IP: UAC2 I/Q and CDC CAT in elf-virtual, user-space client tests"
```

### Task 2: The twin link, five-band acceptance and the kernel-path tools

**Files:**
- Create: `sim/neosim/vboard.py`, `sim/neosim/cwrx.py`, `sim/scenarios/vboard/cw_bands.yaml`, `tools/elf_cw.py`, `tools/vboard-run.sh`, `tools/vboard-attach.sh`, `tools/vboard-detach.sh`
- Test: `sim/tests/test_vboard.py`, `firmware/tests/test_vboard_cw.py`
- Modify: `sim/neosim/__main__.py` (`vboard`), `Makefile` (`vboard`, `e2e`)

**Interfaces:**
- Consumes: Task 1 `elf-virtual --twin`, `usbip_client`; plan 4.1 `si5351_decode.lo_state`.
- Produces: `vboard.decode_state(regs, relays, path_high, order, hw) -> BoardState`, `vboard.Twin(stations, hw).render(state)`, `vboard.serve(stations, socket)`; `cwrx.iq_from_s24le`, `cwrx.decode(iq, lo, station, workdir)`, `cwrx.best_cer`, `cwrx.LO_OFFSET_HZ`; CLI `python -m neosim vboard STATIONS --socket PATH`.

- [ ] **Step 1: Write the failing tests**

`sim/tests/test_vboard.py`:

```python
from fractions import Fraction

import numpy as np
import pytest

from neosim import hardware, vboard

ORDER = [f"bank{k}_{e}" for k in range(1, 12) for e in ("in", "out")] + ["atten", "preamp"]


def _params(x: Fraction) -> list[int]:
    a, b, c = x.numerator // x.denominator, x.numerator % x.denominator, x.denominator
    f = 128 * b // c
    p1, p2, p3 = 128 * a + f - 512, 128 * b - c * f, c
    return [p3 >> 8 & 0xFF, p3 & 0xFF, p1 >> 16 & 3, p1 >> 8 & 0xFF, p1 & 0xFF,
            (p3 >> 12 & 0xF0) | (p2 >> 16 & 0x0F), p2 >> 8 & 0xFF, p2 & 0xFF]


def _regs_14mhz() -> bytes:
    r = bytearray(256)
    r[3] = 0xFC
    r[16:24] = [0x4F, 0x4F] + [0x80] * 6
    r[26:34] = _params(Fraction(28))                 # PLLA 700 MHz
    r[42:50] = r[50:58] = _params(Fraction(50))      # 14 MHz
    r[166] = 50
    return bytes(r)


def _bits(*names: str) -> int:
    return sum(1 << ORDER.index(n) for n in names)


@pytest.fixture(scope="module")
def hw():
    return hardware.Hardware.load()


def test_a_correct_state_decodes_to_lo_filter_and_front_end(hw):
    s = vboard.decode_state(_regs_14mhz(), _bits("bank7_in", "bank7_out", "preamp"), False, ORDER, hw)
    assert (s.lo_hz, s.filter, s.preamp, s.attenuator, s.reason) == (14e6, 7, True, False, "")


@pytest.mark.parametrize("relays, path, why", [
    (_bits("bank7_in"), False, "no single filter"),                                   # output relay missing
    (_bits("bank7_in", "bank7_out", "bank6_in", "bank6_out"), False, "no single filter"),
    (_bits("bank6_in", "bank6_out"), False, "filter 6 selected for an LO in filter 7"),
    (_bits("bank7_in", "bank7_out"), True, "CLK2 is off"),                            # wrong path select
])
def test_a_wrong_firmware_state_blocks_the_stations(hw, relays, path, why):
    s = vboard.decode_state(_regs_14mhz(), relays, path, ORDER, hw)
    assert why in s.reason


def test_a_blocked_state_renders_noise_only(hw, tmp_path):
    st = tmp_path / "st.yaml"
    st.write_text("name: t\nduration_s: 1.0\nseed: 3\nnoise: {type: awgn, dbm_per_hz: -160}\nstations:\n"
                  "  - {id: c, mode: carrier, rf_hz: 14_010_000, level_dbm: -60}\n")
    twin = vboard.Twin(st, hw)
    good = vboard.decode_state(_regs_14mhz(), _bits("bank7_in", "bank7_out"), False, ORDER, hw)
    bad = vboard.decode_state(_regs_14mhz(), _bits("bank6_in", "bank6_out"), False, ORDER, hw)

    def at_10khz(frames):                            # power in the carrier's bin (LO + 10 kHz), I − jQ
        iq = frames[:, 0].astype(float) - 1j * frames[:, 1]
        spec = np.abs(np.fft.fft(iq)) ** 2
        return spec[int(round(10e3 / 96e3 * len(iq)))]

    assert at_10khz(twin.render(good)) > 1e6 * at_10khz(twin.render(bad))   # 60 dB: the carrier is gone
```

`firmware/tests/test_vboard_cw.py`:

```python
"""Acceptance through the user-space USB/IP client (phase 4 spec §1 item 4, without the kernel):
the virtual board, fed by the twin, is tuned over CAT to CW stations on five bands — 80 m on the
÷4 path, the rest direct — and its isochronous I/Q decodes through the shipping DSP."""
from __future__ import annotations

import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
BUILD = Path(os.environ.get("ELF_FW_BUILD", ROOT / "build" / "firmware"))
STATIONS = ROOT / "sim" / "scenarios" / "vboard" / "cw_bands.yaml"
sys.path.insert(0, str(ROOT / "sim"))

from neosim import cwrx  # noqa: E402
from neosim import usbip_client as u  # noqa: E402
from neosim.tools import ToolNotFoundError, tool_path  # noqa: E402

MAX_CER = 0.05
RECORD_S = 15.0           # one loop of the twin (14 s) plus margin: a whole message is inside


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def vboard(tmp_path_factory):
    exe = BUILD / "elf-virtual"
    if not exe.exists():
        pytest.skip(f"{exe} not built (make fw)")
    try:
        tool_path("neoradio-dsp")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    work = tmp_path_factory.mktemp("vboard")
    env = {**os.environ, "PYTHONPATH": str(ROOT / "sim")}
    twin = subprocess.Popen([sys.executable, "-m", "neosim", "vboard", str(STATIONS), "--socket", "twin.sock"],
                            cwd=work, env=env, stdout=open(work / "twin.log", "w"), stderr=subprocess.STDOUT)
    for _ in range(300):
        if (work / "twin.sock").exists():
            break
        time.sleep(0.1)
    port = _free_port()
    board = subprocess.Popen([str(exe), "--port", str(port), "--twin", "twin.sock", "--seconds", "600"], cwd=work,
                             stdout=subprocess.PIPE, text=True)
    assert "USB/IP on" in board.stdout.readline()
    yield port, work
    board.terminate()
    twin.terminate()
    board.wait(5)
    twin.wait(5)


def _receive(c: u.Client, lo_hz: int, seconds: float) -> bytes:
    assert c.bulk_out(3, f"FA{lo_hz:011d};".encode()).status == 0
    t0 = time.monotonic()
    while True:                                     # the twin renders the new state, then streams it
        r = c.iso_in(1, 8, 582)
        if any(any(p) for p in r.iso):
            break
        assert time.monotonic() - t0 < 60, "no I/Q after tuning"
    raw = b""
    for _ in range(int(seconds * 1000) // 8):
        raw += b"".join(c.iso_in(1, 8, 582).iso)
    return raw


STATION_LIST = yaml.safe_load(STATIONS.read_text())["stations"]


@pytest.mark.parametrize("station", STATION_LIST, ids=[s["id"] for s in STATION_LIST])
def test_cw_station_decodes_through_the_virtual_board(vboard, station):
    port, work = vboard
    rf = int(station["rf_hz"])
    lo = int(rf - cwrx.LO_OFFSET_HZ)
    with u.Client("127.0.0.1", port, timeout=60) as c:
        assert c.set_configuration(1) == 0
        assert c.control(0x21, 0x22, 1, 2).status == 0
        assert c.set_interface(1, 1) == 0
        raw = _receive(c, lo, RECORD_S)
        c.set_interface(1, 0)
    text = cwrx.decode(cwrx.iq_from_s24le(raw), lo, rf, work / station["id"])
    assert cwrx.best_cer(text, station["text"]) <= MAX_CER, text


def test_a_wrong_filter_silences_the_band(vboard):
    """Firmware fault injection from the host side: the attenuator and preamp relays are
    switched over CAT, the bank stays right; then the twin must still decode (sanity), whereas
    tuning outside the bank (60 MHz) is refused by the firmware and leaves the old band."""
    port, _ = vboard
    with u.Client("127.0.0.1", port, timeout=60) as c:
        c.set_configuration(1)
        c.control(0x21, 0x22, 1, 2)
        assert c.bulk_out(3, b"FA00060000000;").status == 0
        got = b""
        while not got.endswith(b";"):
            got += c.bulk_in(3, 64).data
        assert got == b"?;"
```

`sim/scenarios/vboard/cw_bands.yaml`:

```yaml
# Stations the virtual board receives (phase 4 acceptance): CW on five bands, both LO paths
# (80 m is below the 4.8 MHz crossover: the ÷4 path). Levels are at the antenna; the firmware
# tunes the LO 10 kHz below each station and the host demodulates at the station − 600 Hz.
name: virtual board CW, five bands
duration_s: 14.0
seed: 11
noise: {type: awgn, dbm_per_hz: -160}       # quiet band noise
stations:
  - {id: cw80, mode: cw, rf_hz: 3_550_000, level_dbm: -95, text: "CQ DE EP2ABC 80M", wpm: 24}
  - {id: cw40, mode: cw, rf_hz: 7_030_000, level_dbm: -95, text: "CQ DE EP2ABC 40M", wpm: 24}
  - {id: cw20, mode: cw, rf_hz: 14_050_000, level_dbm: -95, text: "CQ DE EP2ABC 20M", wpm: 24}
  - {id: cw15, mode: cw, rf_hz: 21_050_000, level_dbm: -95, text: "CQ DE EP2ABC 15M", wpm: 24}
  - {id: cw6, mode: cw, rf_hz: 50_090_000, level_dbm: -95, text: "CQ DE EP2ABC 6M", wpm: 24}
```

- [ ] **Step 2: Run them to verify they fail**

Run: `cd sim && python3 -m pytest -q tests/test_vboard.py`
Expected: FAIL — `ImportError: cannot import name 'vboard' from 'neosim'`.

- [ ] **Step 3: Implement**

`sim/neosim/vboard.py`:

```python
"""The digital twin behind the virtual board (phase 4 spec §3): elf-virtual sends what its
firmware wrote to the hardware — the Si5351C register image, the relay outputs and the LO path
select — and receives the CS5361's I2S frames for that state.

The state is decoded here independently of the firmware: the LO from the registers (AN619
arithmetic, si5351_decode), the filter, attenuator and preamp from the relay bits (the order the
board netlist wires them, controller.json). The stations are rendered through the hardware
model for exactly that state, so a firmware fault becomes a wrong or silent band:

- no valid quadrature LO (wrong divider, phase offset or path select): receiver noise only;
- a bank state other than one filter's two relays, or a filter that does not cover the LO:
  the stations are blocked (receiver noise only).

Socket protocol (Unix stream, little-endian):
  board → twin  b"S" + 256 register bytes + u32 relay bits + u8 path select    (state changed)
  board → twin  b"R" + u32 n                                                 (want n frames)
  twin → board  b"D" + u32 n + n × (i32 I, i32 Q)       (signed 24-bit ADC codes, L = I, R = Q)
"""
from __future__ import annotations

import json
import socket
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from neosim import hardware, si5351_decode, sources
from neosim.scenario import FS_HZ, WINDOW_HZ, Receiver, Scenario, _station

CHAR_DIR = Path(__file__).resolve().parents[2] / "hardware" / "characterized"
FULL_SCALE = 2 ** 23


@dataclass(frozen=True)
class BoardState:
    lo_hz: float | None
    path: str
    filter: int                    # 0 = muted / invalid bank state
    attenuator: bool
    preamp: bool
    reason: str                    # why the stations are blocked, or ""


def relay_order(char_dir: Path = CHAR_DIR) -> list[str]:
    return json.loads((char_dir / "controller.json").read_text())["relay_chain_order"]


def decode_state(regs: bytes, relays: int, path_high: bool, order: list[str], hw: hardware.Hardware) -> BoardState:
    lo = si5351_decode.lo_state(regs, path_high)
    on = {name for k, name in enumerate(order) if relays >> k & 1}
    banks = sorted({int(n[4:].split("_")[0]) for n in on if n.startswith("bank")})
    filt = banks[0] if len(banks) == 1 and {f"bank{banks[0]}_in", f"bank{banks[0]}_out"} <= on else 0
    reason = lo.reason
    if not reason and filt == 0:
        reason = f"bank relays {sorted(n for n in on if n.startswith('bank'))} select no single filter"
    if not reason:
        want = hw.filter_index(float(lo.lo_hz))
        if want != filt:
            reason = f"filter {filt} selected for an LO in filter {want}"
    return BoardState(None if lo.lo_hz is None else float(lo.lo_hz), lo.path, filt, "atten" in on, "preamp" in on, reason)


class Twin:
    """Renders I/Q for a board state from a stations file (absolute levels, band noise)."""

    def __init__(self, stations_path: Path, hw: hardware.Hardware | None = None):
        d = yaml.safe_load(Path(stations_path).read_text())
        self.name = d.get("name", Path(stations_path).stem)
        self.duration_s = float(d.get("duration_s", 16.0))
        self.seed = int(d.get("seed", 1))
        self.noise_dbm_per_hz = float(d["noise"]["dbm_per_hz"])
        self.stations = tuple(_station(s, self.noise_dbm_per_hz) for s in d["stations"])
        self.hw = hw or hardware.Hardware.load()
        self.order = relay_order()

    def render(self, state: BoardState) -> np.ndarray:
        """int32 frames (n, 2) for the state, looped by the server."""
        lo = state.lo_hz if state.lo_hz is not None else 10e6
        stations = () if state.reason else self.stations
        sc = Scenario(self.name, lo, self.duration_s, self.seed, 0.0, stations, (),
                      Receiver("hardware", state.preamp, 20 if state.attenuator else 0), self.noise_dbm_per_hz)
        rng = np.random.default_rng(self.seed)
        iq, _excluded, _stats = hardware.receive(sc, rng, self.hw, sources.render_station, FS_HZ, WINDOW_HZ)
        # receive() returns di − j·dq; the ADC delivers I = di (left) and Q = dq (right)
        frames = np.empty((len(iq), 2), dtype=np.int32)
        frames[:, 0] = np.round(iq.real * FULL_SCALE).astype(np.int32)
        frames[:, 1] = np.round(-iq.imag * FULL_SCALE).astype(np.int32)
        return frames


def _recv(conn: socket.socket, n: int) -> bytes:
    out = bytearray()
    while len(out) < n:
        chunk = conn.recv(n - len(out))
        if not chunk:
            raise ConnectionError("board closed the link")
        out += chunk
    return bytes(out)


def serve(stations_path: Path, socket_path: Path, log=print) -> None:
    """Serves one board connection after another until interrupted."""
    twin = Twin(stations_path)
    socket_path = Path(socket_path)
    socket_path.unlink(missing_ok=True)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(str(socket_path))
    srv.listen(1)
    log(f"vboard: twin for {twin.name!r} on {socket_path}")
    try:
        while True:
            conn, _ = srv.accept()
            with conn:
                _session(conn, twin, log)
    finally:
        srv.close()
        socket_path.unlink(missing_ok=True)


def _session(conn: socket.socket, twin: Twin, log) -> None:
    frames = np.zeros((FS_HZ // 10, 2), dtype=np.int32)       # silence until the first state
    pos = 0
    cache: dict[BoardState, np.ndarray] = {}
    try:
        while True:
            kind = _recv(conn, 1)
            if kind == b"S":
                regs = _recv(conn, 256)
                relays, path = struct.unpack("<IB", _recv(conn, 5))
                state = decode_state(regs, relays, bool(path), twin.order, twin.hw)
                if state not in cache:
                    cache[state] = twin.render(state)
                frames, pos = cache[state], 0
                lo = "none" if state.lo_hz is None else f"{state.lo_hz / 1e6:.6f} MHz"
                log(f"vboard: LO {lo} ({state.path}), filter {state.filter}, preamp {int(state.preamp)}, "
                    f"atten {int(state.attenuator)}{', BLOCKED: ' + state.reason if state.reason else ''}")
            elif kind == b"R":
                (n,) = struct.unpack("<I", _recv(conn, 4))
                idx = (pos + np.arange(n)) % len(frames)
                pos = (pos + n) % len(frames)
                conn.sendall(b"D" + struct.pack("<I", n) + frames[idx].astype("<i4").tobytes())
            else:
                raise ConnectionError(f"unknown message {kind!r}")
    except ConnectionError as e:
        log(f"vboard: {e}")
```

`sim/neosim/cwrx.py`:

```python
"""Receive side of the virtual-board acceptance (phase 4 spec §4): raw UAC2 I/Q from the board
(S24_3LE, stereo, L = I, R = Q) → the shipping C# DSP in CW mode → the twin's CW decoder."""
from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.io import wavfile

from neosim import decoders
from neosim.dsp_runner import run_dsp
from neosim.scenario import FS_HZ
from neosim.sigmf import write_sigmf

PITCH_HZ = 600.0          # the DSP's CW pitch: the dial sits 600 Hz below the station
LO_OFFSET_HZ = 10_000.0   # the LO sits 10 kHz below the station (clear of DC and the I/Q image)


def iq_from_s24le(raw: bytes) -> np.ndarray:
    """Frames of 3-byte little-endian I then Q → complex I/Q in the twin's convention (I − jQ),
    full scale 1.0."""
    b = np.frombuffer(raw[: len(raw) // 6 * 6], dtype=np.uint8).reshape(-1, 3).astype(np.int32)
    v = b[:, 0] | b[:, 1] << 8 | b[:, 2] << 16
    v = np.where(v >= 1 << 23, v - (1 << 24), v).astype(np.float64) / 2 ** 23
    return v[0::2] - 1j * v[1::2]


def decode(iq: np.ndarray, lo_hz: float, station_hz: float, workdir: Path) -> str:
    workdir = Path(workdir)
    meta = write_sigmf(workdir / "capture", iq, FS_HZ, lo_hz)
    wav = workdir / "cw.wav"
    run_dsp(meta, wav, "cw", station_hz - PITCH_HZ - lo_hz)
    rate, audio = wavfile.read(wav)
    return decoders.decode_cw(audio.astype(np.float64) / 32768.0, rate, pitch_hz=PITCH_HZ)


def best_cer(decoded: str, expected: str) -> float:
    """CER of the best-matching stretch of the decoded text: a looped recording starts and ends
    mid-message, so the full message appears somewhere inside it."""
    words = decoded.split()
    n = len(expected.split())
    best = decoders.cer(decoded, expected)
    for i in range(len(words)):
        for j in range(i + 1, min(len(words), i + n + 2) + 1):
            best = min(best, decoders.cer(" ".join(words[i:j]), expected))
    return best
```

`tools/elf_cw.py`:

```python
#!/usr/bin/env python3
"""Elf CW receiver check: tunes an Elf board (the real one, or the virtual board attached
through vhci_hcd) over its CAT serial port to each CW station in a stations file, records I/Q
from its sound card and decodes the Morse.

  tools/elf_cw.py [STATIONS.yaml] [--seconds S] [--workdir DIR]

The board appears as a standard USB sound card ("Elf HF Receiver", S24_3LE, 96 kHz, stereo:
left = I, right = Q) and a CDC serial port; nothing beyond the kernel's snd-usb-audio and
cdc_acm drivers and `arecord` is needed. Decoding uses the project's shipping C# DSP and the
twin's CW decoder. Exit status 0 when every station decodes within the CER limit."""
from __future__ import annotations

import argparse
import glob
import os
import re
import subprocess
import sys
import termios
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))

from neosim import cwrx  # noqa: E402

CARD_NAME = "Elf HF Receiver"
MAX_CER = 0.05


def find_card() -> int:
    for line in Path("/proc/asound/cards").read_text().splitlines():
        m = re.match(r"\s*(\d+)\s+\[.*\]:.*- (.*)$", line)
        if m and m.group(2).strip() == CARD_NAME:
            return int(m.group(1))
    raise SystemExit(f"no sound card named {CARD_NAME!r} (is the board attached?)")


def find_serial() -> str:
    ports = sorted(glob.glob("/dev/serial/by-id/*Elf*"))
    if not ports:
        raise SystemExit("no Elf serial port under /dev/serial/by-id (is the board attached?)")
    return ports[0]


class Cat:
    def __init__(self, path: str):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0                      # raw: no input, output or local processing
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        a[6][termios.VMIN], a[6][termios.VTIME] = 0, 5
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def ask(self, cmd: str, timeout: float = 2.0) -> str:
        os.write(self.fd, cmd.encode())
        out, t0 = b"", time.monotonic()
        while not out.endswith(b";") and time.monotonic() - t0 < timeout:
            out += os.read(self.fd, 64)
        return out.decode(errors="replace")

    def send(self, cmd: str) -> None:
        os.write(self.fd, cmd.encode())


def record(card: int, seconds: float, path: Path) -> bytes:
    subprocess.run(["arecord", "-q", "-D", f"hw:{card},0", "-f", "S24_3LE", "-c", "2", "-r", "96000", "-t", "raw",
                    "-d", str(int(round(seconds))), str(path)], check=True)
    return path.read_bytes()


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("stations", nargs="?", type=Path, default=ROOT / "sim/scenarios/vboard/cw_bands.yaml")
    ap.add_argument("--seconds", type=float, default=16.0, help="recording per station")
    ap.add_argument("--settle", type=float, default=3.0, help="wait after tuning (relays, twin rendering)")
    ap.add_argument("--workdir", type=Path, default=ROOT / "build/elf_cw")
    args = ap.parse_args(argv)

    card, port = find_card(), find_serial()
    cat = Cat(port)
    ident = cat.ask("ID;")
    print(f"board: card {card}, {port}, {ident}")
    if ident != "ID019;":
        raise SystemExit("the serial port does not answer as an Elf board")
    stations = [s for s in yaml.safe_load(args.stations.read_text())["stations"] if s["mode"] == "cw"]
    failures = 0
    for st in stations:
        rf = int(st["rf_hz"])
        lo = int(rf - cwrx.LO_OFFSET_HZ)
        cat.send(f"FA{lo:011d};")
        if cat.ask("FA;") != f"FA{lo:011d};":
            print(f"FAIL  {st['id']}: the board did not tune to {lo} Hz")
            failures += 1
            continue
        time.sleep(args.settle)
        work = args.workdir / st["id"]
        work.mkdir(parents=True, exist_ok=True)
        raw = record(card, args.seconds, work / "iq.raw")
        text = cwrx.decode(cwrx.iq_from_s24le(raw), lo, rf, work)
        cer = cwrx.best_cer(text, st["text"])
        ok = cer <= MAX_CER
        failures += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {st['id']} {rf / 1e6:.6f} MHz: {text!r} (CER {cer:.3f})")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
```

`tools/vboard-run.sh`:

```sh
#!/bin/sh
# Runs the virtual board: the digital twin (rendering the stations file) and elf-virtual (the
# firmware, serving its USB device over USB/IP on 127.0.0.1:3240). Ctrl-C stops both.
#   tools/vboard-run.sh [STATIONS.yaml]
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
STATIONS=$(realpath "${1:-$ROOT/sim/scenarios/vboard/cw_bands.yaml}")
WORK="$ROOT/build/vboard"
mkdir -p "$WORK"
cd "$WORK"                      # the socket path is relative: Unix socket paths are limited to 107 bytes
rm -f twin.sock
PYTHONPATH="$ROOT/sim" python3 -m neosim vboard "$STATIONS" --socket twin.sock &
TWIN=$!
trap 'kill $TWIN 2>/dev/null' EXIT INT TERM
i=0
while [ ! -S twin.sock ] && [ $i -lt 300 ]; do sleep 0.1; i=$((i + 1)); done
"$ROOT/build/firmware/elf-virtual" --twin twin.sock
```

`tools/vboard-attach.sh`:

```sh
#!/bin/sh
# Attaches the running virtual board (make vboard) to this PC's USB stack through vhci_hcd.
# Needs root for the kernel module and the attach; the board then appears as the
# "Elf HF Receiver" sound card and a /dev/ttyACM serial port. Detach: tools/vboard-detach.sh
set -e
sudo modprobe vhci-hcd
sudo usbip attach -r 127.0.0.1 -b 1-1
sleep 2
grep -A1 "Elf HF Receiver" /proc/asound/cards || echo "sound card not up yet: check dmesg"
ls /dev/serial/by-id/ 2>/dev/null | grep Elf || echo "serial port not up yet: check dmesg"
```

`tools/vboard-detach.sh`:

```sh
#!/bin/sh
# Detaches every USB/IP device imported from 127.0.0.1 (the virtual board).
set -e
for p in $(usbip port 2>/dev/null | sed -n 's/^Port \([0-9][0-9]*\):.*/\1/p'); do
    sudo usbip detach -p "$p"
done
```

`sim/neosim/__main__.py (after the `run` parser; the docstring gains the vboard line)`:

```python
    vb = sub.add_parser("vboard", help="serve the digital twin to the virtual board (elf-virtual --twin)")
    vb.add_argument("stations", type=Path)
    vb.add_argument("--socket", type=Path, default=Path("build/vboard/twin.sock"))
    args = parser.parse_args(argv)

    if args.command == "vboard":
        from neosim import vboard
        args.socket.parent.mkdir(parents=True, exist_ok=True)
        try:
            vboard.serve(args.stations, args.socket, log=lambda m: print(m, flush=True))
        except KeyboardInterrupt:
            pass
        return 0
```

`Makefile (append before `clean:`; add `vboard e2e` to .PHONY)`:

```make
# the kernel path: `make vboard`, then tools/vboard-attach.sh (root) and `make e2e` in another shell
vboard: fw
	tools/vboard-run.sh

e2e:
	$(PYTHON) tools/elf_cw.py
```

- [ ] **Step 4: Run them to verify they pass**

Run: `cd sim && python3 -m pytest -q tests/test_vboard.py && cd .. && make fw && cd firmware && python3 -m pytest -q tests/test_vboard_cw.py`
Expected: `6 passed`; then `6 passed` (five stations decode with CER ≤ 0.05, the out-of-range tune answers `?;`), about 90 s.

- [ ] **Step 5: Commit**

```bash
git add firmware sim tools Makefile
git commit -m "feat(vboard): twin link and five-band CW acceptance over USB/IP; kernel-path tools"
```

### Task 3: Verify, README and the attach run

- [ ] **Step 1:** `make test` (includes `fw-test`). Expected: all pass.
- [ ] **Step 2:** README: tick "Whole-board simulation over virtual USB: firmware with TinyUSB attached through USB/IP" only after the kernel attach run; tick "End-to-end test: a Python application reads I/Q from the simulated board and decodes" after `make e2e` passes on the attached board. Add the feature lines. Commit.
- [ ] **Step 3 (with the user, root):** `make vboard` in one shell; `tools/vboard-attach.sh`; `make e2e`. Expected: the "Elf HF Receiver" card and an Elf serial port appear; five PASS lines.
