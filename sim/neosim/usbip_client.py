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
