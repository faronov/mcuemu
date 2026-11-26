#!/usr/bin/env python3
"""Run a system file and check the Zigbee traffic in its air capture.

The check does not depend on the firmware's RAM layout or pins, only on what
goes over the air (a pcap written by a `pcap:` sink, link type 230, IEEE
802.15.4 without FCS):

* the coordinator answers an association request with status 0 (the device
  joined and was given a short address);
* after that the new device sends NWK data frames (--min-up; --min-unicast asks
  for frames addressed to a node, not broadcasts such as Device Announce) and the
  coordinator sends NWK data frames to it (the key, the interview, ...);
* optionally, more unicast uplink frames after a given virtual time (periodic reports),
  one active scan of at least N channels before the join (--min-scan-burst: the
  longest run of MAC beacon requests less than --scan-burst-gap apart; an active scan
  sends one per channel, so 16 is a full 11..26 sweep, while separate shorter sweeps
  do not add up; the capture has no channel field) and regexes that must appear in
  the console (--expect, e.g. the RTT log).

UART and RTT output goes to the console through the system file's own sinks
(`--trace-uart '*=text'` is added unless --no-uart-trace is given); the whole
console output is also written to --log.
"""

from __future__ import annotations

import argparse
import re
import struct
import subprocess
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

BROADCAST_NWK = 0xFFF8  # NWK addresses from here up are broadcasts


@dataclass
class Frame:
    t_ns: int
    kind: int  # 0 beacon, 1 data, 2 ack, 3 command
    seq: int
    dst: Optional[int]  # short address, None for extended/absent
    src: Optional[int]
    payload: bytes
    raw: bytes


def read_pcap(path: Path) -> list[Frame]:
    data = path.read_bytes()
    if len(data) < 24:
        return []
    magic = struct.unpack("<I", data[:4])[0]
    if magic != 0xA1B2C3D4:
        raise SystemExit(f"{path}: unexpected pcap magic {magic:#x}")
    linktype = struct.unpack("<I", data[20:24])[0]
    if linktype != 230:
        raise SystemExit(f"{path}: expected link type 230 (802.15.4 no FCS), got {linktype}")
    frames: list[Frame] = []
    offset = 24
    while offset + 16 <= len(data):
        sec, usec, incl, _orig = struct.unpack("<IIII", data[offset : offset + 16])
        offset += 16
        raw = data[offset : offset + incl]
        offset += incl
        frame = parse_mac(raw, sec * 1_000_000_000 + usec * 1000)
        if frame is not None:
            frames.append(frame)
    return frames


def parse_mac(raw: bytes, t_ns: int) -> Optional[Frame]:
    if len(raw) < 3:
        return None
    fc = raw[0] | raw[1] << 8
    kind = fc & 7
    seq = raw[2]
    if kind == 2:
        return Frame(t_ns, kind, seq, None, None, b"", raw)
    dst_mode = (fc >> 10) & 3
    src_mode = (fc >> 14) & 3
    pan_compress = bool(fc & 0x40)
    pos = 3
    dst = src = None
    if dst_mode:
        pos += 2  # destination PAN
        if dst_mode == 2:
            dst = raw[pos] | raw[pos + 1] << 8
        pos += 2 if dst_mode == 2 else 8
    if src_mode:
        if not pan_compress:
            pos += 2
        if src_mode == 2:
            src = raw[pos] | raw[pos + 1] << 8
        pos += 2 if src_mode == 2 else 8
    return Frame(t_ns, kind, seq, dst, src, raw[pos:], raw)


def nwk_data(frame: Frame) -> Optional[tuple[int, int]]:
    """(dst, src) NWK addresses of a NWK data frame, else None."""
    p = frame.payload
    if frame.kind != 1 or len(p) < 8:
        return None
    nwk_fc = p[0] | p[1] << 8
    if nwk_fc & 3 != 0:
        return None
    return p[2] | p[3] << 8, p[4] | p[5] << 8


def analyse(
    frames: list[Frame],
    min_up: int,
    min_unicast: int,
    min_down: int,
    late_after_ns: int,
    min_late: int,
    min_scan_burst: int = 0,
    scan_burst_gap_ns: int = 200_000_000,
):
    problems: list[str] = []
    joined_at = short = None
    for f in frames:
        if f.kind == 3 and f.payload[:1] == b"\x02" and len(f.payload) >= 4:
            if f.payload[3] == 0:
                joined_at = f.t_ns
                short = f.payload[1] | f.payload[2] << 8
                break
    if short is None:
        return ["no successful association response in the capture"], {}
    # MAC command 0x07 (beacon request) before the association response, grouped into scans.
    scan_burst = burst = 0
    previous = None
    for f in frames:
        if f.t_ns >= joined_at or f.kind != 3 or f.payload[:1] != b"\x07":
            continue
        burst = burst + 1 if previous is not None and f.t_ns - previous < scan_burst_gap_ns else 1
        scan_burst = max(scan_burst, burst)
        previous = f.t_ns
    up = unicast = down = late = 0
    for f in frames:
        if f.t_ns < joined_at:
            continue
        nd = nwk_data(f)
        if nd is None:
            continue
        dst, src = nd
        if src == short:
            up += 1
            if dst < BROADCAST_NWK:
                unicast += 1
                if f.t_ns >= late_after_ns:
                    late += 1
        if dst == short and src == 0:
            down += 1
    stats = {
        "short": short, "joined_at_s": joined_at / 1e9, "up": up, "unicast": unicast, "down": down, "late": late,
        "scan_burst": scan_burst,
    }
    if up < min_up:
        problems.append(f"device {short:#06x} sent {up} NWK data frames, expected {min_up}")
    if unicast < min_unicast:
        problems.append(
            f"device {short:#06x} sent {unicast} unicast NWK data frames, expected {min_unicast}"
        )
    if down < min_down:
        problems.append(f"coordinator sent {down} NWK data frames to {short:#06x}, expected {min_down}")
    if scan_burst < min_scan_burst:
        problems.append(
            f"the longest scan before the join sent {scan_burst} beacon requests, expected {min_scan_burst}"
        )
    if late < min_late:
        problems.append(
            f"only {late} uplink data frames after {late_after_ns / 1e9:.0f} s, expected {min_late}"
        )
    return problems, stats


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("mcuemu")
    ap.add_argument("system", help="system YAML (writes its pcap sink into --workdir)")
    ap.add_argument("--pcap", default="air.pcap", help="pcap file the system writes, in --workdir")
    ap.add_argument("--workdir", default=".")
    ap.add_argument("--for", dest="duration", default=None, help="virtual duration, e.g. 40s")
    ap.add_argument("--log", default="console.log")
    ap.add_argument("--min-up", type=int, default=1, help="NWK data frames from the device (any destination)")
    ap.add_argument("--min-unicast", type=int, default=0, help="of those, unicast ones")
    ap.add_argument("--min-down", type=int, default=1)
    ap.add_argument("--late-after", type=float, default=0.0, help="virtual seconds")
    ap.add_argument("--min-late", type=int, default=0)
    ap.add_argument(
        "--min-scan-burst", type=int, default=0,
        help="beacon requests in the longest scan before the join (16 = a full 11..26 sweep)",
    )
    ap.add_argument(
        "--scan-burst-gap", type=float, default=0.2,
        help="virtual seconds: beacon requests closer than this belong to one scan (default 0.2)",
    )
    ap.add_argument(
        "--expect", action="append", default=[], metavar="REGEX",
        help="regex that must match the console (RTT chunks are re-joined into lines)",
    )
    ap.add_argument("--no-uart-trace", action="store_true")
    ap.add_argument("--timeout", type=int, default=900)
    args = ap.parse_args()

    workdir = Path(args.workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    cmd = [str(Path(args.mcuemu).resolve()), "sim", str(Path(args.system).resolve())]
    if args.duration:
        cmd += ["--for", args.duration]
    if not args.no_uart_trace:
        cmd += ["--trace-uart", "*=text"]
    print("+", " ".join(cmd), flush=True)
    # Stream the console (UART/RTT text) while the emulator runs and keep a copy.
    timer = threading.Timer(args.timeout, lambda: proc.kill())
    with (workdir / args.log).open("w") as log:
        proc = subprocess.Popen(
            cmd, cwd=workdir, text=True, errors="replace",
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        timer.start()
        assert proc.stdout is not None
        for line in proc.stdout:
            sys.stdout.write(line)
            log.write(line)
        proc.wait()
        timer.cancel()
    if proc.returncode != 0:
        print(f"emulator exited with status {proc.returncode}", file=sys.stderr)
        return 1

    # The RTT text sink breaks lines at arbitrary chunk boundaries; join them.
    text = re.sub(r"\n?\[\s*[\d.]+ s\] \S+\.rtt: ", "", Path(workdir / args.log).read_text())
    missing = [p for p in args.expect if not re.search(p, text)]
    for p in missing:
        print(f"FAIL: console never matched /{p}/", file=sys.stderr)

    pcap = workdir / args.pcap
    if not pcap.exists():
        print(f"{pcap} was not written", file=sys.stderr)
        return 1
    problems, stats = analyse(
        read_pcap(pcap),
        args.min_up,
        args.min_unicast,
        args.min_down,
        int(args.late_after * 1e9),
        args.min_late,
        args.min_scan_burst,
        int(args.scan_burst_gap * 1e9),
    )
    if stats:
        print(
            "zigbee: device {short:#06x} joined at {joined_at_s:.2f} s; "
            "{up} uplink ({unicast} unicast, {late} late) and {down} downlink NWK data frames; "
            "longest scan before the join: {scan_burst} beacon requests".format(**stats)
        )
    for p in problems:
        print(f"FAIL: {p}", file=sys.stderr)
    return 1 if problems or missing else 0


if __name__ == "__main__":
    sys.exit(main())
