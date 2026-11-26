#!/usr/bin/env python3
"""An nRF52840 (Embassy Zigbee sensor image) joins a network run by the TLSR8258
sampleGW coordinator: two chips with different cores on one radio medium.

The sensor scans, finds the coordinator's beacon on channel 15, associates
(which needs the coordinator's MAC ACKs to reach the nRF radio in time) and
receives the NWK key in the Transport-Key, then completes the unique
trust-center link key exchange (Request Key, Transport-Key, Verify Key,
Confirm Key) with the Telink stack and commits the network. Needs a sensor image
with the zigbee-rs fixes for the exchange (older builds panic on the first
secured frame).
"""

import os
import re
import subprocess
import sys
import tempfile


def fail(message: str, output: str) -> int:
    print(f"firmware_nrf52840_zigbee_join: {message}", file=sys.stderr)
    if output:
        print("--- emulator output (tail) ---", file=sys.stderr)
        print("\n".join(output.splitlines()[-40:]), file=sys.stderr)
    return 1


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_nrf52840_zigbee_join.py MCUEMU EXAMPLE.yaml", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory() as scratch:
        try:
            result = subprocess.run(
                [sys.argv[1], "sim", sys.argv[2], "--for", "24s"],
                cwd=scratch, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, timeout=300, check=False)
        except subprocess.TimeoutExpired as error:
            output = error.stdout or ""
            if isinstance(output, bytes):
                output = output.decode(errors="replace")
            return fail("emulator timed out", output)
        if result.returncode != 0:
            return fail(f"emulator exited with status {result.returncode}", result.stdout)
        pcap = os.path.join(scratch, "nrf-air.pcap")
        pcap_size = os.path.getsize(pcap) if os.path.exists(pcap) else 0

    # The RTT text sink breaks lines at arbitrary chunk boundaries; join them.
    log = re.sub(r"\n?\[\s*[\d.]+ s\] board\.mcu\.rtt: ", "", result.stdout)
    for what, pattern in [
        ("an active scan that found the coordinator's beacon", r"Found 1 network\(s\)"),
        ("the join request", r"Joining PAN 0x[0-9A-F]+ ch 15"),
        ("association (the coordinator's ACK reached the radio)", r"Joined PAN 0x[0-9A-F]+ ch 15 as 0x[0-9A-F]+"),
        ("the Transport-Key", r"NWK key received from TC"),
        ("a unique TC link key request", r"Sending APSME-REQUEST-KEY to TC"),
        ("the Confirm-Key", r"Confirm-Key SUCCESS from the Trust Center"),
        ("the committed network", r"Commissioning security complete.{1,16}network committed"),
    ]:
        if not re.search(pattern, log):
            return fail(f"never saw {what}", result.stdout)
    if "No ACK after" in log.split("Joined PAN")[0].split("Joining PAN")[-1]:
        return fail("the association request was not acknowledged", result.stdout)
    if pcap_size < 100:
        return fail("the pcap sink captured no frames", result.stdout)
    print("firmware_nrf52840_zigbee_join: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
