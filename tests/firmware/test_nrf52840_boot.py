#!/usr/bin/env python3
"""Boot the Embassy Zigbee sensor image on the nRF52840 model.

The image programs UICR through NVMC on its first start and resets, then brings
up the clocks, GPIO, the RTC time driver and SAADC, configures the RADIO for IEEE
802.15.4, uses RNG and ECB (in either order: builds differ), and goes to sleep in
WFE. The check follows that path in the register trace and requires the core to
end asleep with time passing.
"""

import re
import subprocess
import sys


def fail(message: str, output: str) -> int:
    print(f"firmware_nrf52840_boot: {message}", file=sys.stderr)
    if output:
        print("--- emulator output (tail) ---", file=sys.stderr)
        print("\n".join(output.splitlines()[-40:]), file=sys.stderr)
    return 1


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: test_nrf52840_boot.py MCUEMU FIRMWARE", file=sys.stderr)
        return 2

    command = [
        sys.argv[1], "run", "--model", "NRF52840", "--firmware", sys.argv[2],
        "--for", "3s", "--trace-mmio",
    ]
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, timeout=120, check=False)
    except subprocess.TimeoutExpired as error:
        output = error.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        return fail("emulator timed out", output)
    if result.returncode != 0:
        return fail(f"emulator exited with status {result.returncode}", result.stdout)

    trace = result.stdout
    milestones = [
        # (what, pattern)
        ("UICR write enable (NVMC.CONFIG = WEN)", r"^W 0x4001e504 <- 0x1 "),
        ("HFCLK start request", r"^W 0x40000000 <- 0x1 "),
        ("LFCLK start request", r"^W 0x40000008 <- 0x1 "),
        ("RTC1 start (the time driver)", r"^W 0x40011000 <- 0x1 "),
        ("SAADC VDD channel (PSELP = 9)", r"^W 0x40007510 <- 0x9 "),
        ("RADIO MODE = IEEE 802.15.4 250 kbit/s", r"^W 0x40001510 <- 0xf "),
    ]
    position = 0
    for what, pattern in milestones:
        match = re.compile(pattern, re.MULTILINE).search(trace, position)
        if not match:
            return fail(f"never reached: {what}", trace)
        position = match.start()
    for what, pattern in [("RNG start", r"^W 0x4000d000 <- 0x1 "), ("ECB start", r"^W 0x4000e000 <- 0x1 ")]:
        if not re.compile(pattern, re.MULTILINE).search(trace, position):
            return fail(f"never reached: {what} (after the RADIO setup)", trace)

    resets = len(re.findall(r"^W 0x40000904 <- 0xffffffff ", trace, re.MULTILINE))
    if resets < 2:
        return fail("the image did not restart after programming UICR "
                    f"(saw {resets} start-up sequences)", trace)

    if "stop: time limit (core asleep)" not in trace:
        return fail("the core did not end the run asleep", trace)
    time_match = re.search(r"^virtual time: (\d+) ns$", trace, re.MULTILINE)
    if not time_match or int(time_match.group(1)) < 3_000_000_000:
        return fail("virtual time did not reach 3 s", trace)
    print("firmware_nrf52840_boot: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
