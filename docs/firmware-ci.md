# Firmware tests in CI

`.github/workflows/firmware-tests.yml` runs real firmware on the emulator. It
starts when "Build mcuemu" finishes successfully (or by hand: *Run workflow*,
optionally with the id of a build run) and uses the Linux binary from that build.
It runs on the latest Ubuntu runner. A run started by a build always uses the
workflow file of `main`, whatever branch was built; to try a change to this
workflow (or to the pins below) before merging, run it by hand on the branch,
with the id of the branch's "Build mcuemu" run:
`gh workflow run firmware-tests.yml --ref BRANCH -f run_id=ID`.

## Images

Every image is an artifact (`fw-*`) of its own job, so a broken build fails only
the scenarios that need it.

| Image | Source | How |
| --- | --- | --- |
| `telink-gw.bin` (coordinator) | Telink Zigbee SDK `V3.7.2.0`, `apps/sampleGW` | `tests/ci/telink` (CMake, tc32 GCC from `devbis/tc32` branch `linux`), board `BOARD_8258_EVK_V1P2` |
| `telink-light.bin` | same SDK, `apps/sampleLight` | same |
| `z03mmc.bin` | `devbis/z03mmc` (default branch) | its own CMake, as in its workflow |
| `tlsr8258-plug.bin` | `faronov/zigbee-rs-plugs`, feature `zbeacon-ts011f-512k` | `scripts/tlsr8258-firmware.sh` with the pinned tc32 Rust toolchain (SHA-256 checked) |
| `nrf52840-sensor.bin`, `nrf52840-sensor-sht31.bin` | `devbis/zigbee-rs` at the commit `ZIGBEE_RS_REF` (pinned, see below) | `cargo build --release` in `examples/nrf52840-sensor` |
| `efr32mg21-sensor.bin` | `devbis/zigbee-rs`, same commit, `examples/efr32mg21-sensor` | `cargo build --release --locked` (target `thumbv8m.main-none-eabihf`), `llvm-objcopy -O binary`: an application-slot image (linked at `0x4000`). It logs on the virtual COM port (USART0), the emulator's `uart0` endpoint; it has no RTT |
| `esp32h2-sensor.bin`, `esp32c6-sensor.bin` | `devbis/zigbee-rs`, same ref, `examples/esp32h2-sensor` and `examples/esp32c6-sensor` | `cargo build --release -Z build-std=core,alloc`, then `espflash 4.5.0 save-image --merge` (pinned release, SHA-256 checked) with the product's 4 MB OTA partition table: the whole flash (bootloader at 0, app in `ota_0`) |
| `thermostat-ca51.bin`, `thermostat-zt3l.bin` | release `r02-20261109` of `slacky1965/tuya_thermostat_full_custom_r02_zrd` (`osrctherm_r02_v1.05_CA51F253L3.bin`, `osrctherm_r02_zrd_v1.0.02_ZT3L.bin`) | downloaded, SHA-256 pinned in `THERMOSTAT_*_SHA256` |
| `zephyr-pmtest.bin`, `vendor_sleep_1s.bin`, `phy6252-ts0001.bin` (+ `phy6222-efuse.bin`, `phy6222-rfcal.bin`, `phy6222-rom.bin`) | no source available | committed in `tests/firmware-images/` |
| `efr32-zephyr-router.bin`, `efr32-zephyr-ed.bin` | `devbis/zephyr-tlsr8258`, branch `zigbee-generic-stack`: Zephyr `samples/zigbee/zigbee_shell`, board `slwrb4180a` (RAIL driver, no console), built by that repository's "Build Zigbee firmware" workflow | downloaded from the release `zephyr-zigbee-20261016-deada2` (`efr32mg21-router.bin`, `efr32mg21-end-device.bin`), SHA-256 pinned in `ZEPHYR_EFR32_*_SHA256` |
| `zbdonglee-ncp.bin` | Silicon Labs EZSP NCP 8.2.2 (Sonoff ZBDongle-E build) of `Nerivec/silabs-firmware-builder`, release `v2025.6.2-update1` | downloaded as the `.gbl` (SHA-256 pinned in `EFR32_NCP_GBL_SHA256`), then `scripts/gbl2bin.py` extracts the application |
| `moes-brightness-thermometer.bin` | full-flash dump of the Moes ZigBee "Smart Brightness Thermometer" (ZSS-ZK-THL, Tuya ZS3L) from `https://pvvx.github.io/ZSS-ZK-THL/bin/FullFlashBin.zip` | downloaded, the zip's SHA-256 pinned in `MOES_THERMOMETER_ZIP_SHA256`, `0000000-00C0000.bin` (768 KiB) taken out of it; the scenario erases its NVM3 pages |

The zigbee-rs images come from one pinned commit, `ZIGBEE_RS_REF` (a full SHA of
`devbis/zigbee-rs`, branch `nrf-experiment`): that branch changes the examples' logs,
radio drivers and boards without notice, and following it broke these tests more
than once (the EFR32MG21 sensor moved from RTT to the VCOM UART and from the
scaffold radio driver to `efr32s2-hal`; the nRF52840 and ESP32-C6 logs dropped
lines the scenarios looked for). To move to a newer commit: set `ZIGBEE_RS_REF` to
its SHA on a branch, push it, and run the workflow there by hand (*Run workflow*,
that branch, `run_id` = the id of the "Build mcuemu" run of the branch's head, see
`gh run list --workflow build.yml --branch BRANCH`); fix the scenarios'
`--expect` lines, the models or this page for what changed, then merge.

Adding an image: build it from source if that is easy; otherwise download it
from a stable URL or a GitHub release; commit it to `tests/firmware-images/`
only as a last resort and record where it came from in that folder's README.

## Scenarios

One job per scenario, independent of each other. The scenario files are in
`tests/firmware/scenarios/`; they point at `firmware/<image>.bin`, so they run
locally too when you put the images there.

| Scenario | Passes when |
| --- | --- |
| `zigbee-light`, `zigbee-z03mmc`, `zigbee-plug` | the device gets an Association Response with status 0, then sends NWK data frames to the coordinator and the coordinator sends some back |
| `zigbee-nrf52840` | as above plus, in the defmt log, the scan result, join, Transport-Key, Request Key, Confirm-Key SUCCESS and "network committed"; reports keep arriving after 75 s |
| `zigbee-efr32mg21` | as `zigbee-nrf52840`, on the EFR32MG21 model (the image drives the radio without RAIL, `RsmRadio` in `docs/efr32mg21.md`); a BTN0 press at 2 s starts the join; the expected lines (boot banner, scan, association, Transport-Key, Request Key, Confirm-Key, commit, `CommissioningComplete`) must come from the sensor's console `board.mcu.uart0` |
| `zigbee-esp32h2` | the sensor joins the sampleGW coordinator through the real ESP-IDF bootloader on ROM stand-ins (it scans all channels, backs off and reboots once); the console shows the AES self-test, `JoinedOrResumed`, Confirm-Key SUCCESS and `CommissioningComplete` |
| `zigbee-esp32c6` | the same for the C6 sensor image (the log must show the scan and the association on channel 15; the image no longer logs each scanned channel) |
| `zigbee-efr32-zephyr-router`, `zigbee-efr32-zephyr-ed` | the Zephyr zigbee_shell sample on the EFR32MG21 (RAIL radio driver, no console) gets an Association Response with status 0 from the sampleGW coordinator, judged from the pcap |
| `zigbee-phy6252-ts0001` | the Tuya TS0001 relay (PHY6222 running the dumped ROM, NVM erased) scans, associates with the sampleGW (channel 15), gets the Transport Key and exchanges unicast NWK data, judged by `test_phy6252_ts0001_join.py` from the air sink, per transmitter |
| `tlsr8258 + ca51 thermostat r02` | the ZT3L joins the coordinator and both UARTs of the two-chip board carry framed traffic |
| `moes-brightness-thermometer` | `scripts/moes_brightness_thermometer.py`: the Tuya ZS3L thermometer (NVM3 erased, PB0 long press) joins the EZSP NCP's network: the NCP reports `trustCenterJoinHandler`, `childJoinHandler` and the Device_annce, the firmware reads its CHT8305 and OPT3001 (Lua parts on PA0/PA3) and refreshes the e-ink display (`docs/efr32mg21.md`) |
| `nrf52840-boot`, `zephyr-pm-wake`, `vendor-sleep-1s` | the existing scripts in `tests/firmware/` |

`tests/firmware/zigbee_air_check.py` runs the system file and reads the pcap
sink (link type 230, 802.15.4 without FCS); it does not depend on RAM addresses
or pins. `--expect REGEX` checks the console, with RTT chunks re-joined into lines
(UART lines keep their `[ t s] <endpoint>: ` prefix, so a regex can name the
endpoint a line must come from).

A scenario fails its job when its command fails (the step runs with
`pipefail`; before, the job took `tee`'s status and passed whatever happened). A
matrix entry with `known_failure: <reason>` is a known emulator gap: its job runs
and is shown failed with the reason in the summary, but does not fail the
workflow (`continue-on-error`). Remove the key when the scenario passes again.
Current known failures:

- `zigbee-efr32-zephyr-ed`: with the release image `zephyr-zigbee-20261030-de387d`
  the model faults (fetch at `0xf10002`) at 12.58 s, in RAIL's BUF2 receive
  handler after the end device joined.
- `tlsr8258 + ca51 thermostat r02`: the ZT3L image `v1.0.02` of release
  `r02-20261109` faults at 29 ms (TC32 fetch at `0xfffffffe`);
  `examples/devices/tuya_thermostat.yaml` still splits the image the way `v1.0.01`
  needed (ramcode `0x1ad0`, text at `0x9ad0`).

Some devices join late on purpose: the z03mmc scans once at power-up, before the
coordinator has formed its network, and steers again after 60 s, so its scenario
runs 80 s; the plug gets a short button press at 8 s to start steering.

## Logs

UART and RTT output is printed in the job log (`--trace-uart '*=text'`, or the
`hex`/`text` sinks in the system file) and saved with the pcap in the
`logs-<scenario>` artifact (14 days). Any new scenario that has a UART or RTT
channel must do the same.
