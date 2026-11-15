# Implementation status

The core-first boundary is now in place: TC32 execution is implemented in the
local Unicorn checkout, and the TLSR8258 wrapper calls that backend through the
Unicorn C API. End-to-end vendor-firmware acceptance is still outstanding.

| Work area | Current state |
| --- | --- |
| TC32 in Unicorn | Initial backend implements the covered instruction families, CPU state, CPSR/SPSR/SSR access, `tmrcs` CPSR reads, and C API dispatch. `treti {pc}` restores PC from the IRQ stack and restores the saved mode; 11 backend tests pass, including IRQ stack restoration and instruction-fetch alias tests. Full TC32 vendor-binary coverage is not established. |
| TLSR8258 wrapper | Maps flash, board SRAM, the documented MMIO range, and a firmware-observed register alias into a TC32 `uc_engine`; configures Unicorn to fetch SRAM cache-alias instructions from flash while keeping data reads and writes on SRAM. Uses Unicorn's C API for registers and execution. Board code advances virtual time and injects modeled IRQs around Unicorn execution. |
| TLSR8258 peripherals | GPIO, timer compare/status, fixed 16 MHz system timer and its compare IRQ, UART FIFOs and timing, IRQ masks/sources, analog access, MSPI flash (JEDEC id, unique id, read, page program, sector/block/chip erase, write enable and status; operations complete at once and changes are mirrored into the XIP mapping), AES-128 ECB encrypt/decrypt, and the SDK-used integer divider helper are modeled. AREG `0x40..0x43` exposes a 32 kHz RC or 32.768 kHz crystal counter. Suspend and deep-sleep timer wake are modeled, including the system-timer count correction from the selected 32 kHz calibration. AES completion uses the datasheet's 1000-cycle duration; divider operations complete synchronously. RF TX/RX DMA, including the DMA high-byte offset from `0x800000`, basic RF IRQ status, and packet delivery to the virtual medium are connected. GPIO pull resistors, the I²C master and the SAR ADC misc channel (via DFIFO2) are modeled for the system simulation (see `tlsr8258-register-model.md`). GPIO wake, SRAM retention variants, SPI transfers and I²C slave mode, and DMA channels beyond the RF/ADC paths remain incomplete. |
| Virtual radio | BLE and Zigbee packet transport with channel matching, estimated duration, broadcast/unicast, collision suppression, and successful-reception records. TLSR8258 bridges Zigbee RF TX/RX DMA to this medium and decodes Zigbee channel writes from the RF synthesizer frequency registers `0x801244..0x801245`. BLE is not connected to TLSR8258 RF. For system simulations `VirtualAir` also has a log-distance model (positions, RSSI, sensitivity, SINR capture, half duplex, shared BLE/802.15.4 frequency axis); see `system-config.md`. |
| CLI | Selects F512/F1K, loads raw images at address zero, runs one node or a multi-node Zigbee scenario, prints CPU/time, transmitted frames and successful receptions, and supports PC, MMIO, stack/mode, recent-fault, raw radio-payload, and selected SRAM-range diagnostics. It does not parse ELF. |
| STM32WB55 | Experimental dual-core model: separate Cortex-M4 and Cortex-M0 execution, shared flash/SRAM, configurable raw CPU2 images, and basic IPCC/RCC/PWR/HSEM/RNG/LPTIM behavior. Without a raw CPU2 image, a built-in CPU2 model runs in CPU2's 32 MHz time domain: the Thread RCP model (IPCC/Spinel; Zephyr CPU1 joins the TLSR8258 coordinator and exchanges NWK data after a null-index guard in the Zephyr address-map source) or the ST IEEE 802.15.4 MAC coprocessor model (`cpu2_model: st_mac_802154`; the unmodified zigbee-rs Nucleo-WB55 router associates with the TLSR8258 coordinator, receives the network key and exchanges NWK data, `firmware_stm32wb55_zigbee_rs_router_join`). Synthetic raw CPU2 shared-memory execution is checked. ST's FUS-encrypted wireless-stack image cannot run as raw code, and the real STM32WB wireless PHY is not supported. See [`stm32wb55.md`](stm32wb55.md). |
| PHY6252 (Ai-Thinker PB-03F) | Experimental Cortex-M0 model with 512 KiB XIP flash, 64 KiB SRAM, image-table loader, partial ROM shims, optional external ROM mapping, UART/GPIO/timers/AON/flash and an inferred 802.15.4 path exposed through `run` and `sim`. The PB-03F Zephyr end device joins a TLSR8258 coordinator and exchanges NWK data with either the host ROM shims or the supplied PHY6222 ROM dump. The closed TS0001 relay image scans, associates with the TLSR8258 sample coordinator, receives the Transport Key and exchanges NWK data on its own (`firmware_phy6252_ts0001_join`, run on the dumped PHY6222 ROM; the host ROM stand-ins give the same result). Cold startup through the external ROM's reset vector is not established, and the specific Tuya relay command path remains unverified. See [`phy6252-investigation.md`](phy6252-investigation.md). |
| Zigbee `sampleLight` + `sampleGW` | With the light starting at 5 seconds, the run recorded 70 transmissions and 36 successful receptions. The capture includes the coordinator's Beacon Response, the light's association request, the coordinator's association response, and subsequent bidirectional traffic. At 11.25 seconds, `zb_isDeviceJoinedNwk()` is true for the light (`g_zbNwkCtx + 0x2d` is `0x04`) and the coordinator (`0x96`, also with `permit_join` and `joinAccept` set). The RX DMA layout now counts the two FCS bytes expected by the SDK MAC parser; without them it discarded the Beacon Response. This confirms joining for this firmware pair on channel 15. Source SDK and firmware artifacts remain outside this repository. |
| Zephyr PM | `firmware/zephyr-pmtest.bin` exercises repeated deep timer sleep/wake; the conditional `firmware_zephyr_pm_wake` CTest requires at least four sleep entries and no stuck-sleep result. It passes. The full local Zephyr sleepy-ED image now boots and scans Zigbee channels, but the current sampleGW network run does not show Zephyr joining or entering PM. Therefore the reported 3–5-cycle failure has not yet been reproduced in the full application image. Firmware images remain local and ignored. |

The nRF52840 (Cortex-M4F) has its own page, [`nrf52840.md`](nrf52840.md): core,
peripherals and what is missing (the radio first). The EFR32MG21 (Cortex-M33)
is in [`efr32mg21.md`](efr32mg21.md). The CC2530F256 (MCS-51) is
described in [`cc2530.md`](cc2530.md): CPU, code banking, memory map,
interrupt controller, power modes, the SWRU191F peripherals and the IEEE
802.15.4 radio are modeled, as are the two-wire debug interface (pad-level
DC/DD, DBGLOCK) and CC-Debugger-style programming through it; instruction
timing is the SWRU191F Table 2-3 best case, not cycle accurate.

Two behaviors are explicit inferences. The datasheet documents the register
window at `0x800000..0x83ffff`, while vendor `LoadTblCmdSet` code forms writes
at `0x1000000 + offset`; that observed address range is mapped as a compatibility
alias to the documented registers. The AREG `0x7f` cold-boot value is inferred
from the SDK wake path. Unicorn exposes instruction hooks but not TC32 cycle
counts, so board time currently advances by an estimated three cycles per
instruction; this has not been calibrated against hardware.

The RF bridge test exchanges an 802.15.4 payload between two TLSR8258 board
models through TX/RX DMA buffers and IRQ status. It also checks that firmware
synthesizer writes override the initial host-selected channel and that DMA
addresses use the vendor's high-byte encoding. The `network` CLI command loads
multiple firmware images onto a shared medium and reports successful packet
receptions; `--radio-payloads` includes each transmitted payload as hex, and
`--dump-sram` exposes selected node memory ranges for checking firmware state.
The IRQ return fix removes the sample's previous stack fault. The sampleLight
and sampleGW pair now complete joining when the light starts at 5 seconds. The
medium records the Beacon Response, association request and response, then
bidirectional Zigbee traffic; the SDK joined flag is set on the light. The
specific RX DMA/FCS mismatch and verified run are described above. Keep BLE
deferred and all CPU instruction decoding and execution inside Unicorn.
