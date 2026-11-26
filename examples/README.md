# Examples

| file | what |
|---|---|
| `world.yaml` | A Zigbee network of three devices on one radio medium: a coordinator, a thermostat and a smart plug. The devices come from `devices/` through `include:`. |
| `devices/tuya_thermostat.yaml` | A Tuya-style thermostat: CA51F253L3 main board + ZT3L (TLSR8258) Zigbee module, joined by a UART running the "serial v2" protocol. |
| `devices/tuya_plug.yaml` | A smart plug: TLSR8258 module, relay, LED, button and a BL0937 energy meter on three GPIOs. Defaults are the Zbeacon TS011F board of `zigbee-rs-plugs` (pins and vendor calibration). |
| `plug_only.yaml` | The plug on its own radio medium, with a scenario (load change, button press) and VCD/pcap sinks, for firmware bring-up. |
| `tlsr8269_smoke.yaml` | A bare TLSR8269 (TLSR826x family) running the SDK smoke firmware from `tests/tlsr826x/`: two lines over the DMA UART. |
| `lywsd03mmc.yaml` | A Xiaomi LYWSD03MMC (hardware 1.4) joining a network run by the sampleGW coordinator, with the climate changed from the scenario. |
| `devices/lywsd03mmc_v14.yaml` | LYWSD03MMC hardware B1.4: TLSR8258-class module, SHTC3 sensor and an I2C segment LCD (0x3C) on PC2/PC3, two test-point buttons. The display is decoded by `parts/lcd_lywsd03mmc.lua`. |
| `devices/lywsd03mmc_v15.yaml` | LYWSD03MMC with the UART display (38400 baud, TX PD7) and an SHT4x sensor; the display frames are logged, not decoded. |
| `devices/nrf52840_promicro.yaml` | An nRF52840 on a Pro Micro-style board (nice!nano v2 pins): status LED P0.15, button P0.11; everything else on the header is free to wire. |
| `nrf52840_promicro.yaml` | The board running the Embassy Zigbee sensor image on its own, with a button press from the scenario (it scans and finds nothing). |
| `devices/efr32mg21_brd4181a.yaml` | An EFR32MG21 on the BRD4181A radio board: LED0 on PB0, BTN0 on PD2, the VCOM console on `uart0` (USART0). |
| `efr32mg21_zigbee.yaml` | The board with the zigbee-rs sensor image on one medium with the TLSR8258 sampleGW coordinator: the EFR32 joins, completes the TCLK exchange and reports. The radio is not the silicon's (`docs/efr32mg21.md`). |
| `nrf52840_zigbee.yaml` | The same board with the SHT31 image on one radio medium with the TLSR8258 sampleGW coordinator: the nRF52840 joins the network, completes the TCLK exchange and reports temperature and humidity read from a simulated SHT31 on P0.26/P0.27. Different chips and cores in one world. |
| `cc2530_zigbee.yaml` | A CC2530F256 with the standard Z-Stack DIYRuZ Motion Router image on one medium with the TLSR8258 sampleGW coordinator: the router joins, gets the network key, announces itself and exchanges link status. It is powered on after the coordinator has formed its network (the image scans once and does not rescan). |
| `phy6252-zephyr.yaml` | PB-03F Zephyr Zigbee end device with the TLSR8258 sample gateway; joins and exchanges NWK data in the current emulator. |
| `stm32wb55_zigbee.yaml` | Experimental Zephyr STM32WB55 CPU1 with the built-in Thread RCP model serviced in CPU2's M0 time domain, alongside the TLSR8258 sample gateway. With a local Zephyr null-index guard, CPU1 joins and exchanges NWK data. A separate raw CPU2 image can replace the built-in model and execute on M0; the wireless PHY is not modeled. |
| `phy6252-zephyr-external-rom.yaml` | The same PB-03F image with the external PHY6222 ROM, eFuse and RF calibration inputs. This pairing joins and exchanges NWK data; the binary inputs stay local. |
| `phy6252-ts0001.yaml` | Saved-state TS0001 diagnostic replay with captured eFuse and RF calibration inputs; presses GPIO8 after the Tuya startup reports and records GPIO2 to VCD. The separate `firmware_phy6252_ts0001_join` CTest (first boot, sample coordinator, no other nodes) is the join acceptance test; it passes. |
| `phy6252-ts0001-offline.yaml` | TS0001 local-button replay with the same board inputs and no coordinator. Both physical key inputs are available (GPIO8 and GPIO3); the scenario biases GPIO3 high, presses GPIO8 twice and records GPIO2 to VCD. The firmware's `On/Off` diagnostic callback is still missing in the emulator. |
| `phy6252-ts0001-log-replay.yaml` | 65-second no-coordinator replay of the supplied UART context with one GPIO8 press. Seed 57013 reproduces its three `LL_Rand`-derived reporting intervals; periodic stack counters and the `On/Off` diagnostic are still different or absent. |
| `parts/shtc3.lua`, `parts/sht4x.lua`, `parts/lcd_lywsd03mmc.lua` | The two sensors and the I2C segment display of the LYWSD03MMC. |
| `parts/bl0937.lua` | The BL0937 as a scripted part (CF / CF1 pulse outputs, SEL input). |
| `parts/ds18b20.lua`, `parts/sht3x.lua` | Reference sensor scripts (see `docs/scripting.md`). |

```sh
build/mcuemu sim examples/world.yaml --list-endpoints --for 1ms
build/mcuemu sim examples/world.yaml --for 10s        # writes logs, a VCD and a pcap into the current directory
```

## Inserting a device

A world lists devices, and a device can live in its own file:

```yaml
devices:
  - include: devices/tuya_plug.yaml
    id: plug1                       # optional: rename the device
    params: { position: [3, 0, 0], power_w: 60 }
```

The device file is a YAML document with `version: 1`, an optional `params:`
mapping of defaults and one `device:`. A scalar such as `"${position}"` inside
`device:` is replaced by the parameter of that name (a lone reference keeps its
type: lists and numbers stay lists and numbers). A parameter the file does not
declare is an error, so typos do not pass silently. Paths in the file (firmware,
scripts) are relative to the file. The same file can be inserted several times
under different ids. See `docs/system-config.md`.

## Firmware

No firmware image is stored in the repository. Put the images where the
`params:` defaults point (or pass your own paths):

| device | image | default path |
|---|---|---|
| coordinator | the coordinator sample | `firmware/samplegw_tb03f.bin` |
| PB-03F Zephyr | `zigbee_shell` built for PB-03F | `firmware/phy6252-zephyr-ed.bin` (used by `phy6252-zephyr.yaml`) |
| PHY6222 ROM for PB-03F compatibility run | supplied `rom_10000000_20000.bin` dump | `firmware/phy6222-rom.bin` (used by `phy6252-zephyr-external-rom.yaml`) |
| PHY6222 eFuse input | 32-byte four-block dump for the captured module | `firmware/phy6222-efuse.bin` |
| PHY6222 RF calibration input | four observed `kCal` bytes for the captured module | `firmware/phy6222-rfcal.bin` |
| TS0001 relay | the user-supplied application package or 512 KiB full-flash dump | `firmware/phy6252-ts0001.bin` (used by the investigation scenario with the local PHY6222 eFuse and RF-calibration inputs above) |
| thermostat, ZT3L module | the application built by the thermostat project | `firmware/osrctherm_r02_zrd.bin` |
| thermostat, main board | the CA51F253L3 image | `firmware/osrctherm_r02_v1.02_CA51F253L3.bin` |
| LYWSD03MMC | `z03mmc.bin` from the z03mmc project | `firmware/z03mmc.bin` |
| plug | `tlsr8258-plug.bin` from `zigbee-rs-plugs` (`build-tuya.sh`, 512 KiB board) | `firmware/tlsr8258-plug.bin` |
| nRF52840 board | `nrf52840-sensor.bin`, the Embassy Zigbee sensor image of `zigbee-rs` (built for the nRF52840-DK; the CI artifact `nrf52840-sensor-firmware`) | `firmware/nrf52840-sensor.bin` |
| EFR32MG21 board | `efr32mg21-sensor.bin` of `zigbee-rs` (BRD4181A; the CI artifact `efr32mg21-sensor-firmware`) | `firmware/efr32mg21-sensor.bin` (used by `efr32mg21_zigbee.yaml`) |
| STM32WB55 Zephyr CPU1 | `zephyr.bin` built for `weact_stm32wb55_core` with the Zigbee ED config | `firmware/stm32wb55-zigbee-shell.bin` (used by `stm32wb55_zigbee.yaml`; see the Zephyr source fix in `docs/stm32wb55.md`) |
| STM32WB55 CPU2 | Optional raw executable CPU2 image linked for the address in `cpu2_firmware.offset`; ST FUS wireless-stack packages are encrypted and cannot be loaded directly | `firmware/stm32wb55-cpu2.bin` (optional; configured separately from the Zigbee join example) |
| CC2530 Z-Stack router | `DIYRuZ_Motion_Router.hex` 0.0.5 from the `koptserg/motion` releases | `firmware/DIYRuZ_Motion_Router.hex` (used by `cc2530_zigbee.yaml`) |
| nRF52840 board with SHT31 | the `sensor-sht31` variant of the same image (CI artifact `nrf52840-sensor-sht31-firmware`) | `firmware/nrf52840-sensor-sht31.bin` (used by `nrf52840_zigbee.yaml`) |

The plug's pins and BL0937 ratios default to the Zbeacon TS011F board (relay
PD2, LED PB1 active low, button PA0, CF PB4, CF1 PB5, SEL PD3); for another
board pass `relay_pin`, `cf_pin`, `variant`, ... as params.

## What is and is not modeled

* **Thermostat**: both chips run their real firmware and exchange real frames
  over the UART in both directions (`66 BB ...` in the two log files). The
  CA51F253L3 model has no GPIO, ADC or radio yet, so the display, keys, relay
  and NTC inputs of the main board are not connected to anything. The ZT3L
  application is linked behind a bootloader, which the emulator does not run;
  the file places the application's RAM code at address 0 and the rest at
  flash `0x8000` + offset (the two `zb_*` parameters come from the linker map).
* **Plug**: the BL0937 is a Lua part. CF pulses at `cf_hz_per_w` x watts, CF1
  carries the current or the voltage depending on SEL. Change the load with the
  `plug.meter.power_w` endpoint (also a scenario event or an API call). The
  calibration ratios are round numbers; use the ones your firmware expects.
* **nRF52840**: the CPU, interrupts, clocks, pads, timers, RTC, SAADC, TWIM,
  UARTE, the crypto/random blocks and the 802.15.4 RADIO are modelled (see
  `docs/nrf52840.md`); BLE is not. Pins are `P0.xx`/`P1.xx`.
* **EFR32MG21**: the CPU, interrupts, clocks, pads, EXTI and flash writes are
  modelled; the radio has the register interface of the zigbee-rs driver, not the
  silicon's (see `docs/efr32mg21.md`). Pins are `PA0`..`PD15`.
* **Radio**: log-distance path loss with shadowing; BLE is not connected to
  the TLSR8258 registers, so only Zigbee devices are shown.
