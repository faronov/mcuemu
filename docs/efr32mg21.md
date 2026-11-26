# EFR32MG21 (Cortex-M33)

Silicon Labs EFR32MG21A020F1024 (Series 2), the part of the Zephyr slwrb4180a images
and a superset of the BRD4181A's own part, on which the zigbee-rs images run: the
second Cortex-M chip in mcuemu, on the same `cortexm::Core` as the
nRF52840 but with an ARMv8-M Mainline (Cortex-M33) CPU.

```sh
./build/mcuemu run --model EFR32MG21 --firmware firmware/efr32mg21-sensor.bin --for 2s
./build/mcuemu sim examples/efr32mg21_zigbee.yaml --for 30s
```

`mcuemu run` takes `--firmware IMAGE`, `--for DURATION`, `--steps N` and
`--trace-mmio`. In a system file use `model: efr32mg21`; pads are `PA0`..`PD15`.
The model has no variants, no `supply_mv`, and a fixed 38.4 MHz clock.

## What is and is not the silicon

| Part | Status |
| --- | --- |
| CPU, NVIC, SysTick, FPU, WFE | Cortex-M33 without the Security Extension (one security state, no SAU). 4 priority bits, 51 interrupt lines (`EMU`..`RTCC`). |
| Flash / RAM | 1 MiB at 0, 96 KiB at `0x20000000` (the A020F1024 of the Zephyr slwrb4180a images; the BRD4181A part is smaller and its images fit). USERDATA (`0x0FE00000`) is erased, DEVINFO (`0x0FE08000`) holds the EUI-64 at `+0x1A0` and the unique id at `+0x48`/`+0x4C`
(`SYSTEM_GetUnique`, which Zephyr's hwinfo device id and the 802.15.4 address are built from). |
| Boot | The model has no Gecko Bootloader. An image whose reset vector lies at or above `0x4000` is an application-slot image: it is loaded at `0x4000` and started directly with `VTOR = 0x4000`. Any other image is loaded at 0. |
| Peripheral blocks | `0x40000000 + 0x4000 * n`, with the Series 2 aliases (`+0x1000` set, `+0x2000` clear, `+0x3000` toggle). |
| CMU (`0x40008000`), HFXO0 (`0x4000C000`) | Registers read back; `HFXO.STATUS` follows `CTRL` and the system clock selection (RDY/ENS, first-lock FSMLOCK) so the GSDK start-up sequence completes. The clock is the HFXO's 38.4 MHz from reset; the 20 MHz FSRCO is not modelled. |
| GPIO (`0x4003C000`) | Ports A..D: MODEL/MODEH modes (input, pull, push-pull, wired-or/and), DOUT, DIN, EXTI 0..7 (`EXTIPSEL/PINSEL`, rise/fall, IF/IEN) on `GPIO_EVEN`/`GPIO_ODD`. |
| I2C0 (`0x4A010000`) | Master only, driven through the GPIO routing (`GPIO+0x498` ROUTEEN, `+0x49C` SCLROUTE, `+0x4A0` SDAROUTE; port in bits 1:0, pin in bits 19:16) as an open-drain pair, so a Lua `i2c_slave` on the nets (`examples/parts/cht8305.lua`) sees a real bus. The emlib `I2C_Transfer` register sequence is supported: TXDATA address byte + CMD START, IF.ACK/NACK per byte, RXDATAV with CMD ACK/NACK (or AUTOACK), repeated START, CMD STOP and IF.MSTOP; the bit time follows CLKDIV at 38.4 MHz. Slave mode, clock stretching, arbitration and the bus timeouts are not modelled. INFERRED from emlib and the SDK header (`test_efr32_i2c_master_reads_cht8305` reads a CHT8305 temperature through it); the Tuya ZS3L image has not been seen using I2C0 in the first 120 s after joining. |
| MSC (`0x40030000`) | Page erase (8 KiB) and word programming (bits can only be cleared) by the register protocol of the zigbee-rs flash driver; instantaneous. |
| **RADIO** | Three register interfaces, chosen by what the firmware does; only one of them follows the silicon's registers. See below. |
| DPLL0 (`0x4001C000`) | `IF.LOCK` and `STATUS` follow `EN` at once (inferred from `CMU_DPLLLock`, which waits for the lock flag; no frequency model). |
| USART0 (`0x40058000`, the console) | TXDATA appends each byte to the chip's UART0 output (`run` prints it, `sim` has it as the endpoint `<mcu>.uart0`); STATUS reads TXC, TXBL and TXIDLE as set. Pin routing and the baud rate are not checked. |
| TIMER0..3 (`0x40048000` + `0x4000` n) | The counter only: `CMD.START`/`STOP` with `EN` set, `STATUS.RUNNING`, `CNT` counting core cycles divided by `CFG.PRESC + 1` up to `TOP`, then wrapping to 0 (TIMER0 32-bit, TIMER1..3 16-bit; `TOP` resets to `0xFFFF` on all four, so a 32-bit TIMER0 count needs `TOP` written, as the zigbee-rs HAL does). It keeps counting while the core sleeps (inferred; no EM2 clock gating). No compare/capture, flags or interrupts. The zigbee-rs HAL's busy-wait clock (`efr32s2-hal` `delay.rs`) runs on TIMER0. |
| EMU (`0x40004000`) | `RSTCAUSE` (power-on or system-request reset); `TEMP` reads 1193 (quarter kelvin, 25.1 C; a constant, inferred). The rest are stand-ins. |
| RTCC (`0x48000000` secure, `0x58000000` non-secure) | A 32-bit counter at 32768 Hz divided by `CFG.CNTPRESC` (inferred clock), started by `CMD.START` with `EN`; `STATUS.RUNNING` reads set meanwhile. CC0..CC2 in output-compare mode set `IF.CC0..CC2` (bits 2..4) when the counter reaches `OCVALUE + 1` (inferred from the sleeptimer HAL), the wrap to 0 sets `IF.OF`, and `IF & IEN` raises IRQ 10. The Zephyr kernel tick (CC1) and the zigbee-rs Embassy time driver (CC0, with CC1 for EM2 sleeps) run on it. |
| SEMAILBOX host (`0x4C000000` secure, `0x5C000000` non-secure) | The mailbox to the Secure Element. The SE is not modelled: every command is answered with SUCCESS and no output data (TX_STATUS.TXINT always set; RX_STATUS.RXINT and RX_HEADER after the words the TX_HEADER announces). Inferred from `sli_se_manager_mailbox.c`. Commands that return data (random numbers, keys, crypto) get no data. |
| Stand-in storage | Blocks the model does not implement read and write as plain memory: `0x44000000` (secure-only RADIOAES/BUFC/SMU), `0x5A014000` (HFRCOEM23), and, for the RAIL radio init, `0xB0000000` (FRC RAM), `0xB8000000` (PRORTC/radio registers) and `0x54000000` (the non-secure alias of the secure radio blocks). None of them has behaviour. |
| Everything else | Plain register stand-ins: values read back, no events, no interrupts (LDMA beyond the UART receive, I2C slave mode, CRYPTO, ...). Firmware that waits for one of them stops at the wait; `--trace-mmio` shows where. |

## The radio

The real radio is a sequencer (RAC) running Silicon Labs' RAIL configuration and
microcode; mcuemu does not execute that microcode. Three register interfaces are
modelled, each for the firmware that uses it:

- **The zigbee-rs `efr32s2-hal` driver** (`efr32s2-hal/src/radio`, the
  `efr32mg21-sensor` image since zigbee-rs a8710d66d2): `RsmRadio`
  (`emulator/src/efr32/rsm.cpp`, the register list in `blocks.hpp`). The driver
  uses the silicon's registers (FRC `0xB8004000`, AGC, SYNTH, RAC `0xB8020000`,
  BUFC `0x54004000`) and does the sequencer's work on the CPU: it loads its own
  compact vector table into sequencer RAM, every slot `END` except RXWARM and
  TXWARM/TXPD, which it parks on a self-`JUMP` while it calibrates the
  synthesizer by binary search with the LO counter. The model takes the radio
  over once such a table is loaded, and then holds the RSM (`RAC.STATUS.STATE`)
  in a state whose slot is a self-jump; it sends BUF0 when TXWARM is released
  (`FRC.IF.TXDONE` at the end), stores each frame heard while searching in BUF1
  as `[PHR][PSDU - FCS][RSSI]` (`FRC.IF.RXDONE`; RXOVERFLOW when it does not
  fit) and reports the channel energy in `AGC.RSSI`. INFERRED: the open-loop VCO
  curve (2.52 to 2.36 GHz over `VCOTUNING`), a loop that always locks, zero
  warm-up times (the driver's own waits pace it), the RSSI scale (61 dB above the
  antenna level, what the driver subtracts) and no `RAC.IF.STATECHANGE`
  interrupt. ACKs and address filtering are the driver's (software).
- **RAIL** (Zephyr's RAIL driver, the Silicon Labs EZSP NCP, the Tuya ZS3L image):
  `RailRegs` plus the RAIL sections of `efr32mg21.cpp`, built from what those
  images do (see the sections below). It is not the silicon either: the
  sequencer's effects (RX descriptors in the BUF2 ring, ACKs, PROTIMER events)
  are produced by the model.
- **The older zigbee-rs `efr32s2` scaffold** (`zigbee-mac/src/efr32s2/driver.rs`
  before a8710d66d2, unverified register values): `radio.cpp`, FRC, BUFC,
  SYNTH, RAC and AGC blocks at `0x40090000`, TX and RX FIFOs, RAC commands and
  status, interrupt `FRC_PRI` (IRQ 36), RSSI from the medium. The register map
  is in the header comment of `emulator/src/efr32/radio.cpp`. Current zigbee-rs
  images no longer use it.

Frames go through the shared `mcuemu::VirtualAir` as 802.15.4 MAC frames
without the FCS (timed with it, so a frame lasts as long on air as on a real
radio). The zigbee-rs receivers poll the medium every 16 us and take a frame
whose start-of-frame delimiter had not passed (128 us) when they started
listening, as in the nRF52840 model; the RAIL path polls every 100 us. All of
this timing is inferred.

## Board: BRD4181A

`examples/devices/efr32mg21_brd4181a.yaml`: LED0 on PB0 (active high), BTN0 on
PD2 (the WSTK pull-up, low = pressed). The zigbee-rs `efr32mg21-sensor` image
(an application-slot image linked at `0x4000`, a sleepy end device on the RTCC
with EM2 between polls) logs through `log` on the WSTK virtual COM port, USART0
at 115200 8N1 (`src/vcom_log.rs`), so its console is the endpoint
`board.mcu.uart0`: lines such as `18.251 I [NWK] Joined PAN 0xBF64 ch 15 as
0xB414`. Older images of the sensor logged over RTT; the current one has no
`_SEGGER_RTT`. `examples/efr32mg21_zigbee.yaml` puts the board on one medium with
the TLSR8258 sampleGW coordinator and attaches the console to a `text` sink: a
button press at 2 s starts the join, the first scans find no network yet, and the
retry at about 17 s associates, receives the NWK key and completes the
trust-center link key exchange (about 19 s in, with zigbee-rs 34cafefe and
`telink-gw.bin`); the sensor then reports the die temperature and a synthetic
humidity from about 78 s. EM2 is the core's WFI with `SLEEPDEEP`: the model keeps
every block running, and the RTCC compare wakes it.

## Silicon Labs EZSP NCP (Sonoff ZBDongle-E image)

The Zigbee NCP builds of the Nerivec `silabs-firmware-builder` releases (e.g.
`sonoff_zbdonglee_zigbee_ncp_8.2.2.0_115200_sw_flow.gbl`, EmberZNet 8.2.2, GSDK 2025.6.2) run
on the model up to the ASH UART. `scripts/gbl2bin.py IN.gbl OUT.bin` extracts the application
(a GBL program-data block for 0x4000); `mcuemu run --model EFR32MG21 --firmware OUT.bin --for 3s`
prints the UART0 bytes. After about 1.06 s (virtual) of NVM3 formatting and RAIL init the NCP
sends the power-on RSTACK, `1A C1 02 02 9B 7B 7E`, which is what a real adapter sends (ctest
`firmware_efr32mg21_ncp_rstack`, needs `firmware/zbdonglee-ncp.bin`).

What it needed beyond the zigbee-rs and Zephyr images (all in the source with the evidence):

- DEVINFO: `+0x1FC` bits 23:16 are checked against 0x80/0x81 (`sl_zigbee_system_common.c`
  assert, line 71; 0xFF made the image reboot in a loop; 0x81 is a guess between the two accepted
  values), `+0x08` MEMINFO gives the flash page size (`1 << ((MEMINFO + 10) & 31)`; NVM3 open rejects
  512 B, the erased value), `+0x0C` MSIZE.
- EMU RSTCAUSE (`+0x94`): bit 0 for a power-up, bit 10 after SYSRESETREQ. Without it the NCP's
  reset classification is "unknown" and it sends an ASH ERROR frame (`C2 02 00`) instead of RSTACK.
- RAIL init polls FRC IF bit 14 and FRC+0x08 bit 8 after a calibration the sequencer would run; both
  read as set (`RailRegs::read`).

Host to NCP: `mcuemu run ... --uart0-send HEX@TIME` (repeatable) writes bytes to USART0 at that virtual
time, and in `sim` the chip's UART is a normal endpoint (`uart_rx`). The NCP receives through an LDMA
ring (UARTDRV/iostream), so the model has a minimal `Ldma` (`blocks.cpp`): channel registers at `+0x5C +
0x30*n` (CFG, LOOP, CTRL, SRC, DST, LINK), descriptor load by LINKLOAD, and transfers from USART0 RXDATA,
which advance DST, count XFERCNT down and raise CHDONE/IF/IRQ 21. `scripts/ash_frame.py` builds and parses
ASH frames. Verified exchange (`firmware_efr32mg21_ncp_rstack`): RST (`1A C0 38 BC 7E`) makes the NCP
reboot and answer `C1 02 0B 0A 52 7E` (reset code 0x0B), and the EZSP `version` command returns protocol
0x12, stack 0x8220, i.e. EmberZNet 8.2.2. The NCP's ASH decoder drops bytes until it sees a flag or cancel
byte, so a host must send `1A` (or `7E`) before its first frame after a reset (the firmware's behaviour, not
the model's).

One inference the NCP forced: EMU RSTCAUSE bit 6 is a system-request reset (the image's table maps it
to "software reset, read the reset marker in RAM"), which makes the answer to RST a plain RSTACK 0x0B.

LDMA: CHx_LINK bit 1 is "link valid" and bit 0 "relative address" (the NCP tests bit 1). A LINKLOAD takes effect at
once, also on an enabled channel. The driver parks the receive channel on a "dummy" descriptor (source and
destination the same RAM byte, linked back to the receive descriptor) every 1.5 ms; "DST == that byte" is its
"ring empty" signal. The model lets such a no-op descriptor take the next USART request without reading RXDATA, so
the byte reaches the receive descriptor it links to. (An earlier version ignored LINKLOAD on enabled channels; that
made the firmware read phantom bytes forever and kept its main loop from ever running.)

Radio iteration (NCP, EZSP `startScan`; `scripts/efr32_trace.py` names the registers): the receiver goes RXWARM (RAC
STATUS.STATE 1) to RXSEARCH (2) about 40 us after the RX request, and AGC reports a valid RSSI (STATUS1.RSSISTATE
nonzero, RSSIINT -100 dBm) in those states. The NCP's `RAIL_GetRssi` waits for both (an energy scan hung on the RAC
STATUS poll before that).

PROTIMER time base (found through the NCP): PRECNTTOP (+0x30, image value 0x1233) is the prescaler period
in 1/256 core cycles (integer top in bits 31:8, fraction in 7:0), 19.2 cycles = 2 MHz, and the image's own time
conversion (constant 0x7FFEAAAB at RAM 0x2000433C) turns a WRAPCNT count into 0.49998 us. A compare match is
scheduled at its exact cycle.

Active scan, end to end (`firmware_efr32mg21_ncp_scan`): RAIL sends the beacon request through the hardware
listen-before-talk path (AGC LBT/CCA setup, PROTIMER LBT registers at +0x40..+0x54 and +0x74/+0x78, then PROTIMER CMD
bit 16 and a sequencer command in RAC SR0), not through a host write of RAC CMD.TXEN. INFERRED: the model treats
PROTIMER CMD bit 16 as "LBT started, channel clear" and sends the frame in BUF0 after a fixed ~200 us, then raises
FRC TXRAWEVENT like a TXEN transmit. A Telink coordinator on channel 11 answers with a beacon, the RX path delivers it
and the NCP reports `networkFoundHandler` (EZSP 0x1B) to the host. The sequencer microcode is still not modelled, so
other RAIL features that depend on it (ACK handling by the sequencer, CSMA backoff timing, TX/RX state timing) are
approximations. The host must send `hex:` UART scenario events (see docs/system-config.md) or use `--uart0-send`; a
host that polls `callback` with wrong ASH sequence/ack numbers gets ASH ERROR 0x06.

Forming a network and joining devices (`scripts/ncp_host.py`, a reactive EZSP/ASH host over `mcuemu serve`; `scripts/ncp_drive.py`
is the older scenario-based one): the host sends `setConfigurationValue` (stack profile 2, security level 5), `networkInit`,
`setInitialSecurityState` (note the order: bitmask, *pre-configured TC link key*, then network key), `formNetwork` and
`permitJoining`. The NCP then forms a network, answers beacon requests, and a Zephyr end device (TLSR8258) and the zigbee-rs
sensor associate with it: association request ACKed, data request ACKed with Frame Pending, association response, APS
Transport-Key. What made that work in the model:
- UART RX is paced at 115200 baud (a zero-time burst overran the 128-byte ring) and LDMA REQDIS is honoured.
- The ACK logic also reads the PAN and short address RAIL keeps in sequencer RAM (0xA0001EE0..EEC, one byte per word in bits 15:8).
- A MAC data request raises RAC IF bit 19 with the requester's address staged in BUF1 and SEQRAM 0xB0001EB4 (RAC SR1 bits 7:6
  = address mode); the stack's `RAIL_IEEE802154_SetFramePending` toggles RAC SR2 bit 4, which the ACK carries. RAC STATE reads
  RXFRAME while the event is open. RAIL's packet filterMask (BUF2 descriptor byte 3) is set for frames addressed to the NCP.
- The Secure Engine mailbox now executes AES-128 ECB (command 0x04000100): the stack builds its CCM* and key hashing on it, and
  the transport key decrypts correctly with the key-transport key.

LDMA CHDONE: it means the whole descriptor chain has finished, not one descriptor. The NCP's UART driver parks the receive
channel on the dummy descriptor and then rebuilds "the rest of the running descriptor" from CHx_CTRL.XFERCNT unless CHDONE
is set; with CHDONE raised by the parking descriptor it built a descriptor of 2048 bytes, the UART RX ring (128 bytes) was
overrun and the neighbouring buffer-management data was overwritten. At about 11.6 s the stack hit an assert
(`buffer-management.c:1345`, a buffer handle outside the pool), reset itself with reason 0x0701 and later sent
ASH ERROR 0x06. CHDONE is now set only when a descriptor without a valid link ends.

With that fixed the zigbee-rs sensor (`efr32mg21-sensor.bin`, then an image with the `0x40090000` scaffold radio driver) joins the NCP's network end to end:

    python3 scripts/ncp_host.py join.yaml --root . --for 30s --event "at=8s endpoint=board.key.pressed value=true" \
        --event "at=8.2s endpoint=board.key.pressed value=false"

(an NCP `ncp` plus the BRD4181A board with the sensor on one air; the YAML must sit inside `--root`.) The host sees
`trustCenterJoinHandler` (the standard secured join, about 10.7 s), `childJoinHandler` and the ZDO Device_annce as an
`incomingMessageHandler` (about 12.7 s). `MCUEMU_EFR32_TRACE=FILE` logs a sim chip's MMIO.

Tests: `firmware_efr32mg21_ncp_join_efr32` (Zephyr ED on an EFR32MG21) and `firmware_efr32mg21_ncp_join_esp32h2` (zigbee-rs sensor on an
ESP32-H2, which also covers a RISC-V and an ARM engine in one process) run `ncp_host.py` for 26 s and expect `trustCenterJoinHandler`.
Some devices need luck with the NCP's random beacon-response delay (7..190 ms): the TLSR8258 plug scans 71 ms per channel,
once, and only joined with a later start (`start_at: 13s`); the Z03MMC scans at power-up and again after 60 s.

### Tuya ZS3L thermometer (ZSS-ZK-THL, TS0222), status

Sensors (Moes ZSS-ZK-THL "Smart Brightness Thermometer", TS0222): the firmware reads them at boot even without a network, over a
bit-banged I2C bus on **PA0 (SDA) and PA3 (SCL)** (SDA is released by switching PA0 between push-pull low and input; found
from the GPIO mode and DOUT traffic, then confirmed by the addresses it probes). A spy slave saw two devices: **0x40**, the
CHT8305 (`examples/parts/cht8305.lua`), and **0x44**, an OPT3001 light sensor (`examples/parts/opt3001.lua`: it reads register
0x7E, the manufacturer id, writes the limits 0x03 and 0x02 and the configuration 0xCE11, then reads the result). Both Lua parts
count `reads` / `writes` endpoints. The battery is measured with IADC0 SINGLE = SUPPLY (`0x1001`). The e-ink display is a 9-bit 3-wire SPI (data/command bit first, MSB first, sampled on the SCK rising edge, one frame per CS pulse)
on PD0 (SCK), PC2 (DATA) and PB1 (CS), decoded by `examples/parts/epd_zsszkthl.lua`: init commands `04 00 01 30 03`, waveform tables
(`20 23 24 25 26`, 15 bytes each), `18` + 18 bytes of segment RAM, `11 ff` = refresh. Only the temperature moved the RAM in the first
14 s (humidity 10..90 % and 5..5000 lx left it unchanged); the bit layout of the segments is not plain 7-segment bytes (0.0 and 0.1 °C
give the same RAM, the tenths change bytes 7..9) and is not decoded. PA5, PC1 and PD1 are other control pads. `examples/devices/tuya_zs3l_thermometer.yaml` wires all of this, and `scripts/moes_brightness_thermometer.py DUMP` runs the whole scenario (NCP, erased NVM3, the PB0 long press, both sensors and the display) and prints the join events the NCP saw, the sensor read counts and the display refreshes with the last segment RAM (`--temperature`, `--humidity`, `--lux`, `--channel`, `--keep-nvm`, `--no-press`).

The full-flash dump of pvvx's ZSS-ZK-THL page (Gecko bootloader at 0, app at `0x4000`, NVM3 at `0xB6000..0xBFFFF`) runs
(`firmware: [{file: dump.bin, offset: 0, size: 0x4000}, {file: dump.bin, offset: 0x4000, from: 0x4000}]`) but does **not**
join the NCP's network yet. What it needed in the model: the `0x4A000000` low-energy window (WDOG, I2C0, ACMP, LETIMER as
registers; IADC0 completes at once), the secure aliases `0xA0`/`0xA8` of the radio blocks, `0xA801803C` reading 1 before it is
written, and dropped (not faulting) CPU stores to flash. In the factory-new state (NVM3 erased) it sleeps for 4 h; PB0 (EXTI0,
falling edge, found from its GPIO set-up) wakes it, and a long press starts the radio: from then on it wakes every ~0.4 s for an
RX window (RAC CMD `0x20`/`0x100`, PROTIMER) but never writes RAC CMD.TXEN, so no frame reaches the air. I2C0 is not touched in
the first 45 s, so the CHT8305 is not what blocks it.

What the long press starts is an active scan: the application calls the stack's scan start (`0x2CECC`: type 1, one channel
bit per call, duration 3) for 14, 15, 19, 20, 24 and 25 (channel 11 is the idle channel between them), and the stack's RAIL
layer sends a beacon request through `0x31B8C` -> `0x35088` (the RAIL transmit start). It failed with EmberStatus 0x8C
(PHY_TX_BUSY, three retries per channel, then the next channel) because the start peeks byte 3/4 of the frame in BUF0 with
`BUFC BUF0 STATUS` (+0x28, bits 12:0) as the length check, and the model did not derive that count from
WRITEOFFSET - READOFFSET (now it does). This image also leaves the SYNTH channel field (`0xA8018044`) at 0 and sets the carrier
as a word at `0xA8018034` (channel 11 = `0x1F50AAA`, +`0x10AAA` per channel), which `rail_channel()` now follows. With both,
the beacon request reaches the air and a NCP on the same channel answers it with a beacon (checked with `--channel 24`).

Receive: this RAIL keeps the BUF2 descriptor ring inside its context at `0x20006A70` (sixteen 8-byte entries at +0x10, so
the context is BUF2 ADDR - 0x10; two halfwords at +0 and +2 are the pending-bytes-plus-7 and the consumed bytes; the entry
that the BUF2 threshold handler `0x3B254` takes next is `(h0 + h1 - 7) >> 3 & 15`, the same formula the NCP's ring uses).
`deliver_rail_rx()` falls back to that layout when `rx_context()` finds no NCP-style structure. With it the BUF2 threshold
(BUFC IF bit 18) is taken, `0x3B254` reads the descriptor (status 7 -> RAIL event bit 8) and `0x3B168` consumes it (bit 15 of the
entry's first halfword is set), for the NCP's beacon as well.

The scan hung because of the PROTIMER time base: the image's timers are PROTIMER compare channels (`+0x100+16n`, CTRL
`0x11`, WRAP at +0x0C; the MAC's end-of-dwell callback is armed through `0x37644` as `WRAPCNT + delay`), and the model's
WRAPCNT *read* returned `cycles / divisor` while `service()` compared against `cycles * 256 / divisor` (the 2 MHz count), so
every compare was already in the past for the match test and its IF bit was never set. Both now use the 2 MHz count
(`cycles * 256 / (PRECNTTOP + 0x100)`). With that the scan steps through its channels, and on the NCP's channel the device
sends a second beacon request, then an **Association Request** (acked by the NCP, `--channel 24`), about 0.1 s after the beacon.

The next stall was the data request: its LBT start waits for RAC STATUS.STATE to report RX, and this image requests RX through
RXENSRCEN bit 4 (`0x10`) rather than bit 1, which the model did not treat as an RX request. Both bits count now. With that the
device **associates**: Data Request, Association Response (short address, `trustCenterJoinHandler` EZSP 0x24 at the host), the
NCP's APS Transport-Key command, then a Data Request every 200 ms (checked with `--channel 24`; the device scans channels 14,
15, 19, 20, 24, 25 and 11 in that order of its own, so the NCP has to be on one of them).

The stack's AES is RADIOAES (`0x44000000`), a CRYPTOACC-style engine fed through scatter-gather descriptors (FETCHADDR +0x04,
PUSHADDR +0x14, CTRL +0x34, CMD +0x38, STATUS +0x3C). `radioaes.cpp` models it from `sli_protocol_crypto_radioaes.c`: ECB, CBC, CTR
and CCM/CCM* (RFC 3610 vector 1 and FIPS-197 in `test_efr32_radioaes`). With it the device decrypts the APS Transport Key,
installs the network key and sends NWK-secured frames (an End Device Timeout Request and the Device_annce broadcast; a captured
frame verifies with the NWK key and openssl).

Both chips had the same EUI-64 (`0x0123456789ABCDEF`), so the NCP ignored the joiner's secured frames as its own. The mcu key
`eui64: 0x...` (efr32mg21 only) sets the device information page's EUI-64; with distinct ids the NCP reports
`trustCenterJoinHandler`, `childJoinHandler` and the ZDO Device_annce (`incomingMessageHandler`, cluster 0x0013).

Result: the Tuya ZS3L ZSS-ZK-THL image (full-flash dump of pvvx's page, NVM3 erased, a long press on PB0 as the pairing
trigger, the NCP on one of the device's scan channels) joins the NCP's network in the emulator. `firmware_efr32mg21_ncp_join_efr32`
now also uses distinct EUI-64s and requires the childJoinHandler and the Device_annce.

The `MCUEMU_EFR32_TRACE_PC=FILE` + `MCUEMU_EFR32_TRACE_PC_RANGE=FIRST:LAST[:FROM_NS]` and `MCUEMU_EFR32_TRACE_NODE=NAME` environment
variables (sim), `scripts/ncp_host.py --peek MCU:ADDR:LEN` and `--watch MCU:ADDR:LEN@FROM-TO` were added while finding this.

Not yet tried: `joinNetwork` (the NCP as a joiner). From the image, the UART is USART0 with routes
written as TX = PB1, RX = PB0 (decoded with the alphabetical Series 2 `USARTROUTE` layout: CS, CTS, RTS, RX, SCLK, TX; not
confirmed against a schematic).

## RAIL registers

The Silicon Labs RAIL library drives the radio through blocks outside the
`0x40090000` scaffold map. `RailRegs` (`emulator/src/efr32/blocks.cpp`) holds them
(and, as plain storage, the registers of the zigbee-rs `efr32s2-hal` driver, which
`RsmRadio` gives their behaviour).
Addresses are the secure view (the non-secure `0xB…` view folds onto `0xA…`):

| Block | Address | Use |
| --- | --- | --- |
| RDMEM | `0xA0000000` (SEQRAM, 4 KiB), `0xA0002000` (FRCRAM) | RAM; RAIL copies the sequencer image here from flash (`0x2B65C` in the Zephyr image). No aliases. |
| RAC | `0xA8020000` | State and requests. Set/clear/toggle aliases at `+0x1000/+0x2000/+0x3000`. |
| PROTIMER | `0xA801C000` | Image layout: `IF` at `+0x64`, `IEN` at `+0x70` (RAIL sets IEN with `0x3001804`, clears IF with the same mask). The vendor v5.8.0 header is shifted by `+0x10` for these (IF `+0x74`, IEN `+0x80`, PRECNTTOP `+0x40` against `+0x30` in the image). |
| BUFC | `0x44004000`, `0x54004000` | Buffer controller. |

Hooks, all inferred from the code that polls them (not documented):

- `RAC+0x0C` bits `[27:24]` are the radio state: 0 is idle, 7 to 9 and 12 are
  active states (`recovered_rac_state_code_matches`). No hook: the stored value is used.
- `RAC+0x08` bit 5 is a request the hardware clears when done. RAIL sets it and
  polls it to zero, so it is cleared on write.
- `SEQRAM+0x1EA0` is polled with masks `0x800`, `0x20000`, `0x40000`. No hook yet.
- PROTIMER (`Efr32mg21::service`, layout from `recovery/reconstructed.c`): `WRAPCNT`
  (`+0x20`) advances once per `PRECNTTOP+0x100` core cycles (`+0x30`). Compare channel
  n is at `+0x100+16n`: `CTRL` bit 0 enable, bit 1 CCMODE (0 compare, 1 capture), so
  `CTRL & 3 == 1` is an enabled compare; `WRAP` at `+0x0C`. When WRAPCNT passes WRAP,
  IF bit `8+n` is set (`+0x64`), and IRQ 36 is `IF & IEN` (`+0x70`). The earlier
  experiments found that bit 11 (CC3) reaches the RAIL callback. Bit selection from experiments with forced `IF` words: `0x800`, `0x1000` and
  `0x2000000` reach the callback; `0x1`, `0x4` and `0x1000000` do not. `+0x14` is not
  needed. The callback runs, but transmit still fails with status 14, so something
  else in the chain is still missing.
- Series 2 aliases apply to the register blocks, not to RDMEM.

Event path (from an ordered trace of a run where the callback fires): 
`FRC_IRQHandler` → `sli_rail_e5d70455…` (calls the hook at `0x200042A0` if set)
→ `sli_rail_978af8…` (stores the event flag and ANDs the global event word at
`0x20004028` with the config masks in the struct at `0x20001D88`) →
`sli_rail_2c0c7b…` → `sli_rail_d1d4eb…` (ANDs the handle's mask pair at
`+0x158` with the event bits; if nonzero, tail-calls the callback pointer at
`0x20000730`, which is `sl_802154_rail_events_callback`). So the callback is
reached through a pointer and a RAM event word, not through a hardware vector.

Transmit gate (solved): the transmit path (`sli_rail_3d0930...`, reached from
`sl_802154`) returns status 14 unless the PROTIMER status word at image `+0x14`
bit 0 is set. In the image layout that word is the vendor STATUS register
(header `+0x24`): the image uses header - 0x10 for the PROTIMER registers, which
checks out for PRECNTTOP, IF, IEN, WRAPCNT and CTRL. STATUS.RUNNING is set by CMD
START (bit 0) or RTCSYNCSTART (bit 1) and cleared by CMD STOP (bit 2), per the
vendor header. The model does this, and the refusal is gone (60 M steps: no
status-14 errors).

PROTIMER WRAPCNT (image `+0x20`) is a live counter while RUNNING: it counts
core cycles divided by PRECNTTOP+0x100. Without it, `sl_rail_get_time` busy-waits
forever.

Transmit completion (found from the RAIL event table, `sl_rail_types.h`): the
completion event is `TX_PACKET_SENT` (bit 24 of the low event word). In
`FRC_IRQHandler` it is set when `FRC.IF & FRC.IEN` has bit 15 (`0x8000`), at
`0xA8004068` / `0xA800406C`. Tested: one event window (a single pulse of IRQ 34 with
that bit forced) runs the event callback once and the stack then goes to sleep, so
it waits for a further event (probably an ACK or receive timeout, delivered by a RAIL
timer). Forcing the bit for the whole run keeps the core busy in the handler. The
next candidate is the RAIL multi-timer (PROTIMER TOUT or RTCC compare); not yet
checked.

Event bits from `RAC.IF & RAC.IEN` (`r9` in `FRC_IRQHandler`, RAC at `0xA8020000`,
IF `+0x1C`, IEN `+0x20`): `0x400000` gives `RX_ACK_TIMEOUT` (bit 1 of the low event
word), `0x10000` gives `RX_TIMEOUT` (bit 12), `0x100000` gives `RX_FILTER_PASSED`
(bit 17). The stack waits for an ACK after a transmit, so `RX_ACK_TIMEOUT` is the
likely next event. A windowed test (TX-sent window, then an ACK-timeout window, both
with the line pulsed every 100 us) did not progress the firmware. Sampling shows
that the run then spent most of its time in the FRC event path (an interrupt storm
created by the test pulses), so that way of pulsing is wrong and the result is not
conclusive. A correct test needs one level change per event, not repeated pulses.

Transmit completion (inferred, hardware-side): a write of RAC CMD.TXEN (RAC `+0x10`
bit 0) starts a frame. The chip raises FRC IF bit 15 (`TX_PACKET_SENT`) 1 ms later
(a fixed frame length; the TX buffer is not read) and then RAC IF bit 22
(`RX_ACK_TIMEOUT`) after the 864 us ACK wait, because no ACK comes back: the RAIL
model does not receive frames from the air. Both flags go through their IF/IEN
pairs to FRC (34) and RAC_SEQ (38). These are hardware events; the firmware's own
FRC and RAC handlers and RAIL callbacks run unchanged. With this model the router
image no longer stalls after init: it reaches the commissioning retry
(`app_bdb ... retry scheduled`) at 0.75 s intervals, with no asserts or faults in
3 s of virtual time.

Findings for the frame path (from a trace of the router image, observed):
TX frames are written byte by byte to BUFC BUF0 `WRITEDATA` (`0x54004020`, with
`WRITEOFFSET` `0x54004010` advancing); the first byte is the PHR (frame length).
The first frame is a beacon request (`0x0a 0x03 0x08 ...`). BUF3 `ADDR` points into
SRAM (`0x20001de4`), so buffer RAM is host-visible memory. The RDMEM sequencer
microcode (`0xB0000000...`, about 4 KiB) is loaded by the firmware at start-up; the
hardware sequencer runs it, and the model does not. The channel is set through the
SYNTH frequency word (`0xB8018034` = `0x01F50AAB` for the router's scan); the mapping
from that word to an 802.15.4 channel is not established yet.

RAIL TX to the air (done): the frame is the PHR plus PSDU in the BUF0 buffer RAM
(BUF0 `ADDR` `+0x0C`, host SRAM; read at TX start, WRITEDATA bytes as fallback). The
channel is the SYNTH channel field (`0xB8018044`, 11..26). `Radio::transmit_fifo` sends
it on the virtual air with the zigbee radio's PHR rule. In `sim` the router's beacon
requests now appear in the pcap (`03 08 seq ff ff ff ff 07`), about one every 50 ms.
Frames are sent every ~50 ms during the scan, with no RX started yet.

RAIL RX (attempted, not yet effective): RX is on while RAC RXENSRCEN bit 1 is set
(the RX request the firmware makes before it starts a receive). RAC STATUS reports
the RX state (code 1 = RAIL state 2) while it is on. Frames heard on the SYNTH channel
are written to the BUF1 RAM (BUF1 ADDR `+0x3C`, inferred as the RX buffer) with
WRITEOFFSET set, and FRC RXDONE (bit 4) is raised. None of this is confirmed by the
firmware yet: no BUFC READDATA is read.

Sim observations (2026-10-17): the EFR32 console is now a sim UART endpoint
(`board.mcu.uart0`), so the Zephyr router's log can be attached to a file. The
router scans all 16 channels once (16 beacon requests, 0.8 s), then its commissioning
retries every 5 s send no new requests. The coordinator (`firmware/telink-gw.bin`)
first beacons at about 21 s of virtual time (seen in the zigbee-rs sensor scenario
too), so a 20 s run is too short to see a join attempt. The router's first scan ends
before that, and the stack then waits on a RAIL event that is not produced.

Ruled out as the scan window end (traced): PROTIMER compare channels are configured
in mode 3, not compare mode 1, so the model's compare logic never fires for them; and
RAC STIMER/SR registers (`+0x64…+0x7C`) are written only once at boot. The window end
is therefore most likely produced by the RAIL sequencer microcode in RDMEM, which the
model does not execute.

Also ruled out: RAC RX_TIMEOUT (IF bit 16) as the window end. The firmware never
enables that interrupt (RAC IEN SET writes only touch bits 2, 18, 19, 21, 22), and a
50 ms window model using it did not change the router's behaviour (reverted).

Retry path (firmware side, sample `zigbee_shell`): every 5 s `app_bdb_commissioning_retry`
calls `app_bdb_start_commissioning` -> `bdb_networkSteerStart` -> `bdb_topLevelCommissioning`,
which returns busy only if the BDB state is not idle, the NWK manager is not idle, or
`commissioningStatus` is IN_PROGRESS. In a 20 s run the NWK manager is idle (state 0,
scan channels 0x07fff800, discovery callback set; read with `run --dump-memory`), and the
BDB task still completes each retry (status 0x03 is logged) without sending a new beacon
request. The next step is what `BDB_EVT_COMMISSIONING_START` does for a router that has
not joined; `g_bdbAttrs` is not in the router ELF symbols.

`mcuemu run` (EFR32) accepts `--dump-memory ADDRESS:COUNT`, which prints bytes from the end
of the run through the core's memory map.

Verified retry chain (Zephyr zigbee sources, `zephyr/subsys/zigbee`): the BDB task
for a router that has not joined calls `bdb_networkSteerFactoryNew` ->
`zb_nwkDiscovery` -> `zdo_nwkDiscoveryStart`. The gates pass (scan attempts 1, time
between scans 100 ms, NWK manager idle, no discovery timer), so `NWK_NLME_NWK_DISCOVERY_REQ`
is posted. The Zigbee thread drains the NWK and MAC queues on every pass, so the request
is not lost there. `bdb_nwkDiscCnfCb` then runs `zb_assocJoinReq`; when that fails it sets
NO_NETWORK (status 0x03) and the application logs the retry. The first pass sends 16 beacon
requests; the retries send none, so the NWK layer's MAC active scan is not starting on
the second and later passes. Next: the MAC scan state after the first pass (a scan-busy
flag or a missing scan-done callback in the stack library).

TX buffer (traced, then confirmed by the on-air result): the firmware's FIFO writer
(`sli_rail_d098…`) stores the first four bytes of a frame through BUF0 WRITEDATA
(`0x54004020`, one byte per 32-bit store, value in the low lane) and
copies the rest with memcpy into the buffer RAM at BUF0 ADDR + WRITEOFFSET. Each FIFO
store advances WRITEOFFSET. So the model writes every WRITEDATA byte into RAM at ADDR +
WRITEOFFSET, advances WRITEOFFSET, clears it on CMD.CLEAR, and reads the frame from RAM
by its PHR. With that, the router's beacon requests are complete on the air
(`03 08 seq ff ff ff ff 07`), and the coordinator answers them with beacons
(`0x0080`, 18 in 100 s). The router does not receive them yet.

RX (inferred, not confirmed by the firmware): RX is on while RAC RXENSRCEN bit 1 is set.
A received frame is written to the BUF2 buffer RAM (BUF2 ADDR `0x4400406C`) and its length
goes into BUF2 WRITEOFFSET. BUFC IF bit 18 (BUF2 threshold) is deliberately not raised:
with it set the firmware's transmit path stops after its first scan (49 beacon requests in
30 s, against 138 without it). The RAC status codes for the receive states are RXWARM 1,
RXSEARCH 2 and RXFRAME 3; RXSEARCH while searching and an RXFRAME pulse on delivery both
left the transmit path stopped or had no measured effect, so the status stays RXWARM.
With the current model the coordinator's beacons (`0x0080`) appear on the air (6 in 30 s),
but the router does not receive them.

BUF2 threshold handler (disassembled, `FRC_IRQHandler` bit-18 branch, `sli_rail_0906929b…`
at 0x1EFA8): it reads the RAIL RX context from the global at 0x20004048 (fields `+0x3A`,
`+0x3C`, `+0x3E`, `+0x18`), writes the next threshold to BUF2 THRESHOLDCTRL
(`0x54004000 + 0x8C`), clears BUFC IF bit 18 through its CLR alias, and stores the received
frame descriptor in the global at 0x200042A8, which the rest of the RX code reads. So the
firmware expects bit 18 to fire for a real received frame, with its RX context set up by
RAIL init. The model raised the bit without that context; that is the likely reason the
transmit path stopped (not measured directly). Next: check the RX context at 0x20004048 in
the trace, and only then raise the bit for a delivered frame.

MAC scan state after the first pass (checked against `g_zbMacCtx`, `tl_zb_mac.h`, status at
`0x20003D69`): at the end of a 20 s run the status is NORMAL (0) and the current channel is 11,
so the MAC is not stuck in ACTIVE_SCAN. The second sweep therefore does not stop on that
check. Still open: the second `MAC_MLME_SCAN_REQ` may fail at the handler's parameter check
(`scanDuration > 7`, from the runtime `g_bdbAttrs.scanDuration`, which is not in the ELF
symbols), which returns a confirm at once without any RF sweep.

Current state (committed build, 30 s): the scans repeat. The retries send 138 beacon
requests (16 for the first pass), and each retry logs `commissioning pending` after its
scan. The coordinator answers with beacons (6 in 30 s), but the router does not receive
them, so every scan ends with no network. So the blocker is the receive path, not the
scan restart. Checked on the way: `g_bdbAttrs` is at 0x200039E8 (found by the default
bytes of `g_bdbAttrsDft` in SRAM), with primaryChannelSet 0x07FFF800, scanDuration 3,
commissioningMode 2 (network steer), commissioningStatus 3 (NO_NETWORK), so the BDB
parameters are valid. The earlier "stall after the first scan" statement applied to a
build before the TX buffer fix.

RX context (dumped at the end of a 30 s run): the pointer at 0x20004048 is 0x20001D90. Its
fields: ring base (32-bit) at +0x18 = 0x20001F38 (the BUF2 ADDR), size at +0x3A = 128, write
index at +0x3C and read index at +0x3E. The threshold handler places the frame descriptor at
base + ((write + read - 7) & (size - 1) & ~7). The model now places delivered frames there
(inferred, not confirmed by the firmware). Setting BUFC IF bit 18 stops the transmit path
even with this placement (49 beacon requests in 30 s against 138), and a 100 s trace has no
firmware read of BUFC IF at all (0x540040E0), so the stop mechanism is still unknown. The
bit stays down.

BUF2 threshold flag, isolated: raising BUFC IF bit 18 alone, with no frame written to RAM,
also stops the transmit path (49 beacon requests in 30 s against 138). The firmware reads
no BUF2 register (0x54004068..0x540040A8) and no BUFC IF (0x540040E0) in a 100 s trace, so
the stop is not a poll of this block. Its cause is open.

Traced divergence (sim trace, flag vs no flag, first differing line): the firmware does read
BUFC IF, at its secure address `0x440040E0` (the earlier 0x540040E0 search missed it). With
the BUF2 bit set, `FRC_IRQHandler` takes the bit-18 branch (the threshold helper, IF_CLR for
bit 18, BUF2 THRESHOLDCTRL = 0xF) and afterwards does not run the rest of the event handling.
Without the bit the handler runs 1162 more times and acknowledges RX_ACK_TIMEOUT
(RAC IF 0x400000); with it, the handler is never entered again, no transmit happens, and no
pending timer retry is printed, so the core stops making peripheral accesses. Causes not
checked yet: a critical section left open by the branch (BASEPRI), or the frame content the
RAIL RX code reads from the descriptor.

Critical sections in `FRC_IRQHandler` are balanced before the BUF2 branch: enter 0x1B452 /
exit 0x1B488, enter 0x1B49A / exit 0x1B4C4. The branch (call at 0x1B5F0) runs after both exits
and opens no section of its own, so an open BASEPRI section is not the explanation. Left
open: what the core does after the BUF2 branch, and whether RAIL reads the frame descriptor
(0x200042A8) and waits on it.

Cause of the earlier stop (found in the traced run): the core was not waiting, it was in
RAILCb_AssertFailed, an endless loop after a RAIL assert. The threshold handler switches on
the frame status, which is the low three bits of the descriptor byte at +2 (the value is
returned by `sli_rail_0906929b...` at 0x1EFA8). Status 1..7 selects an RX event; the default
branch (0x1B694) asserts (file 3, line 1568). A descriptor with status 0 therefore asserts.
Statuses 6 and 7 both set event bit 3 (RX_PACKET_RECEIVED). The model now writes a
three-byte header `00 00 06` before the frame in the ring, and raises BUFC IF bit 18 on
delivery. With this, transmits continue (336 beacon requests and 18 coordinator beacons in
100 s) and no assert is reached. The header layout is inferred from the handler; the router
still does not join, so the frame is either not parsed by RAIL or not passed up the stack.

After a delivered frame (fake frame in run mode, status 6): the threshold handler runs once
without assert and `FRC_IRQHandler` continues. It then spins on PROTIMER WRAPCNT
(`0xB801C020`, the pointer at literal 0x1B680 -> 0x2B644), which is a legitimate wait for the
next wrap. Afterwards the event word stored at 0x20004024 is 0x56 and the mask at 0x20004028
is 0x0E, and the 802.15.4 callback region (0x17700..0x177FF) is never executed. The handler
clears event bits with `bic r4, r4, #0xDF` after the status switch, so status 6 and 7
(event bit 3, 0x8) are dropped there. Status 3 (0x800) survives that mask but the callback is
still not reached in a run mode test. So RX_PACKET_RECEIVED is not produced by this path yet.

After the threshold switch, the post-event path (0x1B6A0 onward) reads BUF2 CTRL (`0x54004068`)
and STATUS (`0x54004088`) and checks two RAIL globals ([r3] at 0x1B704, [fp] at 0x1B70C). Only
when those pass, and when a bit of the sequencer RAM word `0xB0001ED8` (read at 0x1B924) matches
the event, does the code reach the 802.15.4 callback. Setting the BUF2 STATUS count to the
frame length does not change that: the callback region is still not reached (0 hits in 8 s).
The sequencer RAM word is written by the firmware at start-up (0x56) and would be updated by
the sequencer microcode when a frame arrives. The model keeps RDMEM as plain storage, so that
update is missing. That, not the descriptor or the count, is the next gap.

Sequencer handshake word (RDMEM `0xB0001EA0`, RAIL-side view): after a delivered frame the
firmware reads it about 770 times in 3 s. Two helpers use it: `sli_rail_10ac1...` (0x1C4F0)
returns 1 when every bit of its mask is set in the word, and `sli_rail_fc431c...` (0x1C4CA)
writes masked fields into it. The masks at the call sites are 0x20000, 0x800 and 0x40000
(bits 17, 11 and 18), and a fourth call at 0x1D69C uses a mask set earlier. In the values the
firmware reads back (0x17E09B0 and 0x17E0990) bits 17, 11 and 18 are already set, so these
checks pass. The repeated poll at 0x1C4F2 is therefore on another mask (not yet found).
The word is written by the firmware itself; its acknowledgment bits, if the sequencer sets
any, are not visible from the firmware side yet.

Mask at the 0x1D69C call: 0x800 (set by `mov r1, #2048` just before it), so the four call
sites use 0x800, 0x20000, 0x40000 and 0x800 again. All of them are set in the values the
firmware reads back, so these helpers do not block the receive path. The remaining gates
are the RAIL globals tested at 0x1B704/0x1B70C and the bits of SEQRAM word 0xB0001ED8
(read at 0x1B924), which the model does not update after a frame.

RAIL gate flags (addresses from the literal pool): A = 0x200048D0, B = 0x200048CF,
C = 0x200048CC, D = 0x200048CB; the RX handle struct is 0x20001D88. In a normal run without
any received frame all four are 0 at the end. `sli_rail_6d89...` (0x1BD6C) clears B, and
if A is set it clears A and calls `sli_rail_2c4d...` (0x1BD1C), which writes RAC SET/CLR
registers (0xB8021000 / 0xB8022000). The threshold path tests A first (0x1B6EC: if A is 0 the
path exits), so A is the flag that must be raised for a received frame. No code in the
disassembly writes A or C directly (only reads and clears were found), so the writer is
reached through a computed pointer; not located yet.

Gate flag B (0x200048CF) is set to 1 by the FRC handler (0x1BBE8) when the helper
`sli_rail_12973c...` (0x2081C) returns RAC state 3 (RXFRAME) and a status byte is 3, so RAIL
treats a frame as being received while RAC is in RXFRAME. The model now holds RXFRAME from a
frame's delivery until the firmware clears BUFC IF bit 18, then returns to RXWARM. The
transmit path is unchanged (138 beacon requests in 30 s). The callback is still not reached:
the handler exits at 0x1B6EC because gate flag A (0x200048D0) is 0, and no direct writer of A
was found in the disassembly (writes through a computed pointer are likely). Flag A is the
next thing to locate.

Gate flag A (0x200048D0, RAIL-private symbol `sli_rail_45d7f5df...`) has one writer in the
RAIL archive: `sli_rail_0c014d5b...` (rail_rx.o), which sets A to 1 and writes RAC SR3 bits
4..5 (`0xB8021000 + 0x7C`). That function belongs to the RAIL_ScheduleRx path, and no symbol or
call of it is linked in the router image. The regular receive start in the image
(`RAIL_StartRx` -> `sli_rail_7a1b...` -> `sli_rail_dadf...` at 0x1D384) sets RXENSRCEN bit 1
and does not touch A. So in this firmware nothing sets A, and the threshold branch exits at
0x1B6EC. Also, FRC handler's RXDONE branch (IF bit 4) exists, but IEN bit 4 is never set in
the traces. The question is therefore which path RAIL uses to report a regular received
packet on this build; not answered yet.

Interrupt enables checked over the 100 s trace: FRC IEN is set only for bits 1, 13 and 15
(bit 4, RXDONE, is never enabled). BUFC IEN is set to 0x50000 (bits 16 and 18, the BUF2
overflow and threshold). So the BUF2 threshold is the receive route in this build. The
function-pointer slot the helper at 0x1E904 calls through (0x200042A4) is 0 at the end of
a run, so that helper does nothing here; the event callback slot (0x20000730) is set
(0x17759, sl_802154_rail_events_callback). Flag A still has no writer among the linked
functions. The remaining question is what sets A (or clears the 0x1B6EC exit) in a regular
receive.

Status and decision (2026-10-17): the join of the Zephyr RAIL pair is deferred. Reached and
kept: beacon requests go out on the air with correct frames (336 in 100 s), the coordinator
answers with beacons, the firmware runs without faults, and the RX delivery path up to the
BUF2 threshold runs without asserts. Not reached: the RAIL receive callback. The blocker is
the writer of gate flag A (0x200048D0), which is not among the linked functions. Next step:
a sim trace of RAM writes to 0x200048D0 with the writing PC during a real reception (needs a
write watch on core RAM, which the core does not have yet).

Watch on gate flag A (0x200048D0), temporary per-instruction byte compare in the core: the
byte never changes, in a standalone run and in the sim run with the coordinator (6 beacons
and 138 requests in 30 s, so frames are delivered). The firmware never sets flag A in this
build, so the receive completion through the threshold branch is never taken. The route that
reports a received packet to the stack is therefore not the one this branch implements; the
next place to look is the RAC/sequencer side of the same handler (the entries at 0x1B5E6 and
0x1B3EC above the threshold branch), not the flag.

Traced path after a delivered frame (instruction trace, `run --trace-pc-range`): with BUFC IRQ
32 raised for IF & IEN, the FRC handler runs after the delivery: the threshold branch (0x1B5F0,
which calls the classifier 0x1EFA8), the switch (status 6 -> 0x1B8E6, event bit 3), the common
point 0x1B6A0, then the gate at 0x1B6EA: flag A is 0, so the branch goes to 0x1B6F0 and then
0x1B75A, and the handler returns. The 802.15.4 callback region is not reached. The image has
no store that sets flag A: the only byte store to 0x200048D0 is a clear (0x1BD72), and no
base-plus-offset store sets it. FRC IEN never has bit 4 (RXDONE). So in this build the receive
completion cannot pass the gate as the firmware is linked.

`mcuemu run` (EFR32) accepts `--trace-pc-range START:END`, which prints each executed
instruction in the range with its cycle count.

RX configuration checked for the gate: `RAIL_ConfigRxOptions` (0x28AEC) is linked and called from
three init sites. It updates the RX-options word at handle +0xA0 and applies the option bits to
FRC, MODEM and PROTIMER registers (0x1E174 writes FRC CTRL-related words). It does not write flag
A (0x200048D0) and does not set FRC IEN bit 4. So the RX configuration is not what opens the gate.

Corrections from the driver's own code (`zephyr` EFR32 driver and the RAIL helpers):
the RX-event callback `sl_802154_rail_events_callback` is reached after a delivered frame, and
it calls `RAIL_GetRxPacketInfo(handle, 3, &info)` for the RX-packet event (bit 3). It copies the
packet (`sl_rail_copy_rx_packet`) only if that returns a handle and the info status is 7
(READY_SUCCESS; 6 is READY_CRC_ERROR). With the descriptor layout from the notes (8 bytes, word
0 = BUF1 end offset, byte 4 low bits = status, BUF1 payload) RAIL_GetRxPacketInfo still returns
0, so no packet is found. Its helper `sli_rail_052f8b...` (0x1F074) works on the ring context
(base +0x18, size +0x3A, write +0x3C, read +0x3E) and returns "no packet" when the index
difference is below a threshold. The exact ring-index semantics are the next thing to model;
guessed index updates produced asserts (classifier default branch) or the same "no packet".

Receive path, confirmed in the driver's code (2026-10-17): the event callback
`sl_802154_rail_events_callback` gets RX_PACKET_RECEIVED (mask bit 3, set by `RAIL_ConfigEvents`
with 0xFF20E00A/0x222), calls `RAIL_GetRxPacketInfo(handle, 3, &info)`, then
`RAIL_GetRxPacketDetailsAlt`, then `sl_rail_copy_rx_packet`. Each step needs its condition:
- Packet info: BUF2 STATUS count (0x54004088) must exceed the descriptor distance; the model
  writes 8 (one 8-byte descriptor) at delivery.
- Descriptor: word 0 = BUF1 end offset; byte +2 low three bits = status 7 (READY_SUCCESS).
- Details: `packetBytes = end - BUF1 read offset - 8` (statuses 6/7, trailing data); the first
  byte of the BUF1 data must equal packetBytes + 1 (the PHR). So the BUF1 frame is the PHR plus
  the PSDU, followed by eight appended bytes (zeros in the model).
- Copy: `sl_rail_get_rx_time_sync_word_end` must return 0, then `sl_rail_copy_rx_packet`.

Each delivered frame goes after the previous one: BUF1 WRITEOFFSET advances by the stored length
and the read offset is set to the frame start, so the descriptor's end offset is the frame start
plus the stored length. The earlier model wrote every frame at the start of BUF1, so a later
beacon overwrote the earlier one and its PHR check failed (the first byte was not packetBytes + 1).
With the frames placed separately, the PHR check passes for the coordinator's beacons in the sim,
and the router sends association requests (command 0x01, capability 0x8E, 17 in 30 s), each
acknowledged by the coordinator (ACK, sequence 0x80). The committed model before this change sent
none. The association response has not been seen in the capture yet, so the join is not confirmed.
The ring-index choices (BUF1 offsets, BUF2 status 8) are inferred from RAIL's checks.

After the copy (MAC side, traced with the coordinator in sim and with fake frames in run mode):
the packet reaches `zb_radio_l2_rx_poll` (0x6A54, the L2 queue drain) and the MAC receive task
(function at 0x13624, `mac_rxDataParse`), which parses the MAC header with `tl_zbMacHdrParse`
and drops the frame when the header is longer than the frame or when the MAC status is not
the one the frame type needs (for a beacon, type 0). The beacon-notify handler
(`tl_zbMacMlmeBeaconNotifyIndicationHandler`, 0xF70C) was not reached in the sim trace, and the
discovery scan confirm (0xF6DC) ran for the fake beacon. The sim PC-range counts for the MAC
task are inconsistent (the entry instruction counted once in 30 s while the task ran several
times), so they are not used as evidence. Next: confirm the MAC status at the time a real beacon
arrives (g_zbMacCtx.status at 0x20003D69) and the header-length check.

Beacon injected in run mode at 5.2, 5.5, 6.0 and 6.5 s (during the repeated scans): the
beacon-notify handler (0xF70C) never runs, and the MAC receive task (0x13624, `mac_rxDataParse`)
is not entered at all after the injection. So the injected frame does not reach the MAC task
in run mode, even though the same delivery path reaches the L2 queue. This contradicts the
sim observation (real frames reached the MAC task), so either the injected frame fails the
buffer conversion (`tl_phyRxBufTozbBuf`) or the task posting differs between the two modes.
Not resolved; the next check is the return value of `tl_phyRxBufTozbBuf` for a delivered frame.

Frame type decides whether RAIL reports the packet at all (run mode, one injected frame at 5.0 s
with the PHR convention and eight appended bytes): a data frame enters the RX callback
(`sl_802154_rail_events_callback`, 2 entries) and the copy, while a beacon (frame control 0x0000)
does not enter the callback, the packet-info helper or the copy. The 5.5 s and later beacon
injections also do not reach the MAC receive. The packet descriptor's third byte (filterMask,
from the helper's description) is the next candidate: it is zero in the model, and beacons
need the matching filter bit. Not verified yet.

Frame filter, checked (2026-10-17): the RAIL frame-type table (`sl_rail_frame_type_t`: is_valid,
address_filter_mask; type 0 = beacon in the lowest bit) is not set by the Zephyr driver, so RAIL's
defaults apply. The descriptor filterMask byte (+3) has no effect on the callback: with masks 00
to FF the injected beacon behaves the same. The callback is entered at the injection moment
(cycles 192001085-86) and returns at once for a beacon, while a data frame continues to
`RAIL_GetRxPacketInfo`. So the early return is decided by the event word or the handle masks,
not by the filter byte. Not resolved; the event word the handler builds for a beacon is the
next thing to compare with the data frame's.

Callback gate, first test (2026-10-17): `sl_802154_rail_events_callback` starts with
`lsls r1, r7, #10; bpl`, i.e. it exits when bit 21 of r7 is clear. Bit 21 is
`SL_RAIL_EVENT_IEEE802154_DATA_REQUEST_COMMAND` in `sl_rail_types.h`, so r7 is probably the
event word's high half or the mask used there, not the low event word (to confirm). The handler
sets 0x200000 at 0x1B8EC only when the handle word at +0xA0 has bits 6/7 (0xC0) set and the RAC
state is RXFRAME with two RAC registers clear (`0x1B802`-`0x1B824`). In the model the handle word
+0xA0 is 0x08 at the end of a run (the value the firmware's RX-options calls wrote), so bits 6/7
are clear and this branch is not taken. The firmware's own configuration therefore does not
enable the data-request path; how data frames still reach the copy is not explained yet.

Callback entry, register-level (`run --trace-pc-range 0x17758:0x17760 --trace-pc-regs`, 2026-10-17):
at entry r0 is the handle (0x20001D90), r2 is the event word low half (0x8, RX_PACKET_RECEIVED) and
r3 the high half (0). The callback copies r2 to r7 and tests bit 21 of r7 (`lsls r1, r7, #10`);
here it is clear, so both a data-type and a beacon-type injected frame leave at 0x17ADA at once.
The earlier observation that a data frame reached the copy came from a later callback entry with
another event word, not from this first entry. So the first-entry gate does not distinguish the
frame types; the difference, if any, must come from the event word built for the later entry.
Not resolved.

Receive after transmit (2026-10-17): the driver sets `tx_transitions.success = SL_RAIL_RF_STATE_RX`
(`ieee802154_silabs_efr32.c`, RAIL config), so a successful transmit leaves the radio receiving. The
model now sets RXENSRCEN bit 1 at TX_PACKET_SENT, which is the RX request the model tracks. The
coordinator's beacons (frame control 0x0080, 26 bytes, channel 15) are now received by the router
(10.4, 11.5, 16.9, 22.5, 28.1 s in a 30 s run). Before, they were missed because RX was off after
transmit. The stack still reports `commissioning pending`, so the beacon does not yet reach the
scan result or the join; the MAC handling of the received beacon is the next thing to check.

Beacon after RX is enabled (sim, 30 s, real coordinator, PC-range trace): the MAC receive task
(0x13624, `mac_rxDataParse`) runs once for the beacon and takes the path that calls
`tl_zbPhyIndication` (0x125E8 via the copy at 0x136A6). The trace of the following code is long
and repetitive (channel changes through `tl_zbMacChannelSet`/`rf_setChannel` and the scan
loop), so the beacon's route from the PHY indication to the scan result is not yet read. The
beacon-notify handler (0xF70C) and the MCPS data indication (0xD898) did not run in 30 s; the
scan-confirm handler (0xF6DC) ran 56 times. Next: follow `tl_zbPhyIndication` to its beacon
branch.

tl_zbPhyIndication (0x125E8, the function the beacon reaches): it dispatches on the frame type
(first byte of the frame, `& 7`); type 0 (beacon) first goes through a hook in RAM (0x20003958),
then, in an active scan (MAC status 2) adds the beacon to the scan list (`tl_zbMacActiveScanListAdd`).
In the sim with the coordinator the function runs once in 30 s, and the first byte it sees is 0x0C
(the PHR of our frame), so the type is 4 and the frame is dropped at the type check. The driver
strips the PHR before copying (`sl_802154_is_valid_pkt` moves the data pointer by one and
`sl_rail_copy_rx_packet` is a plain memcpy from that pointer), so the copy itself should start at
the frame control. The mismatch is between the copied data and the MAC's buffer; not resolved.

Findings (2026-10-17, sim with the coordinator):
- The PHY indication input is correct for a received frame: the copied data starts at the frame
  control (FC 0x0803, the router's own beacon request). So the PHR stripping and the copy work.
  That single event (2.37 s) is a frame sent by the coordinator (`coordinator.zb`, checked with the
  air source name), not the router's own transmission: the air already excludes a node's own frames.
  The coordinator's frame has the same bytes as a beacon request, so its origin is still open.
- The coordinator's beacons never reach `tl_zbPhyIndication` or `mac_rxDataParse` in 30 s, so they
  are dropped earlier, in the RAIL callback (the RX-packet event does not lead to the copy for them).
- The RX-option word at handle +0xA0 is changed by three RAIL_ConfigRxOptions calls: init (mask 8,
  value 8), `silabs_efr32_energy_scan_start` (mask 0x80, value 0x80) and
  `sl_802154_energy_scan_cleanup` (mask 0x80, value 0). So bits 6/7 are an energy-scan setting; the
  firmware never sets them for ordinary receive. The callback's first bit-21 gate therefore is not the
  beacon cause in the normal receive path; the cause is still open.

Beacon path in the callback, after the receive-after-transmit fix (sim, 30 s, real coordinator):
RX_PACKET_RECEIVED (0x8) reaches the callback at each coordinator beacon (10.4, 11.5, 16.9, 22.5,
28.1 s). From there: `RAIL_GetRxPacketInfo` runs 7 times, `RAIL_GetRxPacketDetailsAlt` 7 times, the
PHR check passes 2 times, the sync-word check once, and the copy once. The cause was that every frame
was written at the start of BUF1, so the second beacon's data pointer read the wrong bytes (see the
RX descriptor notes above). Advancing the write offset per frame, which the code here now does,
makes the PHR check pass for each beacon (checked in the sim by the compare at 0x17E0A: data byte
equals packetBytes + 1). Advancing the BUF2 ring index alone did not change the counts.

Remaining gap (written before the per-frame BUF1 offset fix; the pcap statement below is stale):
RAIL RX does not reach the stack. The firmware never reads BUFC READDATA
in 20 s, so no beacon or association response is delivered. RX start is through RAC
RXENSRCEN bit 1 (set through its SET alias), which has no name in the vendor header;
the RAC state machine that would report RX is not modelled either. `mcuemu sim` with the
Telink coordinator produces an empty pcap (24 bytes), so the router cannot join.
The TX path needs FRC FCD mode and BUFC buffer RAM (`+0xE0/+0xEC` and the BUF
`ADDR`/`CTRL.SIZE` fields); the RX path needs the same buffers filled from the air
with `RX_PACKET_RECEIVED`. Neither is modelled yet.

Current state (3 s of virtual time): with the live counter the busy-wait ends and
the core sleeps. The RTCC log shows the sleeptimer's CC1 is armed far in the future
(OCVALUE 0x80000000, the tickless idle value), so the kernel has no timeout pending:
the zigbee thread is waiting for a radio event. The transmit now starts without
error and waits for its completion. Forcing RAC/FRC event words does not wake the
core by itself. With a 100 us pulse on IRQ 34, 36, 37 or 38 (plus those words) the
core runs 57-60 M instructions in 3 s with no status-14 errors and no asserts, but
the console shows nothing after init: the stack still waits for the transmit
completion. The completion source is not identified yet. RAIL does not configure
any PROTIMER compare channel in mode 1 during the attempts (only capture channels
CC0/CC1, and CC3/CC4 are cleared), so it is not a PROTIMER compare.

Earlier analysis (superseded by the transmit gate above): the first transmit (at 16 ms, right after
init) already fails, and then every channel scan fails, so the busy state is set
during initialisation and is not lost later. RAC state codes (`RAC.STATUS[27:24]`),
decoded by `RAIL_GetRadioState` (`sli_rail_a4b57b…`): 0 is IDLE (1), unless
`0xB801C014` bits 1 or 2 are set (then 4); codes 1-5 and 10 give 2, 6 gives 3,
7, 9, 11 and 12 give 4, 8 gives 5. The stack waits for IDLE, and forcing codes 7-12
after init hangs the run in that wait. So the RAC state is not the refusal cause.
Next candidates: the 802.15.4 layer's own RAM flags (`0x200048CE`, `0x200048D6` are
written around the event dispatcher) and the `sl_802154` start-TX check.

Event word, decoded from `FRC_IRQHandler` (`r4`/`r5` are the low and high 32-bit
event words, `r8` the FRC flags):
- `r9 = RAC.IF & RAC.IEN` (`0xB8020000`, `+0x1C`/`+0x20`, as in `register_map.md`).
  Bit 18 (`0x40000`) of `r9` sets bit 9 of the high word, and `sli_rail_2c0c7b…`
  takes that bit as a success path.
- Bit 0 of the low word comes from `r8 = FRC.IF & FRC.IEN` (`0xB8004000`, `+0x68`/`+0x6C`).

Tested (temporary, 60 M steps): `RAC.IF` = `RAC.IEN` = `0x40000` with lines 37/38
pulsed cuts status-14 errors from 1926 to 32 and does not assert. The remaining 32
do not change with `FRC.IF` = `FRC.IEN` = `0x1` either, and the console still shows
no successful transmit. Single bits 0x1 and 0x10 of `RAC.IF` did nothing, and all
ones asserts (file 59, line 472). ## The Unicorn fork

M33 needed three fork changes on top of the NVIC port (see
[`nrf52840.md`](nrf52840.md)):

- `uc_arm_nvic(UC_ARM_NVIC_NO_SECURITY)` makes the stock Cortex-M33 model a core
  without the Security Extension (clears the feature, SAU regions and the secure
  state) and sets `NSACR` so FP stacking is governed by `CPACR` alone. The NVIC
  models a single security state, so the secure banks would otherwise be
  unreachable.
- `v7m_exception_taken` set `EXC_RETURN.ES` whenever the Security Extension was
  absent, and the exception return of the same core then treated it as an invalid
  return (`UFSR.INVPC` escalating to HardFault on the first IRQ). ES is now set
  only for secure targets.
- `Core::reset(vector_table)` loads SP/PC from the application slot and sets
  `VTOR` there.
- A secure-built image (Zephyr with `CONFIG_ARM_SECURE_FIRMWARE`) stores
  `EXC_RETURN = 0xFFFFFFFD` (S and ES set) in its threads. Without the Security
  Extension the core has one state, so `do_v7m_exception_exit` clears S and ES
  on ARMv8-M instead of taking `UFSR.INVPC` on the first thread switch. ARMv7-M
  is unchanged (there the same bits are part of the return-code checks).
  Untested with a non-secure-built image that does not need it.

### Zephyr router (`router2.bin`) join status

Sim with the Telink coordinator (30 s, `router-uart.yaml`):

- The router's address is the unique id (DEVINFO `+0x48`, read by `SYSTEM_GetUnique`), set
  from the device id since commit `9956e69`. The EUI-64 at `+0x1A0` is a separate value.
- The router's association request is ACKed, polled with a data request (ACK with frame
  pending), and the coordinator's association response (short address `0xFCA5`) is ACKed by
  the router (ACK sequence `0x90`). This is the same exchange as the working sensor image in
  the CI scenario.
- Cause of the missing ACKs: RAIL reports `isAck` in the packet details at byte 13, which is
  set from bit 4 of the descriptor's byte 2 when the status is 6 or 7 (`sli_rail_e331ba91`,
  `0x1D9C8`). The driver handles a short frame only when that flag is set (`0x17E6E`). The
  model now sets it for ACK frames, and it sends the ACK itself when a frame requests one and
  is addressed to the unique id (the hardware does this in RAIL's auto-ACK).
- Open: the driver keeps `ack_tx` set after a received frame that requests an ACK, until
  RAIL reports `TXACK_PACKET_SENT` (SDK bit 25, `0x2000000`). The FRC handler produces it
  from FRC IF bit 1 (TXAFTERFRAMEDONE, enabled in the image) when RAC SR0 bits 13..15 are
  clear; the model now sets that bit when it sends an ACK. Verified in the callback: the
  event word `0x2000008` (RX_PACKET plus TXACK) arrives with the association response.
- Short-address frames: the driver takes the router's short address from RAIL's address
  configuration (`sl_rail_ieee802154_set_short_address`). The value does not reach the RAIL
  register window (no write of `0xFCA5`/`0xA5FC` through RailRegs), so it sits in RAM inside
  RAIL's handle, and the model does not describe that structure. The model instead takes the
  PAN and short address from the association response it acknowledges (an 802.15.4 device
  does the same), and ACKs unicast frames to them that request an ACK. With this, the
  blocked-TX warnings stop (25 in 30 s before, 0 after).
- After the association the router broadcasts NWK Link Status frames (command 0x08 to
  `0xFFFD`, `08 02 fd ff ...`), which a router sends once it is on a network. The coordinator
  answers with NWK frames to `0xFCA5`. The router's console shows no join message (the
  commissioning retries stop after 1.5 s; see below), so the join is judged from the air only.
- The console stop was the RTCC tick count, not the router. `Rtcc::freeze()` moved the time
  base to `now` at every freeze, and the core freezes the counter every few thousand cycles, so
  the fraction of a tick in progress was lost each time. The counter ran about 40% slow, the
  kernel's timeouts fired late, and the log and the Zigbee timers stopped. The base is now
  anchored at the cycle where the counter reached its value. With that, the router prints
  `joined network (bdb status: 0x00)` at 7.8 s of virtual time (commit f7a6705).

## Tests

- `efr32emu-tests`: the M33 exception path (the nRF52 `irq` program, with the
  FPU context), the application-slot boot, CMU/HFXO, GPIO and EXTI, MSC, and the
  radio (frame exchange, state, RSSI).
- `simcore-tests` (`tests/sim/test_efr32.cpp`): pads on nets, config rules.
- `firmware_efr32mg21_zigbee_join` (image `efr32mg21-sensor.bin` and
  `samplegw_tb03f.bin` in `firmware/`): the two-chip join, TCLK exchange and
  commit, judged from the sensor's console (`board.mcu.uart0`).

## Not done

- A radio that matches the silicon (RAIL), BLE.
- EMU/EM2 sleep (clock gating, retention), LDMA, TIMER compare/capture,
  LETIMER/BURTC, I2C slave mode, IADC, CRYPTO/SE (random numbers) and the other
  stand-ins above. RTCC counts and compares, TIMER counts, USART transmits, but
  none is an interrupt-accurate model of the silicon.
- Zephyr images with the RAIL radio driver (`IEEE802154_SILABS_EFR32`, board `slwrb4180a`,
  `samples/zigbee/zigbee_shell` as router and as end device, no console) associate with the
  Telink sampleGW coordinator at about 7.7 s of virtual time (checked on the air, with the
  sim scenario of the zigbee-rs sensor and `telink-gw.bin`, 100 s). The RAIL RX context is
  found by the buffer addresses it holds (`Efr32mg21::rx_context`), because its RAM address
  differs per image; the first build read it from a fixed global and never saw a beacon. The
  ED resets itself about 10 s after the join and restores its short address from storage,
  so the address learned from the association response is kept across the chip reset.
  Open: after the join the router sends Link Status only at 8 s and 22 s in a 100 s run
  (the coordinator's every 15 s), the cause is not found.
- Other family members (MG1 is Series 1 and a different chip, MG24 is a larger
  Series 2 part): `Efr32mg21` is the only class.
- TrustZone: the model has one security state.
