# STM32WB55

`stm32wb55` models the STM32WB55's two application cores in `sim`: CPU1 uses
the shared Cortex-M4 core at 64 MHz, and CPU2 uses a separate Cortex-M0 core at
32 MHz. The two cores have independent CPU state and cycle clocks, and map the
same flash and SRAM storage. CPU1 starts CPU2 by setting C2BOOT. CPU2's vector
table is found in its configured raw image, or in the built-in model ROM when
the modeled Thread RCP is selected.
While CPU2 is running, CPU1 yields at one-microsecond service intervals so the
two instruction streams observe shared SRAM and IPCC changes on a common virtual
timeline. IPCC changes request an earlier service.

## CPU2 image

Provide the CPU2 image separately from the Zephyr CPU1 image:

```yaml
devices:
  - id: wb55
    mcus:
      - id: main
        model: stm32wb55
        firmware: ../firmware/stm32wb55-cpu1.bin
        cpu2_firmware:
          - file: ../firmware/stm32wb55-cpu2.bin
            offset: 0xC0000 # use the CPU2 image's linked flash address
```

`cpu2_firmware` accepts the same `file`, `offset`, `from`, and `size` fields as
`firmware`. The configured bytes are mapped into the chip's shared flash, and
CPU2 begins executing when CPU1 enables C2BOOT. Choose an offset that matches
the binary's link address. The emulator expects executable bytes with a Cortex-M
vector table and rejects images without one. It does not unpack encrypted STM32
wireless-firmware packages distributed for the Firmware Upgrade Service (FUS);
those packages do not contain directly executable CPU2 bytes. ST documents the
wireless-stack images as encrypted and signed for FUS in
[AN5185](https://www.st.com/resource/en/application_note/dm00513965-st-firmware-upgrade-services-for-stm32wb-series-stmicroelectronics.pdf).

CPU2 runs on a separate Cortex-M0 core profile at 32 MHz, with independent CPU
state and a cycle clock. It shares flash and SRAM with CPU1 and accesses the
same IPCC status and masks. While both cores are active, the simulator
schedules CPU2 at one-microsecond intervals against the same virtual timeline;
IPCC updates can request an earlier service. Different raw CPU2 firmware images
can be loaded per simulation and execute as M0 instructions. A synthetic M4/M0
pair has been checked for C2BOOT startup and shared-SRAM communication. The
chip model also provides basic RCC, PWR, HSEM, RNG, and LPTIM register behavior.
Unimplemented peripheral registers read as zero or retain writes.

When `cpu2_firmware` is absent, CPU1 starts a built-in CPU2 model:
`cpu2_model: thread_rcp` (the default, described here) or
`cpu2_model: st_mac_802154` (see "ST IEEE 802.15.4 MAC model" below).
The default is the built-in Thread RCP model. Its
IPCC and Spinel behavior is serviced in CPU2's 32 MHz time domain while the M0
core runs a wait loop; this mode lets the Zephyr CPU1 image join the example
network and exchange Zigbee data. When a raw CPU2 image is configured, the
built-in RCP model is disabled and that image executes on M0 instead. The
official ST Thread RCP package was checked, but it is FUS-encrypted and is
rejected by the raw-image loader. The STM32WB wireless PHY and radio/baseband
peripherals are not modeled, so an arbitrary wireless stack running from a raw
CPU2 image cannot perform on-air operations without a matching radio model.

The Zephyr source used for the join run also needed a null-index guard in
`subsys/zigbee/nwk/nwk_addr_map.c`: the end-device leave path calls
`zb_ExtAddrByShortAddr` with a null `idx`, while that function previously wrote
through it unconditionally. With that guard, the Zephyr CPU1 image joined the
TLSR8258 sample gateway at about 7.8 seconds and exchanged bidirectional NWK
traffic in the 24-second capture through the built-in CPU2 Thread RCP model. The example is
[`../examples/stm32wb55_zigbee.yaml`](../examples/stm32wb55_zigbee.yaml); it
needs the Zephyr CPU1 image and TLSR8258 coordinator image. Add
`cpu2_firmware` only when testing an independently executable raw M0 image.

## ST IEEE 802.15.4 MAC model (`cpu2_model: st_mac_802154`)

ST ships an IEEE 802.15.4 MAC as a CPU2 wireless stack
(`stm32wb5x_Mac_802_15_4_fw.bin`, used with FUS 1.2.0; zigbee-rs validated
MAC 1.24.0). That image is FUS-encrypted, so the emulator cannot run it.
`cpu2_model: st_mac_802154` replaces it with a behavioural model in
`emulator/src/stm32wb55/st_mac.cpp` (`stm32wb55emu::StMac`), serviced in CPU2's
32 MHz time domain while the M0 core idles, like the Thread RCP model. It is
written from the interface, not from ST code:

- mailbox layout and handshake: AN5289 and ST's `tl_mbox.c`, `mbox_def.h`,
  `shci.h` (STMicroelectronics/stm32-mw-wpan, commit `1a66948`, the
  STM32_WPAN middleware of STM32CubeWB);
- MAC SAP: `mac_802_15_4/core/inc/802_15_4_mac_core.h` (message identifiers
  `MAC_802_15_4_MsgIdM4ToM0_t` / `MsgIdM0ToM4_t`, `MAC_802_15_4_CMD_OPCODE_OFFSET`
  0x280), `802_15_4_mac_types.h` (request, confirm and indication structures),
  `802_15_4_mac_errors.h` (status codes) and `mac_802_15_4_core_wb.c`
  (opcode `(0x3F << 9) | (0x280 + id)`, synchronous status in `payload[0]`);
- what a real CPU1 driver relies on: zigbee-rs `zigbee-mac/src/stm32wb/`,
  `stm32wb-hal/src/mac.rs` and `stm32wb-hal/src/st_abi.rs` (which asserts every
  structure size and offset against the ST header at compile time).

### Mailbox

| Step | CPU1 | Model |
| --- | --- | --- |
| Boot | `TL_Init`, `shci_init`, `TL_MM_Init`, `TL_MAC_802_15_4_Init`, C2BOOT | 5 ms after C2BOOT (a model choice) fills the device info table (FUS 1.2.0, wireless stack 1.24.0, `INFO_STACK_TYPE_MAC` 0x40) and queues the SHCI ready event (sub-event 0x9200, `WirelessFwRunning`) in the system spare buffer on channel 2; retried every ms until CPU1's tables exist |
| System command | `SHCI_C2_MAC_802_15_4_Init` (or any SHCI command) on channel 2 | Command Complete, status SUCCESS, in the command buffer |
| MAC request | `TL_CmdPacket_t` in `p_cmdrsp_buffer`, channel 3 set | decodes the request, writes the synchronous status (`TL_EvtPacket_t` payload[0]) and frees channel 3 |
| Confirm / indication | | `TL_EvtPacket_t` in `p_notack_buffer` (payload: 16-bit message id + C structure), channel 3 set toward CPU1; data the structure points to (MSDU, beacon payload, GET value) is placed in CPU2 SRAM2b at 0x2003FC00 |
| Acknowledge | `TL_MAC_802_15_4_SendAck`: CPU1 clears channel 3 | the next notification is delivered (one outstanding at a time, as on CPU2) |
| Buffer release | memory manager on channel 4 | channel freed |

### Primitives

Modeled requests: MLME-RESET, -SET, -GET (all PIB attributes zigbee-rs uses,
plus the IEEE 802.15.4-2006 MAC PIB; macExtendedAddress is ST's 0x6F),
-SCAN (ED, active, passive), -START (non-beacon, BO = 15), -ASSOCIATE,
-ASSOCIATE.response, -DISASSOCIATE (direct and indirect), -ORPHAN.response,
-POLL, MCPS-DATA (direct and indirect) and MCPS-PURGE. Confirms and indications:
RESET, SET, GET, SCAN, START, ASSOCIATE, DISASSOCIATE, POLL, DATA, PURGE confirms;
ASSOCIATE, DISASSOCIATE, BEACON-NOTIFY, COMM-STATUS, ORPHAN, POLL (ST extension)
and MCPS-DATA indications. GTS, RX-ENABLE, SYNC, DPS, sounding and calibration
return `MAC_NOT_IMPLEMENTED_STATUS` (0x02, `802_15_4_mac_types.h`) synchronously.

On the shared `air` the model behaves as a non-beacon IEEE 802.15.4-2006 MAC:
unslotted CSMA-CA (macMinBE/macMaxBE/macMaxCSMABackoffs, CCA from frames in
flight), Imm-Ack after aTurnaroundTime with Frame Pending set when an indirect
transaction waits for the polling device, macAckWaitDuration and
macMaxFrameRetries (NO_ACK), third-level address filtering, a beacon in answer to
a Beacon Request after MLME-START (content from macBeaconPayload and
macAssociationPermit), indirect transactions kept for
macTransactionPersistenceTime, the association handshake as a device
(macResponseWaitTime, then a Data Request) and as a coordinator (COMM-STATUS when
the response is ACKed), and RSSI/LQI from the medium. Confirms and indications
reach CPU1 40 us after the event that caused them (a model choice).
`MCUEMU_STM32WB_TRACE=1` prints every primitive with its CPU2 time.

Not modeled: FUS (no FUS commands, no firmware upgrade, no FUS info table), the
real stack's code and timing, MAC security (requests with a security level get
`UNSUPPORTED_SECURITY`; Zigbee secures at NWK/APS), beacon-enabled PANs, GTS,
orphan scan, RX-ENABLE, frame version 2 and IEs, promiscuous-mode sniffing beyond
data frames, and ED scan levels (a channel reads 0xFF when busy, 0 otherwise). As
zigbee-rs reports for MAC 1.24.0, the active-scan confirm is NO_BEACON with an
empty PAN descriptor list when macAutoRequest is FALSE (beacons arrive as
BEACON-NOTIFY); MLME-POLL.indication is reported for short-address pollers only.

### Running the zigbee-rs router

The zigbee-rs Nucleo-WB55 router (`examples/nucleo-wb55-router` of
github.com/faronov/zigbee-rs; the dongle and STM32WB5MM-DK images share the same
`apps/stm32wb-router` composition) runs unmodified. Build it with the
`thumbv7em-none-eabihf` target and convert it to a raw image:

```sh
cd zigbee-rs/examples/nucleo-wb55-router
cargo build --release --locked
OBJCOPY=$(find "$(rustc --print sysroot)" -name llvm-objcopy -print -quit)
"$OBJCOPY" -O binary target/thumbv7em-none-eabihf/release/nucleo-wb55-router \
    path/to/mcuemu/firmware/stm32wb55-zigbee-rs-router.bin
```

Then, with `firmware/samplegw_tb03f.bin` in place:

```sh
mkdir -p run && cd run
../build/mcuemu sim ../examples/stm32wb55_zigbee_rs_router.yaml --for 30s
```

The router log (USART1, the ST-LINK VCP on the board) is printed by the
example's `text` sink and the air goes to `stm32wb55-zigbee-rs-air.pcap`. The
CTest `firmware_stm32wb55_zigbee_rs_router_join` runs the same system when both
images are present and judges it from the pcap.

To get that image running, the chip model also gained what its Embassy HAL
touches: TIM2 as the embassy time driver (32-bit, PSC/ARR/CCR/DIER/SR/EGR
behaviour of RM0434), DMA1 memory-to-USART1 transfers (completed at once), the
flash controller's KEYR/CR page erase and FLASH_SFR (CPU2 secure area from page
0xE1, matching the products' memory map), UID64 at 0x1FFF7580 (from `eui64`),
PLLRDY following PLLON, the HPREF/PPREx and C2HPREF/SHDHPREF flags, and GPIO
inputs reading their pull resistors (the board's active-low buttons read
released).

Observed with the image built from zigbee-rs `gp-m5` (`f169c61`): the first
scan at 0.04 s finds nothing (the gateway has not formed its network yet), the
second commissioning attempt sees the gateway's beacon on channel 15, associates
(Association Response status 0, for example short address 0xF374 at 7.93 s),
receives the network key in a Transport-Key, starts as a router (MLME-START),
sends Device_annce and unicast NWK frames to the coordinator and receives NWK
unicasts and broadcasts from it. The coordinator then sends its unicasts with a
NWK source-route subframe of relay count 0, which zigbee-rs' NWK frame parser
(`zigbee-nwk/src/frames.rs`) rejects, so the router never sees the
Node_Desc_rsp, gives up after three tries at about 23 s, leaves and rejoins. That
is a zigbee-rs stack behaviour, not a MAC-model one: the MAC model delivered
each of those frames as MCPS-DATA.indication.
