// EFR32MG21: the Cortex-M33 core, the application-slot boot, CMU/HFXO, GPIO,
// MSC and the radio blocks.

#include "../test_support.hpp"
#include "../nrf52/programs.hpp"
#include "cortexm/core.hpp"
#include "efr32emu/efr32mg21.hpp"
#include "mcuemu/radio.hpp"

#include <cstring>

namespace mcuemu_tests {

namespace {

constexpr std::uint32_t Cmu = 0x40008000, Hfxo = 0x4000C000, Msc = 0x40030000, Gpio = 0x4003C000;
constexpr std::uint32_t Set = 0x1000, Clear = 0x2000, Toggle = 0x3000;
constexpr std::uint32_t Radio = 0x40090000;
constexpr std::uint64_t Us = 38; // cycles per microsecond at 38.4 MHz (rounded down)

struct Chip {
    efr32emu::Efr32mg21 chip;

    explicit Chip(const std::vector<std::uint8_t> &image = nrf52_programs::idle) {
        chip.load_firmware(image);
        chip.reset();
    }
    void w(std::uint32_t address, std::uint32_t value) { chip.write(address, value, 4); }
    std::uint32_t r(std::uint32_t address) { return chip.read(address, 4); }
};

} // namespace

// The nRF52 test programs (Thumb-2, FPU) on an M33 without the Security
// Extension: SVC, nested interrupts and the exception return with ES clear.
void test_efr32_m33_exceptions() {
    cortexm::Core::Config config;
    config.model = cortexm::Model::M33;
    config.priority_bits = 4;
    cortexm::Core core(config);
    std::uint8_t *flash = core.map_memory(0, 0x10000, false);
    core.map_memory(0x20000000, 0x10000, true);
    std::memcpy(flash, nrf52_programs::irq.data(), nrf52_programs::irq.size());
    core.reset();
    CHECK(core.run(2000).reason == cortexm::StopReason::Deadline);
    std::uint32_t value = 0;
    core.read_memory(0x2000010c, &value, 4);
    CHECK(value == 4); // SVC handler ran
    core.set_irq_line(3, true);
    core.set_irq_line(3, false);
    CHECK(core.run(2400).reason == cortexm::StopReason::Deadline);
    core.read_memory(0x20000108, &value, 4);
    CHECK(value == 3); // the more urgent nested handler ran
    core.read_memory(0x20000104, &value, 4);
    CHECK(value == 2);
    CHECK(core.reg(4) == 0x44);
    CHECK(core.reg(13) == 0x20001000); // stack balanced: every exception returned cleanly
}

// An application-only image (vector table in the 0x4000 slot) is loaded there
// and started directly, with VTOR pointing at it.
void test_efr32_application_slot_boot() {
    std::vector<std::uint8_t> image(0x20, 0xff);
    const std::uint32_t sp = 0x20010000, reset = 0x4009; // Thumb bit set
    std::memcpy(image.data(), &sp, 4);
    std::memcpy(image.data() + 4, &reset, 4);
    image[8] = 0xfe;
    image[9] = 0xe7; // b .
    Chip c(image);
    CHECK(c.chip.flash()[0x4000 + 8] == 0xfe);
    CHECK(c.chip.flash()[0] == 0xff);
    c.chip.run(100);
    CHECK(c.chip.core().pc() == 0x4008);
    CHECK(c.chip.core().reg(13) == 0x20010000);
    std::uint32_t vtor = 0;
    c.chip.core().read_memory(0xE000ED08, &vtor, 4);
    CHECK(vtor == 0x4000);
}

// The sequence of the zigbee-rs clock driver: FSRCO -> HFXO crystal start ->
// first-lock -> HFXO as system clock, using the set/clear aliases.
void test_efr32_clock_and_hfxo() {
    Chip c;
    CHECK((c.r(Cmu + 0x070) & 7) == 1);                 // FSRCO out of reset
    CHECK((c.r(Hfxo + 0x058) & 1) == 0);                // HFXO off
    c.w(Hfxo + Set + 0x028, 2);                         // DISONDEMAND
    c.w(Hfxo + Clear + 0x028, 1);
    CHECK((c.r(Hfxo + 0x058) & (1u << 16)) == 0);       // ENS clear
    c.w(Hfxo + 0x028, (c.r(Hfxo + 0x028) & ~3u) | 1);   // FORCEEN, on demand
    const std::uint32_t ready = 1u | 2u | (1u << 16) | (1u << 30);
    CHECK((c.r(Hfxo + 0x058) & ready) == ready);        // RDY, COREBIASOPTRDY, ENS, FSMLOCK
    c.w(Hfxo + Set + 0x028, 2);
    CHECK((c.r(Hfxo + 0x058) & (1u << 30)) == 0);       // first lock done
    c.w(Cmu + 0x070, 3);                                // HFXO selected
    CHECK((c.r(Cmu + 0x070) & 7) == 3);
    c.w(Hfxo + Clear + 0x028, 3);                       // FORCEEN and DISONDEMAND off
    CHECK((c.r(Hfxo + 0x058) & 0x10001) == 0x10001);    // on demand for the system clock
}

void test_efr32_gpio_aliases_and_exti() {
    Chip c;
    // PB0 push-pull, driven through the set/clear aliases of DOUT.
    c.w(Gpio + 0x34, 4);
    c.w(Gpio + Set + 0x40, 1);
    CHECK(c.r(Gpio + 0x40) == 1);
    CHECK(c.chip.pad_drive(16) == efr32emu::PadDrive::High);
    CHECK((c.r(Gpio + 0x44) & 1) == 1);                 // DIN follows the output
    c.w(Gpio + Clear + 0x40, 1);
    CHECK(c.chip.pad_drive(16) == efr32emu::PadDrive::Low);
    c.w(Gpio + Toggle + 0x40, 1);
    CHECK(c.chip.pad_drive(16) == efr32emu::PadDrive::High);

    // PD2 input with pull-up, EXTI2 on the falling edge: GPIO_EVEN (IRQ 26).
    c.w(Gpio + 0x94, 2u << 8);                          // mode 2 (input with pull)
    c.w(Gpio + Set + 0xa0, 4);                          // DOUT = 1: pull-up
    CHECK(c.chip.pad_drive(3 * 16 + 2) == efr32emu::PadDrive::PullUp);
    c.w(Gpio + 0x400, 3u << 8);                         // EXTIPSEL2 = port D
    c.w(Gpio + 0x408, 2u << 8);                         // EXTIPINSEL2 = pin 2
    c.w(Gpio + 0x414, 4);                               // EXTIFALL
    c.w(Gpio + Set + 0x424, 4);                         // IEN
    std::uint32_t enable = 1u << 26;
    c.chip.core().write_memory(0xE000E100, &enable, 4); // NVIC: enable IRQ 26
    c.chip.set_pad_input(3 * 16 + 2, true);
    CHECK(c.r(Gpio + 0x420) == 0);
    CHECK(!c.chip.core().interrupt_pending());
    c.chip.set_pad_input(3 * 16 + 2, false);            // button pressed
    CHECK(c.r(Gpio + 0x420) == 4);
    CHECK(c.chip.core().interrupt_pending());
    c.w(Gpio + Clear + 0x420, 4);
    CHECK(c.r(Gpio + 0x420) == 0);
    // The NVIC latched the edge: it stays pending until taken or cleared.
    CHECK(c.chip.core().interrupt_pending());
    std::uint32_t clear_pending = 1u << 26;
    c.chip.core().write_memory(0xE000E280, &clear_pending, 4);
    CHECK(!c.chip.core().interrupt_pending());
    // Rising edges are not selected.
    c.chip.set_pad_input(3 * 16 + 2, true);
    CHECK(c.r(Gpio + 0x420) == 0);

    unsigned pad = 0;
    CHECK(efr32emu::Efr32mg21::parse_pad("PD2", pad) && pad == 50);
    CHECK(efr32emu::Efr32mg21::parse_pad("PB0", pad) && pad == 16);
    CHECK(!efr32emu::Efr32mg21::parse_pad("PE0", pad) && !efr32emu::Efr32mg21::parse_pad("PA16", pad));
    CHECK(efr32emu::Efr32mg21::pad_name(50) == "PD2");
}

void test_efr32_msc_erase_and_program() {
    Chip c;
    const std::uint32_t page = 0x7C000;
    CHECK(c.chip.flash()[page] == 0xff);
    // Locked: nothing happens.
    c.w(Msc + 0x00c, 1);
    c.w(Msc + 0x014, page);
    c.w(Msc + 0x018, 0x12345678);
    CHECK(c.chip.flash()[page] == 0xff);
    CHECK((c.r(Msc + 0x01c) & (1u << 16)) != 0); // REGLOCK

    c.w(Msc + 0x03c, 0x1b71);
    CHECK((c.r(Msc + 0x01c) & (1u << 16)) == 0);
    c.w(Msc + 0x00c, 1);                          // WREN
    c.w(Msc + 0x014, page);
    CHECK((c.r(Msc + 0x01c) & 8) != 0);           // WDATAREADY
    c.w(Msc + 0x018, 0x12345678);
    c.w(Msc + 0x018, 0xf0f0f0f0);
    c.w(Msc + 0x010, 4);                          // WRITEEND
    std::uint32_t word = 0;
    std::memcpy(&word, c.chip.flash() + page, 4);
    CHECK(word == 0x12345678);
    std::memcpy(&word, c.chip.flash() + page + 4, 4);
    CHECK(word == 0xf0f0f0f0);
    // Programming only clears bits.
    c.w(Msc + 0x014, page);
    c.w(Msc + 0x018, 0x0f0f0f0f);
    std::memcpy(&word, c.chip.flash() + page, 4);
    CHECK(word == (0x12345678u & 0x0f0f0f0fu));
    // Erase restores the whole 8 KiB page.
    c.w(Msc + 0x014, page + 0x100);
    c.w(Msc + 0x010, 2);
    CHECK(c.chip.flash()[page] == 0xff && c.chip.flash()[page + 4] == 0xff);
    // An address beyond the flash is invalid.
    c.w(Msc + 0x014, 0x100000);
    CHECK((c.r(Msc + 0x01c) & 4) != 0);
    c.w(Msc + 0x03c, 0);
    CHECK((c.r(Msc + 0x01c) & (1u << 16)) != 0);
}

namespace {

void run_both(Chip &a, Chip &b, std::uint64_t until_us) {
    for (std::uint64_t t = 5; t <= until_us; t += 5) {
        a.chip.run(t * Us);
        b.chip.run(t * Us);
    }
}

} // namespace

void test_efr32_radio_frame_exchange() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("b", {});
    Chip a, b;
    a.chip.connect_radio(air, "a");
    b.chip.connect_radio(air, "b");
    for (Chip *chip : {&a, &b}) {
        chip->w(Radio + 0x200c, 4);           // SYNTH CHNO: channel 15
        chip->w(Radio + 0x301c, 3);           // RAC IEN: TX done and RX done
    }
    std::uint32_t enable = 1u << (36 - 32);
    for (Chip *chip : {&a, &b})
        chip->chip.core().write_memory(0xE000E104, &enable, 4); // ISER1: IRQ 36

    b.w(Radio + 0x1004, 2);                   // BUFC: clear the RX FIFO
    b.w(Radio + 0x3004, 2);                   // RXEN
    CHECK((b.r(Radio + 0x3008) & 0xf) == 2);

    const std::uint8_t mac[5] = {0x41, 0x88, 0x07, 0xaa, 0xbb};
    a.w(Radio + 0x1004, 1);
    a.w(Radio + 0x1040, 7);                   // PHR: 5 bytes + the 2-byte FCS
    for (std::uint8_t byte : mac)
        a.w(Radio + 0x1040, byte);
    a.w(Radio + 0x3004, 1);                   // TXEN
    CHECK((a.r(Radio + 0x3008) & 0xf) == 3);

    run_both(a, b, 1500);
    CHECK((a.r(Radio + 0x3010) & 1) == 1);    // TX done
    CHECK((a.r(Radio + 0x3008) & 0xf) == 1);  // idle again
    CHECK((b.r(Radio + 0x3010) & 2) == 2);    // RX done
    CHECK((b.r(Radio + 0x0030) & 0x10) != 0); // CRC ok
    CHECK(b.chip.core().interrupt_pending()); // FRC_PRI
    CHECK(b.r(Radio + 0x1084) == 5);          // frame length
    std::uint8_t got[5];
    for (std::uint8_t &byte : got)
        byte = static_cast<std::uint8_t>(b.r(Radio + 0x1080));
    CHECK(std::memcmp(got, mac, 5) == 0);
    b.w(Radio + 0x3018, 2);                   // IFC
    b.w(Radio + 0x0038, 0xffffffff);
    b.chip.core().write_memory(0xE000E284, &enable, 4); // ICPR1: the edge was latched
    CHECK(!b.chip.core().interrupt_pending());
    CHECK(air.frames().size() == 1 && air.frames()[0].channel == 15 && air.frames()[0].payload.size() == 5);
}

void test_efr32_radio_state_and_rssi() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("b", {});
    Chip a, b;
    a.chip.connect_radio(air, "a");
    b.chip.connect_radio(air, "b");
    CHECK((a.r(Radio + 0x3008) & 0xf) == 1);  // idle out of reset
    a.w(Radio + 0x3004, 2);
    CHECK((a.r(Radio + 0x3008) & 0xf) == 2);
    a.w(Radio + 0x3004, 8);                   // RXDIS
    CHECK((a.r(Radio + 0x3008) & 0xf) == 1);
    // A quiet channel reads far below the CCA threshold (-60 dBm).
    CHECK(static_cast<std::int8_t>(a.r(Radio + 0x5020) & 0xff) < -100);
    // A long frame on air makes the energy rise on the same channel.
    b.w(Radio + 0x1040, 100);
    for (int i = 0; i != 98; ++i)
        b.w(Radio + 0x1040, 0x55);
    b.w(Radio + 0x3004, 1);
    run_both(a, b, 500);
    CHECK(static_cast<std::int8_t>(a.r(Radio + 0x5020) & 0xff) > -60);
}

// The RTCC counts at 32768 Hz (inferred clock), reads the same through the
// secure and non-secure bases, and sets IF.CC1 at OCVALUE + 1 (the sleeptimer's
// convention) with the interrupt taken on the RTCC line.
void test_efr32_rtcc_counts_and_compares() {
    Chip c;
    c.w(0x58000004, 1);      // EN (non-secure alias)
    c.w(0x4800000c, 1);      // CMD.START
    c.chip.core().advance_to(19200000); // 0.5 s at 38.4 MHz
    CHECK(c.r(0x58000020) == 16384);
    CHECK(c.r(0x48000020) == 16384);
    c.w(0x48000020, 100);    // CNT = 100
    c.w(0x4800003c, 2);      // CC1.CTRL = OUTPUTCOMPARE
    c.w(0x48000040, 110);    // CC1.OCVALUE: match at tick 111
    c.w(0x48000018, 8);      // IEN.CC1
    const std::uint64_t start = c.chip.core().cycles();
    c.chip.run(start + 5000); // about 4 ticks
    CHECK((c.r(0x48000014) & 8) == 0);
    c.chip.run(start + 20000); // about 17 ticks
    CHECK((c.r(0x48000014) & 8) != 0);
    c.w(0x48002014, 8);      // IF_CLR: the clear alias (+0x2000) of IF
    CHECK((c.r(0x48000014) & 8) == 0);
}

// STATUS.RUNNING follows START/STOP, compare channel 0 sets IF.CC0 (bit 2) like CC1, and the
// counter wrapping to 0 sets IF.OF (the zigbee-rs HAL's time driver uses all three).
void test_efr32_rtcc_cc0_running_and_overflow() {
    Chip c;
    CHECK((c.r(0x48000010) & 1) == 0);
    c.w(0x58000004, 1);      // EN
    c.w(0x5800000c, 1);      // CMD.START
    CHECK((c.r(0x48000010) & 1) == 1);
    c.w(0x48000030, 2);      // CC0.CTRL = OUTPUTCOMPARE
    c.w(0x48000034, 9);      // CC0.OCVALUE: match at tick 10
    c.w(0x48000018, 4);      // IEN.CC0
    const std::uint64_t start = c.chip.core().cycles();
    c.chip.run(start + 5000); // about 4 ticks
    CHECK((c.r(0x48000014) & 4) == 0);
    c.chip.run(start + 20000); // about 17 ticks
    CHECK((c.r(0x48000014) & 4) != 0);
    CHECK((c.r(0x48000014) & 1) == 0);
    c.w(0x48000020, 0xfffffff0u); // 16 ticks before the wrap
    c.chip.run(c.chip.core().cycles() + 25000); // about 21 ticks
    CHECK((c.r(0x48000014) & 1) != 0);
    CHECK(c.r(0x48000020) < 16);
    c.w(0x4800000c, 2);      // CMD.STOP
    CHECK((c.r(0x48000010) & 1) == 0);
}

// TIMER0 counts core cycles divided by CFG.PRESC + 1 while started (the zigbee-rs HAL's
// busy-wait clock). TOP resets to 0xFFFF on every timer; only TIMER0 is 32 bits wide.
void test_efr32_timer_counts() {
    Chip c;
    CHECK(c.r(0x4004801c) == 0xffff);   // TIMER0.TOP
    CHECK(c.r(0x4004c01c) == 0xffff);   // TIMER1.TOP
    c.w(0x4004800c, 1);      // START without EN: stays stopped
    CHECK((c.r(0x40048010) & 1) == 0);
    c.w(0x40048030, 1);      // EN
    c.w(0x4004801c, 0xffffffffu); // TOP: the full 32-bit range, as the HAL sets it
    CHECK(c.r(0x4004801c) == 0xffffffffu);
    c.w(0x40048024, 0xfffff000u);
    c.w(0x4004800c, 1);      // CMD.START
    CHECK((c.r(0x40048010) & 1) == 1);
    const std::uint32_t first = c.r(0x40048024);
    c.chip.core().advance_to(c.chip.core().cycles() + 1000);
    CHECK(c.r(0x40048024) - first == 1000);
    c.w(0x40048004, 3u << 18); // CFG.PRESC = DIV4
    const std::uint32_t second = c.r(0x40048024);
    c.chip.core().advance_to(c.chip.core().cycles() + 1000);
    CHECK(c.r(0x40048024) - second == 250);
    c.w(0x4004800c, 2);      // CMD.STOP
    const std::uint32_t stopped = c.r(0x40048024);
    c.chip.core().advance_to(c.chip.core().cycles() + 1000);
    CHECK(c.r(0x40048024) == stopped && (c.r(0x40048010) & 1) == 0);
    c.w(0x4004c030, 1);      // TIMER1: TOP keeps 16 bits; CNT wraps at 0x10000
    c.w(0x4004c01c, 0xffffffffu);
    CHECK(c.r(0x4004c01c) == 0xffff);
    c.w(0x4004c024, 0xfff0);
    c.w(0x4004c00c, 1);
    c.chip.core().advance_to(c.chip.core().cycles() + 0x20);
    CHECK(c.r(0x4004c024) == 0x10);
}

// The radio as the zigbee-rs efr32s2-hal driver runs it (RsmRadio): a vector table of END slots
// hands the radio to the model, a self-JUMP holds a warm state, the LO counter guides the VCO
// search, BUF1 receives [PHR][PSDU][RSSI] and BUF0 is sent when TXWARM is released.
void test_efr32_rsm_radio() {
    constexpr std::uint32_t Seq = 0xB0000000, Rac = 0xB8020000, Frc = 0xB8004000, Synth = 0xB8018000,
                            Agc = 0xB800C000, Bufc = 0x54004000;
    constexpr std::uint32_t End = 0xF800F800;
    auto jump = [](std::uint32_t state) { return (4 * state) << 16 | 0x9802u; };
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("peer", {});
    Chip a;
    a.chip.connect_radio(air, "a");
    auto state = [&] { return (a.r(Rac + 0x0C) >> 24) & 0xf; };
    for (std::uint32_t slot = 0; slot != 16; ++slot)
        a.w(Seq + 4 * slot, slot == 1 ? jump(1) : End); // RXWARM parked
    a.w(Rac + 0x64, Seq);                                // VECTADDR
    a.w(Rac + 0x68, 1);                                  // SEQCTRL.COMPACT: the model takes over
    const std::uint32_t freq = static_cast<std::uint32_t>((2425000000ull << 19) / 38400000u); // channel 15
    a.w(Synth + 0x34, freq);
    a.w(Bufc + 0x08, 2);                                 // BUF0: 256 B
    a.w(Bufc + 0x0C, 0x20001000);
    a.w(Bufc + 0x38, 3);                                 // BUF1: 512 B
    a.w(Bufc + 0x3C, 0x20002000);
    CHECK(state() == 0);
    CHECK(a.r(Agc + 0x10) == 0x8000);                    // RSSI invalid while off

    a.w(Rac + 0x08, 2);                                  // RXENSRCEN.SWRXEN
    CHECK(state() == 1);                                 // held in RXWARM
    auto lo_count = [&](std::uint32_t tuning) {
        a.w(Synth + 0x4C, 16u << 11 | tuning);           // VCOTUNING
        a.w(Synth + 0x2078, 1u << 9);                    // IF_CLR.LOCNTDONE
        a.w(Synth + 0x88, 0x43);                         // LOCNTCTRL: NUMCYCLE 4, ENABLE, CLEAR
        a.w(Synth + 0x88, 0x45);                         // RUN
        CHECK((a.r(Synth + 0x78) & (1u << 9)) != 0);
        return a.r(Synth + 0x8C) & 0x7ffff;
    };
    CHECK(lo_count(0x100) > freq >> 13 && lo_count(0x700) < freq >> 13); // open loop spans the channel
    a.w(Synth + 0x0C, 1);                                // CMD.START: the loop locks
    CHECK(lo_count(0x100) == freq >> 13);
    a.w(Seq + 4, End);                                   // release RXWARM
    CHECK(state() == 2);
    const std::uint32_t rssi = a.r(Agc + 0x10);
    CHECK(rssi != 0x8000 && static_cast<std::int32_t>(rssi << 16) >> 24 == -49); // -110 dBm + 61, quarter dB in 15:6

    const std::vector<std::uint8_t> in = {0x41, 0x88, 0x07, 0xaa, 0xbb};
    air.transmit("peer", {}, mcuemu::RadioProtocol::Zigbee802154, 15, a.chip.time_ns() + 1000, in);
    a.chip.run(a.chip.core().cycles() + 38400);        // 1 ms
    CHECK((a.r(Frc + 0x68) & (1u << 4)) != 0);           // FRC.IF.RXDONE
    CHECK(a.r(Bufc + 0x58) == 7);                        // BUF1.STATUS: PHR + 5 + RSSI
    std::uint8_t stored[7] = {};
    a.chip.core().read_memory(0x20002000, stored, sizeof(stored));
    const std::uint8_t expected[7] = {7, 0x41, 0x88, 0x07, 0xaa, 0xbb, 11}; // ideal medium: -50 dBm + 61
    CHECK(std::memcmp(stored, expected, sizeof(expected)) == 0);

    a.w(Rac + 0x08, 0);                                  // receiver off
    CHECK(state() == 0);
    a.w(Seq + 4 * 8, jump(8));                           // park TXWARM
    a.w(Bufc + 0x30, 1);                                 // BUF0.CMD.CLEAR
    for (std::uint8_t byte : {7, 0x41, 0x88, 0x09, 0xcc, 0xdd})
        a.w(Bufc + 0x20, byte);                          // WRITEDATA
    a.w(Rac + 0x10, 1);                                  // CMD.TXEN
    CHECK(state() == 8);
    const std::size_t sent = air.frames().size();
    a.w(Seq + 4 * 8, End);                               // release TXWARM: the frame goes out
    CHECK(state() == 9);
    CHECK(air.frames().size() == sent + 1 && air.frames().back().channel == 15 &&
          air.frames().back().payload == (std::vector<std::uint8_t>{0x41, 0x88, 0x09, 0xcc, 0xdd}));
    CHECK((a.r(Frc + 0x68) & 1) == 0);
    a.chip.run(a.chip.core().cycles() + 38400);
    CHECK((a.r(Frc + 0x68) & 1) != 0);                   // FRC.IF.TXDONE
    CHECK(state() == 0);                                 // TXPD not held: back to OFF
}

// The SEMAILBOX host answers a command once the words its TX_HEADER announces
// have arrived (the handle word is not stored), with SUCCESS and no output.
void test_efr32_semailbox_answers_commands() {
    Chip c;
    CHECK((c.r(0x4C000040) & (1u << 20)) != 0);   // TX_STATUS.TXINT: room
    c.w(0x4C000050, 16);                          // TX_HEADER: 4 words
    c.w(0x4C000000, 1);                           // command
    c.w(0x4C000000, 2);
    CHECK((c.r(0x4C000044) & (1u << 20)) == 0);   // not yet: 2 of 3 words
    c.w(0x4C000000, 3);
    CHECK((c.r(0x4C000044) & (1u << 20)) != 0);   // RX_STATUS.RXINT
    CHECK((c.r(0x4C000054) & 0x000f0000) == 0);   // RX_HEADER: SLI_SE_RESPONSE_OK
    CHECK((c.r(0x5C000044) & (1u << 20)) != 0);   // the non-secure alias
}

// The RTCC keeps the fraction of a tick when the counter is frozen, so reading it
// many times in a row (each read-modify-write of a register freezes it) must not
// make it run slow.
void test_efr32_rtcc_keeps_tick_fraction() {
    Chip c;
    c.w(0x48000004, 1);
    c.w(0x4800000c, 1);
    // 1171.875 cycles per tick: a freeze every 1000 cycles would lose the remainder each time.
    for (std::uint64_t cycle = 1000; cycle <= 1000 * 1000; cycle += 1000) {
        c.chip.core().advance_to(cycle);
        c.w(0x48000008, 0); // CFG write freezes the counter
    }
    const std::uint32_t expected = static_cast<std::uint32_t>(1000ull * 1000 * 32768 / 38400000);
    const std::uint32_t got = c.r(0x48000020);
    CHECK(got + 1 >= expected && got <= expected);
}

namespace {
// The RAIL register addresses used below (secure view).
constexpr std::uint32_t RacRxEnable = 0xA8020008, RacCmd = 0xA8020010, FrcIf = 0xA8004068;
constexpr std::uint32_t Buf0Addr = 0x4400400C, Buf0WriteOffset = 0x44004010, Buf0WriteData = 0x44004020;
constexpr std::uint32_t Buf2Addr = 0x4400406C, Buf1Addr = 0x4400403C, Buf1WriteOffset = 0x44004040, BufcIf = 0x440040E0;
constexpr std::uint32_t ProtimerCmd = 0xA801C00C, ProtimerStatus = 0xA801C014;
} // namespace

// RAIL's blocks keep what is written, with set/clear/toggle aliases everywhere except the
// sequencer RAM; the PROTIMER start and stop commands drive STATUS.RUNNING.
void test_efr32_rail_registers() {
    Chip c;
    c.w(RacRxEnable, 0x10);
    c.w(RacRxEnable + Set, 0x03);
    CHECK(c.r(RacRxEnable) == 0x13);
    c.w(RacRxEnable + Clear, 0x01);
    c.w(RacRxEnable + Toggle, 0x10);
    CHECK(c.r(RacRxEnable) == 0x02);
    // The sequencer RAM is memory: an address with alias bits is a different word.
    c.w(0xA0000100, 0xdeadbeef);
    c.w(0xA0001100, 0x11111111);
    CHECK(c.r(0xA0000100) == 0xdeadbeef);
    // The non-secure view is the same register.
    CHECK(c.r(0xB8020008) == 0x02);
    CHECK((c.r(ProtimerStatus) & 1) == 0);
    c.w(ProtimerCmd, 1);
    CHECK((c.r(ProtimerStatus) & 1) == 1);
    c.w(ProtimerCmd, 4);
    CHECK((c.r(ProtimerStatus) & 1) == 0);
    // BUFC BUF0 STATUS (+0x28) counts the bytes between READOFFSET and WRITEOFFSET.
    CHECK((c.r(0x44004028) & 0x1fff) == 0);
    c.w(0x44004010, 9);
    CHECK((c.r(0x44004028) & 0x1fff) == 9);
    c.w(0x44004014, 4);
    CHECK((c.r(0x44004028) & 0x1fff) == 5);
    // An unwritten synthesizer divider word reads as 1 (the Tuya image asserts on 0).
    CHECK(c.r(0xA801803C) == 1);
}

// RADIOAES: descriptor chains for CCM (RFC 3610 packet vector 1, as sli_protocol_crypto_radioaes.c builds them)
// and ECB (FIPS-197 appendix B).
namespace {
struct AesDescriptor {
    std::uint32_t address, next, length, tag;
};
} // namespace

void test_efr32_radioaes() {
    Chip c;
    constexpr std::uint32_t Ram = 0x20000000, Buf = Ram + 0x8000, Desc = Ram + 0x9000;
    auto put = [&](std::uint32_t at, const std::vector<std::uint8_t> &bytes) {
        c.chip.core().write_memory(at, bytes.data(), bytes.size());
    };
    auto get = [&](std::uint32_t at, std::size_t n) {
        std::vector<std::uint8_t> bytes(n);
        c.chip.core().read_memory(at, bytes.data(), n);
        return bytes;
    };
    auto chain = [&](std::uint32_t at, const std::vector<AesDescriptor> &list) {
        for (std::size_t i = 0; i != list.size(); ++i) {
            AesDescriptor d = list[i];
            d.next = i + 1 == list.size() ? 1u : at + static_cast<std::uint32_t>(16 * (i + 1));
            c.chip.core().write_memory(at + static_cast<std::uint32_t>(16 * i), &d, sizeof d);
        }
    };
    constexpr std::uint32_t Aes = 0x44000000;
    constexpr std::uint32_t Config = 0x10, Data = 0x0, Last = 0x20, Header = 0x40, Engine = 1;
    // CCM encrypt: key, nonce, AAD and payload of RFC 3610 vector 1 (M = 8, L = 2).
    std::vector<std::uint8_t> key, b0 = {0x59, 0, 0, 0, 3, 2, 1, 0, 0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x00, 0x17};
    for (unsigned i = 0; i != 16; ++i)
        key.push_back(static_cast<std::uint8_t>(0xC0 + i));
    std::vector<std::uint8_t> aad = {0x00, 0x08, 0, 1, 2, 3, 4, 5, 6, 7};
    std::vector<std::uint8_t> plain;
    for (unsigned i = 8; i != 31; ++i)
        plain.push_back(static_cast<std::uint8_t>(i));
    const std::uint32_t mode = 0x2000; // CCM, encrypt, AES-128
    put(Buf, {static_cast<std::uint8_t>(mode), static_cast<std::uint8_t>(mode >> 8), 0, 0});
    put(Buf + 0x10, key);
    put(Buf + 0x20, b0);
    put(Buf + 0x30, aad);
    put(Buf + 0x40, plain);
    chain(Desc, {{Buf, 0, 4, Engine | Config | (0u << 8)},
                 {Buf + 0x10, 0, 16, Engine | Config | (8u << 8)},
                 {Buf + 0x20, 0, 16, Engine | Header},
                 {Buf + 0x30, 0, 10, Engine | Header | (6u << 8)},
                 {Buf + 0x40, 0, 23, Engine | Data | Last | (9u << 8)}});
    // Pusher: discard the 32 header bytes, the data (23), its padding (9), the tag (8) and 8 bytes of padding.
    chain(Desc + 0x100, {{0, 0, 32 | 0x40000000u, 0},
                         {Buf + 0x100, 0, 23, 0},
                         {0, 0, 9 | 0x40000000u, 0},
                         {Buf + 0x120, 0, 8, 0},
                         {0, 0, 8 | 0x40000000u, 0}});
    c.w(Aes + 0x04, Desc);
    c.w(Aes + 0x14, Desc + 0x100);
    c.w(Aes + 0x34, 3);
    c.w(Aes + 0x38, 3);
    CHECK((c.r(Aes + 0x3C) & 3) == 0);
    const std::vector<std::uint8_t> cipher = {0x58, 0x8C, 0x97, 0x9A, 0x61, 0xC6, 0x63, 0xD2, 0xF0, 0x66, 0xD0, 0xC2,
                                              0xC0, 0xF9, 0x89, 0x80, 0x6D, 0x5F, 0x6B, 0x61, 0xDA, 0xC3, 0x84};
    const std::vector<std::uint8_t> mic = {0x17, 0xE8, 0xD1, 0x2C, 0xFD, 0xF9, 0x26, 0xE0};
    CHECK(get(Buf + 0x100, 23) == cipher);
    CHECK(get(Buf + 0x120, 8) == mic);

    // ECB (FIPS-197 appendix B): key 2b7e1516..., plaintext 3243f6a8... -> 3925841d02dc09fbdc118597196a0b32.
    const std::vector<std::uint8_t> ecb_key = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
                                               0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
    const std::vector<std::uint8_t> block = {0x32, 0x43, 0xf6, 0xa8, 0x88, 0x5a, 0x30, 0x8d,
                                             0x31, 0x31, 0x98, 0xa2, 0xe0, 0x37, 0x07, 0x34};
    put(Buf, {0x00, 0x01, 0, 0});
    put(Buf + 0x10, ecb_key);
    put(Buf + 0x40, block);
    chain(Desc, {{Buf, 0, 4, Engine | Config},
                 {Buf + 0x10, 0, 16, Engine | Config | (8u << 8)},
                 {Buf + 0x40, 0, 16, Engine | Data | Last}});
    chain(Desc + 0x100, {{Buf + 0x140, 0, 16, 0}});
    c.w(Aes + 0x04, Desc);
    c.w(Aes + 0x14, Desc + 0x100);
    c.w(Aes + 0x38, 3);
    const std::vector<std::uint8_t> expected = {0x39, 0x25, 0x84, 0x1d, 0x02, 0xdc, 0x09, 0xfb,
                                                0xdc, 0x11, 0x85, 0x97, 0x19, 0x6a, 0x0b, 0x32};
    CHECK(get(Buf + 0x140, 16) == expected);
}

// A transmit takes the frame from the BUF0 buffer RAM on RAC CMD.TXEN, puts it on the air
// on the SYNTH channel, and raises FRC TX_PACKET_SENT when it ends.
void test_efr32_rail_transmit() {
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    Chip a;
    a.chip.connect_radio(air, "a");
    a.w(Buf0Addr, 0x20001000);
    a.w(0xA8018044, 15);
    a.w(Buf0WriteOffset, 0);
    // The FIFO writer stores through WRITEDATA, which advances WRITEOFFSET.
    const std::uint8_t frame[] = {7, 0x41, 0x88, 0x05, 0xaa, 0xbb};
    for (std::uint8_t byte : frame)
        a.w(Buf0WriteData, byte);
    CHECK(a.r(Buf0WriteOffset) == sizeof(frame));
    std::uint8_t stored[sizeof(frame)] = {};
    a.chip.core().read_memory(0x20001000, stored, sizeof(stored));
    CHECK(std::memcmp(stored, frame, sizeof(frame)) == 0);
    a.w(RacCmd, 1);
    CHECK(air.frames().size() == 1 && air.frames()[0].channel == 15 && air.frames()[0].payload.size() == 5);
    CHECK((a.r(FrcIf) & (1u << 15)) == 0);
    a.chip.run(a.chip.core().cycles() + 30 * 1000); // far longer than 7 bytes on air
    CHECK((a.r(FrcIf) & (1u << 15)) != 0);
    CHECK((a.r(RacRxEnable) & 2) != 0); // a successful transmit leaves the radio receiving
}

// A unicast data frame to this device that asks for an ACK is stored in BUF1, described in
// the BUF2 ring and answered with an ACK carrying its sequence number.
void test_efr32_rail_receive_and_ack() {
    constexpr std::uint64_t Id = 0x0123456789abcdefull;
    mcuemu::VirtualAir air;
    air.set_node("a", {});
    air.set_node("coordinator", {});
    Chip a;
    a.chip.set_device_id(Id);
    a.chip.connect_radio(air, "a");
    // Driver state: the RX context points at a BUF2 ring of 64 bytes, indexes at zero.
    auto store32 = [&](std::uint32_t address, std::uint32_t value) { a.chip.core().write_memory(address, &value, 4); };
    // The context is found by the three buffer addresses it holds at +0x18, +0x1C and +0x20.
    store32(0x20002018, 0x20003000);
    store32(0x2000201C, 0x20005000);
    store32(0x20002020, 0x20001000);
    a.w(Buf0Addr, 0x20001000);
    a.w(Buf2Addr, 0x20003000);
    const std::uint16_t ring_size = 64;
    a.chip.core().write_memory(0x2000203A, &ring_size, 2);
    a.w(Buf1Addr, 0x20005000);
    a.w(RacRxEnable + Set, 2);

    std::vector<std::uint8_t> payload = {0x61, 0xcc, 0x42, 0x34, 0x12}; // data, ACK request, PAN-compressed
    for (unsigned i = 0; i != 8; ++i)
        payload.push_back(static_cast<std::uint8_t>(Id >> (8 * i))); // destination: this device
    for (unsigned i = 0; i != 8; ++i)
        payload.push_back(0x10 + i);                                   // source
    payload.push_back(0x99);
    air.transmit("coordinator", {}, mcuemu::RadioProtocol::Zigbee802154, 11, 100000, payload);
    a.chip.run(a.chip.core().ns_to_cycles(2000000));

    CHECK((a.r(BufcIf) & (1u << 18)) != 0);          // BUF2 threshold flag
    CHECK(a.r(Buf1WriteOffset) == 1 + payload.size() + 8); // PHR + payload, then 8 appended bytes (the FCS is only in the PHR)
    std::uint8_t phr = 0;
    a.chip.core().read_memory(0x20005000, &phr, 1);
    CHECK(phr == payload.size() + 2);
    std::uint8_t descriptor[8] = {};
    a.chip.core().read_memory(0x20003000 + 56, descriptor, sizeof(descriptor)); // (0 + 0 - 7) & 63, rounded down to 8
    CHECK((descriptor[2] & 7) == 7 && (descriptor[2] & 0x10) == 0);
    CHECK(descriptor[0] == a.r(Buf1WriteOffset));
    // The second frame on the air is the ACK: type 2, sequence 0x42.
    CHECK(air.frames().size() == 2);
    CHECK(air.frames()[1].payload == (std::vector<std::uint8_t>{0x02, 0x00, 0x42}));
    CHECK((a.r(FrcIf) & 2) != 0);                    // TXAFTERFRAMEDONE after the ACK

    // A frame to somebody else gets no ACK.
    payload[5] ^= 0xff;
    air.transmit("coordinator", {}, mcuemu::RadioProtocol::Zigbee802154, 11, 3000000, payload);
    a.chip.run(a.chip.core().ns_to_cycles(5000000));
    CHECK(air.frames().size() == 3);
}

} // namespace mcuemu_tests
