// CC2530F256 model gaps closed from SWRU191F: IRCON bit 6 (2.5), Timer 1
// capture from the RF interrupt (9.12 T1CCTLn.RFIRQ) and capture in PM1-PM3,
// the 8-symbol RSSI average (23.10.3, 23.8.12), the clock-loss detector (5.2)
// and the watchdog in PM1-PM3 (16, 4.1).
//
// Programs are hand-encoded MCS-51 machine code; expected values come from
// SWRU191F and from the rules marked "Model-defined" in the model sources.
// No expectation depends on instruction cycle counts: events are checked in
// a power mode, in a CPU stall or on a 1-cycle NOP sled, where every cycle
// is an instruction boundary, and instruction-dependent instants (PM entry,
// register write) are measured (breakpoints, diagnostics).

#include "cc2530_periph_support.hpp"
#include "cc2530_timers.hpp"
#include "mcuemu/radio.hpp"

#include <array>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

namespace {

using namespace cc2530test;
using mcs51emu::Cc2530Oscillator;
using mcuemu::RadioProtocol;
using mcuemu::VirtualAir;
using mcuemu::VirtualRadioNode;

// SFRs (SWRU191F Table 2-1)
constexpr std::uint8_t SP = 0x81, PCON = 0x87, RFIRQF1 = 0x91, ST0 = 0x95, ST1 = 0x96,
                       ST2 = 0x97, SLEEPSTA = 0x9d, CLKCONSTA = 0x9e, IEN0 = 0xa8,
                       T1STAT = 0xaf, IEN1 = 0xb8, SLEEPCMD = 0xbe, RFERRF = 0xbf, IRCON = 0xc0,
                       CLKCONCMD = 0xc6, WDCTL = 0xc9, T1CC0L = 0xda, T1CC1L = 0xdc,
                       T1CC2L = 0xde, RFST = 0xe1, T1CNTL = 0xe2, T1CTL = 0xe4, T1CCTL0 = 0xe5,
                       T1CCTL1 = 0xe6, T1CCTL2 = 0xe7, RFIRQF0 = 0xe9;
// Bit addresses
constexpr std::uint8_t BitP0if = 0xc5, BitStif = 0xc7;
// XREGs (Table 23-5, 5.2)
constexpr std::uint16_t RxFifo = 0x6000, FRMFILT0 = 0x6180, FRMCTRL0 = 0x6189,
                        FSMSTAT1 = 0x6193, CCACTRL0 = 0x6196, RSSI = 0x6198, RSSISTAT = 0x6199,
                        RFIRQM0 = 0x61a3, RFERRM = 0x61a5, XCld = 0x6290;
// Interrupt vectors (Table 2-5)
constexpr std::uint16_t VSt = 0x2b, VP0 = 0x6b;
constexpr std::uint8_t IrconT1if = 0x02, IrconBit6 = 0x40, IrconStif = 0x80;
// Radio command strobes (23.14.9) and flags (23.1.2)
constexpr std::uint8_t IsRxOn = 0xe3, IsRfOff = 0xef;
constexpr std::uint8_t F0RxPktDone = 0x40, F0Sfd = 0x02, ErrStrobe = 0x40, S1Cca = 0x10;
// SLEEPSTA.RST (bits 4:3)
constexpr std::uint8_t RstWatchdog = 0x10, RstClockLoss = 0x18;
constexpr unsigned CauseWatchdog = 2, CauseClockLoss = 3; // Cc2530ResetCause
constexpr unsigned PmIndex1 = 2, PmIndex3 = 4;              // Cc2530PowerMode Pm1, Pm3
// SLEEPCMD values: MODE in bits 1:0, bit 2 kept at its reset value 1.
constexpr std::uint8_t SleepIdle = 0x04, SleepPm1 = 0x05, SleepPm2 = 0x06, SleepPm3 = 0x07;

// Radio timing at 32 MHz (IEEE 802.15.4 2.4 GHz O-QPSK): 512 cycles per
// symbol, 1024 per octet; RX calibration 12 symbols (23.8.2); RSSI_VALID 8
// symbol periods after the receiver is enabled (23.10.3).
constexpr std::uint64_t Symbol = 512, Octet = 1024, Turn = 6144, Shr = 5 * Octet,
                        RssiValid = 4096;
// Clock-loss detector time-outs (5.2): 0.25 ms (32 MHz XOSC), 0.5 ms (32 kHz).
constexpr std::uint64_t CldXosc = 8000, Cld32k = 16000;

// Flash image: LJMP 0x0100 at the reset vector; main at 0x0100 starts with
// MOV SP,#0x30. ISR bodies go to 0x0090-0x00ff.
struct Program {
    Flash flash;
    Program() {
        flash.at(0).op({0x02, 0x01, 0x00});
        flash.at(0x0100).op(mov_dir_imm(SP, 0x30));
    }
    Program &op(std::initializer_list<std::uint8_t> code) {
        flash.op(code);
        return *this;
    }
    Program &op(const std::vector<std::uint8_t> &code) {
        flash.op(code);
        return *this;
    }
    Program &mov(std::uint8_t dir, std::uint8_t value) { return op(mov_dir_imm(dir, value)); }
    Program &xw(std::uint16_t address, std::uint8_t value) { return op(movx_write(address, value)); }
    Program &nop() { return op({0x00}); }
    // SLEEPCMD = mode; PCON = 1; NOP. The instruction after the PCON write is
    // kept off a 4-byte boundary (SWRU191F 4.2).
    Program &enter_power_mode(std::uint8_t sleepcmd) {
        mov(SLEEPCMD, sleepcmd);
        if (((here() + 3u) & 3u) == 0)
            nop();
        return mov(PCON, 0x01).nop();
    }
    // NOPs (1 cycle each, so every cycle is an instruction boundary) up to
    // 0xfff0 (bank 1 at FMAP = 1, about 65000 cycles), then LJMP back to the
    // first NOP. Every checked instant lies within the first pass.
    Program &sled() {
        const std::uint16_t start = here();
        while (here() < 0xfff0)
            nop();
        return op({0x02, hi(start), lo(start)});
    }
    std::uint16_t here() const { return flash.here(); }
    Program &isr(std::uint16_t vector, std::initializer_list<std::uint8_t> body) {
        const std::uint32_t saved = flash.cursor;
        flash.at(vector).op(body);
        flash.cursor = saved;
        return *this;
    }
};

std::uint64_t now(const Cc2530F256 &d) { return d.registers().cycles; }

// Runs until the CPU is about to execute `pc`; returns that cycle.
std::uint64_t run_to(Cc2530F256 &d, std::uint16_t pc, std::uint64_t limit, const char *what) {
    d.set_breakpoint(pc);
    for (;;) {
        const Cc2530RunResult r = d.run(10'000'000, limit);
        if (!r.error.empty()) {
            std::cerr << what << ": run error: " << r.error << '\n';
            ++failures;
            break;
        }
        if (r.break_hit)
            break;
        if (now(d) >= limit) {
            std::cerr << what << ": PC 0x" << std::hex << pc << std::dec << " not reached by cycle "
                      << limit << '\n';
            ++failures;
            break;
        }
    }
    d.set_breakpoint(std::nullopt);
    return now(d);
}

// Runs to the first instruction boundary at or after `cycle`.
void run_until(Cc2530F256 &d, std::uint64_t cycle) {
    while (now(d) < cycle) {
        const Cc2530RunResult r = d.run(10'000'000, cycle);
        if (!r.error.empty()) {
            std::cerr << "run error: " << r.error << '\n';
            ++failures;
            return;
        }
    }
}

// Runs to exactly `cycle` (which must be an instruction boundary).
bool at(Cc2530F256 &d, std::uint64_t cycle, const char *what) {
    run_until(d, cycle);
    if (now(d) != cycle) {
        std::cerr << what << ": wanted boundary " << cycle << ", landed at " << now(d) << '\n';
        ++failures;
        return false;
    }
    return true;
}

// The 32 kHz clock: one period = 976.5625 cycles; period j starts at
// ceil(j * 15625 / 16); the Sleep Timer compare event of value n comes at
// the rising edge in period n, ceil((2n + 1) * 15625 / 32).
std::uint64_t lf_ticks(std::uint64_t cycles) { return cycles * 16 / 15625; }
std::uint64_t period_start(std::uint64_t j) { return (j * 15625 + 15) / 16; }
std::uint64_t rising_edge(std::uint64_t j) { return ((2 * j + 1) * 15625 + 31) / 32; }

std::uint16_t word_sfr(const Cc2530F256 &d, std::uint8_t low) {
    return static_cast<std::uint16_t>(d.read_sfr(low) |
                                      d.read_sfr(static_cast<std::uint8_t>(low + 1)) << 8);
}
std::uint8_t x(const Cc2530F256 &d, std::uint16_t address) { return d.peek_xdata(address); }
std::uint64_t resets(const Cc2530F256 &d, unsigned cause) {
    return d.diagnostics().resets_by_cause[cause];
}
// Resets other than power-on (load() counts one power-on reset).
std::uint64_t later_resets(const Cc2530F256 &d) {
    const auto &by_cause = d.diagnostics().resets_by_cause;
    return by_cause[1] + by_cause[2] + by_cause[3];
}
mcs51emu::cc2530::Timer1 &t1(Cc2530F256 &d) { return *d.peripherals().timer1; }
mcs51emu::cc2530::SleepTimer &st(Cc2530F256 &d) { return *d.peripherals().sleep_timer; }

// Air time of a cycle count (attach_air: ns = cycles * 1000 / 32).
std::uint64_t ns(std::uint64_t cycle) { return cycle * 1000 / 32; }
// IEEE 802.15.4 data frame (no ACK request, PAN ID compression, short
// addresses) to the broadcast PAN/address, without the FCS.
std::vector<std::uint8_t> frame(std::uint8_t seq, std::vector<std::uint8_t> tail) {
    std::vector<std::uint8_t> m{0x41, 0x88, seq, 0xff, 0xff, 0xff, 0xff, 0x02, 0x00};
    m.insert(m.end(), tail.begin(), tail.end());
    return m;
}

// ---------------------------------------------------------------------------
// IRCON bit 6 (SWRU191F 2.5 IRCON: "Must be written 0. Writing a 1 always
// enables the interrupt source"; Figure 2-4 ORs it with STIF into the Sleep
// Timer interrupt request). Nothing clears it but software.
// ---------------------------------------------------------------------------

// ST ISR at 0x0090: INC 0x40; MOV A,0x40; CJNE A,#3,+3; ANL IRCON,#0xBF;
// RETI. It clears IRCON.6 only on its third entry.
Program ircon6_program(std::uint8_t ien0) {
    Program p;
    p.mov(IEN0, ien0).mov(IRCON, IrconBit6).sled();
    p.isr(VSt, {0x02, 0x00, 0x90});
    p.isr(0x0090, {0x05, 0x40, 0xe5, 0x40, 0xb4, 0x03, 0x03, 0x53, IRCON, 0xbf, 0x32});
    return p;
}

void test_ircon6_requests_sleep_timer_interrupt() {
    {
        // EA + STIE: the request stands until the ISR clears the bit, so the
        // ISR is entered again after each RETI.
        Cc2530F256 d;
        load(d, ircon6_program(0xa0).flash);
        run_until(d, 5000);
        CHECK_EQ(d.read_idata(0x40), 3);
        CHECK_EQ(d.read_sfr(IRCON) & (IrconBit6 | IrconStif), 0);
        CHECK_EQ(d.diagnostics().ircon_bit6_writes, 1u);
    }
    {
        // Without STIE the bit is stored and requests nothing; enabling STIE
        // takes the request.
        Cc2530F256 d;
        load(d, ircon6_program(0x80).flash);
        run_until(d, 3000);
        CHECK_EQ(d.read_sfr(IRCON), IrconBit6);
        CHECK_EQ(d.read_idata(0x40), 0);
        d.write_sfr(IEN0, 0xa0);
        run_until(d, 6000);
        CHECK_EQ(d.read_idata(0x40), 3);
        CHECK_EQ(d.read_sfr(IRCON) & IrconBit6, 0);
        CHECK_EQ(d.diagnostics().ircon_bit6_writes, 1u);
    }
    for_each_mode(
        ircon6_program(0xa0).flash, 5000, [](Cc2530F256 &) {},
        [](Cc2530F256 &d, Mode) { CHECK_EQ(d.read_idata(0x40), 3); }, "ircon6 modes");
}

// Model-defined: as a Sleep Timer request, IRCON.6 wakes PM1/PM2 (4.1.2,
// 4.1.3: the Sleep Timer interrupt is a wake-up source) but not PM3 (4.1.4:
// reset or external I/O interrupt only); there it stays pending.
void test_ircon6_wake_rules() {
    for (const bool pm3 : {false, true}) {
        Program p;
        p.mov(IEN1, 0x20).mov(IEN0, 0xa0); // P0IE; EA + STIE
        p.enter_power_mode(pm3 ? SleepPm3 : SleepPm2).sled();
        p.isr(VSt, {0x02, 0x00, 0x90});
        p.isr(0x0090, {0x05, 0x40, 0x53, IRCON, 0xbf, 0x32}); // INC 0x40; ANL IRCON,#0xBF; RETI
        p.isr(VP0, {0xc2, BitP0if, 0x05, 0x41, 0x32});
        Cc2530F256 d;
        load(d, p.flash);
        const Cc2530PowerMode mode = pm3 ? Cc2530PowerMode::Pm3 : Cc2530PowerMode::Pm2;
        if (at(d, 20000, "ircon6 pm"))
            CHECK(d.power_mode() == mode);
        d.write_sfr(IRCON, IrconBit6);
        CHECK_EQ(d.diagnostics().ircon_bit6_writes, 1u);
        run_until(d, 40000);
        if (!pm3) {
            CHECK(d.power_mode() == Cc2530PowerMode::Active);
            CHECK_EQ(d.diagnostics().power_mode_wakes, 1u);
            CHECK_EQ(d.read_idata(0x40), 1);
            CHECK_EQ(d.read_sfr(IRCON) & IrconBit6, 0);
            continue;
        }
        CHECK(d.power_mode() == Cc2530PowerMode::Pm3);
        CHECK_EQ(d.diagnostics().power_mode_wakes, 0u);
        CHECK_EQ(d.read_idata(0x40), 0);
        CHECK_EQ(d.read_sfr(IRCON) & IrconBit6, IrconBit6);
        // An external interrupt wakes PM3; the standing request is then taken.
        d.schedule_interrupt_flag_for_test(50000, 13); // P0IF
        run_until(d, 70000);
        CHECK(d.power_mode() == Cc2530PowerMode::Active);
        CHECK_EQ(d.diagnostics().power_mode_wakes, 1u);
        CHECK_EQ(d.read_idata(0x41), 1);
        CHECK_EQ(d.read_idata(0x40), 1);
        CHECK_EQ(d.read_sfr(IRCON) & IrconBit6, 0);
    }
}

// ---------------------------------------------------------------------------
// Watchdog in PM1-PM3. SWRU191F 16 (introduction): "The contents of the
// 15-bit counter are retained during all power modes, and the Watchdog Timer
// continues counting when entering active mode again"; 4.1.3 lists the
// blocks active in PM2 without the WDT. So the count holds in PM1-PM3 and
// advances in active mode and Idle (4.1.1). Model-defined: counting resumes
// at the next 32 kHz period boundary after the wake. WDCTL = 0x0b: watchdog
// mode, 64 periods (16.3). After the wake the program enters Idle, where
// virtual time advances exactly to the reset.
// ---------------------------------------------------------------------------

void test_wdt_holds_in_pm1() {
    Program p;
    const std::uint16_t w = p.here();
    p.mov(WDCTL, 0x0b);
    p.mov(ST2, 0).mov(ST1, 0).mov(ST0, 100).mov(IEN0, 0xa0);
    p.enter_power_mode(SleepPm1).enter_power_mode(SleepIdle).sled();
    p.isr(VSt, {0xc2, BitStif, 0x32});
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W = run_to(d, w, 2000, "wdt pm1 w");
    // Without the hold the watchdog would have reset near W + 62500.
    if (at(d, 70000, "wdt pm1 70000")) {
        CHECK(d.power_mode() == Cc2530PowerMode::Pm1);
        CHECK_EQ(later_resets(d), 0u);
    }
    // The Sleep Timer compare (100) wakes PM1 at its rising edge.
    const std::uint64_t wake = rising_edge(100);
    if (at(d, wake + 2000, "wdt pm1 idle"))
        CHECK(d.power_mode() == Cc2530PowerMode::Idle);
    const std::uint64_t entry = wake - d.diagnostics().power_mode_cycles_by_mode[PmIndex1];
    const std::uint64_t before = lf_ticks(entry) - lf_ticks(W);
    const std::uint64_t reset = period_start(lf_ticks(wake) + 64 - before);
    if (at(d, reset - 1, "wdt pm1 before reset")) {
        CHECK(d.power_mode() == Cc2530PowerMode::Idle);
        CHECK_EQ(later_resets(d), 0u);
    }
    if (at(d, reset, "wdt pm1 reset")) {
        CHECK_EQ(resets(d, CauseWatchdog), 1u);
        CHECK_EQ(d.read_sfr(SLEEPSTA) & 0x18, RstWatchdog);
    }
}

void test_wdt_holds_in_pm3() {
    Program p;
    const std::uint16_t w = p.here();
    p.mov(WDCTL, 0x0b);
    p.mov(IEN1, 0x20).mov(IEN0, 0x80);
    p.enter_power_mode(SleepPm3).enter_power_mode(SleepIdle).sled();
    p.isr(VP0, {0xc2, BitP0if, 0x32});
    Cc2530F256 d;
    load(d, p.flash);
    const std::uint64_t W = run_to(d, w, 2000, "wdt pm3 w");
    if (at(d, 149999, "wdt pm3 149999")) {
        CHECK(d.power_mode() == Cc2530PowerMode::Pm3);
        CHECK_EQ(later_resets(d), 0u);
    }
    d.schedule_interrupt_flag_for_test(150000, 13); // P0IF wakes PM3
    if (at(d, 160000, "wdt pm3 idle"))
        CHECK(d.power_mode() == Cc2530PowerMode::Idle);
    // In PM3 the 32 kHz clock is off as well (Table 4-1): the reset comes
    // the PM3 time after the 64th period boundary counted from W.
    const std::uint64_t d3 = d.diagnostics().power_mode_cycles_by_mode[PmIndex3];
    const std::uint64_t reset = period_start(lf_ticks(W) + 64) + d3;
    if (at(d, reset - 1, "wdt pm3 before reset"))
        CHECK_EQ(later_resets(d), 0u);
    if (at(d, reset, "wdt pm3 reset")) {
        CHECK_EQ(resets(d, CauseWatchdog), 1u);
        CHECK_EQ(d.read_sfr(SLEEPSTA) & 0x18, RstWatchdog);
    }
}

// ---------------------------------------------------------------------------
// Timer 1 capture from the RF interrupt. SWRU191F 9.12 T1CCTLn.RFIRQ: "When
// set, use RF interrupt for capture instead of regular capture input."
// Model-defined input (Timer1::rf_interrupt_edge): every RF interrupt
// request (23.1.1, rising-edge triggered) is a rising edge; the RF interrupt
// line returning low (no unmasked RFIRQF0/RFIRQF1 flag left) is a falling
// edge. RFERR is a separate interrupt (Table 2-5, interrupt 0).
// Timer 1 runs free at TICKSPD 16 MHz (reset CLKCONSTA): one count per two
// cycles, so captures at even cycle distances differ by half the distance.
// T1CCTL0 = 0x01: pad, rising. T1CCTL1 = 0xc1: RFIRQ, IM, rising.
// T1CCTL2 = 0x82: RFIRQ, falling, no IM.
// ---------------------------------------------------------------------------

Program t1_rf_program(std::uint8_t rfirqm0) {
    Program p;
    p.xw(FRMFILT0, 0x00).xw(RFIRQM0, rfirqm0).xw(RFERRM, ErrStrobe);
    p.mov(T1CCTL0, 0x01).mov(T1CCTL1, 0xc1).mov(T1CCTL2, 0x82).mov(T1CTL, 0x01);
    return p;
}

void test_t1_capture_from_rf_interrupt() {
    // repeat: RXPKTDONE is unmasked too and requests the RF interrupt again.
    for (const bool repeat : {false, true}) {
        const std::uint8_t mask = repeat ? (F0Sfd | F0RxPktDone) : F0Sfd;
        Program p = t1_rf_program(mask);
        const std::uint16_t ready = p.here();
        p.sled();
        VirtualAir air;
        VirtualRadioNode peer(air, "peer");
        Cc2530F256 d;
        load(d, p.flash);
        d.attach_air(air, "dut");
        run_to(d, ready, 2000, "t1 rf ready");

        // RFERRF.STROBEERR ("trying to disable radio when already
        // disabled", 23.1.2) requests RFERR, which is not the RF interrupt.
        if (at(d, 2048, "t1 rf strobe"))
            d.write_sfr(RFST, IsRfOff);
        CHECK_EQ(d.read_sfr(RFERRF) & ErrStrobe, ErrStrobe);
        CHECK_EQ(d.read_sfr(T1STAT), 0);

        // Channel 0 captures its pad: the time reference.
        const std::uint64_t ref_at = 3000;
        if (at(d, ref_at, "t1 rf ref"))
            t1(d).input_changed(0, true);
        CHECK_EQ(d.read_sfr(T1STAT), 0x01);
        const std::uint16_t ref = word_sfr(d, T1CC0L);
        const auto count_at = [&](std::uint64_t c) {
            return static_cast<std::uint16_t>(ref + (c - ref_at) / 2);
        };
        // With RFIRQ = 1 the pad is not the capture input.
        t1(d).input_changed(1, true);
        t1(d).input_changed(2, true);
        t1(d).input_changed(2, false);
        CHECK_EQ(d.read_sfr(T1STAT), 0x01);
        d.write_sfr(T1STAT, 0x00);

        const std::uint64_t t = 4096;
        if (at(d, t, "t1 rf rx on"))
            d.write_sfr(RFST, IsRxOn);
        const std::uint64_t p0 = 16384;
        const auto m = frame(1, {0x11, 0x22, 0x33});
        peer.transmit("", RadioProtocol::Zigbee802154, 11, ns(p0), m);
        const std::uint64_t s = p0 + Shr;
        const std::uint64_t e = s + (1 + m.size() + 2) * Octet;

        if (at(d, s - 1, "t1 rf before sfd")) {
            CHECK_EQ(d.read_sfr(RFIRQF0), 0);
            CHECK_EQ(d.read_sfr(T1STAT), 0);
            CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, 0);
        }
        // SFD: the RF interrupt request captures channel 1 (rising) and,
        // with T1CCTL1.IM, requests the Timer 1 interrupt (9.10).
        if (at(d, s, "t1 rf sfd")) {
            CHECK_EQ(d.read_sfr(RFIRQF0), F0Sfd);
            CHECK_EQ(d.read_sfr(T1STAT), 0x02);
            CHECK_EQ(word_sfr(d, T1CC1L), count_at(s));
            CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, IrconT1if);
        }
        d.write_sfr(T1STAT, 0x00);
        d.write_sfr(IRCON, 0x00);

        // Clearing the flag takes the RF interrupt line low: channel 2
        // captures (falling) without an interrupt request (IM = 0).
        const std::uint64_t c = s + 1000;
        if (at(d, c, "t1 rf clear"))
            d.write_sfr(RFIRQF0, 0x00);
        CHECK_EQ(d.read_sfr(T1STAT), 0x04);
        CHECK_EQ(word_sfr(d, T1CC2L), count_at(c));
        CHECK_EQ(word_sfr(d, T1CC1L), count_at(s));
        CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, 0);
        d.write_sfr(T1STAT, 0x00);

        // RXPKTDONE: a masked flag is no RF interrupt (23.1.2) and captures
        // nothing; unmasked, it is a new request and channel 1 captures.
        if (at(d, e - 1, "t1 rf before done"))
            CHECK_EQ(d.read_sfr(RFIRQF0) & F0RxPktDone, 0);
        if (at(d, e, "t1 rf done")) {
            CHECK_EQ(d.read_sfr(RFIRQF0) & F0RxPktDone, F0RxPktDone);
            CHECK_EQ(d.read_sfr(T1STAT), repeat ? 0x02 : 0x00);
            CHECK_EQ(word_sfr(d, T1CC1L), count_at(repeat ? e : s));
            CHECK_EQ(d.read_sfr(IRCON) & IrconT1if, repeat ? IrconT1if : 0);
        }
        d.write_sfr(T1STAT, 0x00);
        const std::uint64_t c2 = e + 1000;
        if (at(d, c2, "t1 rf clear done"))
            d.write_sfr(RFIRQF0, 0x00);
        CHECK_EQ(d.read_sfr(T1STAT), repeat ? 0x04 : 0x00);
        CHECK_EQ(word_sfr(d, T1CC2L), count_at(repeat ? c2 : c));
    }
}

// Capture in PM1-PM3. SWRU191F Table 4-1 / 4.1.2-4.1.4: neither high-
// frequency oscillator runs in PM1-PM3, so the system clock that drives the
// Timer 1 counter and synchronizes its capture inputs (9.7) is stopped; the
// counter holds (4.1.3: Timer 1 is not among the PM2 blocks). Model-defined:
// an edge in PM1-PM3 is lost, it is not latched for the wake.
void test_t1_no_capture_in_power_mode() {
    Program p;
    p.mov(T1CCTL0, 0x03).mov(T1CTL, 0x01); // both edges
    p.mov(ST2, 0).mov(ST1, 0).mov(ST0, 100).mov(IEN0, 0xa0);
    p.enter_power_mode(SleepPm2).sled();
    p.isr(VSt, {0xc2, BitStif, 0x32});
    Cc2530F256 d;
    load(d, p.flash);
    if (at(d, 50000, "t1 pm2")) {
        CHECK(d.power_mode() == Cc2530PowerMode::Pm2);
        t1(d).input_changed(0, true);
        t1(d).input_changed(0, false);
        CHECK_EQ(d.read_sfr(T1STAT), 0);
        CHECK_EQ(word_sfr(d, T1CC0L), 0);
    }
    run_until(d, rising_edge(100) + 1000);
    CHECK(d.power_mode() == Cc2530PowerMode::Active);
    t1(d).input_changed(0, true);
    CHECK_EQ(d.read_sfr(T1STAT), 0x01);
    CHECK(word_sfr(d, T1CC0L) != 0);
}

// ---------------------------------------------------------------------------
// RSSI: SWRU191F 23.10.3 "The RSSI value is the received power averaged over
// eight symbol periods (128 us)"; RSSISTAT.RSSI_VALID is set once the
// receiver has been enabled for eight symbol periods. 23.8.12: CCA is
// updated at every new RSSI sample, four system-clock cycles later.
// Model-defined: one new average per symbol period, at RSSI_VALID + k * 512
// cycles, over the eight preceding symbol periods; levels are averaged in mW.
// VirtualAir ideal propagation: a frame is heard at -50 dBm over a -100 dBm
// noise floor; the register adds the 73 dB offset (23.10.3).
// ---------------------------------------------------------------------------

// RSSI register for n of the 8 averaged symbol periods at -50 dBm and the
// rest at -100 dBm: round(10 log10((n * 1e-5 + (8 - n) * 1e-10) / 8)) + 73.
constexpr std::array<std::uint8_t, 9> RssiRamp{0xe5, 0x0e, 0x11, 0x13, 0x14,
                                               0x15, 0x16, 0x16, 0x17};

void test_rssi_eight_symbol_average() {
    // scan: FRMCTRL0.ENERGY_SCAN = 1 (23.10.3: the peak RSSI is kept; CCA
    // still uses the current average).
    for (const bool scan : {false, true}) {
        Program p;
        p.xw(CCACTRL0, 0xf8); // CCA threshold -8 (register units); CCA_HYST 2 (reset)
        if (scan)
            p.xw(FRMCTRL0, 0x50); // AUTOCRC (reset) | ENERGY_SCAN
        const std::uint16_t ready = p.here();
        p.sled();
        VirtualAir air;
        VirtualRadioNode peer(air, "peer");
        Cc2530F256 d;
        load(d, p.flash);
        d.attach_air(air, "dut");
        run_to(d, ready, 2000, "rssi ready");
        const std::uint64_t t = 4096;
        if (at(d, t, "rssi rx on"))
            d.write_sfr(RFST, IsRxOn);
        // RX calibration (12 symbol periods, 23.9.2), then 8 more until the
        // first average is complete.
        const std::uint64_t v = t + Turn + RssiValid;
        if (at(d, v - 1, "rssi before valid")) {
            CHECK_EQ(x(d, RSSISTAT), 0x00);
            CHECK_EQ(x(d, RSSI), 0x80);
            CHECK_EQ(x(d, FSMSTAT1) & S1Cca, 0);
        }
        if (at(d, v, "rssi valid")) {
            CHECK_EQ(x(d, RSSISTAT), 0x01);
            CHECK_EQ(x(d, RSSI), 0xe5);
        }
        if (at(d, v + 4, "cca valid"))
            CHECK_EQ(x(d, FSMSTAT1) & S1Cca, S1Cca);

        const std::uint64_t p0 = v + 12 * Symbol;
        const auto m = frame(2, {0x11, 0x22, 0x33, 0x44});
        peer.transmit("", RadioProtocol::Zigbee802154, 11, ns(p0), m);
        // The average rises over the first eight symbol periods of the
        // frame and changes only at the sample instants.
        for (unsigned n = 0; n <= 8; ++n) {
            const std::uint64_t sample = p0 + n * Symbol;
            if (n > 0 && at(d, sample - 1, "rssi before sample"))
                CHECK_EQ(x(d, RSSI), RssiRamp[n - 1]);
            if (at(d, sample, "rssi sample"))
                CHECK_EQ(x(d, RSSI), RssiRamp[n]);
            // The first sample above the threshold clears CCA 4 cycles later.
            if (n == 1) {
                if (at(d, sample + 3, "cca before busy"))
                    CHECK_EQ(x(d, FSMSTAT1) & S1Cca, S1Cca);
                if (at(d, sample + 4, "cca busy"))
                    CHECK_EQ(x(d, FSMSTAT1) & S1Cca, 0);
            }
        }
        // After the frame the average falls over eight symbol periods; the
        // energy scan keeps the peak.
        const std::uint64_t e = p0 + Shr + (1 + m.size() + 2) * Octet;
        if (at(d, e, "rssi frame end")) {
            CHECK_EQ(d.read_sfr(RFIRQF0) & F0RxPktDone, F0RxPktDone);
            CHECK_EQ(x(d, RSSI), 0x17);
        }
        for (unsigned j = 1; j <= 8; ++j) {
            const std::uint64_t sample = e + j * Symbol;
            if (at(d, sample - 1, "rssi decay before"))
                CHECK_EQ(x(d, RSSI), scan ? 0x17 : RssiRamp[9 - j]);
            if (at(d, sample, "rssi decay"))
                CHECK_EQ(x(d, RSSI), scan ? 0x17 : RssiRamp[8 - j]);
        }
        if (at(d, e + RssiValid + 3, "cca before clear"))
            CHECK_EQ(x(d, FSMSTAT1) & S1Cca, 0);
        if (at(d, e + RssiValid + 4, "cca clear"))
            CHECK_EQ(x(d, FSMSTAT1) & S1Cca, S1Cca);
    }
}

// The RSSI appended to a received frame (AUTOCRC = 1): 23.9.7, Figure 23-15
// field descriptions: "The RSSI value is measured over the first eight
// symbols following the SFD." LogDistance propagation: "weak" at 10 m is heard at -60 dBm,
// "strong" at 1 m at -40 dBm. The strong frame starts four symbol periods
// after the weak frame's SFD: half the window at -60 dBm, half at -40 dBm
// (the model hears the strongest frame on the channel) gives
// 10 log10((1e-6 + 1e-4) / 2) = -42.97 dBm, register 30 = 0x1e. The
// strong frame destroys the weak one (SINR -20 dB): CRC_OK = 0.
void test_rssi_appended_average() {
    Program p;
    p.xw(FRMFILT0, 0x00);
    const std::uint16_t ready = p.here();
    p.sled();
    VirtualAir air;
    mcuemu::Propagation prop;
    prop.model = mcuemu::Propagation::Model::LogDistance;
    air.set_propagation(prop);
    air.set_node("dut", {0, 0, 0});
    air.set_node("weak", {10, 0, 0});
    air.set_node("strong", {1, 0, 0});
    VirtualRadioNode weak(air, "weak"), strong(air, "strong");
    Cc2530F256 d;
    load(d, p.flash);
    d.attach_air(air, "dut");
    run_to(d, ready, 2000, "rssi fifo ready");
    const std::uint64_t t = 4096;
    if (at(d, t, "rssi fifo rx on"))
        d.write_sfr(RFST, IsRxOn);
    const std::uint64_t v = t + Turn + RssiValid;
    const std::uint64_t p0 = v + 4 * Symbol;
    const auto m = frame(3, {0x11, 0x22, 0x33});
    weak.transmit("", RadioProtocol::Zigbee802154, 11, ns(p0), m);
    const std::uint64_t s = p0 + Shr;
    strong.transmit("", RadioProtocol::Zigbee802154, 11, ns(s + 4 * Symbol),
                    frame(4, {0x55, 0x66, 0x77, 0x88, 0x99}));
    if (at(d, s, "rssi fifo sfd")) {
        CHECK_EQ(d.read_sfr(RFIRQF0) & F0Sfd, F0Sfd);
        CHECK_EQ(x(d, RSSI), 0x0d); // -60 dBm over the whole window
    }
    const std::uint64_t e = s + (1 + m.size() + 2) * Octet;
    if (at(d, e, "rssi fifo done")) {
        CHECK_EQ(d.read_sfr(RFIRQF0) & F0RxPktDone, F0RxPktDone);
        CHECK_EQ(x(d, RxFifo), m.size() + 2);
        CHECK_EQ(x(d, static_cast<std::uint16_t>(RxFifo + 1 + m.size())), 0x1e);
        CHECK_EQ(x(d, static_cast<std::uint16_t>(RxFifo + 2 + m.size())) & 0x80, 0);
    }
}

// ---------------------------------------------------------------------------
// Clock-loss detector (SWRU191F 5.2, CLD 0x6290) and stop_oscillator().
// "If one of the clocks stops toggling, a clock-loss detector reset is
// generated within a certain maximum time-out period": 0.25 ms for the
// 32 MHz clock, 0.5 ms for the 32 kHz clock; SLEEPSTA.RST = 11 (4.3). "In
// power modes 1 and 2, the clock-loss detector is automatically stopped and
// restarted when the clocks start up again". Model-defined (see
// Cc2530F256::stop_oscillator): the reset comes at exactly the time-out
// after the stop (or after monitoring starts); without the detector the CPU
// stalls with its system clock; a switch to a stopped oscillator never
// completes (4.4.2); the 32 kHz domain freezes with its clock (4.4.3).
// ---------------------------------------------------------------------------

struct ClockProgram {
    Program p;
    std::uint16_t poll{}, wdt{}, ready{};
};

// CLKCONCMD = 0x80: 32 MHz XOSC system clock, 32 kHz RCOSC, TICKSPD and
// CLKSPD 32 MHz; then wait for CLKCONSTA.OSC = 0 (4.4.2), optionally enable
// the detector (CLD.EN) and the watchdog (WDCTL = 0x0b: 64 periods).
ClockProgram clock_program(bool cld, bool wdt) {
    ClockProgram c;
    c.p.mov(CLKCONCMD, 0x80);
    c.poll = c.p.here();
    c.p.op({0xe5, CLKCONSTA, 0x20, 0xe6, 0xfb}); // MOV A,CLKCONSTA; JB ACC.6,poll
    if (cld)
        c.p.xw(XCld, 0x01);
    c.wdt = c.p.here();
    if (wdt)
        c.p.mov(WDCTL, 0x0b);
    c.ready = c.p.here();
    c.p.sled();
    return c;
}

bool in_poll_loop(const Cc2530F256 &d, const ClockProgram &c) {
    const std::uint16_t pc = d.registers().pc;
    return pc >= c.poll && pc < c.poll + 5;
}

void test_clock_loss_xosc() {
    const ClockProgram c = clock_program(true, false);
    Cc2530F256 d;
    load(d, c.p.flash);
    run_to(d, c.ready, 3000, "cld ready");
    CHECK_EQ(d.read_sfr(CLKCONSTA), 0x80);
    CHECK_EQ(x(d, XCld), 0x01);
    const std::uint64_t stop = 4000;
    d.stop_oscillator(Cc2530Oscillator::Xosc32M, stop);
    if (at(d, stop - 1, "cld before stop"))
        CHECK(!d.clock_stalled());
    if (at(d, stop, "cld stop"))
        CHECK(d.clock_stalled());
    const std::uint16_t pc = d.registers().pc;
    const std::uint64_t executed = d.registers().instructions;
    if (at(d, stop + CldXosc - 1, "cld before reset")) {
        CHECK(d.clock_stalled());
        CHECK_EQ(d.registers().pc, pc);
        CHECK_EQ(d.registers().instructions, executed);
        CHECK_EQ(d.diagnostics().clock_stall_cycles, CldXosc - 1);
        CHECK_EQ(later_resets(d), 0u);
    }
    if (at(d, stop + CldXosc, "cld reset")) {
        CHECK_EQ(resets(d, CauseClockLoss), 1u);
        CHECK_EQ(d.read_sfr(SLEEPSTA) & 0x18, RstClockLoss);
        CHECK_EQ(d.read_sfr(CLKCONSTA), 0xc9); // reset: 16 MHz RCOSC
        CHECK_EQ(x(d, XCld), 0x00);
        CHECK_EQ(d.registers().pc, 0);
        CHECK(!d.clock_stalled());
    }
    // The program runs again on the RCOSC; its switch to the stopped XOSC
    // never completes and the detector stays disabled.
    run_until(d, stop + CldXosc + 5000);
    CHECK(in_poll_loop(d, c));
    CHECK_EQ(d.read_sfr(CLKCONCMD), 0x80);
    CHECK_EQ(d.read_sfr(CLKCONSTA), 0xc9);
    CHECK_EQ(later_resets(d), 1u);
    // Model-defined: a power-on reset brings the oscillators back.
    d.power_on_reset();
    run_to(d, c.ready, 3000, "cld after por");
    CHECK_EQ(d.read_sfr(CLKCONSTA), 0x80);
    CHECK_EQ(d.read_sfr(SLEEPSTA) & 0x18, 0);
}

void test_clock_loss_32k() {
    for (const bool cld : {true, false}) {
        const ClockProgram c = clock_program(cld, false);
        Cc2530F256 d;
        load(d, c.p.flash);
        run_to(d, c.ready, 3000, "cld32k ready");
        const std::uint64_t stop = 4000;
        d.stop_oscillator(Cc2530Oscillator::Rcosc32k, stop);
        std::uint32_t frozen = 0;
        if (at(d, stop, "cld32k stop")) {
            frozen = st(d).count();
            CHECK_EQ(frozen, lf_ticks(stop));
        }
        if (at(d, stop + Cld32k - 1, "cld32k before reset")) {
            CHECK_EQ(st(d).count(), frozen);
            CHECK(!d.clock_stalled()); // the CPU runs on the 32 MHz XOSC
            CHECK_EQ(later_resets(d), 0u);
        }
        if (at(d, stop + Cld32k, "cld32k reset")) {
            CHECK_EQ(resets(d, CauseClockLoss), cld ? 1u : 0u);
            CHECK_EQ(later_resets(d), cld ? 1u : 0u);
            if (cld)
                CHECK_EQ(d.read_sfr(SLEEPSTA) & 0x18, RstClockLoss);
            else
                CHECK_EQ(st(d).count(), frozen);
        }
    }
}

// Without the detector the CPU stalls; the watchdog, clocked by the running
// 32 kHz RCOSC (16.1), still resets the chip, which then runs on the RCOSC.
void test_xosc_stop_without_detector() {
    const ClockProgram c = clock_program(false, true);
    Cc2530F256 d;
    load(d, c.p.flash);
    const std::uint64_t w = run_to(d, c.wdt, 3000, "stall wdt");
    run_to(d, c.ready, 3000, "stall ready");
    const std::uint64_t stop = 4000;
    d.stop_oscillator(Cc2530Oscillator::Xosc32M, stop);
    const std::uint64_t reset = period_start(lf_ticks(w) + 64);
    if (at(d, reset - 1, "stall before wdt")) {
        CHECK(d.clock_stalled());
        CHECK_EQ(d.diagnostics().clock_stall_cycles, reset - 1 - stop);
        CHECK_EQ(later_resets(d), 0u);
    }
    if (at(d, reset, "stall wdt reset")) {
        CHECK_EQ(resets(d, CauseWatchdog), 1u);
        CHECK_EQ(d.read_sfr(SLEEPSTA) & 0x18, RstWatchdog);
        CHECK(!d.clock_stalled());
    }
    run_until(d, reset + 5000);
    CHECK(in_poll_loop(d, c));
    CHECK_EQ(later_resets(d), 1u);
    CHECK_EQ(d.diagnostics().clock_stall_cycles, reset - stop);
}

// SLEEPSTA bit 6 (XOSC_STB) and bit 5 (HFRC_STB) are the oscillator-stable flags
// the standard Z-Stack HAL polls after `SLEEPCMD &= ~OSC_PD` (inferred, see
// Cc2530F256::sleepsta_value). A stopped oscillator clears its flag.
void test_sleepsta_oscillator_stable() {
    Program p;
    Cc2530F256 d;
    load(d, p.flash);
    CHECK_EQ(d.read_sfr(SLEEPSTA) & 0x60, 0x60);
    d.stop_oscillator(Cc2530Oscillator::Xosc32M, 100);
    run_until(d, 200);
    CHECK_EQ(d.read_sfr(SLEEPSTA) & 0x60, 0x20);
}

// A switch to an already stopped XOSC never completes (4.4.2); with the
// detector enabled, monitoring of the new selection starts at the CLKCONCMD
// write and the reset follows the time-out later. The program waits in Idle
// (exact virtual time); the write instant lies within the writing
// instruction, between the two breakpoints.
void test_switch_to_stopped_xosc() {
    Program p;
    p.xw(XCld, 0x01);
    const std::uint16_t write = p.here();
    p.mov(CLKCONCMD, 0x80);
    const std::uint16_t after = p.here();
    p.enter_power_mode(SleepIdle).sled();
    Cc2530F256 d;
    load(d, p.flash);
    d.stop_oscillator(Cc2530Oscillator::Xosc32M, 0);
    const std::uint64_t c0 = run_to(d, write, 3000, "switch write");
    const std::uint64_t c1 = run_to(d, after, 3000, "switch after");
    if (at(d, c0 + CldXosc - 1, "switch before reset")) {
        CHECK(d.power_mode() == Cc2530PowerMode::Idle);
        CHECK_EQ(d.read_sfr(CLKCONCMD), 0x80);
        CHECK_EQ(d.read_sfr(CLKCONSTA), 0xc9);
        CHECK(!d.clock_stalled());
        CHECK_EQ(later_resets(d), 0u);
    }
    std::uint64_t reset = 0;
    for (std::uint64_t t = c0 + CldXosc; t <= c1 + CldXosc && reset == 0; ++t)
        if (at(d, t, "switch reset scan") && resets(d, CauseClockLoss) != 0)
            reset = t;
    CHECK(reset != 0);
    CHECK_EQ(d.read_sfr(SLEEPSTA) & 0x18, RstClockLoss);
}

// Model-defined (docs/cc2530.md "Clock loss"): a stopped oscillator stands
// for a damaged crystal, so a power cut keeps it stopped and the switch to it
// never completes; the power-on reset of a firmware load brings it back.
void test_power_cut_keeps_stopped_oscillator() {
    const ClockProgram c = clock_program(false, false);
    Cc2530F256 d;
    load(d, c.p.flash);
    d.stop_oscillator(Cc2530Oscillator::Xosc32M, 0);
    d.power_cut();
    CHECK_EQ(d.read_sfr(SLEEPSTA) & 0x18, 0x00); // power-on
    run_until(d, 3000);
    CHECK(in_poll_loop(d, c));
    CHECK_EQ(d.read_sfr(CLKCONSTA), 0xc9);
    CHECK_EQ(d.diagnostics().power_cuts, 1u);

    load(d, c.p.flash);
    run_to(d, c.ready, 3000, "load after cut");
    CHECK_EQ(d.read_sfr(CLKCONSTA), 0x80);
}

// 5.2: in PM1/PM2 the detector "is automatically stopped and restarted when
// the clocks start up again". The XOSC stops while the chip is in PM1/PM2:
// no reset there; the wake (Sleep Timer compare 100) restarts monitoring
// and the chip, back on the RCOSC (4.1), waits in Idle for the reset.
void test_clock_loss_suspended_in_pm1_pm2() {
    for (const bool pm1 : {true, false}) {
        ClockProgram c = clock_program(true, false);
        Program &p = c.p;
        p.flash.at(c.ready);
        p.mov(ST2, 0).mov(ST1, 0).mov(ST0, 100).mov(IEN0, 0xa0);
        p.enter_power_mode(pm1 ? SleepPm1 : SleepPm2).enter_power_mode(SleepIdle).sled();
        p.isr(VSt, {0xc2, BitStif, 0x32});
        Cc2530F256 d;
        load(d, p.flash);
        d.stop_oscillator(Cc2530Oscillator::Xosc32M, 50000);
        if (at(d, 90000, "cld pm1/pm2")) {
            CHECK(d.power_mode() == (pm1 ? Cc2530PowerMode::Pm1 : Cc2530PowerMode::Pm2));
            CHECK_EQ(later_resets(d), 0u);
        }
        const std::uint64_t wake = rising_edge(100);
        if (at(d, wake + 2000, "cld pm idle")) {
            CHECK(d.power_mode() == Cc2530PowerMode::Idle);
            CHECK_EQ(d.read_sfr(CLKCONSTA) & 0x40, 0x40);
            CHECK_EQ(d.diagnostics().power_mode_wakes, 1u);
        }
        if (at(d, wake + CldXosc - 1, "cld pm before reset"))
            CHECK_EQ(later_resets(d), 0u);
        if (at(d, wake + CldXosc, "cld pm reset")) {
            CHECK_EQ(resets(d, CauseClockLoss), 1u);
            CHECK_EQ(d.read_sfr(SLEEPSTA) & 0x18, RstClockLoss);
        }
    }
}

// 5.2: "Before entering power mode 3, switch to the 16-MHz RCOSC and disable
// the clock-loss detector." Model-defined: PM3 with CLD.EN = 1 is counted and the detector treated as
// stopped, as in PM1/PM2; the P0 wake restarts it.
void test_clock_loss_pm3_entry() {
    ClockProgram c = clock_program(true, false);
    Program &p = c.p;
    p.flash.at(c.ready);
    p.mov(IEN1, 0x20).mov(IEN0, 0x80);
    p.enter_power_mode(SleepPm3).sled();
    p.isr(VP0, {0xc2, BitP0if, 0x32});
    Cc2530F256 d;
    load(d, p.flash);
    d.stop_oscillator(Cc2530Oscillator::Xosc32M, 20000);
    if (at(d, 60000, "cld pm3")) {
        CHECK(d.power_mode() == Cc2530PowerMode::Pm3);
        CHECK_EQ(d.diagnostics().clock_loss_pm3_entries, 1u);
        CHECK_EQ(later_resets(d), 0u);
    }
    d.schedule_interrupt_flag_for_test(60000, 13); // P0IF
    if (at(d, 60000 + CldXosc - 1, "cld pm3 before reset")) {
        CHECK(d.power_mode() == Cc2530PowerMode::Active);
        CHECK_EQ(later_resets(d), 0u);
    }
    if (at(d, 60000 + CldXosc, "cld pm3 reset"))
        CHECK_EQ(resets(d, CauseClockLoss), 1u);
}

// Only the selected clocks are watched (5.2: "It is assumed that the 32-MHz
// XOSC is selected as system clock source"; the 32 kHz clock is the selected
// 32 kHz RCOSC or XOSC; Model-defined: the 16 MHz RCOSC is not watched): stopping the 32 kHz XOSC
// and the 16 MHz RCOSC while the chip runs on the 32 MHz XOSC and the
// 32 kHz RCOSC changes nothing; neither does stopping the XOSC while the
// RCOSC is the system clock.
void test_unselected_oscillators_not_watched() {
    {
        const ClockProgram c = clock_program(true, false);
        Cc2530F256 d;
        load(d, c.p.flash);
        run_to(d, c.ready, 3000, "unwatched ready");
        d.stop_oscillator(Cc2530Oscillator::Xosc32k, 4000);
        d.stop_oscillator(Cc2530Oscillator::Rcosc16M, 4000);
        if (at(d, 40000, "unwatched")) {
            CHECK_EQ(later_resets(d), 0u);
            CHECK(!d.clock_stalled());
            CHECK_EQ(st(d).count(), lf_ticks(40000));
        }
    }
    {
        Program p;
        p.xw(XCld, 0x01).sled();
        Cc2530F256 d;
        load(d, p.flash);
        d.stop_oscillator(Cc2530Oscillator::Xosc32M, 4000);
        if (at(d, 40000, "unwatched rcosc")) {
            CHECK_EQ(d.read_sfr(CLKCONSTA), 0xc9);
            CHECK_EQ(later_resets(d), 0u);
            CHECK(!d.clock_stalled());
        }
    }
}

// Fast, step and trace runs agree on a detector reset and on a stall ended
// by the watchdog.
void test_clock_loss_modes() {
    for (const bool cld : {true, false}) {
        const ClockProgram c = clock_program(cld, !cld);
        for_each_mode(
            c.p.flash, cld ? 30000 : 80000,
            [](Cc2530F256 &d) { d.stop_oscillator(Cc2530Oscillator::Xosc32M, 3000); },
            [cld](Cc2530F256 &d, Mode) {
                CHECK_EQ(resets(d, cld ? CauseClockLoss : CauseWatchdog), 1u);
                CHECK_EQ(later_resets(d), 1u);
            },
            cld ? "clock loss modes" : "clock stall modes");
    }
}

// Radio paths (8-symbol RSSI, CCA, RF-interrupt capture) agree in fast, step
// and trace runs. The program enables RX itself.
void test_radio_gap_modes() {
    Program p;
    p.xw(FRMFILT0, 0x00).xw(RFIRQM0, F0Sfd | F0RxPktDone).xw(FRMCTRL0, 0x50);
    p.mov(T1CCTL1, 0xc1).mov(T1CCTL2, 0x82).mov(T1CTL, 0x01).mov(RFST, IsRxOn);
    p.sled();
    std::vector<std::unique_ptr<VirtualAir>> airs;
    std::vector<std::unique_ptr<VirtualRadioNode>> peers;
    for_each_mode(
        p.flash, 45000,
        [&](Cc2530F256 &d) {
            airs.push_back(std::make_unique<VirtualAir>());
            peers.push_back(std::make_unique<VirtualRadioNode>(*airs.back(), "peer"));
            d.attach_air(*airs.back(), "dut");
            peers.back()->transmit("", RadioProtocol::Zigbee802154, 11, ns(20480),
                                   frame(5, {0x01, 0x02, 0x03, 0x04}));
        },
        [](Cc2530F256 &d, Mode) {
            CHECK_EQ(d.read_sfr(T1STAT) & 0x02, 0x02);
            CHECK_EQ(d.read_sfr(RFIRQF0) & (F0Sfd | F0RxPktDone), F0Sfd | F0RxPktDone);
            CHECK_EQ(x(d, RSSI), 0x17);
        },
        "radio gap modes");
}

} // namespace

int main() {
    test_ircon6_requests_sleep_timer_interrupt();
    test_ircon6_wake_rules();
    test_wdt_holds_in_pm1();
    test_wdt_holds_in_pm3();
    test_t1_capture_from_rf_interrupt();
    test_t1_no_capture_in_power_mode();
    test_rssi_eight_symbol_average();
    test_rssi_appended_average();
    test_clock_loss_xosc();
    test_clock_loss_32k();
    test_xosc_stop_without_detector();
    test_sleepsta_oscillator_stable();
    test_switch_to_stopped_xosc();
    test_power_cut_keeps_stopped_oscillator();
    test_clock_loss_suspended_in_pm1_pm2();
    test_clock_loss_pm3_entry();
    test_unselected_oscillators_not_watched();
    test_clock_loss_modes();
    test_radio_gap_modes();
    return finish("cc2530 minor gaps");
}
