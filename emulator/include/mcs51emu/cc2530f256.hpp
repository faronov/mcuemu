#ifndef MCS51EMU_CC2530F256_HPP
#define MCS51EMU_CC2530F256_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

struct uc_struct;

namespace mcuemu {
class VirtualAir;
}

namespace mcs51emu {

namespace cc2530 {
class Peripheral;
struct Soc;
} // namespace cc2530

// Active: CPU running. Idle: PCON.IDLE with SLEEPCMD.MODE = 0 (all clocks
// run, CPU stopped). Pm1-Pm3: PCON.IDLE with SLEEPCMD.MODE 1-3 (SWRU191F
// 4.1): Pm1/Pm2 stop the 32 MHz XOSC and 16 MHz RCOSC (the 32 kHz domain
// keeps running), Pm3 also stops the 32 kHz domain.
enum class Cc2530PowerMode : std::uint8_t { Active, Idle, Pm1, Pm2, Pm3 };
// SLEEPSTA.RST encoding (SWRU191F 4.3).
enum class Cc2530ResetCause : std::uint8_t { PowerOn = 0, External = 1, Watchdog = 2, ClockLoss = 3 };
// Oscillators a host can declare stopped (Cc2530F256::stop_oscillator):
// the 32 MHz crystal and 16 MHz RC system-clock sources and the two 32 kHz
// clock sources (SWRU191F 4.4.1).
enum class Cc2530Oscillator : std::uint8_t { Xosc32M, Rcosc16M, Xosc32k, Rcosc32k };
// What the chip does to a pad.
enum class Cc2530PinDrive : std::uint8_t { HiZ, Low, High, PullUp, PullDown };

// One UART frame with line detail beyond the byte (SWRU191F 17.1.4): the
// 9th bit, stop-bit errors and a break. `data` is the byte as an LSB-first
// receiver sees it on the wire (the convention of plain host bytes). A
// default frame is a plain byte: well formed in the receiver's format.
struct Cc2530UartFrame {
    enum class Ninth : std::uint8_t {
        Expected,  // received: the 9th bit the receiver expects (parity or
                   // D9), if it uses one; transmitted: BIT9 = 0, no 9th bit
        Zero,      // a 9th bit of 0 on the wire
        One,       // a 9th bit of 1 on the wire
        BadParity, // received: the inverse of the expected 9th bit
    };
    std::uint8_t data{};
    Ninth ninth{Ninth::Expected};
    bool stop1_error{}; // the first stop bit is at the wrong level
    bool stop2_error{}; // the second stop bit is at the wrong level
    // Break: the line stays at the start-bit level for the whole frame.
    // The other fields are ignored.
    bool line_break{};
    bool operator==(const Cc2530UartFrame &) const = default;
};

struct Cc2530RunResult {
    std::size_t instructions{};
    std::uint16_t pc{};
    std::uint8_t fmap{};
    std::uint64_t cycles{};
    bool step_limit_reached{};
    bool break_hit{};
    // The CPU is in the power mode entered through PCON.IDLE: no enabled
    // interrupt request is pending and no scheduled event within the cycle
    // limit has woken it (see Cc2530F256::run).
    bool sleeping{};
    // At the return: the debug interface holds the CPU halted (SWRU191F
    // 3.3), or RESET_N is held low (Cc2530F256::set_reset_pin).
    bool debug_halted{};
    bool reset_held{};
    std::string error; // CPU exception or unsupported SoC behavior
};

// Response to one debug-interface command (SWRU191F Table 3-1).
struct Cc2530DebugIfResponse {
    std::vector<std::uint8_t> bytes;
    // False: the command was undefined or refused (DEBUG_LOCKED, a chip
    // erase in progress, a power mode, CPU not halted, ...); the response
    // then repeats the status byte (model-defined).
    bool accepted{};
    std::uint64_t cycle{}; // virtual time at which it was executed
};

// A firmware image as Cc2530F256::load_firmware reads it.
struct Cc2530FlashImage {
    std::vector<std::uint8_t> bytes; // 256 KiB, 0xff where not programmed
    std::vector<bool> programmed;    // bytes the image file defines
};

struct Cc2530Registers {
    std::uint16_t pc{};
    std::uint8_t a{}, b{}, psw{}, sp{}, dpl{}, dph{};
    std::array<std::uint8_t, 8> r{}; // the bank selected by PSW.RS
    std::uint8_t fmap{}, memctr{}, mpage{};
    std::uint64_t cycles{}, instructions{};
    // DPTR1 and the data pointer select (DPL/DPH above are DPTR0).
    std::uint8_t dpl1{}, dph1{}, dps{};
    // Interrupt controller: bit L of interrupt_levels is set while an ISR of
    // priority level L is in progress; interrupt_inhibit is the one
    // instruction after RETI during which no interrupt is taken. The posted
    // request is the one the core takes at the next boundary if its level
    // allows (vector 0 and source -1: none).
    std::uint8_t interrupt_levels{};
    bool interrupt_inhibit{};
    std::uint16_t posted_vector{};
    std::uint8_t posted_level{};
    int posted_source{-1};
};

// ---- Flash fault model (docs/cc2530.md "Flash fault injection") ----
// SWRU191F 6.2.2 multiple-write limits since the last page erase (a 0 written
// at most twice to a bit, a 32-bit word written at most 8 times, a page at
// most 1024 times; otherwise "any bit of a 32-bit flash word" becomes
// nondeterministic). Report: violations are only counted, programming stays a
// deterministic AND. Nondeterministic: model-defined fault injection, a
// violating word takes a seeded pseudo-random value (not hardware-accurate).
enum class Cc2530FlashLimitPolicy : std::uint8_t { Report, Nondeterministic };
// Result of a page erase or word write cut by a reset or a power cut.
// SWRU191F does not specify it; every outcome is model-defined. Abandon: no
// flash change. Partial: a seeded subset of the bits the operation changes
// lands, in proportion to the elapsed fraction of the operation time.
// Complete: the full effect is applied.
enum class Cc2530FlashInterrupted : std::uint8_t { Abandon, Partial, Complete };
struct Cc2530FlashFaultConfig {
    Cc2530FlashLimitPolicy limit_policy{Cc2530FlashLimitPolicy::Report};
    Cc2530FlashInterrupted interrupted{Cc2530FlashInterrupted::Abandon};
    // Seeds every pseudo-random outcome (same seed, same results).
    std::uint64_t seed{0};
    // SRAM content after a power-on reset or power cut (SWRU191F leaves it
    // undefined); nullopt: seeded pseudo-random bytes.
    std::optional<std::uint8_t> sram_fill{std::uint8_t{0}};
    // Record every flash operation in Cc2530F256::flash_log().
    bool log{false};
};
enum class Cc2530FlashOpKind : std::uint8_t { PageErase, WordWrite, ChipErase };
enum class Cc2530FlashOutcome : std::uint8_t {
    Completed, // effect applied when the operation time elapsed
    Aborted,   // page locked: FCTL.ABORT, no effect
    Abandoned, // cut: no effect
    Partial,   // cut: a seeded subset of the bits changed
    Forced,    // cut: full effect applied
};
// What ended an operation before its time.
enum class Cc2530FlashCut : std::uint8_t { None, Reset, PowerCut };
// 6.2.2 limits broken by a word write (Cc2530FlashLogEntry::violations).
inline constexpr std::uint8_t Cc2530FlashBitLimit = 0x01;  // 3rd+ 0-write to a bit
inline constexpr std::uint8_t Cc2530FlashWordLimit = 0x02; // 9th+ write to a word
inline constexpr std::uint8_t Cc2530FlashPageLimit = 0x04; // 1025th+ write to a page
struct Cc2530FlashLogEntry {
    std::uint64_t cycle{}; // virtual time at which the outcome was decided
    Cc2530FlashOpKind op{};
    std::uint32_t address{}; // physical byte address (page start for an erase)
    unsigned page{};
    Cc2530FlashOutcome outcome{};
    Cc2530FlashCut cut{};
    Cc2530ResetCause reset_cause{}; // for cut == Reset
    std::uint64_t elapsed_cycles{}; // operation time that had elapsed
    std::uint32_t changed_bits{};   // flash bits the operation changed
    std::uint8_t violations{};      // Cc2530Flash*Limit bits
};
enum class Cc2530FaultAction : std::uint8_t { PowerCut, Reset };
enum class Cc2530FlashOpFilter : std::uint8_t { Any, Erase, Write };
// Fires `action` `delay_cycles` of HF time after the start of the nth page
// erase or word write (counted from when the trigger is added, locked pages
// excluded) that matches `op` and `page`. Fires once.
struct Cc2530FlashTrigger {
    Cc2530FlashOpFilter op{Cc2530FlashOpFilter::Any};
    std::uint64_t nth{1};
    std::optional<unsigned> page;
    std::uint64_t delay_cycles{0};
    Cc2530FaultAction action{Cc2530FaultAction::PowerCut};
};

// Counters for accesses the model accepts but does not emulate, so a run can
// show which unmodeled hardware the firmware touched.
struct Cc2530Diagnostics {
    std::uint64_t unmapped_xdata_reads{};
    std::uint64_t unmapped_xdata_writes{};
    std::uint64_t read_only_xdata_writes{}; // info page, XBANK, read-only XREGs, P0-P2 alias
    std::uint64_t read_only_sfr_writes{};   // CLKCONSTA, SLEEPSTA
    std::uint64_t fmap_writes{};
    std::uint64_t memctr_writes{};
    std::uint64_t interrupts_taken{};
    // Flags set through raise_interrupt_flag_for_test (not by a peripheral).
    std::uint64_t test_injected_flags{};
    // Flags set by the virtual-time event queue (schedule_interrupt_flag_for_test).
    std::uint64_t scheduled_flag_events{};
    // Peripheral service() calls made by the deadline scheduler.
    std::uint64_t peripheral_services{};
    // Interrupt-delivery contract check (R1-F10). Counts instruction
    // boundaries at which the request posted to the core differed from the
    // one recomputed from the flag, enable and priority SFRs, i.e. a change
    // that bypassed update_interrupts(). The boundary check repairs the post,
    // so trace mode and single-stepping would otherwise mask such a defect.
    // Must stay 0.
    std::uint64_t stale_interrupt_posts{};
    // PCON.IDLE. Entries count every write of PCON.IDLE = 1; wakes include
    // immediate ones (a waking request already pending at the write).
    // idle_cycles is the virtual time spent in any power mode.
    std::uint64_t power_mode_entries{};
    std::uint64_t power_mode_wakes{};
    std::uint64_t power_mode_immediate_wakes{};
    std::uint64_t idle_cycles{};
    // Per Cc2530PowerMode (index 0, Active, unused): entries and virtual time.
    std::array<std::uint64_t, 5> power_mode_entries_by_mode{};
    std::array<std::uint64_t, 5> power_mode_cycles_by_mode{};
    // Resets per Cc2530ResetCause, including the power-on reset of a load.
    std::array<std::uint64_t, 4> resets_by_cause{};
    // CPU stalls requested by the flash controller and their virtual time.
    std::uint64_t cpu_stalls{};
    std::uint64_t cpu_stall_cycles{};
    // Virtual time the CPU waited for a stopped system clock
    // (stop_oscillator), and PM3 entries with CLD.EN = 1, which SWRU191F 5.2
    // tells firmware to avoid (the detector is then treated as stopped).
    std::uint64_t clock_stall_cycles{};
    std::uint64_t clock_loss_pm3_entries{};
    // Entries that break the SWRU191F 4.2 placement rule: the instruction
    // after the one setting PCON.IDLE starts on a 4-byte boundary.
    std::uint64_t idle_alignment_violations{};
    // Writes of 1 to IRCON bit 6 (SWRU191F: "Must be written 0. Writing a 1
    // always enables the interrupt source"; modeled as a sleep timer
    // interrupt request, Figure 2-4).
    std::uint64_t ircon_bit6_writes{};
    // Debug interface: virtual time with the CPU halted by it, with RESET_N
    // held low, and DEBUG_INSTR instructions executed (not counted in
    // registers().instructions).
    std::uint64_t debug_halted_cycles{};
    std::uint64_t reset_pin_cycles{};
    std::uint64_t debug_instructions{};
    // XREGs no peripheral model owns (plain latches).
    std::map<std::uint16_t, std::uint64_t> unmodeled_xreg_reads;
    std::map<std::uint16_t, std::uint64_t> unmodeled_xreg_writes;
    // Accesses a peripheral model accepts without emulating the selected
    // function (key: XREG address, or 0x7000 | SFR address).
    std::map<std::uint16_t, std::uint64_t> unmodeled_feature_reads;
    std::map<std::uint16_t, std::uint64_t> unmodeled_feature_writes;
    std::vector<std::string> load_warnings;
    // Flash (SWRU191F 6): erases applied in full and word writes applied
    // (also partially) per 2 KiB page, operations cut by a reset or power
    // cut (any outcome), and word writes breaking each 6.2.2 limit.
    std::array<std::uint64_t, 128> flash_page_erases{};
    std::array<std::uint64_t, 128> flash_page_writes{};
    std::uint64_t flash_interrupted_erases{};
    std::uint64_t flash_interrupted_writes{};
    std::uint64_t flash_bit_limit_violations{};
    std::uint64_t flash_word_limit_violations{};
    std::uint64_t flash_page_limit_violations{};
    // Power cuts (power_cut(), scheduled or triggered) and the last one's time.
    std::uint64_t power_cuts{};
    std::uint64_t last_power_cut_cycle{};
};

// TI CC2530F256 SoC around the generic Unicorn MCS-51 core (SWRU191F).
//
// CODE has three identities that must not be confused: the linker's virtual
// address, the 16-bit CPU logical address, and the physical flash offset. The
// lower CODE half (0x0000-0x7fff) is always physical bank 0; the upper half
// shows bank FMAP, i.e. physical FMAP * 0x8000 + (logical - 0x8000). With
// MEMCTR.XMAP set, logical 0x8000-0x9fff is the 8 KiB SRAM instead.
//
// Modeled: flash, SRAM and its DATA/IDATA/XDATA aliases, FMAP, MEMCTR
// (XBANK and XMAP), MPAGE (MOVX @Ri page), the dual data pointer (DPS,
// DPTR0/DPTR1), the XDATA map (XREG, SFR alias, info page, XBANK), the
// 18-source interrupt controller with its four priority levels, priority
// groups, polling order, nesting and the post-RETI instruction rule, the
// CLKCONCMD -> CLKCONSTA mirror, power modes PM0-PM3 (PCON.IDLE with
// SLEEPCMD.MODE), resets, a deterministic virtual-time interrupt-event queue
// and the peripherals below (docs/cc2530.md lists them and their limits).
//
// Timing contract (docs/cc2530.md "Timing contract"):
// registers().cycles is the virtual time base, one cycle per 32 MHz CPU
// clock. An instruction advances it by its SWRU191F Table 2-3 count (MCS51
// core cycle model UC_MCS51_CYCLE_MODEL_CC253X), the best case without flash
// or XDATA wait states, the same count for a conditional branch taken or
// not. An interrupt entry advances it by 7 (SWRU191F 2.5.2: one cycle to
// detect the request plus the 6-cycle LCALL) and counts as one instruction.
// Time spent in a power mode advances cycles but not instructions.
// CLKCONCMD does not change the rate.
//
// Interrupt-delivery contract: every change to an interrupt flag, enable or
// priority SFR, whether by firmware, host, test hook or scheduled event, goes
// through sfr_write()/set_interrupt_flags() and update_interrupts(), which
// post the resulting request to the core before the next instruction
// boundary. The core evaluates the posted request at every instruction
// boundary in every execution mode.
//
// Peripherals (src/mcs51/cc2530_*.hpp) own their SFRs and XREGs; the core
// dispatches CPU, DMA and host accesses to the owner and runs a deadline
// scheduler: a peripheral's deadline() is serviced at the first instruction
// boundary at which cycles >= deadline, in every execution mode, exactly
// like the test events above (test events first at equal times, then
// peripherals in construction order). XREGs no model owns stay plain
// latches and are counted in diagnostics().unmodeled_xreg_*.
class Cc2530F256 {
public:
    static constexpr std::size_t FlashSize = 256 * 1024;
    static constexpr std::size_t BankSize = 0x8000;
    static constexpr unsigned BankCount = 8;
    static constexpr std::size_t SramSize = 8 * 1024;
    static constexpr std::uint16_t IdataXdataAlias = 0x1f00;
    // cc2530-zigbee keeps NV pages 125/126 and the lock/config page 127 out of
    // CODE images (docs/BANKED_CODE.md); the lock bits are the last 16 bytes.
    static constexpr std::uint32_t ReservedFlashStart = 0x3e800;
    static constexpr std::uint32_t LockBitsStart = 0x3fff0;
    static constexpr std::uint16_t InfoPageStart = 0x7800;
    static constexpr std::uint16_t IeeeAddressOffset = 0x780c;

    Cc2530F256();
    ~Cc2530F256();
    Cc2530F256(const Cc2530F256 &) = delete;
    Cc2530F256 &operator=(const Cc2530F256 &) = delete;

    // Intel HEX with physical flash addresses (type 02 or 04 extended address
    // records for banks 1-7; llvm-objcopy emits 02 below 1 MiB) or a raw binary
    // that starts at physical address 0. ELF images are rejected: convert them
    // with llvm-objcopy -O ihex, which uses the LMAs.
    // Erases the flash first, then performs a power-on reset.
    void load_firmware(const std::string &path);
    // Erases the flash, programs `image` at physical `address` and performs a
    // power-on reset (used by tests).
    void load_flash(std::uint32_t address, const std::vector<std::uint8_t> &image);
    // Parses an image file the way load_firmware does, without loading it.
    static Cc2530FlashImage read_firmware_image(const std::string &path);

    // Power-on reset: virtual time and SRAM cleared (SRAM power-on content is
    // undefined; the model fills it with flash_fault_config().sram_fill, 0 by
    // default), then a reset. A flash operation in flight ends with the
    // configured interrupted outcome. reset() keeps SRAM content and virtual
    // time, like a watchdog or external reset (SLEEPSTA.RST = 01).
    void power_on_reset();
    void reset();

    // Runs at most `instruction_limit` instructions, stopping at the first
    // instruction boundary where the cycle counter has reached `cycle_limit`,
    // before executing a breakpoint (except as the first instruction of this
    // call), when PCON enters a power mode, or on a fault.
    //
    // Scheduled events are applied at the first instruction boundary at which
    // cycles >= their time, before the core evaluates interrupts there; all
    // events due at the boundary where run() returns have been applied.
    //
    // In a power mode no instruction executes. Virtual time advances to the
    // next scheduled event, or to `cycle_limit` if that comes first (a finite
    // limit only), and every enabled pending request (IEN0.EA, its enable bit
    // and its flag set, whatever its priority) wakes the CPU. The instruction
    // after the one that set PCON.IDLE then executes before any ISR
    // (SWRU191F 4.2), also when the request was already pending at the
    // PCON write. If virtual time advanced in the power mode during this
    // call, run() may return at `cycle_limit` without executing an
    // instruction.
    Cc2530RunResult run(std::size_t instruction_limit,
                        std::uint64_t cycle_limit = UINT64_MAX);
    void set_breakpoint(std::optional<std::uint16_t> logical_pc) noexcept {
        breakpoint_ = logical_pc;
    }
    // Called at the end of every chip reset (power-on, power cut, external,
    // watchdog, clock loss), also inside run(), with the cause.
    void set_reset_listener(std::function<void(Cc2530ResetCause)> listener) {
        reset_listener_ = std::move(listener);
    }

    // ---- RESET_N and the two-wire debug interface (SWRU191F Chapter 3) ----
    // Not related to set_breakpoint() above, which is the emulator's own
    // breakpoint. The debug interface works on the pads: DC is P2.2 and DD
    // is P2.1 (set_pin_input / pin_drive), see docs/cc2530.md.
    //
    // Level on the RESET_N pin (true: high, released). Low resets the chip
    // (SLEEPSTA.RST = External) and holds it in reset; releasing it starts
    // the CPU at 0x0000, or enters debug mode with the CPU halted if DC
    // fell twice while RESET_N was low (3.1). Inside run() the change is
    // applied at the next instruction boundary.
    void set_reset_pin(bool high);
    bool reset_pin_held() const noexcept { return reset_held_; }
    // The RESET_N / DC entry sequence in one step: an external reset, then
    // debug mode with the CPU halted at 0x0000.
    void debug_if_enter();
    bool debug_if_active() const;
    bool debug_if_halted() const;
    // DBGLOCK as latched at the last reset (DEBUG_LOCKED, 3.4.1).
    bool debug_if_locked() const;
    std::uint8_t debug_if_status() const; // Table 3-3
    std::uint8_t debug_if_config() const; // Table 3-2
    // Command-level host port (model-defined): queues one complete command
    // whose bytes reach the interface one every 128 cycles (DC at 2 MHz)
    // from now; run() processes it, and debug_if_take_response() returns
    // the response once it is complete. Throws std::logic_error when not in
    // debug mode or while a command is outstanding or the DC/DD wire is in
    // a command, std::invalid_argument for a malformed command.
    void debug_if_submit(const std::vector<std::uint8_t> &command);
    std::optional<Cc2530DebugIfResponse> debug_if_take_response();
    // Cycle at which the last submitted command's final byte arrives; its
    // response is complete then (STEP_INSTR: after the stepped instruction).
    std::uint64_t debug_if_due_cycle() const;
    // debug_if_submit(), then run() until the response is available. Throws
    // std::runtime_error on a run error or without a response within one
    // second of virtual time.
    Cc2530DebugIfResponse debug_if_command(const std::vector<std::uint8_t> &command);
    // Table 3-1 lengths: input bytes including the command byte (for
    // BURST_WRITE pass the second byte too), output bytes.
    static unsigned debug_if_input_length(std::uint8_t command, std::uint8_t second = 0);
    static unsigned debug_if_output_length(std::uint8_t command);

    // An unconditional branch to itself with no interrupt deliverable and no
    // scheduled event left: nothing can make the CPU leave it.
    bool idle_forever() const;
    // A scheduled event, pending reset or peripheral may still raise an
    // interrupt or reset; false: nothing on the chip can end a power mode.
    bool may_wake() const;

    // Side-effect-free inspection.
    std::uint8_t read_sfr(std::uint8_t address) const;
    std::uint8_t read_idata(std::uint8_t address) const;
    std::uint8_t peek_xdata(std::uint16_t address) const;
    std::uint8_t read_code(std::uint16_t logical) const;
    std::uint8_t read_flash(std::uint32_t physical) const;
    // Physical flash offset behind a logical CODE address, or nullopt when
    // XMAP maps it to SRAM.
    std::optional<std::uint32_t> code_physical(std::uint16_t logical) const;
    Cc2530Registers registers() const;
    std::uint16_t pc() const;
    std::uint8_t fmap() const noexcept { return sfr_[0x9f]; }
    std::uint8_t memctr() const noexcept { return sfr_[0xc7]; }
    // In a power mode entered through PCON.IDLE (see Cc2530RunResult).
    bool sleeping() const noexcept { return sleep_requested_; }
    // The power mode in effect (Active while running; the mode requested by
    // the PCON write becomes effective at the following instruction boundary).
    Cc2530PowerMode power_mode() const noexcept { return power_mode_; }
    // Between an interrupt vector entry and its RETI (any priority level).
    bool in_interrupt() const { return engine_levels() != 0; }

    // Host-side writes. write_sfr applies the register's side effects.
    void write_sfr(std::uint8_t address, std::uint8_t value);
    void write_sram(std::uint16_t address, std::uint8_t value);
    void set_ieee_address(std::uint64_t address);

    // TEST ONLY. Sets the CPU interrupt flag of SWRU191F Table 2-5 source
    // `source` (0..17) exactly as a firmware write to that flag register
    // would, and nothing else: no timer, DMA, radio or other peripheral state
    // is touched, so it does not pretend a peripheral raised the event.
    // Counted in diagnostics().test_injected_flags. Not reachable through
    // the system layer, JSON-RPC or MCP. Throws std::out_of_range for
    // source > 17.
    void raise_interrupt_flag_for_test(unsigned source);
    // TEST ONLY. As above, but sets only the bits of `flag_mask` (a non-empty
    // subset of the source's flag bits, e.g. ENCIF_1 = 0x02 of S0CON).
    // Throws std::invalid_argument for any other mask.
    void raise_interrupt_flag_for_test(unsigned source, std::uint8_t flag_mask);

    // TEST ONLY until peripherals exist. Deterministic asynchronous interrupt
    // event: at the first instruction boundary at which the cycle counter is
    // >= `at_cycle` (immediately at the next boundary if it already is), the
    // flag bits `flag_mask` of `source` are set through the same path a
    // peripheral uses, without any firmware SFR write. Events due at the same
    // boundary apply in (at_cycle, scheduling order). A reset discards all
    // events. Counted in diagnostics().scheduled_flag_events.
    void schedule_interrupt_flag_for_test(std::uint64_t at_cycle, unsigned source);
    void schedule_interrupt_flag_for_test(std::uint64_t at_cycle, unsigned source,
                                          std::uint8_t flag_mask);
    std::size_t scheduled_event_count() const noexcept { return events_.size(); }
    std::optional<std::uint64_t> next_event_cycle() const noexcept {
        if (events_.empty())
            return std::nullopt;
        return events_.front().at;
    }

    // Earliest virtual time at which a scheduled test event or a peripheral
    // needs service (UINT64_MAX: none).
    std::uint64_t next_deadline() const;

    // ---- host side of the peripheral models ----
    // Pads: `port` 0-2, `pin` 0-7 (P2 has pins 0-4).
    // External digital level on a pad; nullopt: not driven by the host.
    void set_pin_input(unsigned port, unsigned pin, std::optional<bool> level);
    // External analog voltage on a pad (ADC, comparator, op-amp inputs).
    void set_pin_analog_mv(unsigned port, unsigned pin, std::optional<std::uint32_t> millivolts);
    Cc2530PinDrive pin_drive(unsigned port, unsigned pin) const;
    // Level seen by the pad's input buffer.
    bool pin_level(unsigned port, unsigned pin) const;
    // Called on every change of a pad's drive, with the virtual time.
    void set_pin_listener(
        std::function<void(unsigned port, unsigned pin, Cc2530PinDrive drive, std::uint64_t cycle)>
            listener);
    void set_temperature_celsius(double celsius);
    void set_vdd_mv(std::uint32_t millivolts);
    // USART `index` 0/1 in UART mode.
    bool uart_can_receive(unsigned index) const;
    void uart_receive(unsigned index, std::uint8_t byte);
    // Queues a frame with an explicit 9th bit, a parity or stop-bit error or
    // a break, in order with plain bytes; a plain frame equals uart_receive.
    void uart_receive_frame(unsigned index, const Cc2530UartFrame &frame);
    std::vector<std::uint8_t> uart_take_transmitted(unsigned index);
    // While on (off at construction), every transmitted UART frame is also
    // recorded with its 9th bit for uart_take_transmitted_frames; the byte
    // stream of uart_take_transmitted is unchanged.
    void uart_capture_frames(unsigned index, bool on);
    std::vector<Cc2530UartFrame> uart_take_transmitted_frames(unsigned index);
    void set_spi_device(unsigned index, std::function<std::uint8_t(std::uint8_t)> device);
    // Seeds deterministic noise sources (ADC analog noise, radio RFRND and
    // CSP RANDXY).
    void set_noise_seed(std::uint64_t seed);
    // Fault injection (SWRU191F 5.2, docs/cc2530.md "Clock loss"): from
    // virtual time `at_cycle` (the next instruction boundary if that has
    // passed) oscillator `osc` no longer runs, as with a damaged crystal. It
    // stays stopped across every reset except a power-on reset (which a
    // firmware load includes). With CLD.EN = 1 a stopped monitored clock
    // causes a clock-loss reset (SLEEPSTA.RST = 11); a stopped system clock
    // stalls the CPU; a stopped 32 kHz clock stops the sleep timer and the
    // watchdog; a switch to a stopped oscillator never completes.
    void stop_oscillator(Cc2530Oscillator osc, std::uint64_t at_cycle);
    // The CPU waits for its stopped system clock: run() advances virtual
    // time (peripherals and resets) but executes no instruction.
    bool clock_stalled() const;
    // Connects the IEEE 802.15.4 radio to a shared air as node `node`.
    // Virtual time maps to air time as ns = ns_offset + cycles * 1000 / 32.
    void attach_air(mcuemu::VirtualAir &air, std::string node, std::uint64_t ns_offset = 0);

    // ---- flash fault model (docs/cc2530.md "Flash fault injection") ----
    // Kept across loads and resets.
    void set_flash_fault_config(const Cc2530FlashFaultConfig &config);
    const Cc2530FlashFaultConfig &flash_fault_config() const;
    // Power cut now (between runs): a flash operation in flight ends with
    // the configured interrupted outcome, SRAM takes the power-on fill, then
    // a power-on reset (SLEEPSTA.RST = 00, SWRU191F 4.3). Flash, the info
    // page and virtual time are kept (the supply returns at once).
    void power_cut();
    // Power cut at the first instruction boundary at which cycles >=
    // `at_cycle`, in every power mode. Survives resets.
    void schedule_power_cut(std::uint64_t at_cycle);
    // Arms a flash-event trigger (see Cc2530FlashTrigger). Survives resets.
    void add_flash_trigger(const Cc2530FlashTrigger &trigger);
    // Scheduled power cuts and armed triggers that have not fired yet.
    std::size_t pending_power_faults() const;
    // Flash operations recorded while flash_fault_config().log is set;
    // take_flash_log() hands them over and clears the log.
    const std::vector<Cc2530FlashLogEntry> &flash_log() const;
    std::vector<Cc2530FlashLogEntry> take_flash_log();
    // The whole 256 KiB physical flash (for saving it across runs).
    const std::vector<std::uint8_t> &flash_image() const noexcept { return flash_; }
    // Replaces the whole flash with a saved image (exactly FlashSize bytes)
    // and performs a power-on reset; diagnostics restart, load_warnings are
    // kept.
    void restore_flash(const std::vector<std::uint8_t> &image);
    // Internal peripheral objects (src/mcs51/cc2530_*.hpp), for white-box
    // tests and the system layer.
    cc2530::Soc &peripherals() noexcept { return *soc_; }

    void set_trace(std::ostream *output) noexcept { trace_output_ = output; }
    const Cc2530Diagnostics &diagnostics() const noexcept { return diagnostics_; }

    static unsigned instruction_size(std::uint8_t opcode) noexcept;

private:
    class BusImpl;
    static std::uint64_t sfr_mmio_read(uc_struct *uc, std::uint64_t offset,
                                       unsigned size, void *user_data);
    static void sfr_mmio_write(uc_struct *uc, std::uint64_t offset,
                               unsigned size, std::uint64_t value,
                               void *user_data);
    static std::uint64_t xdata_mmio_read(uc_struct *uc, std::uint64_t offset,
                                         unsigned size, void *user_data);
    static void xdata_mmio_write(uc_struct *uc, std::uint64_t offset,
                                 unsigned size, std::uint64_t value,
                                 void *user_data);
    static void code_hook(uc_struct *uc, std::uint64_t address,
                          std::uint32_t size, void *user_data);

    std::uint8_t xdata_value(std::uint16_t address) const;
    std::uint8_t xdata_read(std::uint16_t address);
    void xdata_write(std::uint16_t address, std::uint8_t value);
    std::uint8_t sfr_read(std::uint8_t address);
    std::uint8_t sfr_peek(std::uint8_t address) const;
    std::uint8_t sleepsta_value() const;
    void sfr_write(std::uint8_t address, std::uint8_t value);
    // After a CPU/DMA/host access reached a peripheral: a lowered deadline
    // must stop the fast interpreter at the right boundary.
    void peripheral_accessed();
    void refresh_flash(std::uint32_t begin, std::uint32_t end);
    void rebuild_views();
    void install_code_view();
    struct Request {
        int source{-1};
        std::uint8_t level{};
    };
    Request pending_interrupt() const;
    void update_interrupts();
    // update_interrupts() at an instruction boundary where no input may have
    // changed without a post: counts a stale post first (R1-F10).
    void check_interrupts_at_boundary();
    bool sync_interrupt_consumption();
    // The single path for a non-firmware flag change (events, test hook,
    // future peripherals).
    void set_interrupt_flags(unsigned source, std::uint8_t flag_mask);
    std::uint8_t checked_flag_mask(unsigned source, std::uint8_t flag_mask) const;
    void apply_due_events();
    std::uint64_t engine_cycles() const;
    void wake_from_power_mode();
    // At an instruction boundary: makes a requested power mode effective,
    // then wakes if the mode's wake rule holds. True if it woke.
    void settle_power_mode();
    bool wake_request(Cc2530PowerMode mode) const;
    void notify_power_mode(Cc2530PowerMode from, Cc2530PowerMode to);
    bool apply_pending_reset();
    std::uint64_t hf_now() const;
    std::uint64_t lf_now() const;
    // Mirrors the core's decision at the next instruction boundary; the
    // post-RETI inhibit is ignored where only eventual delivery matters.
    bool interrupt_deliverable(bool honor_inhibit) const;
    void enter_power_mode();
    void request_stop(std::string error);
    void check_error(int error, const char *operation) const;
    void set_code_hook_enabled(bool enabled);
    void trace_instruction(std::uint16_t pc);
    std::uint16_t engine_irq_vector() const;
    std::uint8_t engine_levels() const;
    bool engine_inhibit() const;
    void reset_core(Cc2530ResetCause cause, bool count = true);
    void create_peripherals();
    void request_power_fault(Cc2530FaultAction action);
    void power_on_sram_fill();
    // Clock loss (stop_oscillator, SWRU191F 4.4 and 5.2).
    bool oscillator_stopped(Cc2530Oscillator osc, std::uint64_t at) const;
    bool cpu_clock_stopped() const;
    std::uint64_t next_oscillator_stop() const;
    std::uint64_t clock_loss_deadline() const;
    // CLKCONSTA for `command` when the system clock currently is the RCOSC
    // (`on_rcosc`) or the XOSC.
    std::uint8_t clock_status(std::uint8_t command, bool on_rcosc) const;
    void set_clock_status(std::uint8_t status);
    // Freezes or resumes the 32 kHz domain at `at` (PM3, stopped clock).
    void refresh_lf_domain(std::uint64_t at);
    // The CPU must not execute: RESET_N low or halted by the debug interface.
    bool cpu_held() const;
    // DEBUG_INSTR (cc2530::Bus::debug_execute).
    std::uint8_t debug_execute(const std::uint8_t *code, unsigned length);
    void apply_pending_reset_pin();

    uc_struct *uc_{};
    std::size_t code_hook_{};
    bool code_hook_installed_{};

    // Physical flash, and one zero-copy 64 KiB CODE view per FMAP value:
    // view N = flash[0, 0x8000) followed by flash[N * 0x8000, +0x8000).
    std::vector<std::uint8_t> flash_;
    std::vector<std::uint8_t> views_;
    // The CODE view while MEMCTR.XMAP is set. Bytes 0x8000-0x9fff of this
    // buffer *are* the SRAM: XDATA 0x0000-0x1fff and IDATA (SRAM 0x1f00-
    // 0x1fff) are mapped onto them, so code written to SRAM and executed
    // through XMAP is always coherent.
    std::vector<std::uint8_t> xmap_view_;
    std::uint8_t *sram_{};
    std::uint8_t *idata_{};
    std::array<std::uint8_t, 256> sfr_{};
    std::array<std::uint8_t, 256> sfr_special_read_{};
    std::array<std::uint8_t, 256> sfr_special_write_{};
    std::array<std::uint8_t, 0x400> xreg_{};
    std::array<std::uint8_t, 0x800> info_page_{};

    std::unique_ptr<BusImpl> bus_;
    std::unique_ptr<cc2530::Soc> soc_;
    // Construction order = scheduler tie-break order.
    std::vector<std::unique_ptr<cc2530::Peripheral>> peripherals_;
    std::array<cc2530::Peripheral *, 256> sfr_owner_{};
    std::array<cc2530::Peripheral *, 0x400> xreg_owner_{};
    Cc2530PowerMode power_mode_{Cc2530PowerMode::Active};
    Cc2530PowerMode requested_mode_{Cc2530PowerMode::Active};
    std::uint64_t power_mode_since_{};
    std::uint64_t hf_stopped_{}; // completed time in PM1-PM3
    std::uint64_t lf_stopped_{}; // completed frozen time of the 32 kHz domain
    // Start of the current 32 kHz domain freeze (PM3 or a stopped 32 kHz
    // clock source).
    std::optional<std::uint64_t> lf_frozen_since_;
    // stop_oscillator() times per Cc2530Oscillator (UINT64_MAX: running);
    // bit N of osc_stops_applied_: stop N has been applied at a boundary.
    std::array<std::uint64_t, 4> osc_stop_at_{UINT64_MAX, UINT64_MAX, UINT64_MAX, UINT64_MAX};
    std::uint8_t osc_stops_applied_{};
    // Since when the clock-loss detector watches the current clocks.
    std::uint64_t cld_since_{};
    std::optional<Cc2530ResetCause> pending_reset_;
    // The pending reset is a power cut (power_cut() at the boundary).
    bool pending_power_cut_{};
    std::uint64_t stall_until_{};
    std::uint64_t fast_event_stop_{UINT64_MAX};

    std::uint16_t posted_vector_{};
    std::uint8_t posted_level_{};
    int posted_source_{-1};
    struct Event {
        std::uint64_t at{};
        std::uint64_t sequence{};
        unsigned source{};
        std::uint8_t flag_mask{};
    };
    // Sorted by (at, sequence).
    std::vector<Event> events_;
    std::uint64_t event_sequence_{};
    std::optional<std::uint16_t> breakpoint_;
    bool break_hit_{};
    std::function<void(Cc2530ResetCause)> reset_listener_;
    std::uint64_t hook_calls_{};
    std::uint64_t trace_cycle_limit_{UINT64_MAX};
    bool sleep_requested_{};
    bool running_fast_{};
    // Inside uc_emu_start (run() or a DEBUG_INSTR).
    bool engine_running_{};
    bool debug_executing_{};
    bool reset_held_{};
    // RESET_N changes made inside the engine, applied at the next boundary.
    std::vector<bool> pending_reset_levels_;
    // Timer 1/3/4 time base suspended by the debug interface (Table 3-2).
    bool timers_suspended_{};
    std::uint64_t timers_suspended_since_{}; // hf_now() when suspended
    std::uint64_t timers_suspended_total_{}; // completed suspended HF time
    std::string stop_error_;
    Cc2530Diagnostics diagnostics_;
    std::ostream *trace_output_{};
};

} // namespace mcs51emu

#endif
