#include "sim/cc2530f256_mcu.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace sim {

namespace {

constexpr std::uint64_t NsPerSecond = 1000000000u;

// floor(cycles * 1e9 / hz) and ceil(ns * hz / 1e9) without 128-bit
// intermediates (as in the CA51F253L3 adapter).
std::uint64_t cycles_to_ns(std::uint64_t cycles, std::uint32_t hz) {
    return (cycles / hz) * NsPerSecond + (cycles % hz) * NsPerSecond / hz;
}

std::uint64_t ns_to_cycles_ceil(std::uint64_t ns, std::uint32_t hz) {
    return (ns / NsPerSecond) * hz +
           ((ns % NsPerSecond) * hz + NsPerSecond - 1) / NsPerSecond;
}

std::string hex(std::uint32_t value, int width) {
    std::ostringstream text;
    text << "0x" << std::hex << std::setw(width) << std::setfill('0') << value;
    return text.str();
}

Drive to_drive(mcs51emu::Cc2530PinDrive drive) {
    switch (drive) {
    case mcs51emu::Cc2530PinDrive::Low: return Drive::Low;
    case mcs51emu::Cc2530PinDrive::High: return Drive::High;
    case mcs51emu::Cc2530PinDrive::PullUp: return Drive::PullUp;
    case mcs51emu::Cc2530PinDrive::PullDown: return Drive::PullDown;
    case mcs51emu::Cc2530PinDrive::HiZ: break;
    }
    return Drive::HiZ;
}

constexpr unsigned PinsPerPort[3] = {8, 8, 5}; // P2 has P2.0-P2.4

// "P0.0" .. "P2.4" (TI's P0_0 spelling is accepted too).
bool parse_pad(std::string_view text, unsigned &port, unsigned &pin) {
    if (text.size() != 4 || (text[0] != 'P' && text[0] != 'p') ||
        (text[2] != '.' && text[2] != '_') || text[1] < '0' || text[1] > '2' ||
        text[3] < '0' || text[3] > '7')
        return false;
    port = static_cast<unsigned>(text[1] - '0');
    pin = static_cast<unsigned>(text[3] - '0');
    return pin < PinsPerPort[port];
}

void put_le(std::array<std::uint8_t, Cc2530F256Mcu::CpuSnapshotSize> &block, unsigned at,
            std::uint64_t value, unsigned bytes) {
    for (unsigned i = 0; i < bytes; ++i)
        block[at + i] = static_cast<std::uint8_t>(value >> (8 * i));
}

const char *op_name(mcs51emu::Cc2530FlashOpKind op) {
    switch (op) {
    case mcs51emu::Cc2530FlashOpKind::PageErase: return "erase";
    case mcs51emu::Cc2530FlashOpKind::WordWrite: return "write";
    case mcs51emu::Cc2530FlashOpKind::ChipErase: return "chip-erase";
    }
    return "?";
}

const char *outcome_name(mcs51emu::Cc2530FlashOutcome outcome) {
    switch (outcome) {
    case mcs51emu::Cc2530FlashOutcome::Completed: return "completed";
    case mcs51emu::Cc2530FlashOutcome::Aborted: return "aborted";
    case mcs51emu::Cc2530FlashOutcome::Abandoned: return "abandoned";
    case mcs51emu::Cc2530FlashOutcome::Partial: return "partial";
    case mcs51emu::Cc2530FlashOutcome::Forced: return "forced";
    }
    return "?";
}

const char *reset_cause_name(mcs51emu::Cc2530ResetCause cause) {
    switch (cause) {
    case mcs51emu::Cc2530ResetCause::PowerOn: return "power-on";
    case mcs51emu::Cc2530ResetCause::External: return "external";
    case mcs51emu::Cc2530ResetCause::Watchdog: return "watchdog";
    case mcs51emu::Cc2530ResetCause::ClockLoss: return "clock-loss";
    }
    return "?";
}

std::string violation_names(std::uint8_t violations) {
    std::string text;
    const std::pair<std::uint8_t, const char *> names[] = {
        {mcs51emu::Cc2530FlashBitLimit, "bit"},
        {mcs51emu::Cc2530FlashWordLimit, "word"},
        {mcs51emu::Cc2530FlashPageLimit, "page"}};
    for (const auto &[bit, name] : names)
        if (violations & bit)
            text += (text.empty() ? "" : ",") + std::string(name);
    return text.empty() ? "none" : text;
}

} // namespace

Cc2530F256Mcu::Cc2530F256Mcu(std::string name, std::uint32_t clock_hz)
    : Mcu(std::move(name)), clock_hz_(clock_hz) {
    if (clock_hz == 0)
        throw std::invalid_argument("clock_hz must be positive");
    // Chip pad -> net, stamped with the cycle at which the drive changed.
    device_.set_pin_listener([this](unsigned port, unsigned pin,
                                    mcs51emu::Cc2530PinDrive drive, std::uint64_t cycle) {
        if (Pin *wired = pins_[port * 8 + pin])
            wired->set_drive(to_drive(drive), cycle_ns(cycle));
    });
    // A power-on reset (also a power cut inside a run) restarts the debugger
    // arrival count and re-arms its breakpoint, as reset() does.
    device_.set_reset_listener([this](mcs51emu::Cc2530ResetCause cause) {
        if (cause != mcs51emu::Cc2530ResetCause::PowerOn)
            return;
        debugger_arrivals_ = 0;
        debugger_at_break_ = false;
        device_.set_breakpoint(debugger_pc_);
    });
}

std::uint64_t Cc2530F256Mcu::cycle_ns(std::uint64_t cycle) const {
    return offset_ns_ + cycles_to_ns(cycle, clock_hz_);
}

bool Cc2530F256Mcu::externally_driven() const {
    // A host using the debug interface keeps a halted chip alive as well.
    if (air_ || uart_used_ || reset_pin_ || device_.debug_if_active())
        return true;
    for (const Pin *pin : pins_)
        if (pin)
            return true;
    return false;
}

Cc2530F256Mcu::~Cc2530F256Mcu() {
    // Destruction ends the run: the remaining log, the counters, then the
    // flash image (only after a run, so a failed configuration load does not
    // create or overwrite the file).
    power_listener_ = nullptr; // the endpoint's subscribers may be gone
    try {
        if (flash_log_) {
            after_run();
            write_flash_summary();
        }
        if (ran_ && !persist_file_.empty())
            save_flash();
    } catch (const std::exception &error) {
        std::cerr << "warning: " << name() << ": " << error.what() << '\n';
    }
}

void Cc2530F256Mcu::load_firmware(const std::string &path) {
    device_.load_firmware(path);
    if (!persist_file_.empty() && std::filesystem::exists(persist_file_)) {
        std::ifstream input(persist_file_, std::ios::binary);
        if (!input)
            throw std::invalid_argument("cannot read the flash persist file " + persist_file_);
        const std::vector<std::uint8_t> image{std::istreambuf_iterator<char>(input),
                                              std::istreambuf_iterator<char>()};
        if (image.size() != mcs51emu::Cc2530F256::FlashSize)
            throw std::invalid_argument("flash persist file " + persist_file_ + " holds " +
                                        std::to_string(image.size()) +
                                        " bytes, not the 262144 of a CC2530F256 flash");
        device_.restore_flash(image);
    }
    reset();
}

void Cc2530F256Mcu::reset() {
    device_.power_on_reset();
    seen_power_cuts_ = device_.diagnostics().power_cuts;
    cycles_ = 0;
    state_ = McuState::Running;
    fault_.clear();
    debugger_arrivals_ = 0;
    debugger_at_break_ = false;
    device_.set_breakpoint(debugger_pc_);
}

void Cc2530F256Mcu::set_debugger(std::uint16_t pc, std::vector<DebuggerWrite> writes,
                                 std::optional<DebuggerReport> report) {
    if (writes.empty() && !report)
        throw std::invalid_argument("debugger: no writes or report given");
    if (report && report->hits == 0)
        throw std::invalid_argument("debugger: report hit counts start at 1");
    if (report)
        for (const auto &[start, length] : report->dump.xdata)
            if (length == 0 || start + std::uint64_t{length} > 0x10000)
                throw std::invalid_argument("debugger: report xdata range at " +
                                            hex(start, 4) + " leaves the XDATA space");
    unsigned last = 0;
    for (const DebuggerWrite &write : writes) {
        if (write.hit == 0)
            throw std::invalid_argument("debugger: hit counts start at 1");
        if (write.bytes.empty() ||
            write.xdata + write.bytes.size() > mcs51emu::Cc2530F256::SramSize)
            throw std::invalid_argument(
                "debugger: write at xdata " + hex(write.xdata, 4) + " (" +
                std::to_string(write.bytes.size()) +
                " bytes) leaves XDATA SRAM 0x0000-0x1fff");
        last = std::max(last, write.hit);
    }
    std::unique_ptr<std::ofstream> out;
    if (report) {
        out = std::make_unique<std::ofstream>(report->file, std::ios::out | std::ios::trunc);
        if (!*out)
            throw std::invalid_argument("debugger: cannot create report file " + report->file);
        last = std::max(last, report->hits);
    }
    debugger_pc_ = pc;
    debugger_writes_ = std::move(writes);
    debugger_last_hit_ = last;
    debugger_report_out_ = std::move(out);
    debugger_report_hits_ = report ? report->hits : 0;
    debugger_report_dump_ = report ? std::move(report->dump) : mcs51emu::Cc2530HitDump{};
    debugger_arrivals_ = 0;
    debugger_at_break_ = false;
    device_.set_breakpoint(pc);
}

void Cc2530F256Mcu::debugger_arrive() {
    ++debugger_arrivals_;
    debugger_at_break_ = true;
    if (debugger_report_out_ && debugger_arrivals_ <= debugger_report_hits_) {
        mcs51emu::write_cc2530_hit(*debugger_report_out_, device_, debugger_arrivals_,
                                   device_.registers().instructions, debugger_report_dump_);
        debugger_report_out_->flush();
        if (!*debugger_report_out_)
            throw std::runtime_error("debugger: writing the report failed");
    }
    for (const DebuggerWrite &write : debugger_writes_)
        if (write.hit == debugger_arrivals_)
            for (std::size_t i = 0; i < write.bytes.size(); ++i)
                device_.write_sram(static_cast<std::uint16_t>(write.xdata + i), write.bytes[i]);
    if (debugger_arrivals_ >= debugger_last_hit_)
        device_.set_breakpoint(std::nullopt); // nothing left to write: run free
}

std::uint64_t Cc2530F256Mcu::ns_to_cycles(std::uint64_t ns) const {
    return ns_to_cycles_ceil(ns, clock_hz_);
}

void Cc2530F256Mcu::stop_oscillator(mcs51emu::Cc2530Oscillator osc, std::uint64_t at_ns) {
    const std::uint64_t at_cycle =
        at_ns <= offset_ns_ ? 0 : ns_to_cycles_ceil(at_ns - offset_ns_, clock_hz_);
    device_.stop_oscillator(osc, at_cycle);
}

std::uint64_t Cc2530F256Mcu::time_ns() const {
    return offset_ns_ + cycles_to_ns(cycles_, clock_hz_);
}

McuState Cc2530F256Mcu::run_until(std::uint64_t target_ns) {
    if (air_ && !air_attached_) {
        // Attached on the first run, once the world has set the start
        // offset that maps chip cycles to air time.
        device_.attach_air(*air_, air_node_, offset_ns_);
        air_attached_ = true;
    }
    while (state_ == McuState::Running && time_ns() < target_ns) {
        // First cycle whose time is at or past the target, counted from the
        // MCU's own power-on.
        const std::uint64_t target_cycle =
            ns_to_cycles_ceil(target_ns - offset_ns_, clock_hz_);
        // The core does not break on the first instruction of a run, so an
        // arrival at the boundary where the previous quantum ended is
        // counted here.
        const bool debugger_armed =
            debugger_pc_ && debugger_arrivals_ < debugger_last_hit_;
        if (debugger_armed && !debugger_at_break_ && device_.pc() == *debugger_pc_)
            debugger_arrive();
        const mcs51emu::Cc2530RunResult result =
            device_.run(std::numeric_limits<std::size_t>::max(), target_cycle);
        cycles_ = result.cycles;
        ran_ = true;
        after_run();
        if (result.break_hit)
            debugger_arrive();
        else if (result.instructions != 0)
            debugger_at_break_ = false;
        if (!result.error.empty()) {
            state_ = McuState::Fault;
            fault_ = result.error;
        } else if (result.sleeping && !externally_driven() && !device_.may_wake()) {
            // Asleep, no event, peripheral or reset can end the power mode and
            // nothing outside the chip (net, air, UART) is connected that
            // could wake it. Peripheral deadlines that cannot wake the CPU
            // (for example the sleep timer compare with STIE clear) do not
            // keep the core running.
            state_ = McuState::Sleeping;
            fault_ = "PCON power mode entered with no wake source at pc=" +
                     hex(result.pc, 4);
        } else if (!externally_driven() && device_.idle_forever()) {
            state_ = McuState::Halted;
            fault_ = "idle self-loop at pc=" + hex(result.pc, 4) + " fmap=" +
                     std::to_string(result.fmap) +
                     " with no deliverable interrupt";
        }
    }
    return state_;
}

void Cc2530F256Mcu::set_flash(FlashSetup setup) {
    std::unique_ptr<std::ofstream> log;
    if (!setup.log.empty()) {
        log = std::make_unique<std::ofstream>(setup.log, std::ios::out | std::ios::trunc);
        if (!*log)
            throw std::invalid_argument("flash: cannot create log file " + setup.log);
        setup.fault.log = true;
    }
    device_.set_flash_fault_config(setup.fault);
    for (const std::uint64_t cycle : setup.cut_cycles)
        device_.schedule_power_cut(cycle);
    for (const mcs51emu::Cc2530FlashTrigger &trigger : setup.triggers)
        device_.add_flash_trigger(trigger);
    persist_file_ = std::move(setup.persist);
    flash_log_ = std::move(log);
}

void Cc2530F256Mcu::after_run() {
    if (flash_log_) {
        for (const mcs51emu::Cc2530FlashLogEntry &entry : device_.take_flash_log()) {
            *flash_log_ << "time_ns=" << cycle_ns(entry.cycle) << " cycle=" << entry.cycle
                        << " op=" << op_name(entry.op) << " page=" << entry.page
                        << " address=" << hex(entry.address, 5)
                        << " outcome=" << outcome_name(entry.outcome);
            if (entry.cut == mcs51emu::Cc2530FlashCut::Reset)
                *flash_log_ << " cut=reset cause=" << reset_cause_name(entry.reset_cause);
            else if (entry.cut == mcs51emu::Cc2530FlashCut::PowerCut)
                *flash_log_ << " cut=power-cut";
            *flash_log_ << " elapsed=" << entry.elapsed_cycles
                        << " changed_bits=" << entry.changed_bits
                        << " violations=" << violation_names(entry.violations) << '\n';
        }
    }
    const mcs51emu::Cc2530Diagnostics &diag = device_.diagnostics();
    if (diag.power_cuts != seen_power_cuts_) {
        seen_power_cuts_ = diag.power_cuts;
        const std::uint64_t at = cycle_ns(diag.last_power_cut_cycle);
        if (flash_log_)
            *flash_log_ << "time_ns=" << at << " cycle=" << diag.last_power_cut_cycle
                        << " event=power-cut count=" << diag.power_cuts << '\n';
        if (power_listener_)
            power_listener_(at, diag.power_cuts);
    }
    if (flash_log_) {
        flash_log_->flush();
        if (!*flash_log_)
            throw std::runtime_error("flash: writing the log failed");
    }
}

void Cc2530F256Mcu::write_flash_summary() {
    const mcs51emu::Cc2530Diagnostics &diag = device_.diagnostics();
    *flash_log_ << "summary power_cuts=" << diag.power_cuts
                << " interrupted_erases=" << diag.flash_interrupted_erases
                << " interrupted_writes=" << diag.flash_interrupted_writes
                << " bit_limit_violations=" << diag.flash_bit_limit_violations
                << " word_limit_violations=" << diag.flash_word_limit_violations
                << " page_limit_violations=" << diag.flash_page_limit_violations << '\n';
    for (unsigned page = 0; page < diag.flash_page_erases.size(); ++page)
        if (diag.flash_page_erases[page] || diag.flash_page_writes[page])
            *flash_log_ << "summary page=" << page << " erases=" << diag.flash_page_erases[page]
                        << " writes=" << diag.flash_page_writes[page] << '\n';
    flash_log_->flush();
}

void Cc2530F256Mcu::save_flash() const {
    // Through a temporary file, so an interrupted save leaves the old image.
    const std::string temporary = persist_file_ + ".tmp";
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        const std::vector<std::uint8_t> &image = device_.flash_image();
        output.write(reinterpret_cast<const char *>(image.data()),
                     static_cast<std::streamsize>(image.size()));
        if (!output)
            throw std::runtime_error("cannot write the flash persist file " + temporary);
    }
    std::filesystem::rename(temporary, persist_file_);
}

void Cc2530F256Mcu::catch_up(std::uint64_t world_ns) {
    // A stopped MCU stays behind the world clock; the cut belongs at the
    // world time of the request.
    if ((state_ == McuState::Sleeping || state_ == McuState::Halted) && world_ns > time_ns()) {
        state_ = McuState::Running;
        fault_.clear();
        run_until(world_ns);
    }
}

void Cc2530F256Mcu::revive() {
    if (state_ == McuState::Sleeping || state_ == McuState::Halted) {
        state_ = McuState::Running;
        fault_.clear();
    }
}

void Cc2530F256Mcu::host_power_cut(std::uint64_t world_ns) {
    catch_up(world_ns);
    device_.power_cut();
    revive();
    after_run();
}

void Cc2530F256Mcu::host_reset(std::uint64_t world_ns) {
    catch_up(world_ns);
    device_.reset();
    revive();
    after_run();
}

Cc2530PowerEndpoint::Cc2530PowerEndpoint(std::string name, Cc2530F256Mcu &mcu,
                                         std::function<std::uint64_t()> world_now)
    : name_(std::move(name)), mcu_(mcu), world_now_(std::move(world_now)) {
    mcu_.set_power_listener([this](std::uint64_t time_ns, std::uint64_t count) {
        for (const auto &[id, sink] : sinks_)
            sink(EndpointEvent{time_ns, name_, Value{static_cast<std::int64_t>(count)}});
    });
}

Value Cc2530PowerEndpoint::get() const {
    return Value{static_cast<std::int64_t>(mcu_.power_cuts())};
}

void Cc2530PowerEndpoint::set(const Value &value) {
    const auto *text = std::get_if<std::string>(&value);
    if (text && *text == "cut")
        mcu_.host_power_cut(world_now_());
    else if (text && *text == "reset")
        mcu_.host_reset(world_now_());
    else
        throw std::invalid_argument("power takes \"cut\" or \"reset\"");
}

std::size_t Cc2530PowerEndpoint::subscribe(EndpointSink sink) {
    const std::size_t id = next_id_++;
    sinks_.emplace(id, std::move(sink));
    return id;
}

std::vector<std::string> Cc2530F256Mcu::pin_names() const {
    std::vector<std::string> names;
    for (unsigned port = 0; port != 3; ++port)
        for (unsigned pin = 0; pin != PinsPerPort[port]; ++pin)
            names.push_back("P" + std::to_string(port) + "." + std::to_string(pin));
    return names;
}

void Cc2530F256Mcu::connect_pin(std::string_view pad, Pin &pin) {
    if (pad == "RESET_N") {
        if (reset_pin_)
            throw std::invalid_argument("pad RESET_N is already connected");
        reset_pin_ = &pin;
        // SWRU191F 4.3 / 3.1: low holds the chip in reset; DC edges while it
        // is low select debug mode at the release. Model-defined: a floating
        // net counts as high (no reset), as with a pull-up on the board.
        const auto apply = [this](Level level) { device_.set_reset_pin(level != Level::Low); };
        pin.net().subscribe([apply](const NetChange &change) { apply(change.level); });
        apply(pin.net().level());
        return;
    }
    unsigned port = 0, bit = 0;
    if (!parse_pad(pad, port, bit))
        throw std::invalid_argument("CC2530F256 has no pad named '" + std::string(pad) +
                                    "' (expected P0.0-P0.7, P1.0-P1.7, P2.0-P2.4 or RESET_N)");
    const unsigned index = port * 8 + bit;
    if (pins_[index])
        throw std::invalid_argument("pad " + std::string(pad) + " is already connected");
    pins_[index] = &pin;
    // Net -> pad: the line level (or the analog voltage) the pad sees.
    const auto apply = [this, port, bit](Level level, std::uint32_t millivolts) {
        if (level == Level::Floating) {
            device_.set_pin_input(port, bit, std::nullopt);
            device_.set_pin_analog_mv(port, bit, std::nullopt);
        } else {
            device_.set_pin_input(port, bit, level == Level::High);
            device_.set_pin_analog_mv(port, bit, millivolts);
        }
    };
    pin.net().subscribe([apply](const NetChange &change) { apply(change.level, change.millivolts); });
    apply(pin.net().level(), pin.net().millivolts());
    // Pad -> net: whatever the pad drives now.
    pin.set_drive(to_drive(device_.pin_drive(port, bit)), time_ns());
}

void Cc2530F256Mcu::debug_if_check_reset_pin(const char *what) const {
    if (reset_pin_ && reset_pin_->net().level() == Level::Low)
        throw std::runtime_error(std::string("debug interface: ") + what + " while net '" +
                                 reset_pin_->net().name() + "' holds RESET_N low");
}

void Cc2530F256Mcu::debug_if_enter() {
    if (state_ == McuState::Fault)
        throw std::runtime_error("debug interface: the MCU stopped with a fault: " + fault_);
    debug_if_check_reset_pin("cannot enter debug mode");
    device_.debug_if_enter();
    // The entry sequence is a reset (3.1): whatever stopped the MCU (an idle
    // loop, a power mode without a wake source) is gone.
    state_ = McuState::Running;
    fault_.clear();
    debugger_at_break_ = false;
}

void Cc2530F256Mcu::debug_if_submit(const std::vector<std::uint8_t> &command) {
    if (state_ == McuState::Fault)
        throw std::runtime_error("debug interface: the MCU stopped with a fault: " + fault_);
    device_.debug_if_submit(command);
}

std::optional<mcs51emu::Cc2530DebugIfResponse> Cc2530F256Mcu::debug_if_take_response() {
    return device_.debug_if_take_response();
}

std::uint64_t Cc2530F256Mcu::debug_if_due_ns() const {
    return cycle_ns(device_.debug_if_due_cycle());
}

void Cc2530F256Mcu::debug_if_reset_pulse() {
    if (state_ == McuState::Fault)
        throw std::runtime_error("debug interface: the MCU stopped with a fault: " + fault_);
    debug_if_check_reset_pin("cannot pulse RESET_N");
    device_.set_reset_pin(false);
    device_.set_reset_pin(true);
    state_ = McuState::Running;
    fault_.clear();
    debugger_at_break_ = false;
}

std::uint64_t Cc2530F256Mcu::time_after_cycles(std::uint64_t cycles) const {
    return cycle_ns(device_.registers().cycles + cycles);
}

void Cc2530F256Mcu::attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                                 const std::vector<std::string> &protocols) {
    for (const std::string &protocol : protocols)
        if (protocol != "zigbee")
            throw std::invalid_argument("cc2530f256: the radio is IEEE 802.15.4 only; '" +
                                        protocol + "' is not supported");
    if (clock_hz_ != 32000000)
        throw std::invalid_argument(
            "cc2530f256: the radio needs the 32 MHz time base; remove 'clock_hz'");
    if (air_)
        throw std::invalid_argument("cc2530f256: the radio is already attached");
    air_ = &air;
    air_node_ = node;
}

bool Cc2530F256Mcu::uart_rx(unsigned index, std::uint8_t byte) {
    uart_used_ = true;
    if (index >= 2 || !device_.uart_can_receive(index))
        return false;
    device_.uart_receive(index, byte);
    return true;
}

std::vector<std::uint8_t> Cc2530F256Mcu::uart_take_tx(unsigned index) {
    if (index >= 2)
        return {};
    return device_.uart_take_transmitted(index);
}

namespace {

// The two frame structs mirror each other field for field.
mcs51emu::Cc2530UartFrame to_chip(const UartFrame &frame) {
    return {.data = frame.data,
            .ninth = static_cast<mcs51emu::Cc2530UartFrame::Ninth>(frame.ninth),
            .stop1_error = frame.stop1_error,
            .stop2_error = frame.stop2_error,
            .line_break = frame.line_break};
}

UartFrame from_chip(const mcs51emu::Cc2530UartFrame &frame) {
    return {.data = frame.data,
            .ninth = static_cast<UartFrame::Ninth>(frame.ninth),
            .stop1_error = frame.stop1_error,
            .stop2_error = frame.stop2_error,
            .line_break = frame.line_break};
}

constexpr bool same_ninth(UartFrame::Ninth a, mcs51emu::Cc2530UartFrame::Ninth b) {
    return static_cast<int>(a) == static_cast<int>(b);
}
static_assert(same_ninth(UartFrame::Ninth::Expected, mcs51emu::Cc2530UartFrame::Ninth::Expected) &&
              same_ninth(UartFrame::Ninth::Zero, mcs51emu::Cc2530UartFrame::Ninth::Zero) &&
              same_ninth(UartFrame::Ninth::One, mcs51emu::Cc2530UartFrame::Ninth::One) &&
              same_ninth(UartFrame::Ninth::BadParity, mcs51emu::Cc2530UartFrame::Ninth::BadParity));

} // namespace

bool Cc2530F256Mcu::uart_rx_frame(unsigned index, const UartFrame &frame) {
    uart_used_ = true;
    if (index >= 2 || !device_.uart_can_receive(index))
        return false;
    device_.uart_receive_frame(index, to_chip(frame));
    return true;
}

void Cc2530F256Mcu::uart_capture_tx_frames(unsigned index, bool on) {
    if (index < 2)
        device_.uart_capture_frames(index, on);
}

std::vector<UartFrame> Cc2530F256Mcu::uart_take_tx_frames(unsigned index) {
    std::vector<UartFrame> frames;
    if (index >= 2)
        return frames;
    for (const mcs51emu::Cc2530UartFrame &frame : device_.uart_take_transmitted_frames(index))
        frames.push_back(from_chip(frame));
    return frames;
}

std::uint8_t Cc2530F256Mcu::read_byte(std::uint32_t address) const {
    if (address < XdataWindow)
        return device_.read_code(static_cast<std::uint16_t>(address));
    if (address < IdataWindow)
        return device_.peek_xdata(static_cast<std::uint16_t>(address - XdataWindow));
    if (address >= IdataWindow && address < IdataWindow + 0x100)
        return device_.read_idata(static_cast<std::uint8_t>(address - IdataWindow));
    if (address >= SfrWindow + 0x80 && address < SfrWindow + 0x100)
        return device_.read_sfr(static_cast<std::uint8_t>(address - SfrWindow));
    if (address >= CpuSnapshotWindow && address < CpuSnapshotWindow + CpuSnapshotSize) {
        std::array<std::uint8_t, CpuSnapshotSize> block{};
        const mcs51emu::Cc2530Registers regs = device_.registers();
        put_le(block, CpuSnapshotPc, regs.pc, 2);
        block[CpuSnapshotFmap] = regs.fmap;
        block[CpuSnapshotMemctr] = regs.memctr;
        block[CpuSnapshotA] = regs.a;
        block[CpuSnapshotB] = regs.b;
        block[CpuSnapshotPsw] = regs.psw;
        block[CpuSnapshotSp] = regs.sp;
        block[CpuSnapshotDpl] = regs.dpl;
        block[CpuSnapshotDph] = regs.dph;
        block[CpuSnapshotMpage] = regs.mpage;
        block[CpuSnapshotFlags] =
            static_cast<std::uint8_t>((device_.sleeping() ? 1u : 0u) |
                                      (device_.in_interrupt() ? 2u : 0u) |
                                      (device_.idle_forever() ? 4u : 0u));
        for (unsigned i = 0; i < 8; ++i)
            block[CpuSnapshotR0 + i] = regs.r[i];
        put_le(block, CpuSnapshotPhysicalPc,
               device_.code_physical(regs.pc).value_or(0xffffffffu), 4);
        put_le(block, CpuSnapshotCycles, regs.cycles, 8);
        put_le(block, CpuSnapshotInstructions, regs.instructions, 8);
        put_le(block, CpuSnapshotInterrupts, device_.diagnostics().interrupts_taken, 4);
        block[CpuSnapshotDpl1] = regs.dpl1;
        block[CpuSnapshotDph1] = regs.dph1;
        block[CpuSnapshotDps] = regs.dps;
        block[CpuSnapshotIrqLevels] = regs.interrupt_levels;
        put_le(block, CpuSnapshotPostedVector, regs.posted_vector, 2);
        block[CpuSnapshotPostedLevel] = regs.posted_level;
        block[CpuSnapshotIrqInhibit] = regs.interrupt_inhibit ? 1 : 0;
        return block[address - CpuSnapshotWindow];
    }
    if (address >= FlashWindow && address < FlashWindow + mcs51emu::Cc2530F256::FlashSize)
        return device_.read_flash(address - FlashWindow);
    throw std::runtime_error("CC2530F256: no memory window at " + hex(address, 8));
}

void Cc2530F256Mcu::read_memory(std::uint32_t address, std::uint8_t *out,
                                std::size_t length) {
    if (length > std::numeric_limits<std::uint32_t>::max() - address)
        throw std::runtime_error("CC2530F256: read wraps the address space");
    // Validate the whole range first so a failed read leaves `out` untouched.
    for (std::size_t i = 0; i < length; ++i)
        (void)read_byte(address + static_cast<std::uint32_t>(i));
    for (std::size_t i = 0; i < length; ++i)
        out[i] = read_byte(address + static_cast<std::uint32_t>(i));
}

void Cc2530F256Mcu::write_memory(std::uint32_t address, const std::uint8_t *data,
                                 std::size_t length) {
    const auto inside = [&](std::uint32_t start, std::uint32_t size) {
        return address >= start && length <= size && address - start <= size - length;
    };
    if (inside(XdataWindow, mcs51emu::Cc2530F256::SramSize)) {
        for (std::size_t i = 0; i < length; ++i)
            device_.write_sram(static_cast<std::uint16_t>(address - XdataWindow + i), data[i]);
        return;
    }
    if (inside(IdataWindow, 0x100)) {
        for (std::size_t i = 0; i < length; ++i)
            device_.write_sram(static_cast<std::uint16_t>(
                                   mcs51emu::Cc2530F256::IdataXdataAlias +
                                   (address - IdataWindow + i)),
                               data[i]);
        return;
    }
    throw std::runtime_error("CC2530F256: writes are limited to SRAM (" +
                             hex(XdataWindow, 8) + "+0x2000) and IDATA (" +
                             hex(IdataWindow, 8) + "+0x100)");
}

} // namespace sim
