// The EFR32MG21 radio state machine as the zigbee-rs efr32s2-hal driver drives it from the CPU, without RAIL
// or sequencer microcode. What is modelled and what is inferred is listed with the class (blocks.hpp).

#include "efr32/blocks.hpp"

#include <algorithm>

namespace efr32emu {

namespace {

// Secure-view addresses (the 0x5/0xB non-secure aliases fold onto them before the call).
constexpr std::uint32_t Frc = 0xA8004000u, FrcIf = Frc + 0x068, FrcIen = Frc + 0x06C;
constexpr std::uint32_t AgcRssi = 0xA800C010u;
constexpr std::uint32_t Synth = 0xA8018000u, SynthCmd = Synth + 0x00C, SynthFreq = Synth + 0x034,
                        SynthIffreq = Synth + 0x038, SynthVcotuning = Synth + 0x04C, SynthIf = Synth + 0x078,
                        SynthLocntCtrl = Synth + 0x088, SynthLocntStatus = Synth + 0x08C;
constexpr std::uint32_t Rac = 0xA8020000u, RacRxEnSrcEn = Rac + 0x008, RacStatus = Rac + 0x00C, RacCmd = Rac + 0x010,
                        RacVectAddr = Rac + 0x064, RacSeqCtrl = Rac + 0x068, RacRfStatus = Rac + 0x098,
                        RacPaEnCtrl = Rac + 0x0A4;
constexpr std::uint32_t Bufc = 0x44004000u;
constexpr std::uint32_t BufCtrl = 0x00, BufAddr = 0x04, BufWriteOffset = 0x08, BufReadOffset = 0x0C, BufStatus = 0x20,
                        BufCmd = 0x28;
constexpr std::uint32_t buf(unsigned n) { return Bufc + 0x08 + 0x30 * n; }

constexpr std::uint32_t SeqRam = 0xA0000000u, SeqRamEnd = 0xA0004000u;
constexpr unsigned SeqSlots = 16;
constexpr std::uint32_t SeqEnd = 0xF800F800u, SeqJump = 0x9802u;

constexpr std::uint32_t FrcIfTxDone = 1u << 0, FrcIfTxUnderflow = 1u << 3, FrcIfRxDone = 1u << 4,
                        FrcIfRxOverflow = 1u << 8;
constexpr std::uint32_t RacCmdTxEn = 1u << 0, RacCmdClearRxOverflow = 1u << 6;
constexpr std::uint32_t SynthCmdStart = 1u << 0, SynthCmdStop = 1u << 1, SynthCmdEnableIf = 1u << 2,
                        SynthCmdDisableIf = 1u << 3;
constexpr std::uint32_t LocntClear = 1u << 1, LocntRun = 1u << 2, SynthIfLocntDone = 1u << 9;
constexpr std::uint32_t BufCmdClear = 1u << 0, BufOffsetMask = 0x1FFF;
constexpr unsigned FrcIrq = 34;

// The receiver polls the medium every 16 us (one symbol), as the scaffold radio does.
constexpr std::uint64_t PollNs = 16000;
// INFERRED: AGC/RSSI-trailer level minus antenna level (see the class comment).
constexpr int RssiOffsetDb = 61;
// SYNTH.FREQ units: Hz * 2^19 / 38.4 MHz.
constexpr std::uint64_t synth_units(std::uint64_t hz) { return (hz << 19) / 38400000u; }
constexpr std::uint64_t OpenLoopHigh = synth_units(2520000000u), OpenLoopLow = synth_units(2360000000u);
constexpr std::uint32_t VcoTuningMax = 0x7FF;

std::uint32_t fold(std::uint32_t address) {
    const std::uint32_t top = address & 0xF0000000u;
    return (top == 0x50000000u || top == 0xB0000000u) ? address - 0x10000000u : address;
}

// Register blocks resolve the set/clear/toggle aliases; sequencer RAM is plain memory.
std::uint32_t register_of(std::uint32_t address) {
    return (address >= SeqRam && address < SeqRamEnd) ? address : address & ~0x3000u;
}

} // namespace

std::uint64_t RsmRadio::now() const { return chip_.core().cycles(); }

void RsmRadio::reset() {
    active_ = false;
    state_ = Off;
    tx_end_ = next_poll_ = Never;
    listening_since_ns_ = 0;
    synth_running_ = synth_if_ = false;
    locnt_ = 0;
}

// Takes over the radio once the firmware loaded a vector table whose states either end at once or
// park on a self-jump (the driver's seq_init writes the slots, then VECTADDR, then SEQCTRL).
void RsmRadio::check_vector_table() {
    if (active_ || !(regs_.stored(RacSeqCtrl) & 1u))
        return;
    const std::uint32_t table = fold(regs_.stored(RacVectAddr));
    if (table < SeqRam || table + 4 * SeqSlots > SeqRamEnd)
        return;
    for (unsigned slot = 0; slot < SeqSlots; ++slot) {
        const std::uint32_t address = table + 4 * slot;
        const std::uint32_t word = regs_.peek(address);
        const bool self_jump = (word & 0xFFFFu) == SeqJump && (word >> 16) == address - SeqRam;
        if (word != SeqEnd && !self_jump)
            return;
    }
    active_ = true;
    state_ = Off;
}

bool RsmRadio::held(State state) const {
    const std::uint32_t slot = fold(regs_.stored(RacVectAddr)) + 4 * static_cast<std::uint32_t>(state);
    const std::uint32_t word = regs_.peek(slot);
    return (word & 0xFFFFu) == SeqJump && (word >> 16) == slot - SeqRam;
}

std::uint8_t RsmRadio::channel() const {
    // SYNTH.FREQ is the carrier: 2405 MHz + 5 MHz * (channel - 11).
    const std::uint64_t freq = regs_.stored(SynthFreq);
    const std::uint64_t hz = freq * 38400000u >> 19;
    if (hz < 2405000000u - 2500000u)
        return 11;
    return static_cast<std::uint8_t>(std::min<std::uint64_t>(26, 11 + (hz - 2405000000u + 2500000u) / 5000000u));
}

std::uint32_t RsmRadio::lo_count() const {
    const unsigned numcycle = (regs_.stored(SynthLocntCtrl) >> 4) & 0xFu;
    const unsigned shift = numcycle <= 17 ? 17 - numcycle : 0;
    std::uint64_t lo = 0;
    if (synth_running_) {
        lo = regs_.stored(SynthFreq);
        if (synth_if_) {
            const std::uint32_t iffreq = regs_.stored(SynthIffreq);
            lo = (iffreq & (1u << 20)) ? lo + (iffreq & 0xFFFFFu) : lo - (iffreq & 0xFFFFFu);
        }
    } else {
        const std::uint32_t tuning = std::min(regs_.stored(SynthVcotuning) & 0x7FFu, VcoTuningMax);
        lo = OpenLoopHigh - (OpenLoopHigh - OpenLoopLow) * tuning / VcoTuningMax;
    }
    return static_cast<std::uint32_t>(lo >> shift) & 0x7FFFFu;
}

void RsmRadio::clear_buffer(std::uint32_t buffer) {
    regs_.write(buffer + BufWriteOffset, 0);
    regs_.write(buffer + BufReadOffset, 0);
}

void RsmRadio::update_irq() {
    chip_.core().set_irq_line(FrcIrq, (regs_.stored(FrcIf) & regs_.stored(FrcIen)) != 0);
}

void RsmRadio::enter(State state, std::uint64_t at) {
    state_ = state;
    next_poll_ = Never;
    if (state == RxSearch) {
        listening_since_ns_ = chip_.core().cycles_to_ns(at);
        next_poll_ = at + chip_.core().ns_to_cycles(PollNs);
        chip_.request_service(next_poll_);
    }
}

void RsmRadio::start_transmit(std::uint64_t at) {
    const std::uint32_t tx = buf(0);
    const std::uint32_t start = regs_.stored(tx + BufAddr) + (regs_.stored(tx + BufReadOffset) & BufOffsetMask);
    std::uint8_t phr = 0;
    std::vector<std::uint8_t> fifo;
    if (chip_.core().read_memory(start, &phr, 1) && phr >= 2) {
        fifo.resize(1u + phr);
        if (!chip_.core().read_memory(start, fifo.data(), fifo.size()))
            fifo.clear();
    }
    if (fifo.empty()) {
        regs_.set_bits(FrcIf, FrcIfTxUnderflow);
        enter(TxPd, at);
        return;
    }
    const std::uint64_t airtime_ns = air_.transmit_fifo(fifo, channel());
    tx_end_ = at + chip_.core().ns_to_cycles(airtime_ns);
    enter(Tx, at);
    chip_.request_service(tx_end_);
}

void RsmRadio::advance(std::uint64_t at) {
    for (;;) {
        if (state_ == RxWarm && !held(RxWarm)) {
            enter(RxSearch, at);
        } else if (state_ == TxWarm && !held(TxWarm)) {
            start_transmit(at);
        } else if (state_ == Tx && at >= tx_end_) {
            tx_end_ = Never;
            regs_.set_bits(FrcIf, FrcIfTxDone);
            enter(TxPd, at);
        } else if (state_ == TxPd && !held(TxPd)) {
            enter((regs_.stored(RacRxEnSrcEn) & 0xFFu) ? RxWarm : Off, at);
        } else {
            break;
        }
    }
    update_irq();
}

void RsmRadio::deliver(const mcuemu::RadioFrame &frame) {
    const std::uint32_t rx = buf(1);
    const std::uint32_t size = 64u << std::min<std::uint32_t>(regs_.stored(rx + BufCtrl) & 7u, 6u);
    const std::uint32_t write = regs_.stored(rx + BufWriteOffset) & BufOffsetMask;
    const std::uint32_t read = regs_.stored(rx + BufReadOffset) & BufOffsetMask;
    const std::uint32_t used = (write - read) & BufOffsetMask;
    std::vector<std::uint8_t> stored;
    stored.reserve(frame.payload.size() + 2);
    stored.push_back(static_cast<std::uint8_t>(frame.payload.size() + 2)); // PHR counts the FCS
    stored.insert(stored.end(), frame.payload.begin(), frame.payload.end());
    stored.push_back(static_cast<std::uint8_t>(std::clamp(frame.rssi_dbm + RssiOffsetDb, -128, 127)));
    if (used + stored.size() > size) {
        regs_.set_bits(FrcIf, FrcIfRxOverflow);
        enter(RxOverflow, now());
        return;
    }
    // The buffer RAM: BUFn.ADDR is an offset from RAMBASEADDR (reset 0x20000000) on xG21.
    const std::uint32_t base = 0x20000000u | (regs_.stored(rx + BufAddr) & 0xFFFFFFu);
    for (std::size_t i = 0; i < stored.size(); ++i) {
        const std::uint32_t address = base + (write + static_cast<std::uint32_t>(i)) % size;
        chip_.core().write_memory(address, &stored[i], 1);
    }
    regs_.write(rx + BufWriteOffset, (write + static_cast<std::uint32_t>(stored.size())) & BufOffsetMask);
    regs_.set_bits(FrcIf, FrcIfRxDone);
}

void RsmRadio::poll(std::uint64_t at) {
    if (state_ != RxSearch)
        return;
    for (const mcuemu::RadioFrame &frame :
         air_.receive_since(channel(), chip_.core().cycles_to_ns(at), listening_since_ns_)) {
        if (state_ != RxSearch)
            break;
        deliver(frame);
    }
    if (state_ == RxSearch) {
        next_poll_ = at + chip_.core().ns_to_cycles(PollNs);
        chip_.request_service(next_poll_);
    }
}

std::uint64_t RsmRadio::service(std::uint64_t at) {
    if (!active_)
        return Never;
    advance(at);
    if (state_ == RxSearch && at >= next_poll_)
        poll(at);
    update_irq();
    std::uint64_t next = Never;
    if (state_ == Tx)
        next = tx_end_;
    if (state_ == RxSearch)
        next = std::min(next, next_poll_);
    return next;
}

void RsmRadio::written(std::uint32_t address, std::uint32_t value) {
    const std::uint32_t reg = register_of(address);
    if (reg == RacSeqCtrl || reg == RacVectAddr)
        check_vector_table();
    if (!active_)
        return;
    const std::uint64_t at = now();
    advance(at);
    switch (reg) {
    case RacRxEnSrcEn: {
        const bool requested = (regs_.stored(RacRxEnSrcEn) & 0xFFu) != 0;
        if (requested && state_ == Off) {
            enter(RxWarm, at);
        } else if (!requested && (state_ == RxWarm || state_ == RxSearch)) {
            poll(at); // frames that ended before the receiver stopped
            enter(Off, at);
        }
        break;
    }
    case RacCmd:
        if ((value & RacCmdTxEn) && (state_ == Off || state_ == RxWarm || state_ == RxSearch)) {
            poll(at);
            enter(TxWarm, at);
        }
        if ((value & RacCmdClearRxOverflow) && state_ == RxOverflow)
            enter((regs_.stored(RacRxEnSrcEn) & 0xFFu) ? RxSearch : Off, at);
        break;
    case SynthCmd:
        if (value & SynthCmdStart)
            synth_running_ = true;
        if (value & SynthCmdStop)
            synth_running_ = false;
        if (value & SynthCmdEnableIf)
            synth_if_ = true;
        if (value & SynthCmdDisableIf)
            synth_if_ = false;
        break;
    case SynthLocntCtrl: {
        const std::uint32_t control = regs_.stored(SynthLocntCtrl);
        if (control & LocntClear)
            locnt_ = 0;
        if (control & LocntRun) {
            locnt_ = lo_count();
            regs_.set_bits(SynthIf, SynthIfLocntDone);
        }
        break;
    }
    case buf(0) + BufCmd:
    case buf(1) + BufCmd:
        if (value & BufCmdClear)
            clear_buffer(reg - BufCmd);
        break;
    default: break;
    }
    advance(at);
}

bool RsmRadio::read(std::uint32_t address, std::uint32_t &value) {
    if (!active_)
        return false;
    const std::uint64_t at = now();
    advance(at);
    const std::uint32_t reg = register_of(address);
    switch (reg) {
    case RacStatus:
        value = (regs_.stored(RacStatus) & ~(0xFu << 24)) | (static_cast<std::uint32_t>(state_) << 24);
        return true;
    case RacRfStatus:
        value = (regs_.stored(RacRfStatus) & ~1u) | ((regs_.stored(RacPaEnCtrl) >> 8) & 1u);
        return true;
    case AgcRssi:
        if (state_ == RxSearch) {
            const int raw = air_.energy_dbm(chip_.core().cycles_to_ns(at), channel()) + RssiOffsetDb;
            value = (static_cast<std::uint32_t>(raw * 4) & 0x3FFu) << 6;
        } else {
            value = 0x8000u;
        }
        return true;
    case SynthLocntStatus:
        value = locnt_; // BUSY (bit 19) clear: the count is ready at once
        return true;
    case buf(0) + BufStatus:
    case buf(1) + BufStatus: {
        const std::uint32_t buffer = reg - BufStatus;
        value = (regs_.stored(buffer + BufWriteOffset) - regs_.stored(buffer + BufReadOffset)) & BufOffsetMask;
        return true;
    }
    default: return false;
    }
}

} // namespace efr32emu
