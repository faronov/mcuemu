// CMU, HFXO, MSC and GPIO of the EFR32MG21 (Series 2). Sources: the EFR32xG21
// reference manual register layouts as used by the zigbee-rs HAL
// (efr32mg21-hal: clock.rs, flash.rs, gpio.rs). What the manual does not
// settle is marked "inferred".

#include "efr32/blocks.hpp"
#include "crypto/aes128.hpp"
#include <algorithm>
#include "efr32/rail_map.hpp"

#include <algorithm>
#include <cstring>

namespace efr32emu {

std::uint64_t Block::now() const { return chip_.core().cycles(); }
std::uint64_t Block::clock_hz() const { return chip_.core().clock_hz(); }
void Block::set_irq_level(unsigned irq, bool level) { chip_.core().set_irq_line(irq, level); }

// ---- CMU, HFXO -----------------------------------------------------------------

namespace {
constexpr std::uint32_t LdmaChen = 0x24, LdmaChdis = 0x28, LdmaChstatus = 0x2c, LdmaChdone = 0x34,
                        LdmaReqdis = 0x40, LdmaLinkload = 0x48, LdmaIf = 0x50, LdmaIen = 0x54;
constexpr std::uint32_t ChCtrl = 0x08, ChSrc = 0x0c, ChDst = 0x10, ChLink = 0x14;
// RXDATA (+0x24) and RXDATAX (+0x20) of USART0 in the non-secure and secure views.
bool is_usart0_rxdata(std::uint32_t address) {
    const std::uint32_t reg = address & 0xfffu;
    return (address & ~0xfffu) % 0x10000000u == 0x58000u && (reg == 0x24 || reg == 0x20);
}
} // namespace

void Ldma::reset() {
    RegBlock::reset();
    enabled_ = 0;
    update_irq();
}

std::uint32_t Ldma::read(std::uint32_t reg) {
    if (reg == LdmaChstatus)
        return enabled_;
    return RegBlock::read(reg);
}

void Ldma::update_irq() { set_irq_level(Irq, (regs_[LdmaIf >> 2] & regs_[LdmaIen >> 2]) != 0); }

void Ldma::load_descriptor(unsigned ch, std::uint32_t address) {
    std::uint32_t words[4] = {0, 0, 0, 0};
    if (!chip().core().read_memory(address, words, sizeof words))
        return;
    desc_addr_[ch] = address;
    set_channel_reg(ch, ChCtrl, words[0]);
    set_channel_reg(ch, ChSrc, words[1]);
    set_channel_reg(ch, ChDst, words[2]);
    set_channel_reg(ch, ChLink, words[3]);
}

void Ldma::write(std::uint32_t reg, std::uint32_t value) {
    if (reg == LdmaChdis) {
        enabled_ &= ~value;
        return;
    }
    if (reg == LdmaChen) {
        enabled_ |= value;
        return;
    }
    if (reg == LdmaLinkload) {
        for (unsigned ch = 0; ch < Channels; ++ch) {
            if (!(value & (1u << ch)))
                continue;
            // The NCP's UART driver parks the receive channel on a "dummy" descriptor (source and
            // destination the same RAM byte, linked back to the receive descriptor) to freeze the
            // write pointer, and tells "ring empty" by DST == that RAM byte. So a link load takes
            // effect at once, even for an enabled channel. LINK: bit 1 = link valid, bit 0 =
            // relative address (xG21 reference manual, CHx_LINK; the firmware tests bit 1).
            load_descriptor(ch, channel_reg(ch, ChLink) & ~3u);
            enabled_ |= 1u << ch;
        }
        return;
    }
    RegBlock::write(reg, value);
    if (reg == LdmaIf || reg == LdmaIen)
        update_irq();
}

// One transfer of the descriptor on `ch` has happened; completes it when the count runs out.
void Ldma::transfer_done(unsigned ch) {
    std::uint32_t ctrl = channel_reg(ch, ChCtrl);
    const std::uint32_t count = (ctrl >> 4) & 0x7ffu;
    if (count != 0) {
        set_channel_reg(ch, ChCtrl, (ctrl & ~(0x7ffu << 4)) | ((count - 1) << 4));
        return;
    }
    if (ctrl & (1u << 20))
        regs_[LdmaIf >> 2] |= 1u << ch;
    const std::uint32_t link = channel_reg(ch, ChLink);
    if (link & 2u) {
        std::uint32_t address = link & ~3u;
        if (link & 1u)
            address += desc_addr_[ch];
        load_descriptor(ch, address);
    } else {
        // CHDONE means the whole chain is finished, not one descriptor: the NCP's UART driver
        // rebuilds "the rest of the running descriptor" from XFERCNT unless CHDONE is set, and a
        // CHDONE raised by the parking descriptor made it build a 2048-byte one that overran the ring.
        regs_[LdmaChdone >> 2] |= 1u << ch;
        enabled_ &= ~(1u << ch);
    }
    update_irq();
}

bool Ldma::usart0_rx(std::uint8_t byte) {
    for (unsigned ch = 0; ch < Channels; ++ch) {
        // REQDIS masks a channel's peripheral requests: the NCP's driver sets it around its
        // read-pointer snapshot, and the byte waits in the USART until it is cleared again.
        if (!(enabled_ & (1u << ch)) || (regs_[LdmaReqdis >> 2] & (1u << ch)))
            continue;
        // A memory-to-memory no-op descriptor (the parking one) takes the request without
        // touching RXDATA; the byte stays pending for the descriptor it links to.
        for (unsigned guard = 0; guard < 4 && (enabled_ & (1u << ch)) &&
                                 channel_reg(ch, ChSrc) == channel_reg(ch, ChDst);
             ++guard)
            transfer_done(ch);
        if (!(enabled_ & (1u << ch)) || !is_usart0_rxdata(channel_reg(ch, ChSrc)))
            continue;
        const std::uint32_t ctrl = channel_reg(ch, ChCtrl);
        const std::uint32_t dst = channel_reg(ch, ChDst);
        chip().core().write_memory(dst, &byte, 1);
        if (((ctrl >> 28) & 3u) == 0) // DSTINC: one byte
            set_channel_reg(ch, ChDst, dst + 1);
        transfer_done(ch);
        return true;
    }
    return false;
}

void Emu::reset() {
    RegBlock::reset();
    regs_[Rstcause >> 2] = chip().soft_reset() ? SystemRequestReset : PowerOnReset;
    regs_[Temp >> 2] = RoomTemperature;
}

void Cmu::reset() {
    RegBlock::reset();
    regs_[Sysclkctrl >> 2] = 1; // FSRCO selected out of reset
}

namespace {
constexpr std::uint32_t HfxoCtrl = 0x028, HfxoXtalctrl = 0x018, HfxoStatus = 0x058;
constexpr std::uint32_t ClkselHfxo = 3;
} // namespace

std::uint32_t Hfxo::read(std::uint32_t reg) {
    if (reg != HfxoStatus)
        return RegBlock::read(reg);
    // CTRL.FORCEEN keeps the oscillator on; otherwise it runs on demand while
    // it is the system clock unless DISONDEMAND is set. Start-up takes no time
    // (inferred). FSMLOCK is the first-lock optimisation: on while the
    // oscillator runs with DISONDEMAND clear and the core-bias optimisation
    // has not been skipped (inferred from how the HAL waits on it).
    const std::uint32_t ctrl = regs_[HfxoCtrl >> 2];
    const bool force = ctrl & 1u, disondemand = ctrl & 2u;
    const bool enabled = force || (!disondemand && cmu_.clock_select() == ClkselHfxo);
    std::uint32_t status = 0;
    if (enabled)
        status |= (1u << 0) | (1u << 1) | (1u << 16);
    if (enabled && !disondemand && !(regs_[HfxoXtalctrl >> 2] >> 31))
        status |= 1u << 30;
    return status;
}

// ---- DPLL ------------------------------------------------------------------------

namespace {
constexpr std::uint32_t DpllEn = 0x004, DpllIf = 0x010, DpllStatus = 0x018;
} // namespace

std::uint32_t Dpll::read(std::uint32_t reg) {
    const bool enabled = regs_[DpllEn >> 2] & 1u;
    if (reg == DpllIf)
        return RegBlock::read(reg) | (enabled ? 1u : 0u);
    if (reg == DpllStatus)
        return enabled ? (1u | 2u | (1u << 31)) : 0u; // RDY, ENS, LOCK
    return RegBlock::read(reg);
}

// ---- TIMER -----------------------------------------------------------------------

namespace {
constexpr std::uint32_t TimerCfg = 0x004, TimerCmd = 0x00c, TimerStatus = 0x010, TimerTop = 0x01c,
                        TimerCnt = 0x024, TimerEn = 0x030;
constexpr std::uint32_t TimerCmdStart = 1u << 0, TimerCmdStop = 1u << 1, TimerStatusRunning = 1u << 0;
constexpr std::uint32_t TimerTopReset = 0xffff; // _TIMER_TOP_RESETVALUE, on the 32-bit TIMER0 too
} // namespace

std::uint32_t Timer::count(std::uint64_t at) const {
    if (!running_ || at <= cycle_at_)
        return count_at_;
    const std::uint64_t divisor = ((regs_[TimerCfg >> 2] >> 18) & 0x3ffu) + 1u;
    const std::uint64_t top = (regs_[TimerTop >> 2] & mask_) + 1ull;
    return static_cast<std::uint32_t>((count_at_ + (at - cycle_at_) / divisor) % top);
}

void Timer::freeze() {
    const std::uint64_t n = now();
    count_at_ = count(n);
    cycle_at_ = n;
}

std::uint32_t Timer::read(std::uint32_t reg) {
    if (reg == TimerCnt)
        return count(now());
    if (reg == TimerStatus)
        return running_ ? TimerStatusRunning : 0u;
    return RegBlock::read(reg);
}

void Timer::write(std::uint32_t reg, std::uint32_t value) {
    switch (reg) {
    case TimerCmd:
        freeze();
        if ((value & TimerCmdStart) && (regs_[TimerEn >> 2] & 1u))
            running_ = true;
        if (value & TimerCmdStop)
            running_ = false;
        return;
    case TimerCnt:
        freeze();
        count_at_ = value & mask_;
        return;
    case TimerEn:
        freeze();
        if (!(value & 1u))
            running_ = false;
        RegBlock::write(reg, value);
        return;
    case TimerCfg:
        freeze();
        RegBlock::write(reg, value);
        return;
    case TimerTop:
        freeze();
        RegBlock::write(reg, value & mask_);
        return;
    default: RegBlock::write(reg, value); return;
    }
}

void Timer::reset() {
    RegBlock::reset();
    regs_[TimerTop >> 2] = TimerTopReset;
    count_at_ = 0;
    cycle_at_ = 0;
    running_ = false;
}

// ---- USART -----------------------------------------------------------------------

namespace {
constexpr std::uint32_t UsartStatus = 0x018, UsartTxdataX = 0x038, UsartTxdata = 0x03c;
constexpr std::size_t UartOutputLimit = 1u << 20;
constexpr std::uint32_t UsartStatusTxc = 1u << 5, UsartStatusTxbl = 1u << 6, UsartStatusTxidle = 1u << 13;
} // namespace

std::uint32_t Usart::read(std::uint32_t reg) {
    if (reg == UsartStatus)
        return UsartStatusTxc | UsartStatusTxbl | UsartStatusTxidle;
    return RegBlock::read(reg);
}

void Usart::write(std::uint32_t reg, std::uint32_t value) {
    if (reg == UsartTxdata || reg == UsartTxdataX) {
        // A run that nobody drains stops recording instead of growing without bound.
        if (chip().uart0_output().size() < UartOutputLimit)
            chip().uart0_output().push_back(static_cast<char>(value & 0xffu));
        return;
    }
    RegBlock::write(reg, value);
}

// ---- SEMAILBOX host --------------------------------------------------------------

namespace {
constexpr std::uint32_t SeFifo = 0x000, SeTxStatus = 0x040, SeRxStatus = 0x044, SeTxHeader = 0x050,
                        SeRxHeader = 0x054;
constexpr std::uint32_t SeTxint = 1u << 20, SeRxint = 1u << 20;
} // namespace

std::uint32_t SeMailbox::read(std::uint32_t reg) {
    if (reg == SeTxStatus)
        return SeTxint; // room in the TX FIFO
    return RegBlock::read(reg);
}

void SeMailbox::write(std::uint32_t reg, std::uint32_t value) {
    if (reg == SeFifo) {
        const std::uint32_t words = regs_[SeTxHeader >> 2] / 4;
        fifo_.push_back(value);
        // The header counts the command's words, including one that is not
        // stored (the handle word), so the command is complete one word early.
        if (++words_ >= (words > 1 ? words - 1 : 1)) {
            words_ = 0;
            execute();
            fifo_.clear();
            regs_[SeRxHeader >> 2] = 0; // SLI_SE_RESPONSE_OK
            regs_[SeRxStatus >> 2] |= SeRxint;
        }
        return;
    }
    if (reg == SeTxHeader) {
        words_ = 0;
        fifo_.clear();
    }
    RegBlock::write(reg, value);
}

// The command stream is [command, data_in chain, data_out chain, parameters...] (sli_se_manager_mailbox.c).
// A chain element is {address, next, length}; next == 1 ends it (SLI_SE_DATATRANSFER_STOP, inferred from the
// NCP's descriptors) and the length word carries flag bits above bit 27. AES-ECB (0x04000100 encrypt, 0x04010100 decrypt, inferred from the NCP) takes
// the plain key as the first input element and the data as the last, and writes the result to the output.
void SeMailbox::execute() {
    if (fifo_.size() < 3 || (fifo_[0] != 0x04000100u))
        return; // only AES-ECB encryption is modelled; other commands just succeed
    struct Element {
        std::uint32_t address, length;
    };
    auto chain = [&](std::uint32_t head) {
        std::vector<Element> elements;
        for (unsigned guard = 0; guard < 6 && head != 0; ++guard) {
            std::uint32_t d[3] = {};
            if (!chip().core().read_memory(head, d, sizeof d))
                break;
            elements.push_back({d[0], d[2] & 0x0fffffffu});
            if ((d[1] & 1u) != 0 || d[1] == 0) // next == 1 is SLI_SE_DATATRANSFER_STOP
                break;
            head = d[1];
        }
        return elements;
    };
    const auto in = chain(fifo_[1]);
    const auto out = chain(fifo_[2]);
    if (in.size() < 2 || out.empty() || in.back().length == 0 || in.back().length % 16 != 0)
        return;
    const Element &key = in[in.size() - 2];
    if (key.length != 16)
        return;
    std::uint8_t key_bytes[16];
    std::vector<std::uint8_t> data(in.back().length);
    if (!chip().core().read_memory(key.address, key_bytes, 16) ||
        !chip().core().read_memory(in.back().address, data.data(), data.size()))
        return;
    crypto::Block k;
    std::copy(key_bytes, key_bytes + 16, k.begin());
    for (std::size_t offset = 0; offset < data.size(); offset += 16) {
        crypto::Block block;
        std::copy(data.begin() + offset, data.begin() + offset + 16, block.begin());
        const crypto::Block encrypted = crypto::aes128_encrypt(k, block);
        std::copy(encrypted.begin(), encrypted.end(), data.begin() + offset);
    }
    chip().core().write_memory(out[0].address, data.data(), std::min<std::size_t>(data.size(), out[0].length));
}

void SeMailbox::reset() {
    RegBlock::reset();
    words_ = 0;
}

// ---- RAIL-driven radio registers -----------------------------------------------

namespace {
// RAC + 0x0C, state code in [27:24]. Inferred: 0 is idle (the loop after the RAC
// enable waits for it to be 0), and 7..9 and 12 are the active states that
// rfhal_rac accepts. No hook yet: the state follows what is stored.
using rail::RacStatus;
// SEQRAM + 0x1EA0, polled with masks 0x800, 0x20000 and 0x40000. Inferred: these
// are "operation done" bits the sequencer would set. Forced set on every read
// for now, to see which waits they release.
constexpr std::uint32_t SeqStatus = 0xA0001EA0;
constexpr std::uint32_t SeqReadyBits = (1u << 11) | (1u << 17);

// RDMEM (0xA0000000) is RAM: no set/clear/toggle aliases. The register blocks
// (RAC, BUFC, ...) use the Series 2 aliases: +0x1000 set, +0x2000 clear, +0x3000 toggle.
bool is_rdmem(std::uint32_t address) { return address >= 0xA0000000u && address < 0xA0004000u; }
std::uint32_t register_base(std::uint32_t address) { return is_rdmem(address) ? address : address & ~0x3000u; }
} // namespace

namespace {
constexpr std::uint32_t IadcCmd = 0x0c, IadcStatus = 0x14, IadcIf = 0x24, IadcSingleFifoData = 0x74,
                        IadcSingleData = 0x7c, IadcScanFifoData = 0x84, IadcScanData = 0x8c;
constexpr std::uint32_t IadcResult = 0x8000;
} // namespace

std::uint32_t Iadc::peek(std::uint32_t reg) {
    if (reg == IadcStatus)
        return RegBlock::read(reg) | (single_dv_ ? 0x100u : 0u) | (scan_dv_ ? 0x200u : 0u);
    return RegBlock::read(reg);
}

std::uint32_t Iadc::read(std::uint32_t reg) {
    switch (reg) {
    case IadcSingleFifoData:
        single_dv_ = false;
        return IadcResult;
    case IadcScanFifoData:
        scan_dv_ = false;
        return IadcResult;
    case IadcSingleData:
    case IadcScanData:
        return IadcResult;
    default: return peek(reg);
    }
}

void Iadc::write(std::uint32_t reg, std::uint32_t value) {
    if (reg == IadcCmd) {
        if (value & 1u) {
            single_dv_ = true;
            regs_[IadcIf >> 2] |= (1u << 9) | 1u;
        }
        if (value & 8u) {
            scan_dv_ = true;
            regs_[IadcIf >> 2] |= (1u << 8) | 2u;
        }
        return;
    }
    if (reg == IadcStatus)
        return;
    RegBlock::write(reg, value);
}

std::uint32_t RailRegs::read(std::uint32_t address) {
    const std::uint32_t base = register_base(address);
    const auto it = words_.find(base);
    std::uint32_t value = it != words_.end() ? it->second : 0;
    if (address == SeqStatus)
        value |= SeqReadyBits;
    // BUFC BUF0 STATUS (+0x28): the byte count in bits 12:0, WRITEOFFSET - READOFFSET. INFERRED from the
    // Tuya ZS3L image, whose RAIL peeks byte 3 or 4 of the TX frame in BUF0 before it starts a transmit and
    // fails with EMBER_PHY_TX_BUSY (0x8C) when the count is zero: the frame is copied into the buffer RAM and
    // WRITEOFFSET set to its length, so the count has to follow the offsets.
    if (address == rail::Buf0Status && it == words_.end()) {
        const auto count = [&](std::uint32_t reg) {
            const auto word = words_.find(reg);
            return word != words_.end() ? word->second : 0u;
        };
        value = (count(rail::Buf0WriteOffset) - count(rail::Buf0ReadOffset)) & 0x1fffu;
    }
    // INFERRED: a Tuya EFR32MG21 image (ZS3L thermometer) reads 0xA801803C before the radio configuration
    // table is applied and multiplies its three 3-bit fields (the synthesizer divider); a product of zero
    // makes it assert (udf at 0xef3c). The NCP writes 1 to this word, so an unwritten word reads as 1.
    if (address == 0xA801803C && it == words_.end())
        value = 1;
    // INFERRED stand-ins, not the silicon: the RAIL init in the Silicon Labs Zigbee NCP 8.2.2 image
    // (pc 0x2e2f4) polls FRC IF (0xA8004068) for bit 14 and FRC+0x08 for bit 8 after it starts a
    // calibration that the RAC sequencer would run (this model does not run it). Reporting both as
    // set lets the init finish; what the bits mean on the silicon is not known here.
    if (address == 0xA8004068)
        value |= 0x4000;
    if (address == 0xA8004008)
        value |= 0x100;
    // RAC STATUS.STATE (bits 27:24): code 1 is RX (RAIL state 2), 0 is idle.
    // INFERRED from the NCP's RAIL_GetRssi (energy scan, scan RX): once the receiver is in RXSEARCH or
    // RXFRAME it waits for a nonzero AGC STATUS1.RSSISTATE (bits 17:15) and then reads AGC RSSI
    // (RSSIINT bits 15:8, signed dBm). The model reports a valid state (3 is a guess; any nonzero
    // passes the test) and a quiet -100 dBm channel.
    if (base == rail::AgcStatus1 && (rx_ready_ || rx_frame_))
        value |= 3u << 15;
    if (base == rail::AgcRssi)
        value = (rx_ready_ || rx_frame_) ? (0x9cu << 8) : (0x80u << 8);
    if (base == RacStatus && (rx_enabled() || rx_frame_hold_))
        value = (value & ~(0xfu << 24)) | (((rx_frame_ || rx_frame_hold_) ? 3u : rx_ready_ ? 2u : 1u) << 24);
    return value;
}

std::uint32_t RailRegs::peek(std::uint32_t address) const {
    const auto it = words_.find(address);
    return it != words_.end() ? it->second : 0;
}

std::uint32_t RailRegs::stored(std::uint32_t address) const { return peek(register_base(address)); }

void RailRegs::set_bits(std::uint32_t address, std::uint32_t mask) { words_[address] |= mask; }

void RailRegs::clear_bits(std::uint32_t address, std::uint32_t mask) { words_[address] &= ~mask; }

void RailRegs::write(std::uint32_t address, std::uint32_t value, unsigned) {
    if (is_rdmem(address)) {
        words_[address] = value;
        return;
    }
    // PROTIMER CMD (image layout: header +0x1C moved to +0x0C): START (bit 0) and
    // RTCSYNCSTART (bit 1) set STATUS.RUNNING (+0x14 bit 0), STOP (bit 2) clears it.
    // The transmit path refuses to start while RUNNING is clear (sli_rail_3d0930...).
    if (register_base(address) == rail::ProtimerCmd) {
        if (value & 0x3u)
            words_[rail::ProtimerStatus] |= 1u;
        if (value & 0x4u)
            words_[rail::ProtimerStatus] &= ~1u;
    }
    std::uint32_t &word = words_[register_base(address)];
    switch ((address >> 12) & 3u) {
    case 1: word |= value; break;
    case 2: word &= ~value; break;
    case 3: word ^= value; break;
    default: word = value; break;
    }
    // Inferred: RAC+0x008 bit 5 is a request that the hardware clears when the
    // operation is done. RAIL sets it and polls it to zero (rfhal_rac), so the
    // operation is treated as completing at once.
    if (register_base(address) == rail::RacRxEnable)
        word &= ~(1u << 5);
    // RAC CMD.TXEN starts the transmitter. The frame then ends on air after its
    // duration (the chip models that end, see Efr32mg21::service).
    if (register_base(address) == rail::RacCmd && (value & 1u))
        tx_started_ = true;
}

bool RailRegs::rx_enabled() const {
    const auto it = words_.find(rail::RacRxEnable);
    // Bit 1 is the RX request of the NCP's and Zephyr's RAIL. INFERRED: the Tuya ZS3L image requests RX for its
    // listen-before-talk through bit 4 (0x10) and then polls RAC STATUS.STATE for RX before it starts the LBT.
    return it != words_.end() && (it->second & 0x12u) != 0;
}

bool RailRegs::take_tx_start() {
    const bool started = tx_started_;
    tx_started_ = false;
    return started;
}

// ---- RTCC ------------------------------------------------------------------------

namespace {
constexpr std::uint32_t RtccEn = 0x004, RtccCfg = 0x008, RtccCmd = 0x00c, RtccStatus = 0x010, RtccIf = 0x014,
                        RtccIen = 0x018, RtccCnt = 0x020, RtccCombcnt = 0x024, RtccCc0Ctrl = 0x030;
constexpr std::uint32_t RtccCmdStart = 1u << 0, RtccCmdStop = 1u << 1, RtccStatusRunning = 1u << 0, RtccIfOf = 1u << 0;
constexpr std::uint64_t RtccWrap = 1ull << 32, RtccTickHz = 32768;
// CNTPRESC above DIV64 is not a valid setting; clamping keeps the cycle arithmetic in 64 bits.
constexpr unsigned RtccMaxPresc = 6;
constexpr std::uint32_t rtcc_cc_ctrl(unsigned channel) { return RtccCc0Ctrl + 12u * channel; }
constexpr std::uint32_t rtcc_cc_ocvalue(unsigned channel) { return rtcc_cc_ctrl(channel) + 4u; }
// IF.CCn of the EFR32MG21 (CC0..CC2 at bits 2..4; efr32mg21_rtcc.h).
constexpr std::uint32_t rtcc_if_cc(unsigned channel) { return 1u << (2u + channel); }
} // namespace

bool Rtcc::counting() const { return started_ && (regs_[RtccEn >> 2] & 1u); }

std::uint64_t Rtcc::ticks(std::uint64_t at) const {
    if (!counting() || at <= cycle_at_)
        return ticks_at_;
    const unsigned presc = std::min<unsigned>((regs_[RtccCfg >> 2] >> 4) & 0xfu, RtccMaxPresc);
    return ticks_at_ + (((at - cycle_at_) * RtccTickHz / clock_hz()) >> presc);
}

// The first core cycle at which the counter reaches `tick` (rounded up, so the
// counter is there when the service runs).
std::uint64_t Rtcc::cycle_for(std::uint64_t tick) const {
    const unsigned presc = std::min<unsigned>((regs_[RtccCfg >> 2] >> 4) & 0xfu, RtccMaxPresc);
    const std::uint64_t cycles = ((tick - ticks_at_) * (clock_hz() << presc) + RtccTickHz - 1) / RtccTickHz;
    return cycle_at_ + cycles;
}

// Moves the base to the cycle where the counter reached its current value. The base
// used to move to `now`, so the fraction of a tick in progress was dropped at every
// freeze: the service runs every few thousand cycles and the counter lost about 40%.
void Rtcc::freeze() {
    const std::uint64_t n = now();
    const std::uint64_t t = ticks(n);
    const std::uint64_t anchor = std::min(cycle_for(t), n);
    ticks_at_ = t;
    cycle_at_ = anchor;
}

void Rtcc::arm() {
    std::uint64_t next = Never;
    for (unsigned channel = 0; channel < Channels; ++channel) {
        armed_tick_[channel] = Never;
        if (!counting() || (regs_[rtcc_cc_ctrl(channel) >> 2] & 3u) != 2u) // OUTPUTCOMPARE
            continue;
        // Match when the counter reaches OCVALUE + 1 (32-bit). A target equal to the
        // current count is taken as a full wrap away, not as an immediate match.
        const auto target = static_cast<std::uint32_t>(regs_[rtcc_cc_ocvalue(channel) >> 2] + 1u);
        std::uint64_t delta = static_cast<std::uint32_t>(target - static_cast<std::uint32_t>(ticks_at_));
        if (delta == 0)
            delta = RtccWrap;
        armed_tick_[channel] = ticks_at_ + delta;
        next = std::min(next, armed_tick_[channel]);
    }
    wrap_tick_ = counting() ? (ticks_at_ | (RtccWrap - 1)) + 1 : Never;
    next = std::min(next, wrap_tick_);
    if (next != Never)
        chip().request_service(cycle_for(next));
}

void Rtcc::update_irq() { set_irq_level(Irq, (regs_[RtccIf >> 2] & regs_[RtccIen >> 2]) != 0); }

std::uint32_t Rtcc::read(std::uint32_t reg) {
    if (reg == RtccCnt || reg == RtccCombcnt)
        return static_cast<std::uint32_t>(ticks(now()));
    if (reg == RtccStatus)
        return (RegBlock::read(reg) & ~RtccStatusRunning) | (counting() ? RtccStatusRunning : 0u);
    return RegBlock::read(reg);
}

void Rtcc::write(std::uint32_t reg, std::uint32_t value) {
    if (reg == RtccCmd) {
        freeze();
        if (value & RtccCmdStart)
            started_ = true;
        if (value & RtccCmdStop)
            started_ = false;
        arm();
        return;
    }
    if (reg == RtccCnt) {
        freeze();
        ticks_at_ = value;
        arm();
        return;
    }
    if (reg == RtccIf || reg == RtccIen) {
        RegBlock::write(reg, value);
        update_irq();
        return;
    }
    const bool compare = reg >= rtcc_cc_ctrl(0) && reg < rtcc_cc_ctrl(Channels) && (reg - rtcc_cc_ctrl(0)) % 12u != 8u;
    if (reg == RtccEn || reg == RtccCfg || compare) {
        freeze();
        RegBlock::write(reg, value);
        arm();
        return;
    }
    RegBlock::write(reg, value);
}

std::uint64_t Rtcc::service(std::uint64_t) {
    if (!counting())
        return Never;
    freeze();
    bool changed = false;
    if (ticks_at_ >= wrap_tick_) {
        regs_[RtccIf >> 2] |= RtccIfOf;
        changed = true;
    }
    for (unsigned channel = 0; channel < Channels; ++channel) {
        if (armed_tick_[channel] == Never || ticks_at_ < armed_tick_[channel])
            continue;
        regs_[RtccIf >> 2] |= rtcc_if_cc(channel);
        changed = true;
    }
    if (changed) {
        // Re-arm from the current count: each match fires again one counter wrap later.
        std::array<std::uint64_t, Channels> fired = armed_tick_;
        arm();
        for (unsigned channel = 0; channel < Channels; ++channel) {
            if (fired[channel] != Never && ticks_at_ >= fired[channel] && armed_tick_[channel] != Never) {
                while (armed_tick_[channel] <= ticks_at_)
                    armed_tick_[channel] += RtccWrap;
            }
        }
        update_irq();
    }
    std::uint64_t next = wrap_tick_;
    for (const std::uint64_t tick : armed_tick_)
        next = std::min(next, tick);
    return next == Never ? Never : cycle_for(next);
}

void Rtcc::reset() {
    RegBlock::reset();
    ticks_at_ = 0;
    cycle_at_ = 0;
    wrap_tick_ = Never;
    armed_tick_.fill(Never);
    started_ = false;
    update_irq();
}

// ---- MSC -------------------------------------------------------------------------

namespace {
constexpr std::uint32_t MscWritectrl = 0x00c, MscWritecmd = 0x010, MscAddrb = 0x014, MscWdata = 0x018,
                        MscStatus = 0x01c, MscLock = 0x03c;
constexpr std::uint32_t PageSize = 8192;
} // namespace

void Msc::reset() {
    RegBlock::reset();
    unlocked_ = invalid_address_ = false;
    address_ = 0;
}

std::uint32_t Msc::read(std::uint32_t reg) {
    if (reg == MscStatus) {
        std::uint32_t status = 0;
        if (invalid_address_)
            status |= 1u << 2;
        if (unlocked_ && (regs_[MscWritectrl >> 2] & 1u))
            status |= 1u << 3; // WDATAREADY: programming is instantaneous
        if (!unlocked_)
            status |= 1u << 16; // REGLOCK
        return status;
    }
    if (reg == MscLock)
        return unlocked_ ? 0 : 1;
    return RegBlock::read(reg);
}

void Msc::write(std::uint32_t reg, std::uint32_t value) {
    switch (reg) {
    case MscLock:
        unlocked_ = value == 0x1b71;
        return;
    case MscAddrb:
        if (!unlocked_)
            return;
        address_ = value;
        invalid_address_ = value >= Efr32mg21::FlashSize;
        return;
    case MscWritecmd:
        if (!unlocked_ || !(regs_[MscWritectrl >> 2] & 1u))
            return;
        if ((value & 2u) && !invalid_address_) { // ERASEPAGE
            const std::uint32_t page = address_ & ~(PageSize - 1);
            std::memset(chip_.flash() + page, 0xff, PageSize);
            chip_.flash_changed(page, PageSize);
        }
        return;
    case MscWdata:
        if (!unlocked_ || !(regs_[MscWritectrl >> 2] & 1u) || invalid_address_)
            return;
        // Flash programming can only clear bits.
        std::uint32_t old;
        std::memcpy(&old, chip_.flash() + address_, 4);
        old &= value;
        std::memcpy(chip_.flash() + address_, &old, 4);
        chip_.flash_changed(address_, 4);
        address_ += 4;
        invalid_address_ = address_ >= Efr32mg21::FlashSize;
        return;
    default:
        RegBlock::write(reg, value);
    }
}

// ---- GPIO ------------------------------------------------------------------------

namespace {
constexpr std::uint32_t PortStride = 0x30;
constexpr std::uint32_t RegCtrl = 0x00, RegModel = 0x04, RegModeh = 0x0c, RegDout = 0x10, RegDin = 0x14;
constexpr std::uint32_t RegLock = 0x300, RegExtipsel = 0x400, RegExtipinsel = 0x408, RegExtirise = 0x410,
                        RegExtifall = 0x414, RegIf = 0x420, RegIen = 0x424, RegI2cRouteEn = 0x498;
constexpr unsigned ExtiLines = 8, GpioEvenIrq = 26, GpioOddIrq = 25; // GPIO_EVEN_IRQn / GPIO_ODD_IRQn in efr32mg21*.h
} // namespace

void Gpio::reset() {
    modes_ = {};
    dout_ = {};
    ctrl_ = {};
    extipsel_ = extipinsel_ = extirise_ = extifall_ = if_ = ien_ = 0;
    i2c_route_ = {};
    i2c_low_[0] = i2c_low_[1] = false;
    for (unsigned pad = 0; pad != 64; ++pad)
        last_[pad] = level(pad);
    update_irq();
}

PadDrive Gpio::drive(unsigned pad) const {
    bool high = dout(pad);
    // A pad routed to I2C0 is an open-drain output of the I2C master: low while it holds the line.
    for (int line = 0; line != 2; ++line)
        if (i2c_pad(line == 0) == static_cast<int>(pad))
            high = !i2c_low_[line];
    // Modes: 0 disabled, 1 input, 2/3 input with pull (DOUT picks the
    // direction), 4/5 push-pull, 6 wired-or, 7 wired-or with pull-down,
    // 8..13 wired-and (open drain; 10/11 with pull-up).
    switch (mode(pad)) {
    case 2:
    case 3: return high ? PadDrive::PullUp : PadDrive::PullDown;
    case 4:
    case 5: return high ? PadDrive::High : PadDrive::Low;
    case 6: return high ? PadDrive::High : PadDrive::HiZ;
    case 7: return high ? PadDrive::High : PadDrive::PullDown;
    case 8:
    case 9:
    case 12:
    case 13: return high ? PadDrive::HiZ : PadDrive::Low;
    case 10:
    case 11: return high ? PadDrive::PullUp : PadDrive::Low;
    default: return PadDrive::HiZ;
    }
}

bool Gpio::level(unsigned pad) const {
    if (pad >= 64)
        return false;
    if (external_valid_[pad])
        return external_[pad];
    switch (drive(pad)) {
    case PadDrive::High:
    case PadDrive::PullUp: return true;
    default: return false;
    }
}

void Gpio::set_external(unsigned pad, bool high) {
    if (pad >= 64)
        return;
    external_valid_[pad] = true;
    external_[pad] = high;
    refresh_exti(pad, last_[pad], level(pad));
    last_[pad] = level(pad);
}

void Gpio::changed(unsigned pad) {
    chip_.notify_pad(pad);
    const bool after = level(pad);
    refresh_exti(pad, last_[pad], after);
    last_[pad] = after;
}

void Gpio::refresh_exti(unsigned pad, bool before, bool after) {
    if (before == after)
        return;
    for (unsigned line = 0; line != ExtiLines; ++line) {
        const unsigned port = (extipsel_ >> (4 * line)) & 3u;
        // Inferred: pin select is a 2-bit offset inside the line's group of
        // four pins (lines 0..3 reach pins 0..3, lines 4..7 reach pins 4..7).
        const unsigned selected = ((extipinsel_ >> (4 * line)) & 3u) + 4 * (line / 4);
        if (port * 16 + selected != pad)
            continue;
        if ((after && ((extirise_ >> line) & 1u)) || (!after && ((extifall_ >> line) & 1u)))
            if_ |= 1u << line;
    }
    update_irq();
}

void Gpio::update_irq() {
    const std::uint32_t pending = if_ & ien_ & ((1u << ExtiLines) - 1);
    set_irq_level(GpioEvenIrq, (pending & 0x55u) != 0);
    set_irq_level(GpioOddIrq, (pending & 0xaau) != 0);
}

std::uint32_t Gpio::read(std::uint32_t reg) {
    if (reg < PortCount * PortStride) {
        const unsigned port = reg / PortStride;
        switch (reg % PortStride) {
        case RegCtrl: return ctrl_[port];
        case RegModel: return modes_[port][0];
        case RegModeh: return modes_[port][1];
        case RegDout: return dout_[port];
        case RegDin: {
            std::uint32_t din = 0;
            for (unsigned pin = 0; pin != 16; ++pin)
                if (mode(port * 16 + pin) != 0 && level(port * 16 + pin))
                    din |= 1u << pin;
            return din;
        }
        default: return 0;
        }
    }
    switch (reg) {
    case RegLock: return 0;
    case RegExtipsel: return extipsel_;
    case RegExtipinsel: return extipinsel_;
    case RegExtirise: return extirise_;
    case RegExtifall: return extifall_;
    case RegIf: return if_;
    case RegIen: return ien_;
    case RegI2cRouteEn: return i2c_route_[0];
    case RegI2cRouteEn + 4: return i2c_route_[1];
    case RegI2cRouteEn + 8: return i2c_route_[2];
    default: return 0;
    }
}

void Gpio::write(std::uint32_t reg, std::uint32_t value) {
    if (reg < PortCount * PortStride) {
        const unsigned port = reg / PortStride;
        switch (reg % PortStride) {
        case RegCtrl: ctrl_[port] = value; break;
        case RegModel:
        case RegModeh: {
            std::uint32_t &word = modes_[port][(reg % PortStride) == RegModel ? 0 : 1];
            const std::uint32_t before = word;
            word = value;
            for (unsigned pin = 0; pin != 8; ++pin)
                if (((before ^ value) >> (4 * pin)) & 0xfu)
                    changed(port * 16 + pin + ((reg % PortStride) == RegModel ? 0 : 8));
            break;
        }
        case RegDout: {
            const std::uint32_t before = dout_[port];
            dout_[port] = value & 0xffffu;
            for (unsigned pin = 0; pin != 16; ++pin)
                if (((before ^ dout_[port]) >> pin) & 1u)
                    changed(port * 16 + pin);
            break;
        }
        default: break;
        }
        return;
    }
    switch (reg) {
    case RegExtipsel: extipsel_ = value; break;
    case RegExtipinsel: extipinsel_ = value; break;
    case RegExtirise: extirise_ = value; break;
    case RegExtifall: extifall_ = value; break;
    case RegIf:
        if_ = value;
        update_irq();
        break;
    case RegIen:
        ien_ = value;
        update_irq();
        break;
    case RegI2cRouteEn:
    case RegI2cRouteEn + 4:
    case RegI2cRouteEn + 8: {
        const int before_scl = i2c_pad(true), before_sda = i2c_pad(false);
        i2c_route_[(reg - RegI2cRouteEn) / 4] = value;
        for (int pad : {before_scl, before_sda, i2c_pad(true), i2c_pad(false)})
            if (pad >= 0)
                changed(static_cast<unsigned>(pad));
        break;
    }
    default: break; // LOCK and the rest
    }
}

} // namespace efr32emu

// ---- I2C0 master -----------------------------------------------------------------

namespace efr32emu {

namespace {
constexpr std::uint32_t I2cEn = 0x04, I2cCtrl = 0x08, I2cCmd = 0x0C, I2cState = 0x10, I2cStatus = 0x14, I2cClkdiv = 0x18,
                        I2cRxdata = 0x24, I2cRxdatap = 0x2C, I2cTxdata = 0x34, I2cIf = 0x3C, I2cIen = 0x40;
constexpr std::uint32_t CtrlAutoAck = 1u << 2;
constexpr std::uint32_t CmdStart = 1u << 0, CmdStop = 1u << 1, CmdAck = 1u << 2, CmdNack = 1u << 3, CmdAbort = 1u << 5,
                        CmdClearTx = 1u << 6;
constexpr std::uint32_t IfStart = 1u << 0, IfRstart = 1u << 1, IfTxc = 1u << 3, IfTxbl = 1u << 4, IfRxdatav = 1u << 5,
                        IfAck = 1u << 6, IfNack = 1u << 7, IfMstop = 1u << 8;
constexpr std::uint32_t IfLevels = IfTxbl | IfRxdatav; // follow the buffers, not stored
constexpr unsigned I2c0Irq = 27;
} // namespace

std::uint64_t I2c::quarter() const {
    // SCL period = (Nlow + Nhigh) * (DIV + 1) + 8 cycles with Nlow = Nhigh = 4 (CLHR standard).
    const std::uint64_t div = regs_[I2cClkdiv >> 2] & 0x1ffu;
    return std::max<std::uint64_t>((8 * (div + 1) + 8) / 4, 1);
}

void I2c::reset() {
    RegBlock::reset();
    steps_.clear();
    at_ = 0;
    if_ = ien_ = 0;
    enabled_ = master_ = transmitter_ = nacked_ = bus_busy_ = hold_ = false;
    start_pending_ = stop_pending_ = addr_phase_ = wait_ack_cmd_ = false;
    scl_low_ = sda_low_ = false;
    tx_full_ = rx_full_ = false;
    chip().gpio().set_i2c_low(true, false);
    chip().gpio().set_i2c_low(false, false);
    update_irq();
}

void I2c::apply(bool sda_low, bool scl_low) {
    if (sda_low == sda_low_ && scl_low == scl_low_)
        return;
    // SDA never moves while SCL is high: lower SCL before changing SDA, settle SDA before raising SCL.
    const bool scl_falls = scl_low && !scl_low_;
    scl_low_ = scl_low;
    sda_low_ = sda_low;
    if (scl_falls) {
        chip().gpio().set_i2c_low(true, scl_low);
        chip().gpio().set_i2c_low(false, sda_low);
    } else {
        chip().gpio().set_i2c_low(false, sda_low);
        chip().gpio().set_i2c_low(true, scl_low);
    }
}

void I2c::queue_start() {
    const std::uint64_t q = quarter();
    std::uint64_t at = std::max(at_, now());
    steps_.push_back({at, Kind::Drive, false, scl_low_});
    steps_.push_back({at + q, Kind::Drive, false, false});
    steps_.push_back({at + 2 * q, Kind::Drive, true, false}); // SDA falls while SCL is high
    steps_.push_back({at + 2 * q, bus_busy_ ? Kind::RepeatedStart : Kind::Started, true, false});
    steps_.push_back({at + 3 * q, Kind::Drive, true, true});
    at_ = at + 4 * q;
}

void I2c::queue_write_byte(std::uint8_t value) {
    const std::uint64_t q = quarter();
    std::uint64_t at = std::max(at_, now());
    for (int bit = 7; bit >= 0; --bit) {
        const bool low = ((value >> bit) & 1u) == 0;
        steps_.push_back({at, Kind::Drive, low, true});
        steps_.push_back({at + q, Kind::Drive, low, false});
        steps_.push_back({at + 3 * q, Kind::Drive, low, true});
        at += 4 * q;
    }
    steps_.push_back({at, Kind::Drive, false, true}); // release SDA for the slave's ACK
    steps_.push_back({at + q, Kind::Drive, false, false});
    steps_.push_back({at + 2 * q, Kind::SampleAck, false, false});
    steps_.push_back({at + 3 * q, Kind::Drive, false, true});
    at += 4 * q;
    steps_.push_back({at, Kind::WriteDone, false, true});
    at_ = at;
}

void I2c::queue_read_byte() {
    const std::uint64_t q = quarter();
    std::uint64_t at = std::max(at_, now());
    shift_ = 0;
    for (int bit = 7; bit >= 0; --bit) {
        steps_.push_back({at, Kind::Drive, false, true});
        steps_.push_back({at + q, Kind::Drive, false, false});
        steps_.push_back({at + 2 * q, Kind::SampleBit, false, false});
        steps_.push_back({at + 3 * q, Kind::Drive, false, true});
        at += 4 * q;
    }
    steps_.push_back({at, Kind::ReadDone, false, true});
    at_ = at;
}

void I2c::queue_ack_bit(bool nack) {
    const std::uint64_t q = quarter();
    std::uint64_t at = std::max(at_, now());
    steps_.push_back({at, Kind::Drive, !nack, true});
    steps_.push_back({at + q, Kind::Drive, !nack, false});
    steps_.push_back({at + 3 * q, Kind::Drive, !nack, true});
    steps_.push_back({at + 4 * q, Kind::Drive, false, true});
    at_ = at + 4 * q;
}

void I2c::queue_stop() {
    const std::uint64_t q = quarter();
    std::uint64_t at = std::max(at_, now());
    steps_.push_back({at, Kind::Drive, true, true});
    steps_.push_back({at + q, Kind::Drive, true, false});
    steps_.push_back({at + 2 * q, Kind::Drive, false, false}); // SDA rises while SCL is high
    steps_.push_back({at + 2 * q, Kind::Stopped, false, false});
    at_ = at + 3 * q;
}

void I2c::progress() {
    if (!enabled_ || !steps_.empty())
        return;
    if (start_pending_) {
        start_pending_ = false;
        addr_phase_ = true;
        queue_start();
    }
    if (tx_full_ && (addr_phase_ || (master_ && transmitter_ && !wait_ack_cmd_)) && (bus_busy_ || !steps_.empty())) {
        sent_ = txdata_;
        tx_full_ = false;
        queue_write_byte(sent_);
    } else if (stop_pending_ && steps_.empty()) {
        stop_pending_ = false;
        queue_stop();
    }
    if (!steps_.empty())
        chip().request_service(steps_.front().cycle);
}

std::uint64_t I2c::service(std::uint64_t now_cycle) {
    bool ack = false;
    while (!steps_.empty() && steps_.front().cycle <= now_cycle) {
        const Step step = steps_.front();
        steps_.pop_front();
        const int sda = chip().gpio().i2c_pad(false);
        const bool sda_high = sda < 0 || chip().gpio().level(static_cast<unsigned>(sda));
        switch (step.kind) {
        case Kind::Drive: apply(step.sda_low, step.scl_low); break;
        case Kind::SampleBit: shift_ = static_cast<std::uint8_t>((shift_ << 1) | (sda_high ? 1u : 0u)); break;
        case Kind::SampleAck:
            ack = !sda_high && sda >= 0;
            if_ |= ack ? IfAck : IfNack;
            nacked_ = !ack;
            last_ack_ = ack;
            break;
        case Kind::Started:
            bus_busy_ = master_ = true;
            if_ |= IfStart;
            break;
        case Kind::RepeatedStart: if_ |= IfRstart; break;
        case Kind::WriteDone:
            if (addr_phase_) {
                addr_phase_ = false;
                transmitter_ = (sent_ & 1u) == 0;
                if (last_ack_ && !transmitter_)
                    queue_read_byte(); // the slave's first byte follows its address ACK
            }
            hold_ = true;
            if_ |= IfTxc;
            break;
        case Kind::ReadDone:
            rxdata_ = shift_;
            rx_full_ = true;
            wait_ack_cmd_ = true;
            hold_ = true;
            break;
        case Kind::Stopped:
            if_ |= IfMstop;
            bus_busy_ = master_ = hold_ = false;
            break;
        }
    }
    if (steps_.empty())
        progress();
    update_irq();
    return steps_.empty() ? ~std::uint64_t{0} : steps_.front().cycle;
}

void I2c::update_irq() {
    std::uint32_t flags = if_ | (!tx_full_ && enabled_ ? IfTxbl : 0u) | (rx_full_ ? IfRxdatav : 0u);
    set_irq_level(I2c0Irq, (flags & ien_) != 0);
}

std::uint32_t I2c::peek(std::uint32_t reg) {
    switch (reg) {
    case I2cEn: return enabled_ ? 1u : 0u;
    case I2cState:
        return (bus_busy_ ? 1u : 0u) | (master_ ? 2u : 0u) | (transmitter_ ? 4u : 0u) | (nacked_ ? 8u : 0u) | (hold_ ? 0x10u : 0u);
    case I2cStatus:
        return (!tx_full_ && enabled_ ? 0x80u : 0u) | (!tx_full_ && steps_.empty() && master_ && hold_ ? 0x40u : 0u) |
               (rx_full_ ? 0x100u : 0u);
    case I2cRxdata:
    case I2cRxdatap: return rxdata_;
    case I2cIf: return if_ | (!tx_full_ && enabled_ ? IfTxbl : 0u) | (rx_full_ ? IfRxdatav : 0u);
    case I2cIen: return ien_;
    default: return RegBlock::read(reg);
    }
}

std::uint32_t I2c::read(std::uint32_t reg) {
    const std::uint32_t value = peek(reg);
    if (reg == I2cRxdata && rx_full_) {
        rx_full_ = false;
        // AUTOACK: the master acknowledges the byte as soon as it is read and starts the next one.
        if (wait_ack_cmd_ && (regs_[I2cCtrl >> 2] & CtrlAutoAck)) {
            wait_ack_cmd_ = false;
            queue_ack_bit(false);
            queue_read_byte();
            chip().request_service(steps_.front().cycle);
        }
        update_irq();
    }
    return value;
}

void I2c::write(std::uint32_t reg, std::uint32_t value) {
    switch (reg) {
    case I2cEn:
        enabled_ = (value & 1u) != 0;
        if (!enabled_) {
            steps_.clear();
            chip().gpio().set_i2c_low(true, false);
            chip().gpio().set_i2c_low(false, false);
            scl_low_ = sda_low_ = master_ = bus_busy_ = hold_ = false;
            tx_full_ = rx_full_ = start_pending_ = stop_pending_ = addr_phase_ = wait_ack_cmd_ = false;
        }
        break;
    case I2cCmd:
        if (!enabled_)
            break;
        if (value & CmdAbort) {
            steps_.clear();
            at_ = 0;
            start_pending_ = stop_pending_ = addr_phase_ = wait_ack_cmd_ = hold_ = false;
            apply(false, false);
        }
        if (value & CmdClearTx)
            tx_full_ = false;
        if ((value & CmdAck) && wait_ack_cmd_) {
            wait_ack_cmd_ = false;
            queue_ack_bit(false);
            queue_read_byte();
        }
        if ((value & CmdNack) && wait_ack_cmd_) {
            wait_ack_cmd_ = false;
            queue_ack_bit(true);
        }
        if (value & CmdStart)
            start_pending_ = true;
        if (value & CmdStop)
            stop_pending_ = true;
        progress();
        if (!steps_.empty())
            chip().request_service(steps_.front().cycle);
        break;
    case I2cTxdata:
        if (enabled_) {
            txdata_ = static_cast<std::uint8_t>(value);
            tx_full_ = true;
            progress();
        }
        break;
    case I2cIf: if_ = value & ~IfLevels; break;
    case I2cIen: ien_ = value; break;
    default: RegBlock::write(reg, value); break;
    }
    update_irq();
}

// ---- GPIO I2C routing --------------------------------------------------------------

int Gpio::i2c_pad(bool scl) const {
    const std::uint32_t enable = i2c_route_[0], route = i2c_route_[scl ? 1 : 2];
    if (!(enable & (scl ? 1u : 2u)))
        return -1;
    return static_cast<int>((route & 3u) * 16u + ((route >> 16) & 0xfu));
}

void Gpio::set_i2c_low(bool scl, bool low) {
    bool &state = i2c_low_[scl ? 0 : 1];
    if (state == low)
        return;
    state = low;
    const int pad = i2c_pad(scl);
    if (pad >= 0)
        changed(static_cast<unsigned>(pad));
}

} // namespace efr32emu
