// The EFR32MG21 radio blocks (FRC, BUFC, SYNTH, RAC, MODEM, AGC) in IEEE
// 802.15.4 mode, bridged to mcuemu::VirtualAir.
//
// IMPORTANT: this is NOT a model of the silicon. The real radio is a RAC
// sequencer running Silicon Labs' RAIL configuration and cannot be driven by a
// handful of register writes. This block implements the register interface of
// the zigbee-rs `efr32s2` MAC driver (zigbee-mac/src/efr32s2/driver.rs), whose
// own header calls it a scaffold with unverified register values. It exists so
// that driver's firmware can run in a network; firmware built on RAIL or the
// GSDK radio stack will not work. Current zigbee-rs images use the efr32s2-hal
// driver instead, modelled by RsmRadio (rsm.cpp).
//
// Interface (offsets from 0x40090000):
//   FRC   0x0000: IF at +0x30 (bit 4: CRC ok), IFC at +0x38
//   BUFC  0x1000: CMD +0x04 (bit 0 clears the TX FIFO, bit 1 the RX FIFO),
//                 TX FIFO data +0x40 (PHR first, then the MAC frame),
//                 RX FIFO data +0x80, RX frame length +0x84
//   SYNTH 0x2000: channel number +0x0C (0 = channel 11)
//   RAC   0x3000: CMD +0x04 (bit 0 TXEN, 1 RXEN, 2 TXDIS, 3 RXDIS),
//                 STATUS +0x08 (1 idle, 2 RX, 3 TX), IF +0x10, IFC +0x18,
//                 IEN +0x1C (bit 0 TX done, bit 1 RX done, bit 2 RX overflow)
//   AGC   0x5000: RSSI +0x20 (signed dBm in the low byte)
// The interrupt is FRC_PRI (IRQ 36), level-sensitive on RAC IF & IEN.
//
// Inferred timing: a 40 us ramp before TX and RX; the medium carries MAC
// frames without the FCS, so a frame is 64 us shorter on air than on a real
// radio; the receiver polls the medium every 16 us and, like the nRF model,
// can still lock on a frame whose start-of-frame delimiter has not passed yet
// (128 us). The radio stays in RX after a frame.

#include "efr32/blocks.hpp"

#include <algorithm>
#include <cstring>

namespace efr32emu {

namespace {
constexpr unsigned FrcPriIrq = 36;
constexpr std::uint64_t RampUs = 40, PollUs = 16, SfdNs = 128000;
constexpr std::uint32_t FrcIf = 0x0030, FrcIfc = 0x0038;
constexpr std::uint32_t BufcCmd = 0x1004, BufcTxData = 0x1040, BufcTxCount = 0x1044, BufcRxData = 0x1080,
                        BufcRxCount = 0x1084;
constexpr std::uint32_t SynthChno = 0x200c;
constexpr std::uint32_t RacCmd = 0x3004, RacStatus = 0x3008, RacIf = 0x3010, RacIfc = 0x3018, RacIen = 0x301c;
constexpr std::uint32_t AgcRssi = 0x5020;
constexpr std::uint32_t IfTxDone = 1u << 0, IfRxDone = 1u << 1;
constexpr std::uint32_t FrcCrcOk = 1u << 4;
} // namespace

std::uint64_t Radio::to_ns(std::uint64_t cycles) const { return chip_.core().cycles_to_ns(cycles); }

void Radio::connect(mcuemu::VirtualAir &air, const std::string &node) {
    air_ = &air;
    node_ = node;
}

void Radio::reset() {
    regs_.fill(0);
    state_ = State::Idle;
    rac_if_ = rac_ien_ = frc_if_ = chno_ = 0;
    tx_fifo_.clear();
    rx_fifo_.clear();
    rx_read_ = 0;
    rx_pending_ = tx_active_ = false;
    rx_rssi_ = -127;
    rsm_poll_ns_ = 0;
    update_irq();
}

void Radio::update_irq() { set_irq_level(FrcPriIrq, (rac_if_ & rac_ien_) != 0); }

std::int8_t Radio::energy_dbm(std::uint64_t at_ns) const { return energy_dbm(at_ns, channel()); }

std::int8_t Radio::energy_dbm(std::uint64_t at_ns, std::uint8_t channel) const {
    if (!air_)
        return -110;
    double best = -110;
    const bool ideal = air_->propagation().model == mcuemu::Propagation::Model::Ideal;
    const auto &frames = air_->frames();
    for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
        if (it->end_ns + 20000000ull < at_ns)
            break;
        if (it->protocol != mcuemu::RadioProtocol::Zigbee802154 || it->channel != channel ||
            it->source == node_ || it->start_ns > at_ns || it->end_ns + 128000 < at_ns)
            continue;
        best = std::max(best, ideal ? -50.0 : it->tx_power_dbm - air_->path_loss_db(it->source, node_));
    }
    return static_cast<std::int8_t>(std::clamp(best, -127.0, 0.0));
}

std::uint32_t Radio::peek(std::uint32_t reg) {
    switch (reg) {
    case FrcIf: return frc_if_;
    case BufcTxCount: return static_cast<std::uint32_t>(tx_fifo_.size());
    case BufcRxData: return rx_read_ < rx_fifo_.size() ? rx_fifo_[rx_read_] : 0;
    case BufcRxCount: return static_cast<std::uint32_t>(rx_fifo_.size());
    case SynthChno: return chno_;
    case RacStatus: return static_cast<std::uint32_t>(state_);
    case RacIf: return rac_if_;
    case RacIen: return rac_ien_;
    case AgcRssi:
        return static_cast<std::uint8_t>(rx_pending_ ? rx_rssi_ : energy_dbm(to_ns(now())));
    default: return reg / 4 < regs_.size() ? regs_[reg / 4] : 0;
    }
}

std::uint32_t Radio::read(std::uint32_t reg) {
    const std::uint32_t value = peek(reg);
    if (reg == BufcRxData && rx_read_ < rx_fifo_.size()) {
        if (++rx_read_ >= rx_fifo_.size())
            rx_pending_ = false; // the frame is drained: the next one may arrive
    }
    return value;
}

void Radio::write(std::uint32_t reg, std::uint32_t value) {
    switch (reg) {
    case FrcIfc: frc_if_ &= ~value; break;
    case BufcCmd:
        if (value & 1u)
            tx_fifo_.clear();
        if (value & 2u) {
            rx_fifo_.clear();
            rx_read_ = 0;
            rx_pending_ = false;
        }
        break;
    case BufcTxData: tx_fifo_.push_back(static_cast<std::uint8_t>(value)); break;
    case SynthChno: chno_ = value; break;
    case RacCmd: command(value); break;
    case RacIfc: rac_if_ &= ~value; break;
    case RacIen: rac_ien_ = value; break;
    default:
        if (reg / 4 < regs_.size())
            regs_[reg / 4] = value;
        break;
    }
    update_irq();
}

void Radio::command(std::uint32_t value) {
    const std::uint64_t t = now();
    if ((value & 4u) && state_ == State::Tx && !tx_active_)
        state_ = State::Idle;
    if ((value & 8u) && state_ == State::Rx)
        state_ = State::Idle;
    if (value & 1u) {
        begin_transmit();
    } else if ((value & 2u) && state_ != State::Tx) {
        state_ = State::Rx;
        rx_start_ns_ = last_poll_ns_ = to_ns(t) + RampUs * 1000;
        chip_.request_service(t + PollUs * (clock_hz() / 1000000u));
    }
}

std::uint64_t Radio::transmit_fifo(const std::vector<std::uint8_t> &fifo, std::uint8_t channel) {
    if (fifo.empty())
        return 0;
    const std::size_t phr = fifo[0];
    const std::size_t room = fifo.size() - 1;
    // PHR counts the two FCS bytes the radio appends itself, as in begin_transmit.
    const std::size_t mac = phr >= 2 ? std::min(phr - 2, room) : 0;
    const std::uint64_t airtime = mcuemu::VirtualAir::airtime_ns(mcuemu::RadioProtocol::Zigbee802154, mac + 2);
    std::vector<std::uint8_t> payload(fifo.begin() + 1, fifo.begin() + 1 + mac);
    if (air_ && !payload.empty() && payload.size() <= 125)
        air_->transmit(node_, {}, mcuemu::RadioProtocol::Zigbee802154, channel, to_ns(now()), std::move(payload));
    return airtime;
}

std::vector<std::vector<std::uint8_t>> Radio::receive_fifo(std::uint8_t channel, std::uint64_t now_ns) {
    std::vector<std::vector<std::uint8_t>> frames;
    if (!air_)
        return frames;
    const std::uint64_t after = rail_poll_ns_;
    rail_poll_ns_ = now_ns;
    for (const mcuemu::RadioFrame &frame :
         air_->receive(node_, mcuemu::RadioProtocol::Zigbee802154, channel, now_ns, after)) {
        if (frame.payload.size() + 2 > 127)
            continue;
        std::vector<std::uint8_t> fifo{static_cast<std::uint8_t>(frame.payload.size() + 2)};
        fifo.insert(fifo.end(), frame.payload.begin(), frame.payload.end());
        frames.push_back(std::move(fifo));
    }
    return frames;
}

std::vector<mcuemu::RadioFrame> Radio::receive_since(std::uint8_t channel, std::uint64_t now_ns,
                                                     std::uint64_t listening_since_ns) {
    std::vector<mcuemu::RadioFrame> heard;
    if (!air_)
        return heard;
    const std::uint64_t after = rsm_poll_ns_;
    rsm_poll_ns_ = now_ns;
    for (mcuemu::RadioFrame &frame :
         air_->receive(node_, mcuemu::RadioProtocol::Zigbee802154, channel, now_ns, after)) {
        if (frame.start_ns + SfdNs < listening_since_ns || frame.payload.size() + 2 > 127)
            continue; // the receiver was not searching when the frame began, or no valid PHR
        heard.push_back(std::move(frame));
    }
    return heard;
}

void Radio::begin_transmit() {
    if (tx_fifo_.empty())
        return;
    const std::size_t phr = tx_fifo_[0];
    const std::size_t room = tx_fifo_.size() - 1;
    // PHR counts the two FCS bytes the radio appends itself.
    const std::size_t mac = phr >= 2 ? std::min(phr - 2, room) : 0;
    std::vector<std::uint8_t> payload(tx_fifo_.begin() + 1, tx_fifo_.begin() + 1 + mac);
    tx_fifo_.clear();
    const std::uint64_t start = to_ns(now()) + RampUs * 1000;
    std::uint64_t end = start + mcuemu::VirtualAir::airtime_ns(mcuemu::RadioProtocol::Zigbee802154, mac + 2);
    if (air_ && !payload.empty() && payload.size() <= 125) {
        // The medium may add an ACK frame of its own, so look the frame up by id.
        const std::uint64_t id = air_->transmit(node_, {}, mcuemu::RadioProtocol::Zigbee802154, channel(),
                                                start, std::move(payload));
        end = std::max(end, air_->frames()[id - 1].end_ns);
    }
    state_ = State::Tx;
    tx_active_ = true;
    tx_end_ns_ = end;
    chip_.request_service(chip_.core().ns_to_cycles(end));
}

void Radio::poll_air(std::uint64_t ns) {
    if (!air_)
        return;
    const std::uint64_t after = last_poll_ns_;
    last_poll_ns_ = ns;
    for (const mcuemu::RadioFrame &frame :
         air_->receive(node_, mcuemu::RadioProtocol::Zigbee802154, channel(), ns, after)) {
        if (frame.start_ns + SfdNs < rx_start_ns_)
            continue; // the receiver was not listening yet
        if (rx_pending_)
            continue; // the previous frame has not been read; this one is lost
        rx_fifo_ = frame.payload;
        rx_read_ = 0;
        rx_pending_ = true;
        rx_rssi_ = static_cast<std::int8_t>(std::clamp(frame.rssi_dbm, -127, 0));
        frc_if_ |= FrcCrcOk;
        rac_if_ |= IfRxDone;
        update_irq();
    }
}

std::uint64_t Radio::reschedule() const {
    std::uint64_t next = Never;
    if (tx_active_)
        next = chip_.core().ns_to_cycles(tx_end_ns_);
    if (state_ == State::Rx && air_)
        next = std::min(next, now() + PollUs * (clock_hz() / 1000000u));
    return next;
}

std::uint64_t Radio::service(std::uint64_t t) {
    const std::uint64_t ns = to_ns(t);
    if (tx_active_ && ns >= tx_end_ns_) {
        tx_active_ = false;
        state_ = State::Idle;
        rac_if_ |= IfTxDone;
        update_irq();
    }
    if (state_ == State::Rx)
        poll_air(ns);
    return reschedule();
}

} // namespace efr32emu
