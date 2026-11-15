#include "stm32wb55emu/st_mac.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <limits>

namespace stm32wb55emu {
namespace {

constexpr std::uint64_t Never = std::numeric_limits<std::uint64_t>::max();
// IEEE 802.15.4-2006 2.4 GHz O-QPSK: 62.5 ksymbol/s.
constexpr std::uint64_t SymbolNs = 16000;
constexpr std::uint64_t UnitBackoffNs = 20 * SymbolNs;       // aUnitBackoffPeriod
constexpr std::uint64_t CcaNs = 8 * SymbolNs;                // phyCCADuration
constexpr std::uint64_t TurnaroundNs = 12 * SymbolNs;        // aTurnaroundTime
constexpr std::uint64_t AckWaitNs = 54 * SymbolNs;           // macAckWaitDuration
constexpr std::uint64_t BaseSuperframeNs = 960 * SymbolNs;   // aBaseSuperframeDuration
// CPU2 time from a request (or a received frame) to its confirm (or
// indication). Inferred: the coprocessor answers in tens of microseconds.
constexpr std::uint64_t ProcessingNs = 40000;
constexpr std::size_t MaxPhyPacket = 127;
constexpr std::uint32_t ChannelMask2450 = 0x07fff800u;

enum : std::uint8_t {
    FrameBeacon = 0,
    FrameData = 1,
    FrameAck = 2,
    FrameCommand = 3,
};

enum : std::uint8_t {
    CmdAssociationRequest = 0x01,
    CmdAssociationResponse = 0x02,
    CmdDisassociation = 0x03,
    CmdDataRequest = 0x04,
    CmdOrphanNotification = 0x06,
    CmdBeaconRequest = 0x07,
    CmdCoordinatorRealignment = 0x08,
};

// PIB identifiers (IEEE 802.15.4-2006 tables 23 and 86, as used by the ST
// SAP; 0x6f is ST's macExtendedAddress).
enum : std::uint8_t {
    PibCurrentChannel = 0x00,
    PibChannelsSupported = 0x01,
    PibTransmitPower = 0x02,
    PibCcaMode = 0x03,
    PibCurrentPage = 0x04,
    PibAckWaitDuration = 0x40,
    PibAssociationPermit = 0x41,
    PibAutoRequest = 0x42,
    PibBattLifeExt = 0x43,
    PibBeaconPayload = 0x45,
    PibBeaconPayloadLength = 0x46,
    PibBeaconOrder = 0x47,
    PibBsn = 0x49,
    PibCoordExtendedAddress = 0x4a,
    PibCoordShortAddress = 0x4b,
    PibDsn = 0x4c,
    PibGtsPermit = 0x4d,
    PibMaxCsmaBackoffs = 0x4e,
    PibMinBe = 0x4f,
    PibPanId = 0x50,
    PibPromiscuousMode = 0x51,
    PibRxOnWhenIdle = 0x52,
    PibShortAddress = 0x53,
    PibSuperframeOrder = 0x54,
    PibTransactionPersistenceTime = 0x55,
    PibAssociatedPanCoord = 0x56,
    PibMaxBe = 0x57,
    PibMaxFrameTotalWaitTime = 0x58,
    PibMaxFrameRetries = 0x59,
    PibResponseWaitTime = 0x5a,
    PibExtendedAddress = 0x6f,
};

std::uint16_t get16(const std::uint8_t *p) {
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

std::uint32_t get32(const std::uint8_t *p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

void put16(std::uint8_t *p, std::uint16_t value) {
    p[0] = static_cast<std::uint8_t>(value);
    p[1] = static_cast<std::uint8_t>(value >> 8);
}

void put32(std::uint8_t *p, std::uint32_t value) {
    for (unsigned i = 0; i != 4; ++i)
        p[i] = static_cast<std::uint8_t>(value >> (8 * i));
}

// MAC_addr_t: a union of a 16-bit and a 64-bit address, little endian.
void put_address(std::uint8_t *p, const StMac::Address &address) {
    std::memset(p, 0, 8);
    if (address.mode == 2)
        put16(p, address.short_address);
    else if (address.mode == 3)
        std::memcpy(p, address.extended.data(), 8);
}

StMac::Address sap_address(std::uint8_t mode, const std::uint8_t *pan, const std::uint8_t *p) {
    StMac::Address address;
    address.mode = mode;
    address.pan = get16(pan);
    if (mode == 2)
        address.short_address = get16(p);
    else if (mode == 3)
        std::memcpy(address.extended.data(), p, 8);
    return address;
}

bool same_address(const StMac::Address &a, const StMac::Address &b) {
    if (a.mode != b.mode)
        return false;
    if (a.mode == 2)
        return a.short_address == b.short_address;
    if (a.mode == 3)
        return a.extended == b.extended;
    return false;
}

std::string hex_address(const StMac::Address &address) {
    char text[32];
    if (address.mode == 2) {
        std::snprintf(text, sizeof text, "0x%04x", address.short_address);
    } else if (address.mode == 3) {
        std::snprintf(text, sizeof text, "%02x%02x%02x%02x%02x%02x%02x%02x",
                      address.extended[7], address.extended[6], address.extended[5],
                      address.extended[4], address.extended[3], address.extended[2],
                      address.extended[1], address.extended[0]);
    } else {
        std::snprintf(text, sizeof text, "none");
    }
    return text;
}

std::string hex8(unsigned value) {
    char text[8];
    std::snprintf(text, sizeof text, "0x%02x", value & 0xffu);
    return text;
}

std::string hex16(unsigned value) {
    char text[8];
    std::snprintf(text, sizeof text, "0x%04x", value & 0xffffu);
    return text;
}

} // namespace

StMac::StMac(StMacHost &host, std::uint64_t seed)
    : host_(host), rng_(seed ? seed : 0x9e3779b97f4a7c15ull) {
    reset_pib();
}

void StMac::trace(const std::string &line) {
    if (trace_)
        trace_lines_.push_back(line);
}

std::vector<std::string> StMac::take_trace() {
    std::vector<std::string> lines;
    lines.swap(trace_lines_);
    return lines;
}

std::uint32_t StMac::random() {
    // xorshift64*: CSMA-CA backoffs and the initial sequence numbers.
    rng_ ^= rng_ >> 12;
    rng_ ^= rng_ << 25;
    rng_ ^= rng_ >> 27;
    return static_cast<std::uint32_t>((rng_ * 0x2545f4914f6cdd1dull) >> 32);
}

std::uint8_t StMac::lqi(int rssi_dbm) const {
    // Linear map of -100..-40 dBm onto 0..255 (the ST MAC reports an LQI
    // that saturates for strong links; the exact curve is not documented).
    return static_cast<std::uint8_t>(std::clamp((rssi_dbm + 100) * 255 / 60, 0, 255));
}

void StMac::reset_pib() {
    channel_ = 11;
    page_ = 0;
    tx_power_ = 0;
    cca_mode_ = 1;
    association_permit_ = false;
    auto_request_ = true;
    battery_life_ext_ = false;
    beacon_payload_.fill(0);
    beacon_payload_length_ = 0;
    beacon_order_ = 15;
    superframe_order_ = 15;
    bsn_ = static_cast<std::uint8_t>(random());
    dsn_ = static_cast<std::uint8_t>(random());
    coord_extended_.fill(0);
    coord_short_ = 0xffff;
    gts_permit_ = true;
    max_csma_backoffs_ = 4;
    min_be_ = 3;
    max_be_ = 5;
    pan_id_ = 0xffff;
    promiscuous_ = false;
    rx_on_when_idle_ = false;
    short_address_ = 0xffff;
    transaction_persistence_time_ = 0x01f4;
    associated_pan_coord_ = false;
    max_frame_total_wait_time_ = 1986;
    max_frame_retries_ = 3;
    response_wait_time_ = 32;
}

bool StMac::rx_enabled() const noexcept {
    if (started_ || rx_on_when_idle_ || promiscuous_ || op_ != Op::None)
        return true;
    return !tx_queue_.empty() && tx_queue_.front().waiting_ack;
}

std::optional<StMacNotification> StMac::pop_notification() {
    if (notifications_.empty())
        return std::nullopt;
    StMacNotification notification = std::move(notifications_.front());
    notifications_.pop_front();
    return notification;
}

void StMac::notify(std::uint16_t id, std::vector<std::uint8_t> body,
                   std::vector<std::uint8_t> data, std::uint64_t due_ns) {
    StMacNotification notification;
    notification.payload.reserve(2 + body.size());
    notification.payload.push_back(static_cast<std::uint8_t>(id));
    notification.payload.push_back(static_cast<std::uint8_t>(id >> 8));
    notification.payload.insert(notification.payload.end(), body.begin(), body.end());
    notification.data = std::move(data);
    pending_.push_back({due_ns, std::move(notification)});
}

void StMac::confirm_simple(std::uint16_t id, std::uint8_t status, std::uint64_t due_ns,
                           std::uint8_t second) {
    // MAC_resetCnf_t, MAC_startCnf_t, MAC_pollCnf_t, MAC_setCnf_t (status,
    // PIB attribute) and MAC_purgeCnf_t (handle, status) are 4 octets.
    std::vector<std::uint8_t> body(4, 0);
    if (id == PurgeCnf) {
        body[0] = second;
        body[1] = status;
    } else {
        body[0] = status;
        body[1] = second;
    }
    notify(id, std::move(body), {}, due_ns);
}

std::uint8_t StMac::request(std::uint8_t id, const std::uint8_t *payload, std::size_t size,
                            std::uint64_t now_ns) {
    now_ns_ = std::max(now_ns_, now_ns);
    ++stats_.requests;
    auto need = [&](std::size_t wanted) { return size >= wanted; };
    switch (id) {
    case SetReq: return need(SetReqSize) ? handle_set(payload, now_ns) : InvalidParameter;
    case GetReq: return need(GetReqSize) ? handle_get(payload, now_ns) : InvalidParameter;
    case ResetReq: return need(ResetReqSize) ? handle_reset(payload, now_ns) : InvalidParameter;
    case ScanReq: return need(ScanReqSize) ? handle_scan(payload, now_ns) : InvalidParameter;
    case StartReq: return need(StartReqSize) ? handle_start(payload, now_ns) : InvalidParameter;
    case AssociateReq:
        return need(AssociateReqSize) ? handle_associate(payload, now_ns) : InvalidParameter;
    case AssociateRes:
        return need(AssociateResSize) ? handle_associate_response(payload, now_ns)
                                      : InvalidParameter;
    case DisassociateReq:
        return need(DisassociateReqSize) ? handle_disassociate(payload, now_ns)
                                         : InvalidParameter;
    case OrphanRes:
        return need(OrphanResSize) ? handle_orphan_response(payload, now_ns) : InvalidParameter;
    case PollReq: return need(PollReqSize) ? handle_poll(payload, now_ns) : InvalidParameter;
    case DataReq: return need(DataReqSize) ? handle_data(payload, now_ns) : InvalidParameter;
    case PurgeReq: return need(PurgeReqSize) ? handle_purge(payload, now_ns) : InvalidParameter;
    default:
        // GTS, RX-ENABLE, SYNC, DPS, sounding and calibration are beacon-
        // enabled or UWB features a Zigbee non-beacon PAN never uses.
        trace("request " + hex8(id) + " not modeled");
        return NotImplemented;
    }
}

std::uint8_t StMac::handle_set(const std::uint8_t *p, std::uint64_t now_ns) {
    const std::uint32_t pointer = get32(p);
    const std::uint8_t attribute = p[4];
    std::uint8_t value[MaxBeaconPayload] = {};
    auto load = [&](std::size_t size) { return host_.read(pointer, value, size); };
    std::uint8_t status = Success;
    auto flag = [&](bool &field) {
        if (!load(1))
            status = InvalidParameter;
        else if (value[0] > 1)
            status = InvalidParameter;
        else
            field = value[0] != 0;
    };
    auto byte_in = [&](std::uint8_t &field, unsigned low, unsigned high) {
        if (!load(1) || value[0] < low || value[0] > high)
            status = InvalidParameter;
        else
            field = value[0];
    };
    auto word = [&](std::uint16_t &field) {
        if (!load(2))
            status = InvalidParameter;
        else
            field = get16(value);
    };
    switch (attribute) {
    case PibCurrentChannel: byte_in(channel_, 11, 26); break;
    case PibTransmitPower:
        if (load(1))
            tx_power_ = static_cast<std::int8_t>(value[0]);
        else
            status = InvalidParameter;
        break;
    case PibCcaMode: byte_in(cca_mode_, 1, 3); break;
    case PibCurrentPage: byte_in(page_, 0, 0); break;
    case PibChannelsSupported:
    case PibAckWaitDuration: status = ReadOnly; break;
    case PibAssociationPermit: flag(association_permit_); break;
    case PibAutoRequest: flag(auto_request_); break;
    case PibBattLifeExt: flag(battery_life_ext_); break;
    case PibBeaconPayload: {
        // The SAP passes no length: CPU2 copies aMaxBeaconPayloadLength
        // octets; macBeaconPayloadLength (set next) selects how many count.
        std::size_t readable = MaxBeaconPayload;
        while (readable != 0 && !load(readable))
            --readable;
        if (readable == 0)
            status = InvalidParameter;
        else
            std::copy(value, value + MaxBeaconPayload, beacon_payload_.begin());
        break;
    }
    case PibBeaconPayloadLength:
        byte_in(beacon_payload_length_, 0, static_cast<unsigned>(MaxBeaconPayload));
        break;
    case PibBeaconOrder: byte_in(beacon_order_, 15, 15); break;
    case PibSuperframeOrder: byte_in(superframe_order_, 15, 15); break;
    case PibBsn: byte_in(bsn_, 0, 255); break;
    case PibDsn: byte_in(dsn_, 0, 255); break;
    case PibCoordExtendedAddress:
        if (load(8))
            std::copy(value, value + 8, coord_extended_.begin());
        else
            status = InvalidParameter;
        break;
    case PibCoordShortAddress: word(coord_short_); break;
    case PibGtsPermit: flag(gts_permit_); break;
    case PibMaxCsmaBackoffs: byte_in(max_csma_backoffs_, 0, 5); break;
    case PibMinBe: byte_in(min_be_, 0, max_be_); break;
    case PibPanId: word(pan_id_); break;
    case PibPromiscuousMode: flag(promiscuous_); break;
    case PibRxOnWhenIdle: flag(rx_on_when_idle_); break;
    case PibShortAddress: word(short_address_); break;
    case PibTransactionPersistenceTime: word(transaction_persistence_time_); break;
    case PibAssociatedPanCoord: flag(associated_pan_coord_); break;
    case PibMaxBe: byte_in(max_be_, 3, 8); break;
    case PibMaxFrameTotalWaitTime: word(max_frame_total_wait_time_); break;
    case PibMaxFrameRetries: byte_in(max_frame_retries_, 0, 7); break;
    case PibResponseWaitTime: byte_in(response_wait_time_, 2, 64); break;
    case PibExtendedAddress:
        if (load(8))
            std::copy(value, value + 8, extended_.begin());
        else
            status = InvalidParameter;
        break;
    default: status = UnsupportedAttribute; break;
    }
    trace("MLME-SET " + hex8(attribute) + " -> " + hex8(status));
    confirm_simple(SetCnf, status, now_ns + ProcessingNs, attribute);
    return Success;
}

std::uint8_t StMac::handle_get(const std::uint8_t *p, std::uint64_t now_ns) {
    const std::uint8_t attribute = p[0];
    std::vector<std::uint8_t> value;
    auto u16 = [&](std::uint16_t v) {
        value = {static_cast<std::uint8_t>(v), static_cast<std::uint8_t>(v >> 8)};
    };
    std::uint8_t status = Success;
    switch (attribute) {
    case PibCurrentChannel: value = {channel_}; break;
    case PibChannelsSupported: value = {0x00, 0xf8, 0xff, 0x07}; break;
    case PibTransmitPower: value = {static_cast<std::uint8_t>(tx_power_)}; break;
    case PibCcaMode: value = {cca_mode_}; break;
    case PibCurrentPage: value = {page_}; break;
    case PibAckWaitDuration: value = {54}; break;
    case PibAssociationPermit: value = {association_permit_}; break;
    case PibAutoRequest: value = {auto_request_}; break;
    case PibBattLifeExt: value = {battery_life_ext_}; break;
    case PibBeaconPayload:
        value.assign(beacon_payload_.begin(), beacon_payload_.begin() + beacon_payload_length_);
        break;
    case PibBeaconPayloadLength: value = {beacon_payload_length_}; break;
    case PibBeaconOrder: value = {beacon_order_}; break;
    case PibSuperframeOrder: value = {superframe_order_}; break;
    case PibBsn: value = {bsn_}; break;
    case PibDsn: value = {dsn_}; break;
    case PibCoordExtendedAddress: value.assign(coord_extended_.begin(), coord_extended_.end()); break;
    case PibCoordShortAddress: u16(coord_short_); break;
    case PibGtsPermit: value = {gts_permit_}; break;
    case PibMaxCsmaBackoffs: value = {max_csma_backoffs_}; break;
    case PibMinBe: value = {min_be_}; break;
    case PibPanId: u16(pan_id_); break;
    case PibPromiscuousMode: value = {promiscuous_}; break;
    case PibRxOnWhenIdle: value = {rx_on_when_idle_}; break;
    case PibShortAddress: u16(short_address_); break;
    case PibTransactionPersistenceTime: u16(transaction_persistence_time_); break;
    case PibAssociatedPanCoord: value = {associated_pan_coord_}; break;
    case PibMaxBe: value = {max_be_}; break;
    case PibMaxFrameTotalWaitTime: u16(max_frame_total_wait_time_); break;
    case PibMaxFrameRetries: value = {max_frame_retries_}; break;
    case PibResponseWaitTime: value = {response_wait_time_}; break;
    case PibExtendedAddress: value.assign(extended_.begin(), extended_.end()); break;
    default: status = UnsupportedAttribute; break;
    }
    // MAC_getCnf_t: valuePtr, status, PIB_attribute, value length, stuffing.
    std::vector<std::uint8_t> body(8, 0);
    put32(body.data(), data_address_);
    body[4] = status;
    body[5] = attribute;
    body[6] = static_cast<std::uint8_t>(value.size());
    notify(GetCnf, std::move(body), std::move(value), now_ns + ProcessingNs);
    return Success;
}

std::uint8_t StMac::handle_reset(const std::uint8_t *p, std::uint64_t now_ns) {
    const bool set_default_pib = p[0] != 0;
    // A reset abandons every transaction: no confirms for them follow.
    tx_queue_.clear();
    transactions_.clear();
    op_ = Op::None;
    started_ = false;
    pan_coordinator_ = false;
    if (set_default_pib)
        reset_pib();
    trace(std::string("MLME-RESET set_default_pib=") + (set_default_pib ? "1" : "0"));
    confirm_simple(ResetCnf, Success, now_ns + ProcessingNs);
    return Success;
}

std::uint8_t StMac::handle_scan(const std::uint8_t *p, std::uint64_t now_ns) {
    const std::uint8_t type = p[0];
    const std::uint8_t duration = p[1];
    const std::uint32_t channels = get32(p + 4) & ChannelMask2450;
    if (op_ != Op::None)
        return ScanInProgress;
    // ED, active and passive scans; an orphan scan is not modeled.
    if (type > 2 || duration > 14 || channels == 0 || p[2] != 0)
        return InvalidParameter;
    op_ = Op::Scan;
    scan_ = ScanState{};
    scan_.type = type;
    scan_.channels = channels;
    scan_.duration = duration;
    scan_.saved_channel = channel_;
    scan_.saved_pan = pan_id_;
    // Beacons from every PAN are accepted while scanning (7.5.2.1).
    if (type != 0)
        pan_id_ = 0xffff;
    char text[64];
    std::snprintf(text, sizeof text, "MLME-SCAN type=%u channels=0x%08x duration=%u", type,
                  channels, duration);
    trace(text);
    scan_next_channel(now_ns);
    return Success;
}

void StMac::scan_next_channel(std::uint64_t now_ns) {
    int next = -1;
    for (int channel = scan_.current + 1; channel <= 26; ++channel) {
        if (channel >= 11 && (scan_.channels & (1u << channel))) {
            next = channel;
            break;
        }
    }
    if (next < 0) {
        finish_scan(now_ns);
        return;
    }
    scan_.current = next;
    channel_ = static_cast<std::uint8_t>(next);
    scan_.listening = false;
    const std::uint64_t listen_ns = BaseSuperframeNs * ((1ull << scan_.duration) + 1);
    if (scan_.type == 1) {
        TxJob job;
        Address broadcast{2, 0xffff, 0xffff, {}};
        job.mpdu = build_frame(FrameCommand, false, false, next_dsn(), broadcast, Address{},
                               {CmdBeaconRequest});
        job.purpose = Purpose::BeaconRequest;
        queue_tx(std::move(job), now_ns);
        scan_.listen_until_ns = listen_ns; // relative until the request is on air
    } else {
        scan_.listening = true;
        scan_.listen_until_ns = now_ns + listen_ns;
    }
}

void StMac::finish_scan(std::uint64_t now_ns) {
    op_ = Op::None;
    channel_ = scan_.saved_channel;
    pan_id_ = scan_.saved_pan;
    // MAC_scanCnf_t (185 octets).
    std::vector<std::uint8_t> body(ScanCnfSize, 0);
    std::uint8_t status;
    if (scan_.type == 0) {
        status = Success;
        body[7] = static_cast<std::uint8_t>(scan_.energy.size());
        std::copy(scan_.energy.begin(), scan_.energy.end(), body.begin() + 8);
    } else {
        // The PAN descriptor list stays empty with macAutoRequest FALSE, and
        // the coprocessor then reports NO_BEACON (zigbee-rs observed this on
        // MAC 1.24.0; the beacons themselves arrive as BEACON-NOTIFY).
        status = scan_.descriptors.empty() ? NoBeacon : Success;
        body[7] = static_cast<std::uint8_t>(scan_.descriptors.size());
        for (std::size_t i = 0; i != scan_.descriptors.size(); ++i)
            std::copy(scan_.descriptors[i].begin(), scan_.descriptors[i].end(),
                      body.begin() + 24 + static_cast<std::ptrdiff_t>(i * PanDescriptorSize));
    }
    body[0] = status;
    body[1] = scan_.type;
    body[2] = 0;
    put32(body.data() + 3, 0); // every requested channel was scanned
    trace("MLME-SCAN.confirm status=" + hex8(status) +
          " results=" + std::to_string(static_cast<unsigned>(body[7])));
    notify(ScanCnf, std::move(body), {}, now_ns + ProcessingNs);
}

std::uint8_t StMac::handle_start(const std::uint8_t *p, std::uint64_t now_ns) {
    const std::uint16_t pan = get16(p);
    const std::uint8_t channel = p[2];
    const std::uint8_t beacon_order = p[8];
    const std::uint8_t superframe_order = p[9];
    const bool pan_coordinator = p[10] != 0;
    std::uint8_t status = Success;
    if (channel < 11 || channel > 26 || p[3] != 0)
        status = InvalidParameter;
    else if (beacon_order != 15)
        status = InvalidParameter; // beacon-enabled PANs are not modeled
    else if (short_address_ == 0xffff)
        status = NoShortAddress;
    if (status == Success) {
        pan_id_ = pan;
        channel_ = channel;
        beacon_order_ = beacon_order;
        superframe_order_ = superframe_order == 15 ? 15 : superframe_order;
        pan_coordinator_ = pan_coordinator;
        battery_life_ext_ = p[11] != 0;
        started_ = true;
    }
    trace("MLME-START pan=" + hex16(pan) + " channel=" + std::to_string(channel) +
          " pan_coordinator=" + (pan_coordinator ? "1" : "0") + " -> " + hex8(status));
    confirm_simple(StartCnf, status, now_ns + ProcessingNs);
    return Success;
}

std::uint8_t StMac::handle_associate(const std::uint8_t *p, std::uint64_t now_ns) {
    const std::uint8_t channel = p[0];
    const std::uint8_t mode = p[2];
    if (op_ != Op::None)
        return ScanInProgress;
    if (channel < 11 || channel > 26 || p[1] != 0 || (mode != 2 && mode != 3))
        return InvalidParameter;
    op_ = Op::Associate;
    associate_ = AssociateState{};
    associate_.coordinator = sap_address(mode, p + 4, p + 16);
    associate_.capability = p[3];
    // 7.5.3.1: the device adopts the channel, PAN and coordinator first.
    channel_ = channel;
    pan_id_ = associate_.coordinator.pan;
    if (mode == 2)
        coord_short_ = associate_.coordinator.short_address;
    else
        coord_extended_ = associate_.coordinator.extended;
    Address source{3, 0xffff, 0xffff, extended_};
    TxJob job;
    job.mpdu = build_frame(FrameCommand, false, true, next_dsn(), associate_.coordinator, source,
                           {CmdAssociationRequest, associate_.capability});
    job.purpose = Purpose::AssociationRequest;
    job.ack_request = true;
    trace("MLME-ASSOCIATE coordinator=" + hex_address(associate_.coordinator) + " pan=" +
          hex16(associate_.coordinator.pan) + " channel=" + std::to_string(channel) +
          " capability=" + hex8(associate_.capability));
    queue_tx(std::move(job), now_ns);
    return Success;
}

void StMac::finish_associate(std::uint8_t status, std::uint16_t short_address,
                             std::uint64_t now_ns) {
    op_ = Op::None;
    if (status == Success) {
        short_address_ = short_address;
    } else {
        pan_id_ = 0xffff;
        if (short_address == 0xffff)
            short_address_ = 0xffff;
    }
    // MAC_associateCnf_t (16 octets): short address, status, security.
    std::vector<std::uint8_t> body(16, 0);
    put16(body.data(), short_address);
    body[2] = status;
    trace("MLME-ASSOCIATE.confirm status=" + hex8(status) + " short=" + hex16(short_address));
    notify(AssociateCnf, std::move(body), {}, now_ns + ProcessingNs);
}

std::uint8_t StMac::handle_associate_response(const std::uint8_t *p, std::uint64_t now_ns) {
    if (transactions_.size() >= MaxIndirect)
        return TransactionOverflow;
    Transaction transaction;
    transaction.id = next_transaction_++;
    transaction.kind = TransactionKind::AssociationResponse;
    transaction.destination.mode = 3;
    transaction.destination.pan = pan_id_;
    std::copy(p, p + 8, transaction.destination.extended.begin());
    transaction.assoc_short = get16(p + 8);
    transaction.assoc_status = p[10];
    transaction.expires_ns = now_ns + transaction_persistence_time_ * BaseSuperframeNs;
    trace("MLME-ASSOCIATE.response device=" + hex_address(transaction.destination) +
          " short=" + hex16(transaction.assoc_short) + " status=" +
          hex8(transaction.assoc_status));
    transactions_.push_back(std::move(transaction));
    return Success;
}

std::uint8_t StMac::handle_disassociate(const std::uint8_t *p, std::uint64_t now_ns) {
    const std::uint8_t mode = p[0];
    if (mode != 2 && mode != 3)
        return InvalidParameter;
    const Address device = sap_address(mode, p + 1, p + 4);
    const std::uint8_t reason = p[3];
    const bool indirect = p[12] != 0;
    trace("MLME-DISASSOCIATE device=" + hex_address(device) + " reason=" + hex8(reason) +
          (indirect ? " indirect" : ""));
    if (indirect) {
        if (transactions_.size() >= MaxIndirect)
            return TransactionOverflow;
        Transaction transaction;
        transaction.id = next_transaction_++;
        transaction.kind = TransactionKind::Disassociation;
        transaction.destination = device;
        transaction.reason = reason;
        transaction.expires_ns = now_ns + transaction_persistence_time_ * BaseSuperframeNs;
        transactions_.push_back(std::move(transaction));
        return Success;
    }
    TxJob job;
    Address source{3, pan_id_, 0xffff, extended_};
    job.mpdu = build_frame(FrameCommand, false, true, next_dsn(), device, source,
                           {CmdDisassociation, reason});
    job.purpose = Purpose::Disassociation;
    job.ack_request = true;
    queue_tx(std::move(job), now_ns);
    return Success;
}

std::uint8_t StMac::handle_orphan_response(const std::uint8_t *p, std::uint64_t now_ns) {
    if (p[10] == 0)
        return Success; // not a member: nothing goes on the air (7.5.2.1.4)
    Address orphan{3, 0xffff, 0xffff, {}};
    std::copy(p, p + 8, orphan.extended.begin());
    const std::uint16_t assigned = get16(p + 8);
    Address source{3, pan_id_, 0xffff, extended_};
    std::vector<std::uint8_t> payload{CmdCoordinatorRealignment,
                                      static_cast<std::uint8_t>(pan_id_),
                                      static_cast<std::uint8_t>(pan_id_ >> 8),
                                      static_cast<std::uint8_t>(short_address_),
                                      static_cast<std::uint8_t>(short_address_ >> 8),
                                      channel_,
                                      static_cast<std::uint8_t>(assigned),
                                      static_cast<std::uint8_t>(assigned >> 8)};
    TxJob job;
    job.mpdu = build_frame(FrameCommand, false, true, next_dsn(), orphan, source, payload);
    job.purpose = Purpose::OrphanRealignment;
    job.ack_request = true;
    trace("MLME-ORPHAN.response orphan=" + hex_address(orphan) + " short=" + hex16(assigned));
    queue_tx(std::move(job), now_ns);
    return Success;
}

std::uint8_t StMac::handle_poll(const std::uint8_t *p, std::uint64_t now_ns) {
    const std::uint8_t mode = p[0];
    if (op_ != Op::None)
        return ScanInProgress;
    if (mode != 2 && mode != 3)
        return InvalidParameter;
    op_ = Op::Poll;
    poll_ = PollState{};
    poll_.coordinator = sap_address(mode, p + 20, p + 4);
    TxJob job;
    job.mpdu = build_frame(FrameCommand, false, true, next_dsn(), poll_.coordinator,
                           own_address(true), {CmdDataRequest});
    job.purpose = Purpose::PollDataRequest;
    job.ack_request = true;
    trace("MLME-POLL coordinator=" + hex_address(poll_.coordinator));
    queue_tx(std::move(job), now_ns);
    return Success;
}

void StMac::finish_poll(std::uint8_t status, std::uint64_t now_ns) {
    op_ = Op::None;
    trace("MLME-POLL.confirm status=" + hex8(status));
    confirm_simple(PollCnf, status, now_ns + ProcessingNs);
}

std::uint8_t StMac::handle_data(const std::uint8_t *p, std::uint64_t now_ns) {
    const std::uint32_t pointer = get32(p);
    const std::uint8_t src_mode = p[4];
    const std::uint8_t dst_mode = p[5];
    const Address destination = sap_address(dst_mode, p + 6, p + 8);
    const std::uint8_t length = p[16];
    const std::uint8_t handle = p[17];
    const bool ack = p[18] != 0;
    const bool indirect = p[20] != 0;
    const std::uint8_t security = p[21];
    auto fail = [&](std::uint8_t status) {
        std::vector<std::uint8_t> body(28, 0);
        body[0] = handle;
        body[6] = status;
        trace("MCPS-DATA.confirm handle=" + hex8(handle) + " status=" + hex8(status));
        notify(DataCnf, std::move(body), {}, now_ns + ProcessingNs);
        return Success;
    };
    if ((src_mode != 0 && src_mode != 2 && src_mode != 3) ||
        (dst_mode != 0 && dst_mode != 2 && dst_mode != 3) || (src_mode == 0 && dst_mode == 0))
        return fail(InvalidAddress);
    if (security != 0)
        return fail(UnsupportedSecurity); // Zigbee secures at NWK/APS
    std::vector<std::uint8_t> msdu(length);
    if (length != 0 && !host_.read(pointer, msdu.data(), length))
        return fail(InvalidParameter);
    // Header: FCF, DSN, PAN ID(s) and addresses, plus the 2-octet FCS.
    const std::size_t header = 3 + (dst_mode ? 2u + (dst_mode == 2 ? 2u : 8u) : 0u) +
                               (src_mode ? (src_mode == 2 ? 2u : 8u) : 0u) + 2u;
    if (header + length + 2 > MaxPhyPacket)
        return fail(FrameTooLong);
    if (indirect && dst_mode != 0) {
        if (transactions_.size() >= MaxIndirect)
            return fail(TransactionOverflow);
        Transaction transaction;
        transaction.id = next_transaction_++;
        transaction.kind = TransactionKind::Data;
        transaction.destination = destination;
        transaction.handle = handle;
        transaction.msdu = std::move(msdu);
        transaction.src_mode = src_mode;
        transaction.ack_request = ack;
        transaction.expires_ns = now_ns + transaction_persistence_time_ * BaseSuperframeNs;
        trace("MCPS-DATA indirect handle=" + hex8(handle) + " to " + hex_address(destination));
        transactions_.push_back(std::move(transaction));
        return Success;
    }
    Address source = src_mode == 0 ? Address{} : own_address(src_mode == 2);
    if (src_mode == 3)
        source.mode = 3;
    const bool broadcast = dst_mode == 2 && destination.short_address == 0xffff;
    TxJob job;
    job.mpdu = build_frame(FrameData, false, ack && !broadcast && dst_mode != 0, next_dsn(),
                           destination, source, msdu);
    job.purpose = Purpose::Data;
    job.handle = handle;
    job.ack_request = ack && !broadcast && dst_mode != 0;
    trace("MCPS-DATA handle=" + hex8(handle) + " to " + hex_address(destination) +
          " len=" + std::to_string(static_cast<unsigned>(length)) + (job.ack_request ? " ack" : ""));
    queue_tx(std::move(job), now_ns);
    return Success;
}

std::uint8_t StMac::handle_purge(const std::uint8_t *p, std::uint64_t now_ns) {
    const std::uint8_t handle = p[0];
    const auto found = std::find_if(transactions_.begin(), transactions_.end(),
                                    [&](const Transaction &t) {
                                        return t.kind == TransactionKind::Data &&
                                               t.handle == handle && !t.in_flight;
                                    });
    std::uint8_t status = InvalidHandle;
    if (found != transactions_.end()) {
        transactions_.erase(found);
        status = Success;
    }
    confirm_simple(PurgeCnf, status, now_ns + ProcessingNs, handle);
    return Success;
}

StMac::Address StMac::own_address(bool prefer_short) const {
    Address address;
    address.pan = pan_id_;
    if (prefer_short && short_address_ < 0xfffe) {
        address.mode = 2;
        address.short_address = short_address_;
    } else {
        address.mode = 3;
        address.extended = extended_;
    }
    return address;
}

std::vector<std::uint8_t> StMac::build_frame(std::uint8_t type, bool frame_pending,
                                             bool ack_request, std::uint8_t sequence,
                                             const Address &destination, const Address &source,
                                             const std::vector<std::uint8_t> &payload) {
    // IEEE 802.15.4-2003 frame version: what a 2006 MAC sends for frames
    // without security (7.2.1.1.7).
    const bool compress = destination.mode != 0 && source.mode != 0 &&
                          destination.pan == source.pan;
    std::uint16_t fcf = type & 7u;
    if (frame_pending)
        fcf |= 1u << 4;
    if (ack_request)
        fcf |= 1u << 5;
    if (compress)
        fcf |= 1u << 6;
    fcf |= static_cast<std::uint16_t>((destination.mode & 3u) << 10);
    fcf |= static_cast<std::uint16_t>((source.mode & 3u) << 14);
    std::vector<std::uint8_t> frame{static_cast<std::uint8_t>(fcf),
                                    static_cast<std::uint8_t>(fcf >> 8), sequence};
    auto append_address = [&](const Address &address, bool with_pan) {
        if (address.mode == 0)
            return;
        if (with_pan) {
            frame.push_back(static_cast<std::uint8_t>(address.pan));
            frame.push_back(static_cast<std::uint8_t>(address.pan >> 8));
        }
        if (address.mode == 2) {
            frame.push_back(static_cast<std::uint8_t>(address.short_address));
            frame.push_back(static_cast<std::uint8_t>(address.short_address >> 8));
        } else {
            frame.insert(frame.end(), address.extended.begin(), address.extended.end());
        }
    };
    append_address(destination, true);
    append_address(source, !compress);
    frame.insert(frame.end(), payload.begin(), payload.end());
    return frame;
}

bool StMac::parse_header(const std::vector<std::uint8_t> &mpdu, Header &header) {
    if (mpdu.size() < 3)
        return false;
    header = Header{};
    header.fcf = get16(mpdu.data());
    header.type = header.fcf & 7u;
    header.frame_pending = (header.fcf >> 4) & 1u;
    header.ack_request = (header.fcf >> 5) & 1u;
    header.pan_compression = (header.fcf >> 6) & 1u;
    header.sequence = mpdu[2];
    if (header.type == FrameAck) {
        header.payload_offset = 3;
        return true;
    }
    if ((header.fcf >> 3) & 1u)
        return false; // MAC security: never used by Zigbee, not modeled
    const unsigned dst_mode = (header.fcf >> 10) & 3u;
    const unsigned src_mode = (header.fcf >> 14) & 3u;
    if (dst_mode == 1 || src_mode == 1 || header.type > FrameCommand)
        return false;
    std::size_t offset = 3;
    auto read_address = [&](Address &address, unsigned mode, bool with_pan) {
        address.mode = static_cast<std::uint8_t>(mode);
        if (mode == 0)
            return true;
        if (with_pan) {
            if (offset + 2 > mpdu.size())
                return false;
            address.pan = get16(mpdu.data() + offset);
            offset += 2;
        }
        const std::size_t size = mode == 2 ? 2 : 8;
        if (offset + size > mpdu.size())
            return false;
        if (mode == 2)
            address.short_address = get16(mpdu.data() + offset);
        else
            std::copy(mpdu.begin() + static_cast<std::ptrdiff_t>(offset),
                      mpdu.begin() + static_cast<std::ptrdiff_t>(offset + 8),
                      address.extended.begin());
        offset += size;
        return true;
    };
    if (!read_address(header.destination, dst_mode, true))
        return false;
    const bool src_pan = !(header.pan_compression && dst_mode != 0);
    if (!read_address(header.source, src_mode, src_pan))
        return false;
    if (!src_pan)
        header.source.pan = header.destination.pan;
    header.payload_offset = offset;
    return true;
}

void StMac::queue_tx(TxJob job, std::uint64_t now_ns) {
    tx_queue_.push_back(std::move(job));
    if (tx_queue_.size() == 1)
        start_csma(tx_queue_.front(), now_ns);
}

void StMac::start_csma(TxJob &job, std::uint64_t now_ns) {
    // Unslotted CSMA-CA (7.5.1.4): random(2^BE - 1) unit backoffs, CCA.
    job.nb = 0;
    job.be = min_be_;
    job.on_air = false;
    job.waiting_ack = false;
    const std::uint32_t slots = job.csma ? random() % (1u << job.be) : 0u;
    job.due_ns = now_ns + slots * UnitBackoffNs;
}

void StMac::run_tx(std::uint64_t now_ns) {
    while (!tx_queue_.empty()) {
        TxJob &job = tx_queue_.front();
        if (job.waiting_ack) {
            if (now_ns < job.end_ns + AckWaitNs)
                return;
            if (job.retries >= max_frame_retries_) {
                TxJob done = std::move(job);
                tx_queue_.pop_front();
                ++stats_.no_acks;
                finish_tx(std::move(done), NoAck, now_ns);
                if (!tx_queue_.empty())
                    start_csma(tx_queue_.front(), now_ns);
                continue;
            }
            ++job.retries;
            start_csma(job, now_ns);
            continue;
        }
        if (job.on_air) {
            if (now_ns < job.end_ns)
                return;
            TxJob done = std::move(job);
            tx_queue_.pop_front();
            finish_tx(std::move(done), Success, now_ns);
            if (!tx_queue_.empty())
                start_csma(tx_queue_.front(), now_ns);
            continue;
        }
        if (now_ns < job.due_ns)
            return;
        if (job.csma && host_.channel_busy(channel_, job.due_ns)) {
            ++job.nb;
            job.be = std::min<unsigned>(job.be + 1, max_be_);
            if (job.nb > max_csma_backoffs_) {
                TxJob done = std::move(job);
                tx_queue_.pop_front();
                ++stats_.channel_access_failures;
                finish_tx(std::move(done), ChannelAccessFailure, now_ns);
                if (!tx_queue_.empty())
                    start_csma(tx_queue_.front(), now_ns);
                continue;
            }
            job.due_ns = now_ns + (random() % (1u << job.be)) * UnitBackoffNs;
            return;
        }
        const std::uint64_t start = std::max(job.due_ns + CcaNs + TurnaroundNs, now_ns);
        std::uint64_t end = host_.transmit(channel_, start, job.mpdu);
        if (end == 0)
            end = start + (job.mpdu.size() + 8) * 2 * SymbolNs; // SHR, PHR and FCS too
        ++stats_.frames_sent;
        job.on_air = true;
        job.end_ns = end;
        job.waiting_ack = job.ack_request;
        return;
    }
}

void StMac::finish_tx(TxJob job, std::uint8_t status, std::uint64_t now_ns) {
    const std::uint64_t due = now_ns + ProcessingNs;
    switch (job.purpose) {
    case Purpose::Data: {
        // MAC_dataCnf_t (28 octets): handle, timestamp, ranging, status.
        std::vector<std::uint8_t> body(28, 0);
        body[0] = job.handle;
        put32(body.data() + 1, symbols(job.end_ns));
        body[6] = status;
        trace("MCPS-DATA.confirm handle=" + hex8(job.handle) + " status=" + hex8(status));
        notify(DataCnf, std::move(body), {}, due);
        break;
    }
    case Purpose::AssociationRequest:
        if (op_ != Op::Associate)
            break;
        if (status != Success) {
            finish_associate(status, 0xffff, now_ns);
            break;
        }
        // 7.5.3.1: wait macResponseWaitTime, then extract the response.
        associate_.data_request_at_ns = now_ns + response_wait_time_ * BaseSuperframeNs;
        break;
    case Purpose::AssociationDataRequest:
        if (op_ != Op::Associate)
            break;
        if (status != Success)
            finish_associate(status, 0xffff, now_ns);
        else if (!job.ack_pending_bit)
            finish_associate(NoData, 0xffff, now_ns);
        else {
            associate_.awaiting_response = true;
            associate_.response_deadline_ns = now_ns + max_frame_total_wait_time_ * SymbolNs;
        }
        break;
    case Purpose::PollDataRequest:
        if (op_ != Op::Poll)
            break;
        if (status != Success)
            finish_poll(status, now_ns);
        else if (!job.ack_pending_bit)
            finish_poll(NoData, now_ns);
        else {
            poll_.awaiting_data = true;
            poll_.deadline_ns = now_ns + max_frame_total_wait_time_ * SymbolNs;
        }
        break;
    case Purpose::BeaconRequest:
        if (op_ == Op::Scan && !scan_.listening) {
            scan_.listening = true;
            scan_.listen_until_ns = now_ns + scan_.listen_until_ns;
        }
        break;
    case Purpose::Beacon:
        break;
    case Purpose::IndirectFrame: {
        const auto found = std::find_if(transactions_.begin(), transactions_.end(),
                                        [&](const Transaction &t) {
                                            return t.id == job.transaction;
                                        });
        if (found == transactions_.end())
            break;
        found->in_flight = false;
        if (status != Success)
            break; // stays queued for the next poll until it expires (7.5.6.3)
        const Transaction transaction = *found;
        transactions_.erase(found);
        if (transaction.kind == TransactionKind::Data) {
            std::vector<std::uint8_t> body(28, 0);
            body[0] = transaction.handle;
            put32(body.data() + 1, symbols(job.end_ns));
            body[6] = Success;
            trace("MCPS-DATA.confirm (indirect) handle=" + hex8(transaction.handle) +
                  " status=0x00");
            notify(DataCnf, std::move(body), {}, due);
        } else if (transaction.kind == TransactionKind::AssociationResponse) {
            comm_status(transaction.destination, Success, now_ns);
        } else {
            std::vector<std::uint8_t> body(12, 0);
            body[0] = Success;
            body[1] = transaction.destination.mode;
            put16(body.data() + 2, transaction.destination.pan);
            put_address(body.data() + 4, transaction.destination);
            notify(DisassociateCnf, std::move(body), {}, due);
        }
        break;
    }
    case Purpose::OrphanRealignment: {
        Header header;
        if (parse_header(job.mpdu, header))
            comm_status(header.destination, status, now_ns);
        break;
    }
    case Purpose::Disassociation: {
        Header header;
        std::vector<std::uint8_t> body(12, 0);
        body[0] = status;
        if (parse_header(job.mpdu, header)) {
            body[1] = header.destination.mode;
            put16(body.data() + 2, header.destination.pan);
            put_address(body.data() + 4, header.destination);
        }
        notify(DisassociateCnf, std::move(body), {}, due);
        break;
    }
    }
}

void StMac::comm_status(const Address &destination, std::uint8_t status, std::uint64_t now_ns) {
    // MAC_commStatusInd_t (32 octets).
    std::vector<std::uint8_t> body(32, 0);
    put16(body.data(), pan_id_);
    body[2] = 3;
    body[3] = destination.mode;
    std::copy(extended_.begin(), extended_.end(), body.begin() + 4);
    put_address(body.data() + 12, destination);
    body[20] = status;
    trace("MLME-COMM-STATUS.indication to " + hex_address(destination) + " status=" +
          hex8(status));
    notify(CommStatusInd, std::move(body), {}, now_ns + ProcessingNs);
}

bool StMac::has_transaction_for(const Address &address) const {
    return std::any_of(transactions_.begin(), transactions_.end(), [&](const Transaction &t) {
        return !t.in_flight && same_address(t.destination, address);
    });
}

void StMac::serve_transaction(const Address &requester, std::uint64_t now_ns) {
    auto found = std::find_if(transactions_.begin(), transactions_.end(),
                              [&](const Transaction &t) {
                                  return !t.in_flight && same_address(t.destination, requester);
                              });
    if (found == transactions_.end())
        return;
    const bool more = std::count_if(transactions_.begin(), transactions_.end(),
                                    [&](const Transaction &t) {
                                        return !t.in_flight &&
                                               same_address(t.destination, requester);
                                    }) > 1;
    Transaction &transaction = *found;
    transaction.in_flight = true;
    TxJob job;
    job.purpose = Purpose::IndirectFrame;
    job.transaction = transaction.id;
    switch (transaction.kind) {
    case TransactionKind::Data: {
        Address source = transaction.src_mode == 0 ? Address{}
                                                   : own_address(transaction.src_mode == 2);
        const bool broadcast = transaction.destination.mode == 2 &&
                               transaction.destination.short_address == 0xffff;
        job.ack_request = transaction.ack_request && !broadcast;
        job.mpdu = build_frame(FrameData, more, job.ack_request, next_dsn(),
                               transaction.destination, source, transaction.msdu);
        break;
    }
    case TransactionKind::AssociationResponse: {
        Address destination = transaction.destination;
        destination.pan = pan_id_;
        Address source{3, pan_id_, 0xffff, extended_};
        job.ack_request = true;
        job.mpdu = build_frame(FrameCommand, more, true, next_dsn(), destination, source,
                               {CmdAssociationResponse,
                                static_cast<std::uint8_t>(transaction.assoc_short),
                                static_cast<std::uint8_t>(transaction.assoc_short >> 8),
                                transaction.assoc_status});
        break;
    }
    case TransactionKind::Disassociation: {
        Address source{3, pan_id_, 0xffff, extended_};
        job.ack_request = true;
        job.mpdu = build_frame(FrameCommand, more, true, next_dsn(), transaction.destination,
                               source, {CmdDisassociation, transaction.reason});
        break;
    }
    }
    queue_tx(std::move(job), now_ns);
}

void StMac::expire_transactions(std::uint64_t now_ns) {
    for (auto it = transactions_.begin(); it != transactions_.end();) {
        if (it->in_flight || now_ns < it->expires_ns) {
            ++it;
            continue;
        }
        const Transaction transaction = *it;
        it = transactions_.erase(it);
        if (transaction.kind == TransactionKind::Data) {
            std::vector<std::uint8_t> body(28, 0);
            body[0] = transaction.handle;
            body[6] = TransactionExpired;
            trace("MCPS-DATA.confirm (indirect) handle=" + hex8(transaction.handle) +
                  " status=" + hex8(TransactionExpired));
            notify(DataCnf, std::move(body), {}, now_ns + ProcessingNs);
        } else if (transaction.kind == TransactionKind::AssociationResponse) {
            comm_status(transaction.destination, TransactionExpired, now_ns);
        } else {
            std::vector<std::uint8_t> body(12, 0);
            body[0] = TransactionExpired;
            body[1] = transaction.destination.mode;
            put16(body.data() + 2, transaction.destination.pan);
            put_address(body.data() + 4, transaction.destination);
            notify(DisassociateCnf, std::move(body), {}, now_ns + ProcessingNs);
        }
    }
}

bool StMac::accept(const Header &header) const {
    // Third-level filtering of 7.5.6.2 for data and command frames.
    if (header.destination.mode == 0)
        return pan_coordinator_ && header.source.pan == pan_id_;
    if (header.destination.pan != 0xffff && header.destination.pan != pan_id_)
        return false;
    if (header.destination.mode == 2)
        return header.destination.short_address == 0xffff ||
               header.destination.short_address == short_address_;
    return header.destination.extended == extended_;
}

void StMac::send_ack(const StMacRxFrame &frame, const Header &header, bool pending) {
    // Imm-Ack aTurnaroundTime after the frame (7.5.6.4.2).
    const std::vector<std::uint8_t> ack{static_cast<std::uint8_t>(FrameAck | (pending ? 0x10 : 0)),
                                        0x00, header.sequence};
    host_.transmit(frame.channel, frame.end_ns + TurnaroundNs, ack);
    ++stats_.acks_sent;
}

void StMac::receive(const StMacRxFrame &frame, std::uint64_t now_ns) {
    now_ns_ = std::max(now_ns_, now_ns);
    Header header;
    if (!parse_header(frame.mpdu, header))
        return;
    ++stats_.frames_received;
    if (header.type == FrameAck) {
        if (!tx_queue_.empty() && tx_queue_.front().waiting_ack &&
            frame.mpdu.size() == 3 && header.sequence == tx_queue_.front().mpdu[2] &&
            frame.start_ns >= tx_queue_.front().end_ns) {
            TxJob done = std::move(tx_queue_.front());
            tx_queue_.pop_front();
            done.ack_pending_bit = header.frame_pending;
            finish_tx(std::move(done), Success, now_ns);
            if (!tx_queue_.empty())
                start_csma(tx_queue_.front(), now_ns);
        }
        return;
    }
    if (op_ == Op::Scan) {
        if (header.type == FrameBeacon && scan_.type != 0 && scan_.listening)
            beacon_received(frame, header, now_ns);
        return; // every other frame is discarded during a scan (7.5.2.1)
    }
    if (header.type == FrameBeacon)
        return; // non-beacon PAN: no tracking, so no BEACON-NOTIFY outside a scan
    if (promiscuous_) {
        if (header.type == FrameData)
            deliver_data(frame, header, now_ns);
        return;
    }
    if (!accept(header))
        return;
    const bool broadcast = header.destination.mode == 2 &&
                           header.destination.short_address == 0xffff;
    const std::size_t offset = header.payload_offset;
    const bool data_request = header.type == FrameCommand && offset < frame.mpdu.size() &&
                              frame.mpdu[offset] == CmdDataRequest;
    if (header.ack_request && !broadcast)
        send_ack(frame, header, data_request && has_transaction_for(header.source));
    if (header.type == FrameData)
        deliver_data(frame, header, now_ns);
    else if (header.type == FrameCommand)
        handle_command(frame, header, now_ns);
}

void StMac::deliver_data(const StMacRxFrame &frame, const Header &header, std::uint64_t now_ns) {
    const std::size_t offset = header.payload_offset;
    std::vector<std::uint8_t> msdu(frame.mpdu.begin() + static_cast<std::ptrdiff_t>(offset),
                                   frame.mpdu.end());
    // MAC_dataInd_t (68 octets).
    std::vector<std::uint8_t> body(DataIndSize, 0);
    put32(body.data(), data_address_);
    body[4] = header.source.mode;
    put16(body.data() + 5, header.source.pan);
    put_address(body.data() + 7, header.source);
    body[15] = header.destination.mode;
    put16(body.data() + 16, header.destination.pan);
    put_address(body.data() + 18, header.destination);
    body[26] = static_cast<std::uint8_t>(msdu.size());
    body[27] = lqi(frame.rssi_dbm);
    body[28] = header.sequence;
    put32(body.data() + 29, symbols(frame.start_ns));
    body[65] = static_cast<std::uint8_t>(static_cast<std::int8_t>(std::clamp(frame.rssi_dbm, -127, 0)));
    trace("MCPS-DATA.indication from " + hex_address(header.source) + " to " +
          hex_address(header.destination) + " len=" + std::to_string(msdu.size()));
    notify(DataInd, std::move(body), std::move(msdu), now_ns + ProcessingNs);
    if (op_ == Op::Poll && poll_.awaiting_data && same_address(header.source, poll_.coordinator))
        finish_poll(Success, now_ns);
}

void StMac::handle_command(const StMacRxFrame &frame, const Header &header,
                           std::uint64_t now_ns) {
    const std::size_t offset = header.payload_offset;
    if (offset >= frame.mpdu.size())
        return;
    const std::uint8_t *payload = frame.mpdu.data() + offset;
    const std::size_t size = frame.mpdu.size() - offset;
    const std::uint64_t due = now_ns + ProcessingNs;
    switch (payload[0]) {
    case CmdAssociationRequest: {
        if (!started_ || !association_permit_ || header.source.mode != 3 || size < 2)
            return;
        // MAC_associateInd_t (20 octets).
        std::vector<std::uint8_t> body(20, 0);
        std::copy(header.source.extended.begin(), header.source.extended.end(), body.begin());
        body[8] = payload[1];
        trace("MLME-ASSOCIATE.indication device=" + hex_address(header.source) +
              " capability=" + hex8(payload[1]));
        notify(AssociateInd, std::move(body), {}, due);
        return;
    }
    case CmdAssociationResponse: {
        if (op_ != Op::Associate || size < 4)
            return;
        finish_associate(payload[3], get16(payload + 1), now_ns);
        return;
    }
    case CmdDisassociation: {
        if (header.source.mode != 3 || size < 2)
            return;
        // MAC_disassociateInd_t (20 octets).
        std::vector<std::uint8_t> body(20, 0);
        std::copy(header.source.extended.begin(), header.source.extended.end(), body.begin());
        body[8] = payload[1];
        trace("MLME-DISASSOCIATE.indication device=" + hex_address(header.source));
        notify(DisassociateInd, std::move(body), {}, due);
        return;
    }
    case CmdDataRequest: {
        if (!started_)
            return;
        // Indirect delivery starts after our ACK has gone out.
        if (has_transaction_for(header.source))
            serve_transaction(header.source, frame.end_ns + TurnaroundNs + 11 * 2 * SymbolNs);
        if (header.source.mode == 2) {
            // MLME-POLL.indication (ST extension, MAC_pollInd_t, 9 octets).
            std::vector<std::uint8_t> body(9, 0);
            body[0] = 2;
            put_address(body.data() + 1, header.source);
            notify(PollInd, std::move(body), {}, due);
        }
        return;
    }
    case CmdOrphanNotification: {
        if (!started_ || header.source.mode != 3)
            return;
        // MAC_orphanInd_t (20 octets).
        std::vector<std::uint8_t> body(20, 0);
        std::copy(header.source.extended.begin(), header.source.extended.end(), body.begin());
        trace("MLME-ORPHAN.indication orphan=" + hex_address(header.source));
        notify(OrphanInd, std::move(body), {}, due);
        return;
    }
    case CmdBeaconRequest: {
        if (!started_)
            return;
        // Non-beacon PAN: a beacon is sent with unslotted CSMA-CA (7.5.2.4).
        std::vector<std::uint8_t> payload_bytes;
        const std::uint16_t superframe =
            static_cast<std::uint16_t>((beacon_order_ & 0x0fu) | ((superframe_order_ & 0x0fu) << 4) |
                                       (0x0fu << 8) | (battery_life_ext_ ? 1u << 12 : 0u) |
                                       (pan_coordinator_ ? 1u << 14 : 0u) |
                                       (association_permit_ ? 1u << 15 : 0u));
        payload_bytes.push_back(static_cast<std::uint8_t>(superframe));
        payload_bytes.push_back(static_cast<std::uint8_t>(superframe >> 8));
        payload_bytes.push_back(0); // GTS specification: no descriptors
        payload_bytes.push_back(0); // pending address specification: none
        payload_bytes.insert(payload_bytes.end(), beacon_payload_.begin(),
                             beacon_payload_.begin() + beacon_payload_length_);
        TxJob job;
        job.purpose = Purpose::Beacon;
        job.mpdu = build_frame(FrameBeacon, false, false, bsn_++, Address{}, own_address(true),
                               payload_bytes);
        queue_tx(std::move(job), now_ns);
        return;
    }
    default:
        return;
    }
}

void StMac::beacon_received(const StMacRxFrame &frame, const Header &header,
                            std::uint64_t now_ns) {
    const std::size_t offset = header.payload_offset;
    const std::vector<std::uint8_t> &mpdu = frame.mpdu;
    if (header.source.mode == 0 || offset + 4 > mpdu.size())
        return;
    const std::uint16_t superframe = get16(mpdu.data() + offset);
    std::size_t at = offset + 2;
    const std::uint8_t gts_spec = mpdu[at++];
    const unsigned gts_count = gts_spec & 7u;
    if (gts_count)
        at += 1 + 3 * gts_count;
    if (at >= mpdu.size())
        return;
    const std::uint8_t pending_spec = mpdu[at++];
    const unsigned short_count = pending_spec & 7u;
    const unsigned extended_count = (pending_spec >> 4) & 7u;
    const std::size_t pending_at = at;
    at += 2 * short_count + 8 * extended_count;
    if (at > mpdu.size())
        return;
    std::vector<std::uint8_t> sdu(mpdu.begin() + static_cast<std::ptrdiff_t>(at), mpdu.end());
    if (sdu.size() > MaxBeaconPayload)
        sdu.resize(MaxBeaconPayload);

    // MAC_PAN_Desc_t (24 octets).
    std::array<std::uint8_t, PanDescriptorSize> descriptor{};
    put16(descriptor.data(), header.source.pan);
    descriptor[2] = header.source.mode;
    descriptor[3] = frame.channel;
    put_address(descriptor.data() + 4, header.source);
    descriptor[12] = 0;
    descriptor[13] = (gts_spec >> 7) & 1u;
    put16(descriptor.data() + 14, superframe);
    put32(descriptor.data() + 16, symbols(frame.start_ns));
    descriptor[20] = lqi(frame.rssi_dbm);
    scan_.any_beacon = true;
    if (auto_request_ && scan_.descriptors.size() < MaxPanDescriptors) {
        const bool known = std::any_of(
            scan_.descriptors.begin(), scan_.descriptors.end(), [&](const auto &existing) {
                return std::equal(existing.begin(), existing.begin() + 12, descriptor.begin());
            });
        if (!known)
            scan_.descriptors.push_back(descriptor);
    }
    // 7.1.5.1: a BEACON-NOTIFY.indication accompanies a beacon with a payload
    // or any beacon when macAutoRequest is FALSE.
    if (sdu.empty() && auto_request_)
        return;
    // MLME-BEACON-NOTIFY.indication (MAC_beaconNotifyInd_t, 88 octets).
    std::vector<std::uint8_t> body(BeaconNotifyIndSize, 0);
    put32(body.data(), data_address_);
    std::copy(descriptor.begin(), descriptor.end(), body.begin() + 4);
    std::size_t list = 28;
    std::size_t pending = pending_at;
    for (unsigned i = 0; i != short_count && list + 8 <= 84; ++i, list += 8, pending += 2) {
        body[list] = mpdu[pending];
        body[list + 1] = mpdu[pending + 1];
    }
    for (unsigned i = 0; i != extended_count && list + 8 <= 84; ++i, list += 8, pending += 8)
        std::copy(mpdu.begin() + static_cast<std::ptrdiff_t>(pending),
                  mpdu.begin() + static_cast<std::ptrdiff_t>(pending + 8),
                  body.begin() + static_cast<std::ptrdiff_t>(list));
    body[84] = header.sequence;
    body[85] = pending_spec;
    body[86] = static_cast<std::uint8_t>(sdu.size());
    trace("MLME-BEACON-NOTIFY.indication from " + hex_address(header.source) + " pan=" +
          hex16(header.source.pan) + " channel=" + std::to_string(frame.channel) +
          " permit=" + ((superframe >> 15) ? "1" : "0"));
    notify(BeaconNotifyInd, std::move(body), std::move(sdu), now_ns + ProcessingNs);
}

std::uint64_t StMac::service(std::uint64_t now_ns) {
    now_ns_ = std::max(now_ns_, now_ns);
    run_tx(now_ns);
    if (op_ == Op::Scan && scan_.listening && now_ns >= scan_.listen_until_ns) {
        if (scan_.type == 0)
            scan_.energy.push_back(host_.channel_busy(channel_, now_ns) ? 0xff : 0x00);
        scan_next_channel(now_ns);
        run_tx(now_ns);
    }
    if (op_ == Op::Associate) {
        if (associate_.data_request_at_ns != 0 && now_ns >= associate_.data_request_at_ns) {
            associate_.data_request_at_ns = 0;
            Address coordinator = associate_.coordinator;
            coordinator.pan = pan_id_;
            Address source{3, pan_id_, 0xffff, extended_};
            TxJob job;
            job.mpdu = build_frame(FrameCommand, false, true, next_dsn(), coordinator, source,
                                   {CmdDataRequest});
            job.purpose = Purpose::AssociationDataRequest;
            job.ack_request = true;
            queue_tx(std::move(job), now_ns);
            run_tx(now_ns);
        } else if (associate_.awaiting_response && now_ns >= associate_.response_deadline_ns) {
            finish_associate(NoData, 0xffff, now_ns);
        }
    }
    if (op_ == Op::Poll && poll_.awaiting_data && now_ns >= poll_.deadline_ns)
        finish_poll(NoData, now_ns);
    expire_transactions(now_ns);

    if (!pending_.empty()) {
        std::stable_sort(pending_.begin(), pending_.end(),
                         [](const Pending &a, const Pending &b) { return a.due_ns < b.due_ns; });
        std::size_t ready = 0;
        while (ready != pending_.size() && pending_[ready].due_ns <= now_ns) {
            notifications_.push_back(std::move(pending_[ready].notification));
            ++ready;
        }
        pending_.erase(pending_.begin(), pending_.begin() + static_cast<std::ptrdiff_t>(ready));
    }

    std::uint64_t next = Never;
    if (!pending_.empty())
        next = std::min(next, pending_.front().due_ns);
    if (!tx_queue_.empty()) {
        const TxJob &job = tx_queue_.front();
        if (job.waiting_ack)
            next = std::min(next, job.end_ns + AckWaitNs);
        else if (job.on_air)
            next = std::min(next, job.end_ns);
        else
            next = std::min(next, job.due_ns);
    }
    if (op_ == Op::Scan && scan_.listening)
        next = std::min(next, scan_.listen_until_ns);
    if (op_ == Op::Associate) {
        if (associate_.data_request_at_ns != 0)
            next = std::min(next, associate_.data_request_at_ns);
        if (associate_.awaiting_response)
            next = std::min(next, associate_.response_deadline_ns);
    }
    if (op_ == Op::Poll && poll_.awaiting_data)
        next = std::min(next, poll_.deadline_ns);
    for (const Transaction &transaction : transactions_)
        if (!transaction.in_flight)
            next = std::min(next, transaction.expires_ns);
    return next == Never ? Never : std::max(next, now_ns + 1);
}

} // namespace stm32wb55emu
