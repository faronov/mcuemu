// The built-in model of ST's STM32WB IEEE 802.15.4 MAC coprocessor: request
// parsing (MAC SAP structures of 802_15_4_mac_types.h), confirm and indication
// encoding, the frames it puts on the air, and the IPCC mailbox exchange of the
// chip model (AN5289 reference table, SHCI ready event, MAC command channel).

#include "../test_support.hpp"
#include "stm32wb55emu/st_mac.hpp"
#include "stm32wb55emu/stm32wb55.hpp"

#include <algorithm>
#include <cstring>
#include <map>

namespace mcuemu_tests {

namespace {

using stm32wb55emu::StMac;
using stm32wb55emu::StMacNotification;
using stm32wb55emu::StMacRxFrame;

constexpr std::uint64_t Ms = 1000000;

std::uint16_t get16(const std::vector<std::uint8_t> &bytes, std::size_t at) {
    return static_cast<std::uint16_t>(bytes[at] | (bytes[at + 1] << 8));
}

std::uint32_t get32(const std::vector<std::uint8_t> &bytes, std::size_t at) {
    return static_cast<std::uint32_t>(bytes[at]) | (static_cast<std::uint32_t>(bytes[at + 1]) << 8) |
           (static_cast<std::uint32_t>(bytes[at + 2]) << 16) |
           (static_cast<std::uint32_t>(bytes[at + 3]) << 24);
}

void put16(std::uint8_t *p, std::uint16_t value) {
    p[0] = static_cast<std::uint8_t>(value);
    p[1] = static_cast<std::uint8_t>(value >> 8);
}

void put32(std::uint8_t *p, std::uint32_t value) {
    for (unsigned i = 0; i != 4; ++i)
        p[i] = static_cast<std::uint8_t>(value >> (8 * i));
}

struct Sent {
    std::uint8_t channel;
    std::uint64_t start_ns;
    std::vector<std::uint8_t> mpdu;
};

// CPU1 memory and the medium as the MAC model sees them.
struct FakeHost final : stm32wb55emu::StMacHost {
    std::map<std::uint32_t, std::vector<std::uint8_t>> memory;
    std::vector<Sent> sent;
    bool busy{false};

    bool read(std::uint32_t address, void *out, std::size_t size) override {
        for (const auto &[base, bytes] : memory) {
            if (address >= base && address + size <= base + bytes.size()) {
                std::memcpy(out, bytes.data() + (address - base), size);
                return true;
            }
        }
        return false;
    }
    std::uint64_t transmit(std::uint8_t channel, std::uint64_t start_ns,
                           const std::vector<std::uint8_t> &mpdu) override {
        sent.push_back({channel, start_ns, mpdu});
        return start_ns + (mpdu.size() + 8) * 32000;
    }
    bool channel_busy(std::uint8_t, std::uint64_t) override { return busy; }
};

struct Mac {
    FakeHost host;
    StMac mac{host, 1};
    std::uint64_t now{0};

    Mac() { mac.set_data_address(0x2003fc00); }

    // Advance the model to `until`, servicing it at every timer it asks for.
    void run(std::uint64_t until) {
        while (now < until) {
            const std::uint64_t next = mac.service(now);
            now = std::min(until, std::max(now + 1, next));
        }
        mac.service(now);
    }
    std::vector<StMacNotification> take() {
        std::vector<StMacNotification> all;
        while (auto notification = mac.pop_notification())
            all.push_back(std::move(*notification));
        return all;
    }
    std::uint8_t set(std::uint8_t attribute, std::vector<std::uint8_t> value) {
        host.memory[0x20001000] = std::move(value);
        std::uint8_t request[StMac::SetReqSize] = {};
        put32(request, 0x20001000);
        request[4] = attribute;
        return mac.request(StMac::SetReq, request, sizeof request, now);
    }
    // A frame from another node that ends now.
    void hear(std::vector<std::uint8_t> mpdu, int rssi = -60) {
        StMacRxFrame frame;
        frame.channel = mac.channel();
        frame.end_ns = now;
        frame.start_ns = now - (mpdu.size() + 8) * 32000;
        frame.rssi_dbm = rssi;
        frame.mpdu = std::move(mpdu);
        mac.receive(frame, now);
    }
    // Run until `count` frames went on the air (CSMA-CA backoffs are random).
    void run_until_sent(std::size_t count) {
        for (unsigned step = 0; step != 1000 && host.sent.size() < count; ++step)
            run(now + 10000);
    }
    // The Imm-Ack of the last frame sent, aTurnaroundTime after it ended.
    void ack_last(bool pending = false) {
        const Sent &last = host.sent.back();
        const std::uint64_t end = last.start_ns + (last.mpdu.size() + 8) * 32000;
        run(end + 192000 + 11 * 32000);
        hear({static_cast<std::uint8_t>(pending ? 0x12 : 0x02), 0x00, last.mpdu[2]});
    }
};

const std::vector<std::uint8_t> Extended = {0x33, 0x22, 0x11, 0x00, 0x26, 0xe1, 0x80, 0x00};

void program_router(Mac &m, std::uint16_t pan, std::uint16_t short_address) {
    m.set(0x6f, Extended);
    m.set(0x50, {static_cast<std::uint8_t>(pan), static_cast<std::uint8_t>(pan >> 8)});
    m.set(0x53, {static_cast<std::uint8_t>(short_address),
                 static_cast<std::uint8_t>(short_address >> 8)});
    std::uint8_t start[StMac::StartReqSize] = {};
    put16(start, pan);
    start[2] = 15;
    start[8] = 15;
    start[9] = 15;
    CHECK(m.mac.request(StMac::StartReq, start, sizeof start, m.now) == StMac::Success);
    m.run(m.now + Ms);
    m.take();
}

void test_st_mac_set_get_reset_confirms() {
    Mac m;
    // MAC_setCnf_t: subEvtCode 7, status, PIB attribute.
    CHECK(m.set(0x6f, Extended) == StMac::Success);
    CHECK(m.set(0x00, {30}) == StMac::Success); // out of range channel
    CHECK(m.set(0x99, {0}) == StMac::Success);  // unknown attribute
    m.run(Ms);
    auto confirms = m.take();
    CHECK(confirms.size() == 3);
    if (confirms.size() == 3) {
        CHECK(confirms[0].payload.size() == 2 + 4);
        CHECK(get16(confirms[0].payload, 0) == StMac::SetCnf);
        CHECK(confirms[0].payload[2] == StMac::Success);
        CHECK(confirms[0].payload[3] == 0x6f);
        CHECK(confirms[1].payload[2] == StMac::InvalidParameter);
        CHECK(confirms[2].payload[2] == StMac::UnsupportedAttribute);
    }
    // MAC_getCnf_t points at the value it copies to CPU2 memory.
    const std::uint8_t get[StMac::GetReqSize] = {0x6f, 0, 0, 0};
    CHECK(m.mac.request(StMac::GetReq, get, sizeof get, m.now) == StMac::Success);
    m.run(2 * Ms);
    confirms = m.take();
    CHECK(confirms.size() == 1);
    if (confirms.size() == 1) {
        CHECK(get16(confirms[0].payload, 0) == StMac::GetCnf);
        CHECK(confirms[0].payload.size() == 2 + 8);
        CHECK(get32(confirms[0].payload, 2) == 0x2003fc00);
        CHECK(confirms[0].payload[6] == StMac::Success);
        CHECK(confirms[0].payload[7] == 0x6f);
        CHECK(confirms[0].payload[8] == 8);
        CHECK(confirms[0].data == Extended);
    }
    // MAC_resetCnf_t; a short request is refused synchronously.
    const std::uint8_t reset[StMac::ResetReqSize] = {1, 0, 0, 0};
    CHECK(m.mac.request(StMac::ResetReq, reset, sizeof reset, m.now) == StMac::Success);
    CHECK(m.mac.request(StMac::ResetReq, reset, 2, m.now) == StMac::InvalidParameter);
    CHECK(m.mac.request(StMac::GtsReq, reset, sizeof reset, m.now) == StMac::NotImplemented);
    m.run(3 * Ms);
    confirms = m.take();
    CHECK(confirms.size() == 1 && get16(confirms[0].payload, 0) == StMac::ResetCnf &&
          confirms[0].payload[2] == StMac::Success);
}

void test_st_mac_data_request_ack_and_retries() {
    Mac m;
    program_router(m, 0xab8c, 0x1234);
    m.host.memory[0x20002000] = {0x48, 0x02, 0x00, 0x00, 0x34, 0x12, 0x1e, 0x05};
    // MAC_dataReq_t: msduPtr, src/dst mode, dst PAN, dst address, length,
    // handle, ack, GTS, indirect, security.
    std::uint8_t data[StMac::DataReqSize] = {};
    put32(data, 0x20002000);
    data[4] = 2;
    data[5] = 2;
    put16(data + 6, 0xab8c);
    put16(data + 8, 0x0000);
    data[16] = 8;
    data[17] = 0x42;
    data[18] = 1;
    CHECK(m.mac.request(StMac::DataReq, data, sizeof data, m.now) == StMac::Success);
    m.run(m.now + 3 * Ms);
    CHECK(m.host.sent.size() == 1);
    if (m.host.sent.empty())
        return;
    const std::vector<std::uint8_t> mpdu = m.host.sent[0].mpdu;
    // Data, ack request, PAN ID compression, short addresses (FCF 0x8861).
    CHECK(mpdu.size() == 9 + 8);
    CHECK(get16(mpdu, 0) == 0x8861);
    CHECK(get16(mpdu, 3) == 0xab8c && get16(mpdu, 5) == 0x0000 && get16(mpdu, 7) == 0x1234);
    CHECK(std::equal(mpdu.begin() + 9, mpdu.end(), m.host.memory[0x20002000].begin()));
    CHECK(m.host.sent[0].channel == 15);
    CHECK(m.take().empty()); // the confirm waits for the ACK
    m.ack_last();
    m.run(m.now + Ms);
    auto confirms = m.take();
    CHECK(confirms.size() == 1);
    if (confirms.size() == 1) {
        // MAC_dataCnf_t: handle at 0, timestamp at 1, status at 6 (28 octets).
        CHECK(get16(confirms[0].payload, 0) == StMac::DataCnf);
        CHECK(confirms[0].payload.size() == 2 + 28);
        CHECK(confirms[0].payload[2] == 0x42);
        CHECK(confirms[0].payload[2 + 6] == StMac::Success);
    }

    // Without an ACK: macMaxFrameRetries (3) retransmissions, then NO_ACK.
    data[17] = 0x43;
    CHECK(m.mac.request(StMac::DataReq, data, sizeof data, m.now) == StMac::Success);
    m.run(m.now + 50 * Ms);
    CHECK(m.host.sent.size() == 1 + 4);
    confirms = m.take();
    CHECK(confirms.size() == 1 && confirms[0].payload[2] == 0x43 &&
          confirms[0].payload[2 + 6] == StMac::NoAck);
    CHECK(m.mac.stats().no_acks == 1);

    // A busy channel ends in CHANNEL_ACCESS_FAILURE after macMaxCSMABackoffs.
    m.host.busy = true;
    data[17] = 0x44;
    CHECK(m.mac.request(StMac::DataReq, data, sizeof data, m.now) == StMac::Success);
    m.run(m.now + 100 * Ms);
    CHECK(m.host.sent.size() == 5);
    confirms = m.take();
    CHECK(confirms.size() == 1 && confirms[0].payload[2 + 6] == StMac::ChannelAccessFailure);
}

void test_st_mac_data_indication_and_imm_ack() {
    Mac m;
    program_router(m, 0xab8c, 0x1234);
    m.now += Ms;
    // Coordinator 0x0000 to us, ack requested, NWK payload.
    m.hear({0x61, 0x88, 0x77, 0x8c, 0xab, 0x34, 0x12, 0x00, 0x00, 0xde, 0xad, 0xbe, 0xef}, -55);
    CHECK(m.host.sent.size() == 1);
    if (!m.host.sent.empty()) {
        CHECK(m.host.sent[0].mpdu == (std::vector<std::uint8_t>{0x02, 0x00, 0x77}));
        CHECK(m.host.sent[0].start_ns == m.now + 192000); // aTurnaroundTime
    }
    // Another PAN and someone else's short address are filtered out.
    m.hear({0x61, 0x88, 0x78, 0x8d, 0xab, 0x34, 0x12, 0x00, 0x00, 0x01});
    m.hear({0x61, 0x88, 0x79, 0x8c, 0xab, 0x35, 0x12, 0x00, 0x00, 0x01});
    m.run(m.now + Ms);
    auto indications = m.take();
    CHECK(indications.size() == 1);
    if (indications.size() != 1)
        return;
    const std::vector<std::uint8_t> &p = indications[0].payload;
    // MAC_dataInd_t (68 octets) after the 16-bit identifier.
    CHECK(get16(p, 0) == StMac::DataInd);
    CHECK(p.size() == 2 + 68);
    CHECK(get32(p, 2 + 0) == 0x2003fc00);
    CHECK(p[2 + 4] == 2 && get16(p, 2 + 5) == 0xab8c && get16(p, 2 + 7) == 0x0000);
    CHECK(p[2 + 15] == 2 && get16(p, 2 + 16) == 0xab8c && get16(p, 2 + 18) == 0x1234);
    CHECK(p[2 + 26] == 4);
    CHECK(p[2 + 28] == 0x77);
    CHECK(static_cast<std::int8_t>(p[2 + 65]) == -55);
    CHECK(indications[0].data == (std::vector<std::uint8_t>{0xde, 0xad, 0xbe, 0xef}));
}

void test_st_mac_active_scan_and_associate() {
    Mac m;
    m.set(0x6f, Extended);
    m.set(0x42, {0}); // macAutoRequest = FALSE, as zigbee-rs programs it
    m.run(Ms);
    m.take();
    // MAC_scanReq_t: active scan of channel 15, duration 3.
    std::uint8_t scan[StMac::ScanReqSize] = {};
    scan[0] = 1;
    scan[1] = 3;
    put32(scan + 4, 1u << 15);
    CHECK(m.mac.request(StMac::ScanReq, scan, sizeof scan, m.now) == StMac::Success);
    m.run(m.now + 5 * Ms);
    CHECK(m.host.sent.size() == 1);
    if (!m.host.sent.empty())
        CHECK(m.host.sent[0].mpdu ==
              (std::vector<std::uint8_t>{0x03, 0x08, m.host.sent[0].mpdu[2], 0xff, 0xff, 0xff,
                                         0xff, 0x07}));
    // A Zigbee router beacon: permit join, 15-octet beacon payload.
    std::vector<std::uint8_t> beacon{0x00, 0x80, 0x21, 0x8c, 0xab, 0x00, 0x00,
                                     0xff, 0xcf, 0x00, 0x00};
    const std::vector<std::uint8_t> zigbee{0x00, 0x22, 0x84, 1, 2, 3, 4, 5, 6, 7, 8,
                                           0xff, 0xff, 0xff, 0x00};
    beacon.insert(beacon.end(), zigbee.begin(), zigbee.end());
    m.hear(beacon, -40);
    m.run(m.now + 200 * Ms);
    auto events = m.take();
    CHECK(events.size() == 2);
    if (events.size() != 2)
        return;
    // MLME-BEACON-NOTIFY.indication (MAC_beaconNotifyInd_t, 88 octets).
    const std::vector<std::uint8_t> &notify = events[0].payload;
    CHECK(get16(notify, 0) == StMac::BeaconNotifyInd);
    CHECK(notify.size() == 2 + 88);
    CHECK(get16(notify, 2 + 4) == 0xab8c); // PAN descriptor: coord PAN
    CHECK(notify[2 + 6] == 2 && notify[2 + 7] == 15);
    CHECK(get16(notify, 2 + 8) == 0x0000);
    CHECK(get16(notify, 2 + 18) == 0xcfff); // superframe specification
    CHECK(notify[2 + 24] == 255);           // LQI
    CHECK(notify[2 + 84] == 0x21);          // BSN
    CHECK(notify[2 + 86] == zigbee.size());
    CHECK(events[0].data == zigbee);
    // MAC_scanCnf_t (185 octets): NO_BEACON with an empty descriptor list.
    CHECK(get16(events[1].payload, 0) == StMac::ScanCnf);
    CHECK(events[1].payload.size() == 2 + 185);
    CHECK(events[1].payload[2] == StMac::NoBeacon && events[1].payload[3] == 1);

    // MAC_associateReq_t: channel 15, short coordinator 0x0000 in PAN 0xab8c.
    m.host.sent.clear();
    std::uint8_t associate[StMac::AssociateReqSize] = {};
    associate[0] = 15;
    associate[2] = 2;
    associate[3] = 0x8e;
    put16(associate + 4, 0xab8c);
    put16(associate + 16, 0x0000);
    CHECK(m.mac.request(StMac::AssociateReq, associate, sizeof associate, m.now) ==
          StMac::Success);
    m.run_until_sent(1);
    CHECK(m.host.sent.size() == 1);
    if (m.host.sent.size() != 1)
        return;
    const std::vector<std::uint8_t> request = m.host.sent[0].mpdu;
    CHECK(get16(request, 0) == 0xc823); // command, ack, short dst, extended src
    CHECK(request.back() == 0x8e && request[request.size() - 2] == 0x01);
    m.ack_last();
    // macResponseWaitTime later the device extracts the response.
    m.run(m.now + 400 * Ms);
    CHECK(m.host.sent.size() == 1);
    for (unsigned step = 0; step != 4000 && m.host.sent.size() < 2; ++step)
        m.run(m.now + 50000);
    CHECK(m.host.sent.size() == 2);
    if (m.host.sent.size() != 2)
        return;
    CHECK(m.host.sent[1].mpdu.back() == 0x04); // Data Request command
    m.ack_last(true);
    m.run(m.now + Ms);
    std::vector<std::uint8_t> response{0x63, 0xcc, 0x10, 0x8c, 0xab};
    response.insert(response.end(), Extended.begin(), Extended.end());
    for (std::uint8_t byte : {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08})
        response.push_back(byte);
    for (std::uint8_t byte : {0x02, 0x34, 0xe5, 0x00})
        response.push_back(byte);
    m.hear(response);
    m.run(m.now + Ms);
    CHECK(m.host.sent.size() == 3 && m.host.sent[2].mpdu.size() == 3); // our ACK
    events = m.take();
    CHECK(events.size() == 1);
    if (events.size() == 1) {
        // MAC_associateCnf_t: short address at 0, status at 2 (16 octets).
        CHECK(get16(events[0].payload, 0) == StMac::AssociateCnf);
        CHECK(events[0].payload.size() == 2 + 16);
        CHECK(get16(events[0].payload, 2) == 0xe534);
        CHECK(events[0].payload[4] == StMac::Success);
    }
    CHECK(m.mac.short_address() == 0xe534 && m.mac.pan_id() == 0xab8c);
}

void test_st_mac_parent_association_and_indirect() {
    Mac m;
    program_router(m, 0xab8c, 0x1234);
    m.set(0x41, {1}); // macAssociationPermit
    m.set(0x45, {0x00, 0x22, 0x84});
    m.set(0x46, {3});
    m.run(m.now + Ms);
    m.take();
    const std::vector<std::uint8_t> child = {1, 2, 3, 4, 5, 6, 7, 8};
    // Beacon Request -> beacon from the router's PIB.
    m.hear({0x03, 0x08, 0x50, 0xff, 0xff, 0xff, 0xff, 0x07});
    m.run(m.now + 10 * Ms);
    CHECK(m.host.sent.size() == 1);
    if (!m.host.sent.empty()) {
        const std::vector<std::uint8_t> &beacon = m.host.sent[0].mpdu;
        CHECK(get16(beacon, 0) == 0x8000);
        CHECK(get16(beacon, 3) == 0xab8c && get16(beacon, 5) == 0x1234);
        CHECK(get16(beacon, 7) == 0x8fff); // BO=SO=15, final CAP, permit
        CHECK(std::vector<std::uint8_t>(beacon.end() - 3, beacon.end()) ==
              (std::vector<std::uint8_t>{0x00, 0x22, 0x84}));
    }
    // Association Request from the child -> MLME-ASSOCIATE.indication.
    std::vector<std::uint8_t> request{0x23, 0xc8, 0x51, 0x8c, 0xab, 0x34, 0x12, 0xff, 0xff};
    request.insert(request.end(), child.begin(), child.end());
    request.push_back(0x01);
    request.push_back(0x80);
    m.hear(request);
    m.run(m.now + Ms);
    auto events = m.take();
    CHECK(events.size() == 1);
    if (events.size() == 1) {
        CHECK(get16(events[0].payload, 0) == StMac::AssociateInd);
        CHECK(events[0].payload.size() == 2 + 20);
        CHECK(std::equal(child.begin(), child.end(), events[0].payload.begin() + 2));
        CHECK(events[0].payload[2 + 8] == 0x80);
    }
    // MAC_associateRes_t queues an indirect response.
    std::uint8_t response[StMac::AssociateResSize] = {};
    std::copy(child.begin(), child.end(), response);
    put16(response + 8, 0x5678);
    CHECK(m.mac.request(StMac::AssociateRes, response, sizeof response, m.now) ==
          StMac::Success);
    m.host.sent.clear();
    // The child polls with its extended address: ACK with Frame Pending, then
    // the response, then COMM-STATUS once the child ACKs it.
    std::vector<std::uint8_t> poll{0x63, 0xc8, 0x52, 0x8c, 0xab, 0x34, 0x12};
    poll.insert(poll.end(), child.begin(), child.end());
    poll.push_back(0x04);
    m.hear(poll);
    CHECK(m.host.sent.size() == 1 && m.host.sent[0].mpdu[0] == 0x12);
    m.run_until_sent(2);
    CHECK(m.host.sent.size() == 2);
    if (m.host.sent.size() == 2) {
        const std::vector<std::uint8_t> frame = m.host.sent[1].mpdu;
        CHECK(frame[frame.size() - 4] == 0x02 && get16(frame, frame.size() - 3) == 0x5678 &&
              frame.back() == 0x00);
        m.ack_last();
    }
    m.run(m.now + Ms);
    events = m.take();
    CHECK(events.size() == 1);
    if (events.size() == 1) {
        // MAC_commStatusInd_t (32 octets): destination at 12, status at 20.
        CHECK(get16(events[0].payload, 0) == StMac::CommStatusInd);
        CHECK(events[0].payload.size() == 2 + 32);
        CHECK(events[0].payload[2 + 3] == 3);
        CHECK(std::equal(child.begin(), child.end(), events[0].payload.begin() + 2 + 12));
        CHECK(events[0].payload[2 + 20] == StMac::Success);
    }

    // Indirect MSDU for short 0x5678: a Data Request gets Frame Pending, the
    // frame follows, and DATA.confirm reports delivery; the poll is indicated.
    m.host.memory[0x20002000] = {0xaa, 0xbb};
    std::uint8_t data[StMac::DataReqSize] = {};
    put32(data, 0x20002000);
    data[4] = 2;
    data[5] = 2;
    put16(data + 6, 0xab8c);
    put16(data + 8, 0x5678);
    data[16] = 2;
    data[17] = 0x07;
    data[18] = 1;
    data[20] = 1;
    CHECK(m.mac.request(StMac::DataReq, data, sizeof data, m.now) == StMac::Success);
    m.host.sent.clear();
    m.hear({0x63, 0x88, 0x53, 0x8c, 0xab, 0x34, 0x12, 0x78, 0x56, 0x04});
    CHECK(m.host.sent.size() == 1 && m.host.sent[0].mpdu[0] == 0x12);
    m.run_until_sent(2);
    CHECK(m.host.sent.size() == 2);
    if (m.host.sent.size() == 2) {
        CHECK(get16(m.host.sent[1].mpdu, 5) == 0x5678);
        m.ack_last();
    }
    m.run(m.now + Ms);
    events = m.take();
    CHECK(events.size() == 2);
    if (events.size() == 2) {
        CHECK(get16(events[0].payload, 0) == StMac::PollInd);
        CHECK(events[0].payload.size() == 2 + 9);
        CHECK(events[0].payload[2] == 2 && get16(events[0].payload, 3) == 0x5678);
        CHECK(get16(events[1].payload, 0) == StMac::DataCnf);
        CHECK(events[1].payload[2] == 0x07 && events[1].payload[2 + 6] == StMac::Success);
    }
    // A poll without anything queued is ACKed without Frame Pending.
    m.host.sent.clear();
    m.hear({0x63, 0x88, 0x54, 0x8c, 0xab, 0x34, 0x12, 0x78, 0x56, 0x04});
    CHECK(m.host.sent.size() == 1 && m.host.sent[0].mpdu[0] == 0x02);
}

// IPCC + SRAM2a exchange of the chip with the MAC model selected.
void test_stm32wb55_mac_mailbox() {
    stm32wb55emu::Stm32wb55 chip;
    chip.set_cpu2_model(stm32wb55emu::Stm32wb55::Cpu2Model::StMac802154);
    chip.reset();
    cortexm::Core &core = chip.core();
    auto w32 = [&](std::uint32_t address, std::uint32_t value) {
        std::uint8_t bytes[4];
        put32(bytes, value);
        core.write_memory(address, bytes, 4);
    };
    auto r8 = [&](std::uint32_t address) {
        std::uint8_t byte = 0;
        core.read_memory(address, &byte, 1);
        return byte;
    };
    auto r32 = [&](std::uint32_t address) {
        std::uint8_t bytes[4] = {};
        core.read_memory(address, bytes, 4);
        return static_cast<std::uint32_t>(bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) |
                                          (static_cast<std::uint32_t>(bytes[3]) << 24));
    };
    // AN5289 reference table (TL_Init, shci_init, TL_MM_Init, TL_MAC_802_15_4_Init).
    constexpr std::uint32_t Ref = 0x20030000, Info = 0x20030100, Sys = 0x20030140,
                            MmTable = 0x20030160, MacTable = 0x20030180, SysCmd = 0x20030200,
                            SysQueue = 0x20030300, Spare = 0x20030400, MacCmd = 0x20030600,
                            MacNot = 0x20030800, Value = 0x20030a00;
    w32(Ref + 0, Info);
    w32(Ref + 12, Sys);
    w32(Ref + 16, MmTable);
    w32(Ref + 24, MacTable);
    w32(Sys, SysCmd);
    w32(Sys + 4, SysQueue);
    w32(SysQueue, SysQueue);
    w32(SysQueue + 4, SysQueue);
    w32(MmTable + 4, Spare);
    w32(MacTable, MacCmd);
    w32(MacTable + 4, MacNot);
    constexpr std::uint32_t Ipcc = 0x58000c00;
    chip.write(Ipcc + 0x00, (1u << 16) | 1u, 4); // C1CR: TXFIE, RXOIE
    chip.write(Ipcc + 0x04, 0, 4);               // C1MR: nothing masked
    chip.write(0x5800040c, 1u << 15, 4);          // PWR_CR4.C2BOOT

    std::uint64_t now = 0;
    auto run_to = [&](std::uint64_t ns) {
        now = ns;
        chip.run_cpu2_until(now);
    };
    run_to(10 * Ms);
    // SHCI ready event (sub-event 0x9200, WirelessFwRunning) in the spare buffer.
    CHECK(chip.read(Ipcc + 0x1c, 4) & (1u << 1));
    CHECK(r32(SysQueue) == Spare);
    CHECK(r8(Spare + 8) == 0x12 && r8(Spare + 11) == 0x00 && r8(Spare + 12) == 0x92 &&
          r8(Spare + 13) == 0x00);
    // Wireless firmware: MAC 802.15.4 1.24.0, INFO_STACK_TYPE_MAC.
    CHECK(r32(Info + 16) == 0x01180000u && r32(Info + 24) == 0x40u);

    // MLME-SET macExtendedAddress through the MAC command channel (3).
    for (unsigned i = 0; i != 8; ++i)
        core.write_memory(Value + i, &Extended[i], 1);
    const std::uint16_t set_opcode = (0x3f << 9) | (0x280 + StMac::SetReq);
    std::uint8_t command[12 + StMac::SetReqSize] = {};
    command[8] = 0x00;
    put16(command + 9, set_opcode);
    command[11] = StMac::SetReqSize;
    put32(command + 12, Value);
    command[16] = 0x6f;
    core.write_memory(MacCmd, command, sizeof command);
    chip.write(Ipcc + 0x08, 1u << (16 + 2), 4); // C1SCR: CH3S
    CHECK(chip.read(Ipcc + 0x0c, 4) & (1u << 2));
    run_to(now + 10000);
    // The command channel is released with the synchronous status in the buffer.
    CHECK((chip.read(Ipcc + 0x0c, 4) & (1u << 2)) == 0);
    CHECK(r8(MacCmd + 11) == StMac::Success);
    run_to(now + Ms);
    // MLME-SET.confirm in the notification buffer, signalled on channel 3.
    CHECK(chip.read(Ipcc + 0x1c, 4) & (1u << 2));
    CHECK(r8(MacNot + 10) == 2 + 4);
    CHECK(r8(MacNot + 11) == StMac::SetCnf && r8(MacNot + 12) == 0);
    CHECK(r8(MacNot + 13) == StMac::Success && r8(MacNot + 14) == 0x6f);

    // An unknown MAC command gets NOT_IMPLEMENTED synchronously.
    put16(command + 9, (0x3f << 9) | (0x280 + 0x1f));
    core.write_memory(MacCmd, command, sizeof command);
    chip.write(Ipcc + 0x08, 1u << 2, 4); // C1SCR: CH3C, the notification is consumed
    chip.write(Ipcc + 0x08, 1u << (16 + 2), 4);
    run_to(now + 10000);
    CHECK(r8(MacCmd + 11) == StMac::NotImplemented);
    CHECK((chip.read(Ipcc + 0x1c, 4) & (1u << 2)) == 0);
}

} // namespace

void run_st_mac_tests() {
    test_st_mac_set_get_reset_confirms();
    test_st_mac_data_request_ack_and_retries();
    test_st_mac_data_indication_and_imm_ack();
    test_st_mac_active_scan_and_associate();
    test_st_mac_parent_association_and_indirect();
    test_stm32wb55_mac_mailbox();
}

} // namespace mcuemu_tests

int main() {
    mcuemu_tests::run_st_mac_tests();
    if (mcuemu_tests::failures != 0) {
        std::cerr << mcuemu_tests::failures << " check(s) failed\n";
        return 1;
    }
    return 0;
}
