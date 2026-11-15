#pragma once

// Behavioural model of the IEEE 802.15.4 MAC that ST ships for the STM32WB
// CPU2 (stm32wb5x_Mac_802_15_4_fw.bin, validated against MAC 1.24.0 users).
//
// The real stack is an encrypted FUS image; this is not its code. It answers
// the MAC SAP exactly as CPU1 sees it through the IPCC mailbox:
// - request and notification identifiers: MAC_802_15_4_MsgIdM4ToM0_t and
//   MAC_802_15_4_MsgIdM0ToM4_t in ST's 802_15_4_mac_core.h;
// - payloads: the C structures of 802_15_4_mac_types.h (stm32-mw-wpan
//   1a66948, the STM32_WPAN middleware of STM32CubeWB), whose sizes and field
//   offsets zigbee-rs asserts at compile time in stm32wb-hal/src/st_abi.rs;
// - status codes: 802_15_4_mac_errors.h (IEEE 802.15.4-2011 values).
// Behaviour on the air follows IEEE 802.15.4-2006 for a non-beacon PAN
// (unslotted CSMA-CA, Imm-Ack, indirect transmission, association, active
// and passive scan). Times are in CPU2 nanoseconds.

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace stm32wb55emu {

// Access to the chip that the MAC needs: CPU1-owned memory (MSDUs and PIB
// values the requests point to) and the shared radio medium.
class StMacHost {
public:
    virtual ~StMacHost() = default;
    virtual bool read(std::uint32_t address, void *out, std::size_t size) = 0;
    // Put an MPDU (no FCS) on the air; returns the time its last symbol ends.
    virtual std::uint64_t transmit(std::uint8_t channel, std::uint64_t start_ns,
                                   const std::vector<std::uint8_t> &mpdu) = 0;
    // Clear channel assessment: another node is on the air at `at_ns`.
    virtual bool channel_busy(std::uint8_t channel, std::uint64_t at_ns) = 0;
};

// A frame heard on the medium (MPDU without FCS).
struct StMacRxFrame {
    std::vector<std::uint8_t> mpdu;
    std::uint8_t channel{11};
    std::uint64_t start_ns{0};
    std::uint64_t end_ns{0};
    int rssi_dbm{-50};
};

// One MAC_802_15_4 notification for the CPU1 mailbox: `payload` is the TL
// event payload (16-bit message id + the C structure); `data` is what the
// structure's msduPtr/sduPtr/valuePtr designates. The model has already set
// that pointer to its data area (StMac::data_address()).
struct StMacNotification {
    std::vector<std::uint8_t> payload;
    std::vector<std::uint8_t> data;
};

class StMac {
public:
    // MAC_802_15_4_MsgIdM4ToM0_t.
    enum Request : std::uint8_t {
        AssociateReq = 0x00,
        AssociateRes = 0x01,
        DisassociateReq = 0x02,
        GetReq = 0x03,
        GtsReq = 0x04,
        OrphanRes = 0x05,
        ResetReq = 0x06,
        RxEnableReq = 0x07,
        ScanReq = 0x08,
        SetReq = 0x09,
        StartReq = 0x0a,
        SyncReq = 0x0b,
        PollReq = 0x0c,
        DpsReq = 0x0d,
        SoundingReq = 0x0e,
        CalibrateReq = 0x0f,
        DataReq = 0x10,
        PurgeReq = 0x11,
    };
    // MAC_802_15_4_MsgIdM0ToM4_t.
    enum Notification : std::uint16_t {
        AssociateCnf = 0,
        DisassociateCnf = 1,
        GetCnf = 2,
        GtsCnf = 3,
        ResetCnf = 4,
        RxEnableCnf = 5,
        ScanCnf = 6,
        SetCnf = 7,
        StartCnf = 8,
        PollCnf = 9,
        DpsCnf = 10,
        SoundingCnf = 11,
        CalibrateCnf = 12,
        DataCnf = 13,
        PurgeCnf = 14,
        AssociateInd = 15,
        DisassociateInd = 16,
        BeaconNotifyInd = 17,
        CommStatusInd = 18,
        GtsInd = 19,
        OrphanInd = 20,
        SyncLossInd = 21,
        DpsInd = 22,
        DataInd = 23,
        PollInd = 24,
    };
    // 802_15_4_mac_errors.h; NotImplemented is MAC_NOT_IMPLEMENTED_STATUS of
    // 802_15_4_mac_types.h.
    enum Status : std::uint8_t {
        Success = 0x00,
        NotImplemented = 0x02,
        UnsupportedSecurity = 0xdf,
        FrameTooLong = 0xe5,
        ChannelAccessFailure = 0xe1,
        Denied = 0xe2,
        InvalidHandle = 0xe7,
        InvalidParameter = 0xe8,
        NoAck = 0xe9,
        NoBeacon = 0xea,
        NoData = 0xeb,
        NoShortAddress = 0xec,
        TransactionExpired = 0xf0,
        TransactionOverflow = 0xf1,
        UnsupportedAttribute = 0xf4,
        InvalidAddress = 0xf5,
        ReadOnly = 0xfb,
        ScanInProgress = 0xfc,
    };
    // C structure sizes from 802_15_4_mac_types.h (checked by st_abi.rs).
    static constexpr std::size_t AssociateReqSize = 25;
    static constexpr std::size_t AssociateResSize = 24;
    static constexpr std::size_t DisassociateReqSize = 24;
    static constexpr std::size_t GetReqSize = 4;
    static constexpr std::size_t OrphanResSize = 24;
    static constexpr std::size_t ResetReqSize = 4;
    static constexpr std::size_t ScanReqSize = 20;
    static constexpr std::size_t SetReqSize = 8;
    static constexpr std::size_t StartReqSize = 35;
    static constexpr std::size_t PollReqSize = 24;
    static constexpr std::size_t DataReqSize = 36;
    static constexpr std::size_t PurgeReqSize = 4;
    static constexpr std::size_t ScanCnfSize = 185;
    static constexpr std::size_t DataIndSize = 68;
    static constexpr std::size_t BeaconNotifyIndSize = 88;
    static constexpr std::size_t PanDescriptorSize = 24;
    static constexpr std::size_t MaxPanDescriptors = 6;
    static constexpr std::size_t MaxBeaconPayload = 52; // aMaxBeaconPayloadLength
    static constexpr std::size_t MaxIndirect = 8;

    struct Stats {
        std::uint32_t requests{0};
        std::uint32_t frames_sent{0};
        std::uint32_t frames_received{0};
        std::uint32_t acks_sent{0};
        std::uint32_t no_acks{0};
        std::uint32_t channel_access_failures{0};
    };

    StMac(StMacHost &host, std::uint64_t seed);

    // Where data referenced by a notification is placed (CPU2-owned SRAM2).
    void set_data_address(std::uint32_t address) noexcept { data_address_ = address; }
    std::uint32_t data_address() const noexcept { return data_address_; }
    // Diagnostic lines (one per primitive) for an optional trace sink.
    void set_trace(bool enabled) noexcept { trace_ = enabled; }
    std::vector<std::string> take_trace();

    // One MAC command from the mailbox (`id` from the opcode OCF, `payload`
    // the request structure). Returns the synchronous command status that
    // CPU2 writes back into the command buffer.
    std::uint8_t request(std::uint8_t id, const std::uint8_t *payload, std::size_t size,
                         std::uint64_t now_ns);
    // Frames of the shared medium on the current channel (when rx_enabled()).
    void receive(const StMacRxFrame &frame, std::uint64_t now_ns);
    // Run timers due at `now_ns`; returns the next time the model needs
    // service, or UINT64_MAX.
    std::uint64_t service(std::uint64_t now_ns);

    bool rx_enabled() const noexcept;
    std::uint8_t channel() const noexcept { return channel_; }
    bool has_notification() const noexcept { return !notifications_.empty(); }
    std::optional<StMacNotification> pop_notification();
    const Stats &stats() const noexcept { return stats_; }
    std::uint16_t short_address() const noexcept { return short_address_; }
    std::uint16_t pan_id() const noexcept { return pan_id_; }
    bool started() const noexcept { return started_; }

    // Exposed for unit tests: the MAC header/addressing of a frame.
    struct Address {
        std::uint8_t mode{0}; // 0 none, 2 short, 3 extended
        std::uint16_t pan{0xffff};
        std::uint16_t short_address{0xffff};
        std::array<std::uint8_t, 8> extended{};
    };
    struct Header {
        std::uint16_t fcf{0};
        std::uint8_t type{0};
        bool frame_pending{false};
        bool ack_request{false};
        bool pan_compression{false};
        std::uint8_t sequence{0};
        Address destination;
        Address source;
        std::size_t payload_offset{0};
    };
    static bool parse_header(const std::vector<std::uint8_t> &mpdu, Header &header);
    static std::vector<std::uint8_t> build_frame(std::uint8_t type, bool frame_pending,
                                                 bool ack_request, std::uint8_t sequence,
                                                 const Address &destination,
                                                 const Address &source,
                                                 const std::vector<std::uint8_t> &payload);

private:
    enum class Purpose {
        Data,
        AssociationRequest,
        AssociationDataRequest,
        PollDataRequest,
        BeaconRequest,
        Beacon,
        IndirectFrame,
        OrphanRealignment,
        Disassociation,
    };
    struct TxJob {
        std::vector<std::uint8_t> mpdu;
        Purpose purpose{Purpose::Data};
        std::uint8_t handle{0};
        bool ack_request{false};
        bool csma{true};
        unsigned nb{0};
        unsigned be{3};
        unsigned retries{0};
        std::uint64_t due_ns{0};
        std::uint64_t end_ns{0};
        bool on_air{false};
        bool waiting_ack{false};
        bool ack_pending_bit{false};
        // Indirect transaction this frame delivers (Purpose::IndirectFrame).
        std::uint32_t transaction{0};
    };
    enum class TransactionKind { Data, AssociationResponse, Disassociation };
    struct Transaction {
        std::uint32_t id{0};
        TransactionKind kind{TransactionKind::Data};
        Address destination;
        std::uint8_t handle{0};
        std::vector<std::uint8_t> msdu;
        std::uint8_t src_mode{2};
        bool ack_request{true};
        std::uint16_t assoc_short{0xffff};
        std::uint8_t assoc_status{0};
        std::uint8_t reason{0};
        std::uint64_t expires_ns{0};
        bool in_flight{false};
    };
    enum class Op { None, Scan, Associate, Poll };
    struct ScanState {
        std::uint8_t type{0};
        std::uint32_t channels{0};
        std::uint8_t duration{0};
        int current{-1};
        bool listening{false};
        std::uint64_t listen_until_ns{0};
        std::uint8_t saved_channel{11};
        std::uint16_t saved_pan{0xffff};
        std::vector<std::array<std::uint8_t, PanDescriptorSize>> descriptors;
        std::vector<std::uint8_t> energy;
        bool any_beacon{false};
    };
    struct AssociateState {
        Address coordinator;
        std::uint8_t capability{0};
        bool awaiting_response{false};
        std::uint64_t data_request_at_ns{0};
        std::uint64_t response_deadline_ns{0};
    };
    struct PollState {
        Address coordinator;
        bool awaiting_data{false};
        std::uint64_t deadline_ns{0};
    };
    struct Pending {
        std::uint64_t due_ns{0};
        StMacNotification notification;
    };

    std::uint8_t handle_set(const std::uint8_t *p, std::uint64_t now_ns);
    std::uint8_t handle_get(const std::uint8_t *p, std::uint64_t now_ns);
    std::uint8_t handle_reset(const std::uint8_t *p, std::uint64_t now_ns);
    std::uint8_t handle_scan(const std::uint8_t *p, std::uint64_t now_ns);
    std::uint8_t handle_start(const std::uint8_t *p, std::uint64_t now_ns);
    std::uint8_t handle_associate(const std::uint8_t *p, std::uint64_t now_ns);
    std::uint8_t handle_associate_response(const std::uint8_t *p, std::uint64_t now_ns);
    std::uint8_t handle_disassociate(const std::uint8_t *p, std::uint64_t now_ns);
    std::uint8_t handle_orphan_response(const std::uint8_t *p, std::uint64_t now_ns);
    std::uint8_t handle_poll(const std::uint8_t *p, std::uint64_t now_ns);
    std::uint8_t handle_data(const std::uint8_t *p, std::uint64_t now_ns);
    std::uint8_t handle_purge(const std::uint8_t *p, std::uint64_t now_ns);

    void reset_pib();
    void notify(std::uint16_t id, std::vector<std::uint8_t> body,
                std::vector<std::uint8_t> data = {}, std::uint64_t due_ns = 0);
    void confirm_simple(std::uint16_t id, std::uint8_t status, std::uint64_t due_ns,
                        std::uint8_t second = 0);
    void queue_tx(TxJob job, std::uint64_t now_ns);
    void start_csma(TxJob &job, std::uint64_t now_ns);
    void run_tx(std::uint64_t now_ns);
    void finish_tx(TxJob job, std::uint8_t status, std::uint64_t now_ns);
    void send_ack(const StMacRxFrame &frame, const Header &header, bool pending);
    bool accept(const Header &header) const;
    void handle_command(const StMacRxFrame &frame, const Header &header, std::uint64_t now_ns);
    void deliver_data(const StMacRxFrame &frame, const Header &header, std::uint64_t now_ns);
    void beacon_received(const StMacRxFrame &frame, const Header &header, std::uint64_t now_ns);
    void scan_next_channel(std::uint64_t now_ns);
    void finish_scan(std::uint64_t now_ns);
    void finish_associate(std::uint8_t status, std::uint16_t short_address, std::uint64_t now_ns);
    void finish_poll(std::uint8_t status, std::uint64_t now_ns);
    void expire_transactions(std::uint64_t now_ns);
    bool has_transaction_for(const Address &address) const;
    void serve_transaction(const Address &requester, std::uint64_t now_ns);
    void comm_status(const Address &destination, std::uint8_t status, std::uint64_t now_ns);
    Address own_address(bool prefer_short) const;
    std::uint8_t next_dsn() { return dsn_++; }
    std::uint32_t random();
    std::uint8_t lqi(int rssi_dbm) const;
    std::uint32_t symbols(std::uint64_t ns) const {
        return static_cast<std::uint32_t>(ns / 16000u);
    }
    void trace(const std::string &line);

    StMacHost &host_;
    std::uint64_t rng_;
    std::uint32_t data_address_{0};
    bool trace_{false};
    std::vector<std::string> trace_lines_;
    Stats stats_;

    // PIB (IEEE 802.15.4-2006 tables 71 and 86; ST's macExtendedAddress 0x6f).
    std::uint8_t channel_{11};
    std::uint8_t page_{0};
    std::int8_t tx_power_{0};
    std::uint8_t cca_mode_{1};
    bool association_permit_{false};
    bool auto_request_{true};
    bool battery_life_ext_{false};
    std::array<std::uint8_t, MaxBeaconPayload> beacon_payload_{};
    std::uint8_t beacon_payload_length_{0};
    std::uint8_t beacon_order_{15};
    std::uint8_t superframe_order_{15};
    std::uint8_t bsn_{0};
    std::uint8_t dsn_{0};
    std::array<std::uint8_t, 8> coord_extended_{};
    std::uint16_t coord_short_{0xffff};
    bool gts_permit_{true};
    std::uint8_t max_csma_backoffs_{4};
    std::uint8_t min_be_{3};
    std::uint8_t max_be_{5};
    std::uint16_t pan_id_{0xffff};
    bool promiscuous_{false};
    bool rx_on_when_idle_{false};
    std::uint16_t short_address_{0xffff};
    std::uint16_t transaction_persistence_time_{0x01f4};
    bool associated_pan_coord_{false};
    std::uint16_t max_frame_total_wait_time_{1986};
    std::uint8_t max_frame_retries_{3};
    std::uint8_t response_wait_time_{32};
    std::array<std::uint8_t, 8> extended_{};

    bool started_{false};
    bool pan_coordinator_{false};

    Op op_{Op::None};
    ScanState scan_;
    AssociateState associate_;
    PollState poll_;
    std::deque<TxJob> tx_queue_;
    std::vector<Transaction> transactions_;
    std::uint32_t next_transaction_{1};
    std::vector<Pending> pending_;
    std::deque<StMacNotification> notifications_;
    std::uint64_t now_ns_{0};
};

} // namespace stm32wb55emu
