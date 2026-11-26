#pragma once

#include "cortexm/core.hpp"
#include "mcuemu/radio.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <memory>
#include <string>
#include <vector>

namespace efr32emu {

class Block;
class Cmu;
class Gpio;
class I2c;
class Radio;
class Rtcc;
class Ldma;
class SeMailbox;
class RailRegs;
class RsmRadio;

// What a pad does to its net, as seen from outside the chip.
enum class PadDrive { HiZ, Low, High, PullUp, PullDown };

// Silicon Labs EFR32MG21A020F512 (Series 2): Cortex-M33 (no TrustZone state
// is modelled), 1 MiB flash, 96 KiB RAM (the A020F1024 the Zephyr images for
// the slwrb4180a board are built for; the BRD4181A images fit in it too).
//
// Peripherals sit at 0x40000000 in 16 KiB blocks. Each block has the Series 2
// register aliases: +0x1000 sets, +0x2000 clears and +0x3000 toggles the bits
// written. Registers follow the EFR32xG21 reference manual where the firmware
// we run uses them; each source file says what is documented and what is
// inferred. The RADIO is the exception: see radio.cpp.
//
// The system clock is the 38.4 MHz HFXO from reset (the 20 MHz FSRCO the real
// chip starts on is not modelled).
class Efr32mg21 final : public cortexm::Bus {
public:
    static constexpr std::uint32_t FlashSize = 0x100000;
    static constexpr std::uint32_t RamSize = 0x18000;
    static constexpr std::uint32_t RamBase = 0x20000000;
    static constexpr std::uint32_t UserDataBase = 0x0FE00000;
    static constexpr std::uint32_t DevInfoBase = 0x0FE08000;
    static constexpr std::uint32_t PeripheralBase = 0x40000000;
    static constexpr std::uint32_t PeripheralSize = 0x100000;
    // The application slot: a Gecko Bootloader owns the first 16 KiB.
    static constexpr std::uint32_t AppBase = 0x4000;
    static constexpr std::uint32_t ClockHz = 38400000;
    static constexpr unsigned IrqCount = 51;
    // Ports A..D, 16 pins each; pad = 16 * port + pin.
    static constexpr unsigned PadCount = 64;

    Efr32mg21();
    ~Efr32mg21() override;
    Efr32mg21(const Efr32mg21 &) = delete;
    Efr32mg21 &operator=(const Efr32mg21 &) = delete;

    // Places a flash image at `offset`. An image whose reset vector lies in the
    // application slot (>= AppBase) is an application-only image and is
    // loaded at AppBase: the model has no boot loader, it starts the
    // application directly.
    void load_firmware(const std::vector<std::uint8_t> &image, std::uint32_t offset = 0);
    // (Re)starts the CPU; RAM and flash are kept.
    void reset();

    cortexm::Core &core() noexcept { return *core_; }
    const cortexm::Core &core() const noexcept { return *core_; }
    std::uint64_t time_ns() const { return core_->cycles_to_ns(core_->cycles()); }

    // Logs every peripheral access (address, value, pc) to `out`.
    void set_trace(std::ostream *out) noexcept { trace_ = out; }
    std::ostream *trace() noexcept { return trace_; }

    // ---- pads ("PA0".."PD15") ----------------------------------------------
    using PadObserver = std::function<void(unsigned pad)>;
    void set_pad_observer(PadObserver observer) { pad_observer_ = std::move(observer); }
    PadDrive pad_drive(unsigned pad) const;
    // The level an external circuit holds on the pad (true = high).
    void set_pad_input(unsigned pad, bool high);
    static bool parse_pad(const std::string &name, unsigned &pad);
    static std::string pad_name(unsigned pad);

    // ---- memory ----------------------------------------------------------
    std::uint8_t *flash() noexcept { return flash_; }
    std::uint8_t *ram() noexcept { return ram_; }

    // cortexm::Bus
    std::uint32_t read(std::uint32_t address, unsigned size) override;
    void write(std::uint32_t address, std::uint32_t value, unsigned size) override;

    // Runs until `until_cycle`, restarting after resets.
    cortexm::RunResult run(std::uint64_t until_cycle);
    // True while reset() runs for a reset the firmware requested (SYSRESETREQ) rather than a power-up.
    bool soft_reset() const noexcept { return soft_reset_; }

    // A byte from the host on USART0's receive line. True when a DMA channel took it; false when
    // nothing is listening yet (the caller retries).
    bool uart0_rx(std::uint8_t byte);

    // Connects the RADIO to a shared medium under the node name `node`.
    void connect_radio(mcuemu::VirtualAir &air, const std::string &node);

    // The EUI-64 in the device information page derives from `id`.
    void set_device_id(std::uint64_t id);

    // ---- used by the peripherals ----------------------------------------
    Cmu &cmu() noexcept { return *cmu_; }
    Gpio &gpio() noexcept { return *gpio_; }
    // Bytes the firmware sent through USART0 (the console), in order.
    std::string &uart0_output() noexcept { return uart0_output_; }
    void notify_pad(unsigned pad);
    // Flash words programmed through the MSC; flushes translated code only
    // when the program image itself is touched.
    void flash_changed(std::uint32_t address, std::uint32_t length);
    void request_service(std::uint64_t at_cycle) { core_->request_service(at_cycle); }
    void request_reset() { core_->request_reset(); }

private:
    std::uint64_t service(std::uint64_t now);
    Block &block_at(std::uint32_t slot);
    Block &lp_block_at(std::uint32_t slot);
    // RADIOAES: runs the descriptor chains the firmware pointed FETCHADDR/PUSHADDR at (radioaes.cpp).
    void run_radioaes();

    std::unique_ptr<cortexm::Core> core_;
    std::uint8_t *flash_{nullptr};
    std::uint8_t *ram_{nullptr};
    std::uint8_t *devinfo_{nullptr};
    bool soft_reset_{false};
    std::uint64_t uart_rx_ready_at_{0};
    std::uint8_t *userdata_{nullptr};
    std::ostream *trace_{nullptr};
    std::uint32_t code_end_{0};
    std::uint32_t boot_vector_{0};
    PadObserver pad_observer_;
    std::array<std::unique_ptr<Block>, 64> blocks_;
    // The low-energy peripherals at 0x4A000000 (LETIMER0, IADC0, ACMP0/1, I2C0, HFRCOEM23, WDOG0/1), 16 KiB apart.
    std::array<std::unique_ptr<Block>, 8> lp_blocks_;
    Cmu *cmu_{nullptr};
    Gpio *gpio_{nullptr};
    I2c *i2c_{nullptr};
    std::unique_ptr<Radio> radio_;
    std::string uart0_output_;
    std::unique_ptr<Rtcc> rtcc_;
    Ldma *ldma_{nullptr};
    std::unique_ptr<SeMailbox> se_mailbox_;
    std::unique_ptr<RailRegs> rail_regs_;
    // The radio driven without RAIL (zigbee-rs efr32s2-hal); takes over from the RAIL path once active.
    std::unique_ptr<RsmRadio> rsm_;
    // Core cycle of the end of the frame in flight, and of the ACK timeout after it.
    static constexpr std::uint64_t NoEvent = ~std::uint64_t{0};
    std::uint64_t tx_done_at_{NoEvent}, ack_timeout_at_{NoEvent};
    std::vector<std::uint8_t> tx_frame();
    void start_tx();
    std::uint64_t lbt_tx_at_{~std::uint64_t{0}};
    std::uint64_t pending_ack_at_{~std::uint64_t{0}};
    std::uint8_t pending_ack_seq_{0};
    std::uint64_t pending_rx_at_{~std::uint64_t{0}};
    std::uint64_t data_request_deadline_{0};
    std::vector<std::uint8_t> pending_rx_;
    std::uint8_t rx_filter_mask_{0}, pending_filter_mask_{0};
    void tx_store(std::uint8_t byte);
    std::uint8_t rail_channel() const;
    void deliver_rail_rx(const std::vector<std::uint8_t> &fifo);
    // The radio acknowledges a received frame that requests it and is addressed to this
    // device: by its unique id (extended address) or, after an association response it
    // acknowledged, by the short address and PAN that response assigned.
    bool acknowledge(const std::vector<std::uint8_t> &fifo);
    // Short address and PAN ID taken from the association response (little-endian as sent).
    bool short_address_known_{false};
    std::uint8_t short_address_[2]{}, pan_id_[2]{};
    std::uint64_t rail_rx_poll_at_{0};
    // Core cycle at which a requested receiver finishes warming up (RXWARM -> RXSEARCH).
    std::uint64_t rx_warm_done_at_{~std::uint64_t{0}};
    bool rx_was_enabled_{false};
    std::uint32_t rx_context();
    std::uint32_t rx_context_{0};
    // PROTIMER compare check: next due cycle and the WRAPCNT seen at the previous check, per channel.
    std::uint64_t next_protimer_{0};
    std::uint64_t protimer_last_wrap_[8]{};
};

} // namespace efr32emu
