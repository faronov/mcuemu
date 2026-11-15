#include "stm32wb55emu/stm32wb55.hpp"

#include "stm32wb55emu/st_mac.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace stm32wb55emu {
namespace {

constexpr std::uint32_t SystemMemoryBase = 0x1fff0000;
constexpr std::uint32_t SystemMemorySize = 0x10000;
constexpr std::uint32_t Cpu2ModelVector = SystemMemoryBase + 0xf000;
constexpr std::uint32_t Cpu2ModelCode = SystemMemoryBase + 0xf100;
constexpr std::uint32_t RccBase = 0x58000000;
constexpr std::uint32_t PwrBase = 0x58000400;
constexpr std::uint32_t IpccBase = 0x58000c00;
constexpr std::uint32_t RngBase = 0x58001000;
constexpr std::uint32_t HsemBase = 0x58001400;
constexpr std::uint32_t FlashControlBase = 0x58004000;
constexpr std::uint32_t IpccRxIrq = 44;
constexpr std::uint32_t IpccTxIrq = 45;
constexpr std::uint32_t LptimIrq = 47;
constexpr std::uint32_t RngIrq = 53;
constexpr std::uint32_t Cpu2IpccIrq = 19;
constexpr std::uint32_t Cpu2RngIrq = 20;
constexpr std::uint32_t Cpu2LptimIrq = 13;
constexpr std::uint32_t Cpu2SyncCycles = 64; // one microsecond at the CPU1 clock.
constexpr std::uint32_t Cpu2RadioPollCycles = 640; // 20 us at the CPU2 clock.
constexpr std::uint32_t LptimBase = 0x40007c00;
constexpr std::uint32_t LptimClockHz = 32768;
constexpr std::uint32_t Sram2aBase = 0x20030000;
constexpr std::uint32_t RngEnable = 1u << 2;
constexpr std::uint32_t RngInterruptEnable = 1u << 3;
constexpr std::uint64_t RngReadyTimeNs = 1000; // modeled 64 cycles at the 64 MHz CPU1 clock.
constexpr std::uint64_t Never = std::numeric_limits<std::uint64_t>::max();
constexpr std::uint32_t RadioPollCycles = 1280; // 20 us at 64 MHz.
constexpr std::uint64_t RadioAckTurnaroundNs = 192000; // 12 symbols at 2.4 GHz.
constexpr std::uint32_t Tim2Base = 0x40000000;
constexpr std::uint32_t Tim2Irq = 28;
constexpr std::uint32_t Dma1Base = 0x40020000;
constexpr std::uint32_t Dma1Channel1Irq = 11; // DMA1_Channel1..7 are IRQs 11..17.
constexpr std::uint32_t Usart1Tdr = 0x40013828;
// RM0434 37.2: UID64 (UDN, then device id and ST company id) used as EUI-64.
constexpr std::uint32_t Uid64Address = 0x1fff7580;
// AN5289 4.3: CPU2 reports its ready event once its firmware is up; CPU1
// must have finished TL_Init/shci_init/TL_MM_Init by then. The delay is a
// model choice (no datasheet figure), long enough for those few stores.
constexpr std::uint64_t StMacBootNs = 5000000;
constexpr std::uint64_t StMacBootRetryNs = 1000000;
// SRAM2b top: CPU2-owned memory where the MAC model places the MSDU,
// beacon payload or PIB value a notification points to.
constexpr std::uint32_t StMacDataAddress = 0x2003fc00;
constexpr unsigned MacTableIndex = 6;      // TL_RefTable.p_mac_802_15_4_table
constexpr unsigned MacOpcodeOffset = 0x280; // MAC_802_15_4_CMD_OPCODE_OFFSET

std::uint16_t get16(const std::uint8_t *p) {
    return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
}

std::uint32_t get32(const std::uint8_t *p) {
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
}

void put16(std::uint8_t *p, std::uint16_t value) {
    p[0] = static_cast<std::uint8_t>(value);
    p[1] = static_cast<std::uint8_t>(value >> 8);
}

void put32(std::uint8_t *p, std::uint32_t value) {
    p[0] = static_cast<std::uint8_t>(value);
    p[1] = static_cast<std::uint8_t>(value >> 8);
    p[2] = static_cast<std::uint8_t>(value >> 16);
    p[3] = static_cast<std::uint8_t>(value >> 24);
}

std::uint32_t lane_read(std::uint32_t value, std::uint32_t address, unsigned size) {
    if (size == 4)
        return value;
    const unsigned shift = (address & 3u) * 8;
    return (value >> shift) & (size == 1 ? 0xffu : 0xffffu);
}

std::uint32_t lane_write(std::uint32_t old_value, std::uint32_t value,
                         std::uint32_t address, unsigned size) {
    if (size == 4)
        return value;
    const unsigned shift = (address & 3u) * 8;
    const std::uint32_t mask = (size == 1 ? 0xffu : 0xffffu) << shift;
    return (old_value & ~mask) | ((value << shift) & mask);
}

std::uint16_t hdlc_fcs(const std::uint8_t *data, std::size_t size) {
    std::uint16_t crc = 0xffff;
    for (std::size_t i = 0; i != size; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit != 8; ++bit)
            crc = static_cast<std::uint16_t>((crc & 1u) ? (crc >> 1) ^ 0x8408u : crc >> 1);
    }
    return static_cast<std::uint16_t>(crc ^ 0xffffu);
}

} // namespace

class Stm32wb55::MacHost final : public StMacHost {
public:
    explicit MacHost(Stm32wb55 &chip) : chip_(chip) {}
    bool read(std::uint32_t address, void *out, std::size_t size) override {
        if (address >= FlashBase && address <= FlashBase + FlashSize &&
            size <= FlashBase + FlashSize - address) {
            std::memcpy(out, chip_.flash_ + (address - FlashBase), size);
            return true;
        }
        return chip_.memory_read(address, out, size);
    }
    std::uint64_t transmit(std::uint8_t channel, std::uint64_t start_ns,
                           const std::vector<std::uint8_t> &mpdu) override {
        if (!chip_.air_ || chip_.radio_node_.empty())
            return 0;
        chip_.air_->transmit(chip_.radio_node_, {}, mcuemu::RadioProtocol::Zigbee802154,
                             channel, start_ns, mpdu);
        return chip_.air_->frames().back().end_ns;
    }
    bool channel_busy(std::uint8_t channel, std::uint64_t at_ns) override {
        if (!chip_.air_ || chip_.radio_node_.empty())
            return false;
        return !chip_.air_
                    ->in_flight(chip_.radio_node_, mcuemu::RadioProtocol::Zigbee802154, channel,
                                at_ns, 0)
                    .empty();
    }

private:
    Stm32wb55 &chip_;
};

class Stm32wb55::Cpu2Bus final : public cortexm::Bus {
public:
    explicit Cpu2Bus(Stm32wb55 &chip) : chip_(chip) {}
    std::uint32_t read(std::uint32_t address, unsigned size) override {
        return chip_.read_cpu2_mmio(address, size);
    }
    void write(std::uint32_t address, std::uint32_t value, unsigned size) override {
        chip_.write_cpu2_mmio(address, value, size);
    }

private:
    Stm32wb55 &chip_;
};

Stm32wb55::Stm32wb55() {
    config_.model = cortexm::Model::M4;
    config_.clock_hz = ClockHz;
    config_.irq_count = 64;
    config_.priority_bits = 4;
    core_ = std::make_unique<cortexm::Core>(config_);
    flash_ = core_->map_memory(FlashBase, FlashSize, false);
    ram_ = core_->map_memory(RamBase, RamSize, true);
    system_memory_ = core_->map_memory(SystemMemoryBase, SystemMemorySize, false);
    cortexm::Core::Config cpu2_config;
    cpu2_config.model = cortexm::Model::M0;
    cpu2_config.clock_hz = Cpu2ClockHz;
    cpu2_config.irq_count = 32;
    cpu2_config.priority_bits = 2;
    cpu2_core_ = std::make_unique<cortexm::Core>(cpu2_config);
    cpu2_core_->map_memory_alias(FlashBase, FlashSize, false, flash_);
    cpu2_core_->map_memory_alias(RamBase, RamSize, true, ram_);
    cpu2_core_->map_memory_alias(SystemMemoryBase, SystemMemorySize, false, system_memory_);
    cpu2_bus_ = std::make_unique<Cpu2Bus>(*this);
    mac_host_ = std::make_unique<MacHost>(*this);
    std::memset(flash_, 0xff, FlashSize);
    std::memset(ram_, 0, RamSize);
    std::memset(system_memory_, 0xff, SystemMemorySize);

    // STM32WB55's UID, flash-size field and IPCC database pointer live in the
    // system-memory/flash-controller windows used by the STM32 HAL.
    put32(system_memory_ + (0x1fff7590u - SystemMemoryBase), 0x13579bdfu);
    put32(system_memory_ + (0x1fff7594u - SystemMemoryBase), 0x2468ace0u);
    put32(system_memory_ + (0x1fff7598u - SystemMemoryBase), 0x00cafe55u);
    put32(system_memory_ + (Cpu2ModelVector - SystemMemoryBase), RamBase + RamSize);
    put32(system_memory_ + (Cpu2ModelVector - SystemMemoryBase) + 4,
          Cpu2ModelCode | 1u);
    for (unsigned exception = 2; exception != 48; ++exception)
        put32(system_memory_ + (Cpu2ModelVector - SystemMemoryBase) + exception * 4,
              (Cpu2ModelCode + 2) | 1u);
    // The built-in Thread RCP model services IPCC in the CPU2 domain. Its M0
    // execution loop waits for those modeled firmware events.
    system_memory_[Cpu2ModelCode - SystemMemoryBase] = 0x30; // WFI
    system_memory_[Cpu2ModelCode - SystemMemoryBase + 1] = 0xbf;
    system_memory_[Cpu2ModelCode - SystemMemoryBase + 2] = 0xfe; // B .
    system_memory_[Cpu2ModelCode - SystemMemoryBase + 3] = 0xe7;
    registers_[FlashControlBase + 0x3c] = 0; // SRAM2A starts at 0x20030000.
    registers_[FlashControlBase + 0x14] = 0xc0000000u; // FLASH_CR: LOCK, OPTLOCK
    registers_[RccBase] = (1u << 1) | (1u << 10) | (1u << 17) | (1u << 25);
    registers_[RccBase + 0x98] = 1u << 1; // LSE ready.
    ipcc_c1mr_ = 0xffffffffu;
    ipcc_c2mr_ = 0xffffffffu;

    // STM32 peripheral buses are sparse; this callback provides the ordinary
    // read/write register behavior, with clock-ready and IPCC side effects
    // handled below.
    core_->map_mmio(0x40000000, 0x10000000, *this);
    core_->map_mmio(0x58000000, 0x01000000, *this);
    core_->set_service_handler([this](std::uint64_t now) { return service(now); });
    cpu2_core_->map_mmio(0x40000000, 0x10000000, *cpu2_bus_);
    cpu2_core_->map_mmio(0x58000000, 0x01000000, *cpu2_bus_);
    cpu2_core_->set_service_handler([this](std::uint64_t now) { return service_cpu2(now); });
    const auto flash_write = [this](std::uint32_t address, unsigned size,
                                    std::uint32_t value) {
        if (address < FlashBase || address + size > FlashBase + FlashSize)
            return false;
        for (unsigned i = 0; i < size; ++i)
            flash_[address - FlashBase + i] &= static_cast<std::uint8_t>(value >> (i * 8));
        const std::uint32_t end = address + size;
        if (address < code_end_ && end > FlashBase) {
            core_->request_code_invalidation();
            cpu2_core_->request_code_invalidation();
        }
        return true;
    };
    core_->set_write_protect_handler(flash_write);
    cpu2_core_->set_write_protect_handler(flash_write);
    set_device_id(device_id_);
}

Stm32wb55::~Stm32wb55() = default;

std::uint64_t Stm32wb55::time_ns() const {
    return std::max(core_->cycles_to_ns(core_->cycles()), cpu2_time_ns());
}

std::uint64_t Stm32wb55::core_time_ns(const cortexm::Core &core) const {
    if (&core == cpu2_core_.get())
        return cpu2_time_ns();
    return core.cycles_to_ns(core.cycles());
}

void Stm32wb55::request_service_at_ns(cortexm::Core &core, std::uint64_t requested_ns) {
    std::uint64_t target_cycle;
    if (&core == cpu2_core_.get()) {
        if (!cpu2_started_)
            return;
        const std::uint64_t elapsed_ns = requested_ns > cpu2_boot_time_ns_
                                             ? requested_ns - cpu2_boot_time_ns_
                                             : 0;
        target_cycle = cpu2_boot_cycle_ + core.ns_to_cycles(elapsed_ns);
    } else {
        target_cycle = core.ns_to_cycles(requested_ns);
    }
    core.request_service(std::max(core.cycles() + 1, target_cycle));
}

std::uint64_t Stm32wb55::cpu2_time_ns() const {
    if (!cpu2_started_)
        return 0;
    return cpu2_boot_time_ns_ +
           cpu2_core_->cycles_to_ns(cpu2_core_->cycles() - cpu2_boot_cycle_);
}

cortexm::RunResult Stm32wb55::run_cpu2_until(std::uint64_t target_ns) {
    if (cpu2_stopped_)
        return cpu2_stop_result_;
    if (!cpu2_started_ || target_ns <= cpu2_boot_time_ns_)
        return {};
    const std::uint64_t target = cpu2_boot_cycle_ +
                                 cpu2_core_->ns_to_cycles(target_ns - cpu2_boot_time_ns_);
    while (cpu2_core_->cycles() < target) {
        const cortexm::RunResult result = cpu2_core_->run(target);
        if (result.reason == cortexm::StopReason::Reset) {
            cpu2_core_->reset(FlashBase + cpu2_vector_table_offset_);
            continue;
        }
        if (result.reason == cortexm::StopReason::StepLimit)
            return result;
        if (result.reason != cortexm::StopReason::Deadline) {
            cpu2_stopped_ = true;
            cpu2_stop_result_ = result;
            return result;
        }
    }
    return {};
}

void Stm32wb55::set_device_id(std::uint64_t id) {
    device_id_ = id;
    eui64_.resize(8);
    for (unsigned i = 0; i != 8; ++i)
        eui64_[i] = static_cast<std::uint8_t>(id >> (56 - i * 8));
    put32(system_memory_ + (0x1fff7590u - SystemMemoryBase), static_cast<std::uint32_t>(id));
    put32(system_memory_ + (0x1fff7594u - SystemMemoryBase), static_cast<std::uint32_t>(id >> 32));
    put32(system_memory_ + (0x1fff7598u - SystemMemoryBase), 0x00cafe55u);
    put32(system_memory_ + (Uid64Address - SystemMemoryBase), static_cast<std::uint32_t>(id));
    put32(system_memory_ + (Uid64Address + 4 - SystemMemoryBase),
          static_cast<std::uint32_t>(id >> 32));
    rng_state_ = static_cast<std::uint32_t>(id) ^ static_cast<std::uint32_t>(id >> 32) ^
                 0x9e3779b9u;
    if (rng_state_ == 0)
        rng_state_ = 0x6d2b79f5u;
}

void Stm32wb55::load_firmware(const std::vector<std::uint8_t> &image, std::uint32_t offset) {
    if (offset > FlashSize || image.size() > FlashSize - offset)
        throw std::runtime_error("firmware does not fit in the 1 MiB STM32WB55 flash");
    std::copy(image.begin(), image.end(), flash_ + offset);
    code_end_ = std::max(code_end_, offset + static_cast<std::uint32_t>(image.size()));
    vector_table_offset_ = offset;
    const std::size_t scan = std::min<std::size_t>(image.size(), 0x1000);
    for (std::size_t candidate = 0; candidate + 8 <= scan; candidate += 4) {
        const std::uint32_t initial_sp = get32(image.data() + candidate);
        const std::uint32_t reset_pc = get32(image.data() + candidate + 4);
        if (initial_sp >= RamBase && initial_sp <= RamBase + RamSize &&
            (initial_sp & 3u) == 0 && (reset_pc & 1u) &&
            (reset_pc & ~1u) >= FlashBase + offset &&
            (reset_pc & ~1u) < FlashBase + FlashSize) {
            vector_table_offset_ = offset + static_cast<std::uint32_t>(candidate);
            break;
        }
    }
    core_->invalidate_code();
}

void Stm32wb55::load_cpu2_firmware(const std::vector<std::uint8_t> &image,
                                  std::uint32_t offset) {
    if (offset > FlashSize || image.size() > FlashSize - offset)
        throw std::runtime_error("CPU2 firmware does not fit in the 1 MiB STM32WB55 flash");
    std::copy(image.begin(), image.end(), flash_ + offset);
    code_end_ = std::max(code_end_, offset + static_cast<std::uint32_t>(image.size()));
    if (!cpu2_vector_table_found_)
        cpu2_vector_table_offset_ = offset;
    const std::size_t scan = std::min<std::size_t>(image.size(), 0x1000);
    for (std::size_t candidate = 0; candidate + 8 <= scan; candidate += 4) {
        const std::uint32_t initial_sp = get32(image.data() + candidate);
        const std::uint32_t reset_pc = get32(image.data() + candidate + 4);
        if (initial_sp >= RamBase && initial_sp <= RamBase + RamSize &&
            (initial_sp & 3u) == 0 && (reset_pc & 1u) &&
            (reset_pc & ~1u) >= FlashBase + offset &&
            (reset_pc & ~1u) < FlashBase + FlashSize) {
            if (!cpu2_vector_table_found_) {
                cpu2_vector_table_offset_ = offset + static_cast<std::uint32_t>(candidate);
                cpu2_vector_table_found_ = true;
            }
            break;
        }
    }
    cpu2_image_loaded_ = true;
    core_->invalidate_code();
    cpu2_core_->invalidate_code();
}

void Stm32wb55::reset() {
    ipcc_c1cr_ = 0;
    ipcc_c1mr_ = 0xffffffffu;
    ipcc_c2cr_ = 0;
    ipcc_c2mr_ = 0xffffffffu;
    c1toc2sr_ = 0;
    c2toc1sr_ = 0;
    lptim_isr_ = 0;
    lptim_ier_ = 0;
    lptim_cfgr_ = 0;
    lptim_cr_ = 0;
    lptim_cmp_ = 0;
    lptim_arr_ = 0xffff;
    lptim_epoch_ns_ = core_time_ns(*core_);
    lptim_epoch_count_ = 0;
    rng_cr_ = 0;
    rng_next_ready_ns_ = 0;
    rng_data_ready_ = false;
    registers_.clear();
    registers_[FlashControlBase + 0x3c] = 0;
    registers_[FlashControlBase + 0x14] = 0xc0000000u;
    registers_[RccBase] = (1u << 1) | (1u << 10) | (1u << 17) | (1u << 25);
    registers_[RccBase + 0x98] = 1u << 1;
    rx_notification_outstanding_ = false;
    uart_rx_.clear();
    uart_tx_.clear();
    radio_enabled_ = false;
    radio_raw_stream_ = false;
    short_address_ = 0xffff;
    cpu2_started_ = false;
    cpu2_model_active_ = false;
    cpu2_model_boot_pending_ = false;
    cpu2_model_pending_channels_ = 0;
    cpu2_stopped_ = false;
    cpu2_stop_result_ = {};
    cpu2_boot_time_ns_ = 0;
    cpu2_boot_cycle_ = 0;
    spinel_tid_counter_ = 0;
    rx_queue_.clear();
    spinel_response_pending_ = false;
    pending_tx_response_ = false;
    pending_response_ns_ = 0;
    radio_poll_ns_ = 0;
    last_rx_ns_ = 0;
    cpu2_model_boot_due_ns_ = 0;
    mac_notification_outstanding_ = false;
    st_mac_.reset();
    dma1_ = {};
    dma1_isr_ = 0;
    tim2_cr1_ = 0;
    tim2_dier_ = 0;
    tim2_sr_ = 0;
    tim2_psc_ = 0;
    tim2_psc_active_ = 0;
    tim2_arr_ = 0xffffffffu;
    tim2_ccr_ = {};
    tim2_epoch_cycle_ = core_->cycles();
    tim2_epoch_count_ = 0;
    core_->reset(FlashBase + vector_table_offset_);
}

void Stm32wb55::set_cpu2_model(Cpu2Model model) {
    cpu2_model_kind_ = model;
}

void Stm32wb55::connect_radio(mcuemu::VirtualAir &air, const std::string &node) {
    air_ = &air;
    radio_node_ = node;
    radio_poll_ns_ = time_ns();
    core_->request_service(core_->cycles() + 1);
}

bool Stm32wb55::uart_rx(std::uint8_t byte) {
    if (uart_rx_.size() >= 256)
        return false;
    uart_rx_.push_back(byte);
    return true;
}

std::vector<std::uint8_t> Stm32wb55::uart_take_tx() {
    std::vector<std::uint8_t> bytes;
    bytes.swap(uart_tx_);
    return bytes;
}

std::uint32_t Stm32wb55::read(std::uint32_t address, unsigned size) {
    return read_mmio(address, size);
}

void Stm32wb55::write(std::uint32_t address, std::uint32_t value, unsigned size) {
    write_mmio(address, value, size);
}

std::uint32_t Stm32wb55::read_cpu2_mmio(std::uint32_t address, unsigned size) {
    cortexm::Core *previous = mmio_core_;
    mmio_core_ = cpu2_core_.get();
    try {
        const std::uint32_t value = read_mmio(address, size);
        mmio_core_ = previous;
        return value;
    } catch (...) {
        mmio_core_ = previous;
        throw;
    }
}

void Stm32wb55::write_cpu2_mmio(std::uint32_t address, std::uint32_t value, unsigned size) {
    cortexm::Core *previous = mmio_core_;
    mmio_core_ = cpu2_core_.get();
    try {
        write_mmio(address, value, size);
        mmio_core_ = previous;
    } catch (...) {
        mmio_core_ = previous;
        throw;
    }
}

std::uint32_t Stm32wb55::read_mmio(std::uint32_t address, unsigned size) {
    cortexm::Core &bus_core = mmio_core_ ? *mmio_core_ : *core_;
    if (address >= RngBase && address < RngBase + 0x400) {
        rng_advance(core_time_ns(bus_core));
        const std::uint32_t base = address & ~3u;
        std::uint32_t value = 0;
        switch (base - RngBase) {
        case 0x00: value = rng_cr_; break;
        case 0x04: value = rng_data_ready_ ? 1u : 0u; break;
        case 0x08:
            if (rng_data_ready_) {
                value = rng_next_word();
                rng_data_ready_ = false;
                if (rng_cr_ & RngEnable) {
                    rng_next_ready_ns_ = core_time_ns(bus_core) + RngReadyTimeNs;
                    request_service_at_ns(bus_core, rng_next_ready_ns_);
                }
                rng_update_irq();
            }
            break;
        default: value = registers_[base]; break;
        }
        return lane_read(value, address, size);
    }
    if (address >= Tim2Base && address < Tim2Base + 0x400) {
        tim2_advance(core_->cycles());
        return lane_read(tim2_read(address & 0x3fcu), address, size);
    }
    if (address >= Dma1Base && address < Dma1Base + 0x400)
        return lane_read(dma_read(address & ~3u), address, size);
    if (address >= LptimBase && address < LptimBase + 0x400) {
        const std::uint64_t now_ns = core_time_ns(bus_core);
        lptim_advance(now_ns);
        const std::uint32_t base = address & ~3u;
        std::uint32_t value = 0;
        switch (base - LptimBase) {
        case 0x00: value = lptim_isr_; break;
        case 0x04: value = 0; break;
        case 0x08: value = lptim_ier_; break;
        case 0x0c: value = lptim_cfgr_; break;
        case 0x10: value = lptim_cr_; break;
        case 0x14: value = lptim_cmp_; break;
        case 0x18: value = lptim_arr_; break;
        case 0x1c: value = lptim_counter(now_ns); break;
        default: return 0;
        }
        return lane_read(value, address, size);
    }
    if ((address & ~3u) == 0x40013824) {
        const std::uint32_t value = uart_rx_.empty() ? 0 : uart_rx_.front();
        if (!uart_rx_.empty())
            uart_rx_.pop_front();
        return lane_read(value, address, size);
    }
    if (address >= HsemBase + 0x80 && address < HsemBase + 0x100)
        return lane_read(0x80000400u, address, size); // M4 owns the one-step semaphore.
    if (address >= IpccBase && address < IpccBase + 0x20) {
        switch (address & ~3u) {
        case IpccBase + 0x00: return lane_read(ipcc_c1cr_, address, size);
        case IpccBase + 0x04: return lane_read(ipcc_c1mr_, address, size);
        case IpccBase + 0x08: return 0;
        case IpccBase + 0x0c: return lane_read(c1toc2sr_, address, size);
        case IpccBase + 0x10: return lane_read(ipcc_c2cr_, address, size);
        case IpccBase + 0x14: return lane_read(ipcc_c2mr_, address, size);
        case IpccBase + 0x18: return 0;
        case IpccBase + 0x1c: return lane_read(c2toc1sr_, address, size);
        default: break;
        }
    }
    if (address >= RccBase && address < RccBase + 0x400) {
        std::uint32_t value = registers_[address & ~3u];
        // Prescaler-applied flags (RM0434 8.4.3 HPREF/PPRE1F/PPRE2F, 8.4.32
        // SHDHPREF/C2HPREF): new dividers take effect at once in the model.
        if ((address & ~3u) == RccBase + 0x08)
            value |= 7u << 16;
        else if ((address & ~3u) == RccBase + 0x108)
            value |= 3u << 16;
        return lane_read(value, address, size);
    }
    if (address >= PwrBase && address < PwrBase + 0x100)
        return lane_read(registers_[address & ~3u], address, size);
    if ((address & ~3u) == FlashControlBase + 0x3c)
        return lane_read(0, address, size);
    // FLASH_SFR (RM0434 3.10.18): CPU2 security enabled (FSD = 0) with the
    // secure area from page 0xE1, where FUS 1.2.0 + MAC 1.24.0 install.
    if ((address & ~3u) == FlashControlBase + 0x80)
        return lane_read(0x000000e1u, address, size);

    const std::uint32_t base = address & ~3u;
    std::uint32_t value = registers_[base];
    // GPIO input data follows the output latch for the unwired board pads.
    // An unwired input with a pull resistor reads the pulled level (RM0434
    // 9.3.1), so active-low buttons with pull-ups read released.
    if (base >= 0x48000010 && base < 0x48002000 && ((base - 0x48000000) & 0x3ffu) == 0x10) {
        const std::uint32_t port = base - 0x10;
        const std::uint32_t moder = registers_[port];
        const std::uint32_t pupdr = registers_[port + 0x0c];
        value = registers_[port + 0x14];
        for (unsigned pin = 0; pin != 16; ++pin) {
            if (((moder >> (2 * pin)) & 3u) != 0)
                continue;
            const std::uint32_t pull = (pupdr >> (2 * pin)) & 3u;
            if (pull == 1)
                value |= 1u << pin;
            else if (pull == 2)
                value &= ~(1u << pin);
        }
    }
    // USART status bits that the polling HAL expects to be immediately ready.
    const bool uart_isr =
        (base >= 0x40008000 && base < 0x40008400 && ((base - 0x40008000) & 0x3ffu) == 0x1c) ||
        (base >= 0x40013800 && base < 0x40013c00 && ((base - 0x40013800) & 0x3ffu) == 0x1c);
    if (uart_isr) {
        value |= (1u << 22) | (1u << 21) | (1u << 7) | (1u << 6);
        if ((base & ~0x3ffu) == 0x40013800 && !uart_rx_.empty())
            value |= 1u << 5;
    }
    return lane_read(value, address, size);
}

void Stm32wb55::write_mmio(std::uint32_t address, std::uint32_t value, unsigned size) {
    cortexm::Core &bus_core = mmio_core_ ? *mmio_core_ : *core_;
    const bool from_cpu2 = mmio_core_ == cpu2_core_.get();
    const std::uint32_t base = address & ~3u;
    if (address >= RngBase && address < RngBase + 0x400) {
        const std::uint64_t now_ns = core_time_ns(bus_core);
        rng_advance(now_ns);
        if (base == RngBase) {
            const bool was_enabled = (rng_cr_ & RngEnable) != 0;
            rng_cr_ = lane_write(rng_cr_, value, address, size);
            const bool enabled = (rng_cr_ & RngEnable) != 0;
            if (enabled && !was_enabled) {
                rng_data_ready_ = false;
                rng_next_ready_ns_ = now_ns + RngReadyTimeNs;
                request_service_at_ns(bus_core, rng_next_ready_ns_);
            } else if (!enabled) {
                rng_data_ready_ = false;
                rng_next_ready_ns_ = 0;
            }
            rng_update_irq();
        } else if (base != RngBase + 0x04 && base != RngBase + 0x08) {
            registers_[base] = lane_write(registers_[base], value, address, size);
        }
        return;
    }
    if (address >= Tim2Base && address < Tim2Base + 0x400) {
        tim2_advance(core_->cycles());
        const std::uint32_t offset = address & 0x3fcu;
        const std::uint32_t old_value = offset == 0x10 ? tim2_sr_ : tim2_read(offset);
        tim2_write(offset, lane_write(old_value, value, address, size));
        return;
    }
    if (address >= Dma1Base && address < Dma1Base + 0x400) {
        dma_write(base, lane_write(dma_read(base), value, address, size));
        return;
    }
    if ((base & ~0x3ffu) == 0x40013800 && (base & 0x3ffu) == 0x28) {
        if (registers_[0x40013800] & (1u << 3))
            uart_tx_.push_back(static_cast<std::uint8_t>(lane_write(0, value, address, size)));
        return;
    }
    if (address >= LptimBase && address < LptimBase + 0x400) {
        const std::uint64_t now = core_time_ns(bus_core);
        lptim_advance(now);
        switch (base - LptimBase) {
        case 0x04:
            lptim_isr_ &= ~lane_write(0, value, address, size);
            lptim_update_irq();
            break;
        case 0x08:
            lptim_ier_ = lane_write(lptim_ier_, value, address, size);
            lptim_update_irq();
            break;
        case 0x0c:
            lptim_epoch_count_ = lptim_counter(now);
            lptim_epoch_ns_ = now;
            lptim_cfgr_ = lane_write(lptim_cfgr_, value, address, size);
            break;
        case 0x10: {
            const bool was_running = (lptim_cr_ & 5u) == 5u;
            const std::uint32_t count = lptim_counter(now);
            lptim_cr_ = lane_write(lptim_cr_, value, address, size);
            const bool running = (lptim_cr_ & 5u) == 5u;
            if (was_running && !running) {
                lptim_epoch_count_ = count;
                lptim_epoch_ns_ = now;
            } else if (!was_running && running) {
                lptim_epoch_count_ = 0;
                lptim_epoch_ns_ = now;
            }
            request_service_at_ns(bus_core, now + 1);
            break;
        }
        case 0x14:
            lptim_cmp_ = lane_write(lptim_cmp_, value, address, size) & 0xffffu;
            break;
        case 0x18:
            lptim_arr_ = lane_write(lptim_arr_, value, address, size) & 0xffffu;
            lptim_isr_ |= 1u << 4; // ARROK: the preload register accepted the write.
            lptim_update_irq();
            request_service_at_ns(bus_core, now + 1);
            break;
        default:
            break;
        }
        return;
    }
    if (base == IpccBase + 0x00) {
        if (from_cpu2) {
            ipcc_c2cr_ = lane_write(ipcc_c2cr_, value, address, size);
        } else {
            ipcc_c1cr_ = lane_write(ipcc_c1cr_, value, address, size);
        }
        signal_cpu1_rx(0);
        signal_cpu1_tx(0);
        signal_cpu2_rx(0);
        signal_cpu2_tx(0);
        return;
    }
    if (base == IpccBase + 0x04) {
        if (from_cpu2)
            ipcc_c2mr_ = lane_write(ipcc_c2mr_, value, address, size);
        else
            ipcc_c1mr_ = lane_write(ipcc_c1mr_, value, address, size);
        signal_cpu1_rx(0);
        signal_cpu1_tx(0);
        signal_cpu2_rx(0);
        signal_cpu2_tx(0);
        return;
    }
    if (base == IpccBase + 0x08) {
        if (from_cpu2)
            return;
        const std::uint32_t command = lane_write(0, value, address, size);
        const std::uint32_t clears = command & 0x3fu;
        const std::uint32_t sets = (command >> 16) & 0x3fu;
        if (cpu2_image_loaded_) {
            c1toc2sr_ = (c1toc2sr_ & ~clears) | sets;
        } else {
            c2toc1sr_ &= ~clears;
            if ((clears & (1u << 2)) && st_mac_) {
                // TL_MAC_802_15_4_SendAck: CPU1 released the notification buffer.
                mac_notification_outstanding_ = false;
                request_service_at_ns(*cpu2_core_, core_time_ns(*core_) + 1);
            } else if (clears & (1u << 2)) {
                rx_notification_outstanding_ = false;
                deliver_rx_frame();
            }
            c1toc2sr_ |= sets;
            if (cpu2_model_active_) {
                cpu2_model_pending_channels_ |= sets;
                if (sets)
                    request_service_at_ns(*cpu2_core_, core_time_ns(*core_) + 1);
            } else {
                for (unsigned channel = 1; channel <= 6; ++channel)
                    if (sets & (1u << (channel - 1)))
                        ipcc_command(channel);
            }
        }
        signal_cpu1_rx(0);
        signal_cpu1_tx(0);
        signal_cpu2_rx(0);
        signal_cpu2_tx(0);
        return;
    }
    if (base == IpccBase + 0x10) {
        ipcc_c2cr_ = lane_write(ipcc_c2cr_, value, address, size);
        signal_cpu2_rx(0);
        signal_cpu2_tx(0);
        return;
    }
    if (base == IpccBase + 0x14) {
        ipcc_c2mr_ = lane_write(ipcc_c2mr_, value, address, size);
        signal_cpu2_rx(0);
        signal_cpu2_tx(0);
        return;
    }
    if (base == IpccBase + 0x18) {
        if (!from_cpu2 && cpu2_image_loaded_)
            return;
        const std::uint32_t command = lane_write(0, value, address, size);
        const std::uint32_t clears = command & 0x3fu;
        const std::uint32_t sets = (command >> 16) & 0x3fu;
        c2toc1sr_ = (c2toc1sr_ & ~clears) | sets;
        signal_cpu1_rx(0);
        signal_cpu1_tx(0);
        signal_cpu2_rx(0);
        signal_cpu2_tx(0);
        return;
    }

    std::uint32_t updated = lane_write(registers_[base], value, address, size);
    if (base == RccBase) {
        // PLLRDY/PLLSAI1RDY follow their enables both ways: embassy turns the
        // PLL off and waits for PLLRDY to clear before reprogramming it. The
        // oscillator ready flags stay set as before.
        updated &= ~((1u << 25) | (1u << 27));
        if (updated & (1u << 26)) updated |= 1u << 27;
        if (updated & (1u << 0)) updated |= 1u << 1;
        if (updated & (1u << 8)) updated |= 1u << 10;
        if (updated & (1u << 16)) updated |= 1u << 17;
        if (updated & (1u << 24)) updated |= 1u << 25;
        if (updated & (1u << 12)) updated |= 1u << 13;
    } else if (base == RccBase + 0x08) {
        updated = (updated & ~0xcu) | ((updated & 3u) << 2);
    } else if (base == RccBase + 0x90 || base == RccBase + 0x94) {
        if (updated & 1u) updated |= 2u;
    } else if (base == RccBase + 0x98) {
        if (updated & 1u) updated |= 2u;
    }
    registers_[base] = updated;

    if (base == FlashControlBase + 0x08 && updated == 0xcdef89abu) {
        registers_[FlashControlBase + 0x14] &= ~(1u << 31); // KEYR second key
    } else if (base == FlashControlBase + 0x14) {
        // FLASH_CR (RM0434 3.10.5): page erase completes at once; the
        // programming itself arrives through the flash write handler.
        if (registers_[FlashControlBase + 0x14] & (1u << 31))
            updated |= 1u << 31; // LOCK only clears through KEYR
        if ((updated & (1u << 16)) && (updated & (1u << 1)) && !(updated & (1u << 31))) {
            const std::uint32_t page = (updated >> 3) & 0xffu;
            if (page * 0x1000u < FlashSize) {
                std::memset(flash_ + page * 0x1000u, 0xff, 0x1000u);
                if (page * 0x1000u < code_end_) {
                    core_->request_code_invalidation();
                    cpu2_core_->request_code_invalidation();
                }
            }
            registers_[FlashControlBase + 0x10] |= 1u; // EOP
        }
        updated &= ~(1u << 16);
        registers_[base] = updated;
    }
    if (!from_cpu2 && base == PwrBase + 0x0c && (updated & (1u << 15))) {
        start_cpu2();
    }
    if (base == 0x40021000) {
        // DMA status is sticky in hardware; an IFCR write clears matching flags.
        registers_[0x40021000] &= ~value;
    }
}

std::uint64_t Stm32wb55::service(std::uint64_t now_cycle) {
    const std::uint64_t now_ns = core_->cycles_to_ns(now_cycle);
    if (cpu2_started_ && !cpu2_stopped_)
        (void)run_cpu2_until(now_ns);
    rng_advance(now_ns);
    lptim_advance(now_ns);
    tim2_advance(now_cycle);
    if (!cpu2_model_active_ && air_ && radio_enabled_)
        poll_radio(now_ns);
    if (!cpu2_model_active_ && pending_tx_response_ && now_ns >= pending_tx_.response_ns)
        complete_radio_tx(now_ns);
    if (!cpu2_model_active_ && spinel_response_pending_ &&
        now_ns >= pending_response_ns_) {
        spinel_response_pending_ = false;
        signal_spinel(std::move(pending_spinel_response_));
    }

    std::uint64_t next = Never;
    if (!cpu2_model_active_ && air_ && radio_enabled_)
        next = now_cycle + RadioPollCycles;
    if (cpu2_started_ && !cpu2_stopped_)
        next = std::min(next, now_cycle + Cpu2SyncCycles);
    if (!cpu2_model_active_ && pending_tx_response_) {
        const std::uint64_t due = core_->ns_to_cycles(pending_tx_.response_ns);
        next = std::min(next, std::max(now_cycle + 1, due));
    }
    if (!cpu2_model_active_ && spinel_response_pending_)
        next = std::min(next, std::max(now_cycle + 1,
                                       core_->ns_to_cycles(pending_response_ns_)));
    if ((rng_cr_ & RngEnable) && !rng_data_ready_ && rng_next_ready_ns_ != 0)
        next = std::min(next, std::max(now_cycle + 1,
                                      core_->ns_to_cycles(rng_next_ready_ns_)));
    const std::uint64_t lptim_due_ns = lptim_next_event_ns(now_ns);
    if (lptim_due_ns != Never)
        next = std::min(next, std::max(now_cycle + 1, core_->ns_to_cycles(lptim_due_ns)));
    const std::uint64_t tim2_due = tim2_next_event(now_cycle);
    if (tim2_due != Never)
        next = std::min(next, std::max(now_cycle + 1, tim2_due));
    return next;
}

std::uint64_t Stm32wb55::service_cpu2(std::uint64_t now_cycle) {
    const std::uint64_t now_ns = core_time_ns(*cpu2_core_);
    if (cpu2_model_active_ && st_mac_) {
        const std::uint64_t due_ns = st_mac_service(now_ns);
        rng_advance(now_ns);
        lptim_advance(now_ns);
        std::uint64_t next = Never;
        auto cycle_at = [&](std::uint64_t at_ns) {
            const std::uint64_t elapsed_ns = at_ns > cpu2_boot_time_ns_
                                                 ? at_ns - cpu2_boot_time_ns_
                                                 : 0;
            return std::max(now_cycle + 1,
                            cpu2_boot_cycle_ + cpu2_core_->ns_to_cycles(elapsed_ns));
        };
        if (due_ns != Never)
            next = cycle_at(due_ns);
        if (air_ && st_mac_->rx_enabled())
            next = std::min(next, now_cycle + Cpu2RadioPollCycles);
        if ((rng_cr_ & RngEnable) && !rng_data_ready_ && rng_next_ready_ns_ != 0)
            next = std::min(next, cycle_at(rng_next_ready_ns_));
        return next;
    }
    if (cpu2_model_active_) {
        if (cpu2_model_boot_pending_) {
            initialize_wireless_info();
            signal_ready_event();
            cpu2_model_boot_pending_ = false;
        }
        const std::uint32_t pending = cpu2_model_pending_channels_;
        cpu2_model_pending_channels_ = 0;
        for (unsigned channel = 1; channel <= 6; ++channel) {
            if (pending & (1u << (channel - 1)))
                ipcc_command(channel);
        }
        if (air_ && radio_enabled_)
            poll_radio(now_ns);
        if (pending_tx_response_ && now_ns >= pending_tx_.response_ns)
            complete_radio_tx(now_ns);
        if (spinel_response_pending_ && now_ns >= pending_response_ns_) {
            spinel_response_pending_ = false;
            signal_spinel(std::move(pending_spinel_response_));
        }
    }
    rng_advance(now_ns);
    lptim_advance(now_ns);
    std::uint64_t next = Never;
    auto cycle_at = [&](std::uint64_t due_ns) {
        const std::uint64_t elapsed_ns = due_ns > cpu2_boot_time_ns_
                                             ? due_ns - cpu2_boot_time_ns_
                                             : 0;
        return std::max(now_cycle + 1,
                        cpu2_boot_cycle_ + cpu2_core_->ns_to_cycles(elapsed_ns));
    };
    if ((rng_cr_ & RngEnable) && !rng_data_ready_ && rng_next_ready_ns_ != 0)
        next = std::min(next, cycle_at(rng_next_ready_ns_));
    if (cpu2_model_active_ && air_ && radio_enabled_)
        next = std::min(next, now_cycle + Cpu2RadioPollCycles);
    if (cpu2_model_active_ && pending_tx_response_)
        next = std::min(next, cycle_at(pending_tx_.response_ns));
    if (cpu2_model_active_ && spinel_response_pending_)
        next = std::min(next, cycle_at(pending_response_ns_));
    const std::uint64_t lptim_due_ns = lptim_next_event_ns(now_ns);
    if (lptim_due_ns != Never)
        next = std::min(next, cycle_at(lptim_due_ns));
    return next;
}

void Stm32wb55::rng_advance(std::uint64_t now_ns) {
    if ((rng_cr_ & RngEnable) && !rng_data_ready_ && rng_next_ready_ns_ != 0 &&
        now_ns >= rng_next_ready_ns_) {
        rng_data_ready_ = true;
        rng_next_ready_ns_ = 0;
        rng_update_irq();
    }
}

void Stm32wb55::rng_update_irq() {
    const bool pending = (rng_cr_ & (RngEnable | RngInterruptEnable)) ==
                             (RngEnable | RngInterruptEnable) &&
                         rng_data_ready_;
    core_->set_irq_line(RngIrq, pending);
    cpu2_core_->set_irq_line(Cpu2RngIrq, pending);
}

std::uint32_t Stm32wb55::rng_next_word() {
    std::uint32_t value = rng_state_;
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    rng_state_ = value ? value : 0x6d2b79f5u;
    return rng_state_;
}

std::uint32_t Stm32wb55::lptim_counter(std::uint64_t now_ns) const {
    if ((lptim_cr_ & 5u) != 5u || now_ns <= lptim_epoch_ns_)
        return lptim_epoch_count_;
    const std::uint32_t prescaler = 1u << ((lptim_cfgr_ >> 9) & 7u);
    const std::uint64_t frequency = LptimClockHz / prescaler;
    const std::uint64_t elapsed_ns = now_ns - lptim_epoch_ns_;
    return lptim_epoch_count_ +
           static_cast<std::uint32_t>(elapsed_ns * frequency / 1000000000ull);
}

void Stm32wb55::lptim_advance(std::uint64_t now_ns) {
    if ((lptim_cr_ & 5u) != 5u || now_ns <= lptim_epoch_ns_)
        return;
    const std::uint32_t prescaler = 1u << ((lptim_cfgr_ >> 9) & 7u);
    const std::uint64_t frequency = LptimClockHz / prescaler;
    unsigned catchup = 0;
    while (catchup++ < 65536) {
        const std::uint64_t target_count = static_cast<std::uint64_t>(lptim_arr_) + 1;
        if (target_count <= lptim_epoch_count_)
            break;
        const std::uint64_t count_delta = target_count - lptim_epoch_count_;
        const std::uint64_t elapsed_ns =
            (count_delta * 1000000000ull + frequency - 1) / frequency;
        const std::uint64_t due_ns = lptim_epoch_ns_ + elapsed_ns;
        if (due_ns > now_ns)
            break;
        lptim_epoch_ns_ = due_ns;
        lptim_epoch_count_ = 0;
        lptim_isr_ |= 1u << 1; // ARRM: the counter reloaded from ARR.
    }
    lptim_update_irq();
}

std::uint64_t Stm32wb55::lptim_next_event_ns(std::uint64_t now_ns) const {
    if ((lptim_cr_ & 5u) != 5u)
        return Never;
    const std::uint32_t prescaler = 1u << ((lptim_cfgr_ >> 9) & 7u);
    const std::uint64_t frequency = LptimClockHz / prescaler;
    const std::uint64_t target_count = static_cast<std::uint64_t>(lptim_arr_) + 1;
    if (target_count <= lptim_epoch_count_)
        return now_ns + 1;
    const std::uint64_t count_delta = target_count - lptim_epoch_count_;
    const std::uint64_t elapsed_ns =
        (count_delta * 1000000000ull + frequency - 1) / frequency;
    return std::max(now_ns + 1, lptim_epoch_ns_ + elapsed_ns);
}

void Stm32wb55::lptim_update_irq() {
    const bool pending = (lptim_isr_ & lptim_ier_) != 0;
    core_->set_irq_line(LptimIrq, pending);
    cpu2_core_->set_irq_line(Cpu2LptimIrq, pending);
}

void Stm32wb55::ipcc_command(unsigned channel) {
    if (st_mac_) {
        st_mac_command(channel, core_time_ns(*cpu2_core_));
        return;
    }
    const std::uint32_t bit = 1u << (channel - 1);
    if (channel == 2)
        complete_system_command();
    else if (channel == 5) {
        const std::uint32_t packet = thread_table_entry(1);
        if (packet >= RamBase && packet + 16 < RamBase + RamSize) {
            const std::uint8_t *buffer = ram_ + (packet - RamBase);
            const std::size_t length = buffer[11];
            std::vector<std::uint8_t> frame;
            if (length >= 2 && length <= 255 && hdlc_decode(buffer + 12, length, frame))
                handle_spinel_request(frame.data(), frame.size());
        }
    }
    c1toc2sr_ &= ~bit;
    signal_cpu1_tx(channel);
}

void Stm32wb55::complete_system_command() {
    const std::uint32_t sys_table = ref_table_entry(3);
    if (sys_table >= RamBase && sys_table + 8 <= RamBase + RamSize) {
        const std::uint32_t packet = memory_u32(sys_table);
        if (packet >= RamBase && packet + 16 < RamBase + RamSize) {
            std::uint8_t *buffer = ram_ + (packet - RamBase);
            const std::uint16_t opcode = get16(buffer + 9);
            // HCI Command Complete event in the command buffer, which SHCI
            // copies from byte zero before releasing its waiting semaphore.
            buffer[0] = 0x04;
            buffer[1] = 0x0e;
            buffer[2] = 4;
            buffer[3] = 1;
            put16(buffer + 4, opcode);
            buffer[6] = 0;
        }
    }
    c1toc2sr_ &= ~(1u << 1);
    signal_cpu1_tx(2);
}

void Stm32wb55::start_cpu2() {
    if (cpu2_started_)
        return;
    cpu2_boot_time_ns_ = core_->cycles_to_ns(core_->cycles());
    cpu2_boot_cycle_ = cpu2_core_->cycles();
    cpu2_started_ = true;
    cpu2_stopped_ = false;
    cpu2_stop_result_ = {};
    if (cpu2_image_loaded_) {
        cpu2_core_->reset(FlashBase + cpu2_vector_table_offset_);
        signal_cpu2_rx(0);
        core_->request_service(core_->cycles() + 1);
        return;
    }
    cpu2_model_active_ = true;
    cpu2_model_boot_pending_ = true;
    cpu2_model_pending_channels_ = 0;
    if (cpu2_model_kind_ == Cpu2Model::StMac802154) {
        st_mac_ = std::make_unique<StMac>(*mac_host_, device_id_ ^ 0x5354u);
        st_mac_->set_data_address(StMacDataAddress);
        st_mac_->set_trace(!cpu2_trace_name_.empty());
        cpu2_model_boot_due_ns_ = cpu2_boot_time_ns_ + StMacBootNs;
        mac_notification_outstanding_ = false;
        radio_poll_ns_ = cpu2_boot_time_ns_;
    }
    cpu2_core_->reset(Cpu2ModelVector);
    cpu2_core_->set_interrupts_masked(true);
    cpu2_core_->request_service(cpu2_core_->cycles() + 1);
    core_->request_service(core_->cycles() + 1);
}

void Stm32wb55::initialize_wireless_info() {
    const std::uint32_t info = ref_table_entry(0);
    if (info < RamBase || info + 28 > RamBase + RamSize)
        return;
    std::uint8_t *table = ram_ + (info - RamBase);
    // Stack type THREAD MTD and the memory fields exposed by SHCI_GetWirelessFwInfo.
    put32(table + 16, 0x01000001u);
    put32(table + 20, (20u << 24) | (10u << 16) | (192u << 8) | 220u);
    put32(table + 24, 0x11u);
    put32(table + 28, 0);
}

bool Stm32wb55::memory_read(std::uint32_t address, void *out, std::size_t size) const {
    if (address >= RamBase && address <= RamBase + RamSize &&
        size <= RamBase + RamSize - address) {
        std::memcpy(out, ram_ + (address - RamBase), size);
        return true;
    }
    return false;
}

bool Stm32wb55::memory_write(std::uint32_t address, const void *data, std::size_t size) {
    if (address >= RamBase && address <= RamBase + RamSize &&
        size <= RamBase + RamSize - address) {
        std::memcpy(ram_ + (address - RamBase), data, size);
        return true;
    }
    return false;
}

std::uint32_t Stm32wb55::memory_u32(std::uint32_t address) const {
    std::array<std::uint8_t, 4> bytes{};
    if (!memory_read(address, bytes.data(), bytes.size()))
        return 0;
    return get32(bytes.data());
}

void Stm32wb55::memory_put32(std::uint32_t address, std::uint32_t value) {
    std::array<std::uint8_t, 4> bytes{};
    put32(bytes.data(), value);
    (void)memory_write(address, bytes.data(), bytes.size());
}

std::uint32_t Stm32wb55::ref_table_entry(unsigned index) const {
    const std::uint32_t ipccdba = registers_.contains(FlashControlBase + 0x3c)
                                      ? registers_.at(FlashControlBase + 0x3c) & 0x3fffu
                                      : 0;
    return memory_u32(Sram2aBase + ipccdba * 4 + index * 4);
}

std::uint32_t Stm32wb55::thread_table_entry(unsigned index) const {
    const std::uint32_t table = ref_table_entry(2);
    return table >= RamBase && table + (index + 1) * 4 <= RamBase + RamSize
               ? memory_u32(table + index * 4)
               : 0;
}

void Stm32wb55::signal_cpu1_rx(unsigned) {
    const bool pending = (ipcc_c1cr_ & 1u) && (c2toc1sr_ & ~ipcc_c1mr_ & 0x3fu);
    core_->set_irq_line(IpccRxIrq, pending);
    if (cpu2_started_)
        core_->request_service(core_->cycles() + 1);
}

void Stm32wb55::signal_cpu1_tx(unsigned) {
    // TXFIE: a channel that is free (C1TOC2SR bit clear) and not masked by
    // its CHnFM bit (C1MR bits 16..21) requests the TX-free interrupt.
    const bool pending = (ipcc_c1cr_ & (1u << 16)) &&
                         ((~c1toc2sr_) & (~ipcc_c1mr_ >> 16) & 0x3fu);
    core_->set_irq_line(IpccTxIrq, pending);
    if (cpu2_started_)
        core_->request_service(core_->cycles() + 1);
}

void Stm32wb55::signal_cpu2_rx(unsigned) {
    if (!cpu2_started_)
        return;
    const bool rx_pending = (ipcc_c2cr_ & 1u) &&
                            (c1toc2sr_ & ~ipcc_c2mr_ & 0x003fu);
    const bool tx_pending = (ipcc_c2cr_ & (1u << 16)) &&
                            ((~c2toc1sr_) & (~ipcc_c2mr_ >> 16) & 0x3fu);
    cpu2_core_->set_irq_line(Cpu2IpccIrq, rx_pending || tx_pending);
}

void Stm32wb55::signal_cpu2_tx(unsigned) {
    signal_cpu2_rx(0);
}

bool Stm32wb55::signal_ready_event() {
    return signal_system_event(0x9200);
}

bool Stm32wb55::signal_system_event(std::uint16_t subevent) {
    const std::uint32_t sys_table = ref_table_entry(3);
    const std::uint32_t mm_table = ref_table_entry(4);
    if (sys_table < RamBase || sys_table + 8 > RamBase + RamSize ||
        mm_table < RamBase || mm_table + 8 > RamBase + RamSize)
        return false;
    const std::uint32_t queue = memory_u32(sys_table + 4);
    const std::uint32_t packet = memory_u32(mm_table + 4);
    if (queue < RamBase || packet < RamBase || packet + 16 > RamBase + RamSize)
        return false;
    const std::uint32_t tail = memory_u32(queue + 4);
    memory_put32(packet, queue);
    memory_put32(packet + 4, tail == 0 ? queue : tail);
    if (tail >= RamBase && tail + 4 <= RamBase + RamSize)
        memory_put32(tail, packet);
    memory_put32(queue + 4, packet);
    std::uint8_t *event = ram_ + (packet - RamBase);
    event[8] = 0x12; // TL_SYSEVT_PKT_TYPE
    event[9] = 0xff;
    event[10] = 4;
    put16(event + 11, subevent);
    event[13] = 0;
    event[14] = 0;
    c2toc1sr_ |= 1u << 1;
    signal_cpu1_rx(2);
    return true;
}

void Stm32wb55::signal_spinel(std::vector<std::uint8_t> frame) {
    if (rx_notification_outstanding_) {
        rx_queue_.push_front(std::move(frame));
        return;
    }
    const std::uint32_t packet = thread_table_entry(0);
    if (packet < RamBase || packet + 12 > RamBase + RamSize)
        return;
    const std::vector<std::uint8_t> encoded = hdlc_encode(frame);
    if (encoded.size() > 247 || packet + 11 + 8 + encoded.size() > RamBase + RamSize)
        return;
    std::uint8_t *buffer = ram_ + (packet - RamBase);
    std::fill(buffer, buffer + 11 + 8 + encoded.size(), 0);
    buffer[8] = 0x0c; // TL_OTNOT_PKT_TYPE
    buffer[9] = 0xff;
    buffer[10] = static_cast<std::uint8_t>(8 + encoded.size());
    put32(buffer + 11, 0);
    put32(buffer + 15, static_cast<std::uint32_t>(encoded.size()));
    std::copy(encoded.begin(), encoded.end(), buffer + 19);
    rx_notification_outstanding_ = true;
    c2toc1sr_ |= 1u << 2;
    signal_cpu1_rx(3);
}

void Stm32wb55::handle_spinel_request(const std::uint8_t *frame, std::size_t size) {
    if (size < 3 || (frame[0] & 0xc0u) != 0x80u)
        return;
    const std::uint64_t now_ns = time_ns();
    const std::uint8_t tid = frame[0] & 0x0fu;
    const std::uint8_t command = frame[1];
    std::size_t used = 0;
    std::uint32_t property = 0;
    if (!read_varint(frame + 2, size - 2, used, property))
        return;
    const std::uint8_t *value = frame + 2 + used;
    const std::size_t value_size = size - 2 - used;

    if (property == 0x71 && command == 3) {
        handle_spinel_tx(value, value_size, tid);
        return;
    }
    if (command == 3) {
        switch (property) {
        case 0x20:
            radio_enabled_ = value_size && value[0] != 0;
            if (radio_enabled_) {
                if (cpu2_model_active_)
                    request_service_at_ns(*cpu2_core_, now_ns + 1);
                else
                    core_->request_service(core_->cycles() + 1);
            }
            break;
        case 0x21:
            if (value_size) radio_channel_ = value[0];
            if (cpu2_model_active_)
                request_service_at_ns(*cpu2_core_, now_ns + 1);
            else
                core_->request_service(core_->cycles() + 1);
            break;
        case 0x25: if (value_size) radio_tx_power_ = value[0]; break;
        case 0x24: if (value_size) cca_threshold_ = static_cast<std::int8_t>(value[0]); break;
        case 0x37:
            radio_raw_stream_ = value_size && value[0] != 0;
            if (radio_raw_stream_) {
                if (cpu2_model_active_)
                    request_service_at_ns(*cpu2_core_, now_ns + 1);
                else
                    core_->request_service(core_->cycles() + 1);
            }
            break;
        default: break;
        }
    }

    std::vector<std::uint8_t> response{static_cast<std::uint8_t>(0x80u | tid), 6};
    append_varint(response, property);
    if (command == 2) {
        std::vector<std::uint8_t> result = spinel_value(property);
        response.insert(response.end(), result.begin(), result.end());
    } else if (command == 3) {
        response.insert(response.end(), value, value + value_size);
    } else {
        return;
    }
    signal_spinel(std::move(response));
}

void Stm32wb55::handle_spinel_tx(const std::uint8_t *value, std::size_t size,
                                 std::uint8_t tid) {
    if (size < 10 || pending_tx_response_)
        return;
    const std::size_t psdu_size = get16(value);
    if (psdu_size < 2 || psdu_size > 129 || size < psdu_size + 8)
        return;
    const std::size_t mac_size = psdu_size - 2;
    std::vector<std::uint8_t> mpdu(value + 2, value + 2 + mac_size);
    const std::uint16_t fcf = mpdu.size() >= 2 ? get16(mpdu.data()) : 0;
    const bool wants_ack = (fcf & (1u << 5)) != 0;
    const bool broadcast = !wants_ack || (mpdu.size() > 5 && mpdu[5] == 0xff && mpdu[6] == 0xff);
    const std::uint8_t channel = value[psdu_size + 2];
    const std::uint64_t now_ns = time_ns();
    RadioTx tx;
    tx.channel = channel;
    tx.wants_ack = wants_ack;
    tx.broadcast = broadcast;
    tx.sequence = mpdu.size() > 2 ? mpdu[2] : 0;
    tx.finish_ns = now_ns;
    tx.response_ns = now_ns;
    if (air_ && channel >= 11 && channel <= 26) {
        tx.frame_id = air_->transmit(radio_node_, {}, mcuemu::RadioProtocol::Zigbee802154,
                                     channel, now_ns, std::move(mpdu));
        const auto &registered = air_->frames().back();
        tx.finish_ns = registered.end_ns;
        tx.response_ns = tx.finish_ns + (wants_ack ? 1000000u : 0u);
    }
    pending_tx_ = tx;
    pending_tx_response_ = true;
    pending_tid_ = tid;
    pending_property_ = 0x71;
    if (cpu2_model_active_)
        request_service_at_ns(*cpu2_core_, tx.finish_ns);
    else
        core_->request_service(std::max(core_->cycles() + 1, core_->ns_to_cycles(tx.finish_ns)));
}

void Stm32wb55::poll_radio(std::uint64_t now_ns) {
    if (!air_ || radio_node_.empty())
        return;
    const std::uint64_t after = std::min(radio_poll_ns_, now_ns);
    std::vector<mcuemu::RadioFrame> frames = air_->receive(
        radio_node_, mcuemu::RadioProtocol::Zigbee802154, radio_channel_, now_ns, after);
    radio_poll_ns_ = now_ns;
    last_rx_ns_ = now_ns;
    for (const auto &frame : frames) {
        const std::vector<std::uint8_t> &mpdu = frame.payload;
        const std::uint16_t fcf = mpdu.size() >= 3 ? get16(mpdu.data()) : 0;
        const unsigned frame_type = fcf & 7u;
        const unsigned destination_mode = (fcf >> 10) & 3u;
        const unsigned source_mode = (fcf >> 14) & 3u;
        std::size_t payload_offset = 3;
        bool addressed_to_us = false;
        if (destination_mode != 0 && (destination_mode == 2 || destination_mode == 3)) {
            if (payload_offset + 2 <= mpdu.size()) {
                payload_offset += 2; // destination PAN ID
                const std::size_t address_size = destination_mode == 2 ? 2 : 8;
                if (payload_offset + address_size <= mpdu.size()) {
                    if (destination_mode == 2) {
                        addressed_to_us = get16(mpdu.data() + payload_offset) == short_address_ &&
                                          short_address_ != 0xffff;
                    } else {
                        addressed_to_us = true;
                        for (unsigned i = 0; i != 8; ++i)
                            addressed_to_us &= mpdu[payload_offset + i] == eui64_[7 - i];
                    }
                    payload_offset += address_size;
                }
            }
        }
        if (source_mode != 0 && !(fcf & (1u << 6)))
            payload_offset += 2; // source PAN ID when PAN compression is clear
        if (source_mode == 2)
            payload_offset += 2;
        else if (source_mode == 3)
            payload_offset += 8;
        if (payload_offset > mpdu.size())
            addressed_to_us = false;

        if (addressed_to_us && frame_type == 3 && payload_offset + 4 <= mpdu.size() &&
            mpdu[payload_offset] == 0x02 && mpdu[payload_offset + 3] == 0) {
            short_address_ = get16(mpdu.data() + payload_offset + 1);
        }

        if (addressed_to_us && (fcf & (1u << 5)) && frame_type != 0 && frame_type != 2) {
            const std::vector<std::uint8_t> ack = {0x02, 0, mpdu[2]};
            air_->transmit(radio_node_, frame.source, mcuemu::RadioProtocol::Zigbee802154,
                           frame.channel, frame.end_ns + RadioAckTurnaroundNs, ack);
        }

        bool ack_for_tx = false;
        if (pending_tx_response_ && pending_tx_.wants_ack && mpdu.size() >= 3) {
            ack_for_tx = (fcf & 7u) == 2u && frame.payload[2] == pending_tx_.sequence;
            if (ack_for_tx) {
                pending_tx_.ack = frame.payload;
                pending_tx_.ack_rssi = static_cast<std::int8_t>(frame.rssi_dbm);
            }
        }
        if (!ack_for_tx && radio_raw_stream_)
            queue_rx_frame(frame);
    }
    if (!rx_notification_outstanding_)
        deliver_rx_frame();
}

void Stm32wb55::complete_radio_tx(std::uint64_t now_ns) {
    (void)now_ns;
    pending_tx_response_ = false;
    std::vector<std::uint8_t> value;
    std::uint32_t status = pending_tx_.wants_ack && pending_tx_.ack.empty() ? 17u : 0u;
    append_varint(value, status);
    value.push_back(0); // pending
    value.push_back(0); // header updated
    const std::size_t ack_size = status == 0 ? pending_tx_.ack.size() + (pending_tx_.ack.empty() ? 0 : 2) : 0;
    value.push_back(static_cast<std::uint8_t>(ack_size));
    value.push_back(static_cast<std::uint8_t>(ack_size >> 8));
    if (ack_size != 0) {
        value.insert(value.end(), pending_tx_.ack.begin(), pending_tx_.ack.end());
        const std::uint16_t fcs = ieee802154_fcs(pending_tx_.ack);
        value.push_back(static_cast<std::uint8_t>(fcs));
        value.push_back(static_cast<std::uint8_t>(fcs >> 8));
    }
    value.push_back(static_cast<std::uint8_t>(pending_tx_.ack_rssi));
    std::vector<std::uint8_t> response{static_cast<std::uint8_t>(0x80u | pending_tid_), 6};
    append_varint(response, 0x00);
    response.insert(response.end(), value.begin(), value.end());
    pending_spinel_response_ = std::move(response);
    spinel_response_pending_ = true;
    pending_response_ns_ = time_ns();
}

void Stm32wb55::queue_rx_frame(const mcuemu::RadioFrame &frame) {
    if (frame.payload.size() > 127)
        return;
    const std::uint16_t fcs = ieee802154_fcs(frame.payload);
    std::vector<std::uint8_t> value;
    const std::uint16_t len = static_cast<std::uint16_t>(frame.payload.size() + 2);
    value.push_back(static_cast<std::uint8_t>(len));
    value.push_back(static_cast<std::uint8_t>(len >> 8));
    value.insert(value.end(), frame.payload.begin(), frame.payload.end());
    value.push_back(static_cast<std::uint8_t>(fcs));
    value.push_back(static_cast<std::uint8_t>(fcs >> 8));
    value.push_back(static_cast<std::uint8_t>(static_cast<std::int8_t>(frame.rssi_dbm)));
    value.push_back(static_cast<std::uint8_t>(-100));
    value.push_back(0); value.push_back(0); // flags
    value.push_back(2); value.push_back(0); // phy data size
    value.push_back(frame.channel);
    value.push_back(200); // LQI
    std::vector<std::uint8_t> spinel{0x80, 6};
    append_varint(spinel, 0x71);
    spinel.insert(spinel.end(), value.begin(), value.end());
    rx_queue_.push_back(std::move(spinel));
}

void Stm32wb55::deliver_rx_frame() {
    if (!rx_notification_outstanding_ && !rx_queue_.empty() && radio_enabled_ && radio_raw_stream_) {
        std::vector<std::uint8_t> frame = std::move(rx_queue_.front());
        rx_queue_.pop_front();
        signal_spinel(std::move(frame));
    }
}

std::vector<std::uint8_t> Stm32wb55::spinel_value(std::uint32_t property) const {
    switch (property) {
    case 0x08: return eui64_;
    case 0x20: return {static_cast<std::uint8_t>(radio_enabled_)};
    case 0x21: return {radio_channel_};
    case 0x24: return {static_cast<std::uint8_t>(cca_threshold_)};
    case 0x25: return {radio_tx_power_};
    case 0x26: return {static_cast<std::uint8_t>(-50)};
    case 0x34: return std::vector<std::uint8_t>(8, 0);
    case 0x35: return {0xff, 0xff};
    case 0x36: return {0xff, 0xff};
    case 0x38: return {0};
    default: return {};
    }
}

std::uint16_t Stm32wb55::ieee802154_fcs(const std::vector<std::uint8_t> &bytes) {
    std::uint16_t crc = 0;
    for (const std::uint8_t byte : bytes) {
        crc ^= byte;
        for (unsigned bit = 0; bit != 8; ++bit)
            crc = static_cast<std::uint16_t>((crc & 1u) ? (crc >> 1) ^ 0x8408u : crc >> 1);
    }
    return crc;
}

bool Stm32wb55::hdlc_decode(const std::uint8_t *data, std::size_t size,
                            std::vector<std::uint8_t> &frame) {
    std::vector<std::uint8_t> raw;
    bool in_frame = false;
    bool escaped = false;
    for (std::size_t i = 0; i != size; ++i) {
        const std::uint8_t byte = data[i];
        if (byte == 0x7e) {
            if (in_frame && raw.size() >= 5) {
                const std::uint16_t expected = get16(raw.data() + raw.size() - 2);
                if (hdlc_fcs(raw.data(), raw.size() - 2) == expected) {
                    raw.resize(raw.size() - 2);
                    frame = std::move(raw);
                    return true;
                }
            }
            in_frame = true;
            escaped = false;
            raw.clear();
            continue;
        }
        if (!in_frame)
            continue;
        if (byte == 0x7d) {
            escaped = true;
            continue;
        }
        raw.push_back(escaped ? static_cast<std::uint8_t>(byte ^ 0x20u) : byte);
        escaped = false;
    }
    return false;
}

std::vector<std::uint8_t> Stm32wb55::hdlc_encode(const std::vector<std::uint8_t> &frame) {
    std::vector<std::uint8_t> result{0x7e};
    const std::uint16_t fcs = hdlc_fcs(frame.data(), frame.size());
    auto append = [&result](std::uint8_t byte) {
        if (byte == 0x7e || byte == 0x7d || byte == 0x11 || byte == 0x13 || byte == 0xf8) {
            result.push_back(0x7d);
            result.push_back(static_cast<std::uint8_t>(byte ^ 0x20u));
        } else {
            result.push_back(byte);
        }
    };
    for (std::uint8_t byte : frame)
        append(byte);
    append(static_cast<std::uint8_t>(fcs));
    append(static_cast<std::uint8_t>(fcs >> 8));
    result.push_back(0x7e);
    return result;
}

void Stm32wb55::append_varint(std::vector<std::uint8_t> &out, std::uint32_t value) {
    do {
        std::uint8_t byte = static_cast<std::uint8_t>(value & 0x7fu);
        value >>= 7;
        if (value)
            byte |= 0x80u;
        out.push_back(byte);
    } while (value);
}

bool Stm32wb55::read_varint(const std::uint8_t *data, std::size_t size,
                           std::size_t &used, std::uint32_t &value) {
    value = 0;
    used = 0;
    unsigned shift = 0;
    while (used < size && shift < 32) {
        const std::uint8_t byte = data[used++];
        value |= static_cast<std::uint32_t>(byte & 0x7fu) << shift;
        if ((byte & 0x80u) == 0)
            return true;
        shift += 7;
    }
    return false;
}

std::uint64_t Stm32wb55::st_mac_service(std::uint64_t now_ns) {
    std::uint64_t next = Never;
    if (cpu2_model_boot_pending_) {
        if (now_ns >= cpu2_model_boot_due_ns_) {
            // The ready event needs CPU1's system queue and spare buffer
            // (shci_init, TL_MM_Init); retry until they are published.
            initialize_mac_wireless_info();
            if (signal_ready_event())
                cpu2_model_boot_pending_ = false;
            else
                cpu2_model_boot_due_ns_ = now_ns + StMacBootRetryNs;
        }
        if (cpu2_model_boot_pending_)
            next = cpu2_model_boot_due_ns_;
    }
    const std::uint32_t pending = cpu2_model_pending_channels_;
    cpu2_model_pending_channels_ = 0;
    for (unsigned channel = 1; channel <= 6; ++channel)
        if (pending & (1u << (channel - 1)))
            st_mac_command(channel, now_ns);
    if (air_ && !radio_node_.empty()) {
        if (st_mac_->rx_enabled()) {
            const std::uint64_t after = std::min(radio_poll_ns_, now_ns);
            for (const mcuemu::RadioFrame &frame :
                 air_->receive(radio_node_, mcuemu::RadioProtocol::Zigbee802154,
                               st_mac_->channel(), now_ns, after)) {
                StMacRxFrame received;
                received.mpdu = frame.payload;
                received.channel = frame.channel;
                received.start_ns = frame.start_ns;
                received.end_ns = frame.end_ns;
                received.rssi_dbm = frame.rssi_dbm;
                st_mac_->receive(received, now_ns);
            }
        }
        radio_poll_ns_ = now_ns;
    }
    next = std::min(next, st_mac_->service(now_ns));
    st_mac_deliver_notification();
    st_mac_flush_trace(now_ns);
    return next;
}

void Stm32wb55::st_mac_command(unsigned channel, std::uint64_t now_ns) {
    const std::uint32_t bit = 1u << (channel - 1);
    switch (channel) {
    case 2:
        // SHCI system command (SHCI_C2_MAC_802_15_4_Init and any other):
        // a Command Complete with status SUCCESS.
        complete_system_command();
        return;
    case 3: {
        // TL_MAC_802_15_4_SendCmd: the request sits in p_cmdrsp_buffer as a
        // TL_CmdPacket_t (8-octet header, type, opcode, plen, payload); the
        // synchronous status goes back in the same buffer as a TL_EvtPacket_t
        // whose payload[0] MAC_802_15_4_CallBack_Processing reads.
        const std::uint32_t table = ref_table_entry(MacTableIndex);
        const std::uint32_t buffer = memory_u32(table);
        std::array<std::uint8_t, 12 + 255> packet{};
        std::uint8_t status = StMac::NotImplemented;
        if (buffer >= RamBase && memory_read(buffer, packet.data(), 12)) {
            const std::uint16_t opcode = get16(packet.data() + 9);
            const std::size_t length = packet[11];
            const unsigned ocf = opcode & 0x3ffu; // MASK_CMD_CODE_OCF
            if (memory_read(buffer + 12, packet.data() + 12, length) &&
                ocf >= MacOpcodeOffset && ocf < MacOpcodeOffset + 0x20u)
                status = st_mac_->request(static_cast<std::uint8_t>(ocf - MacOpcodeOffset),
                                          packet.data() + 12, length, now_ns);
            const std::uint8_t response[4] = {0x09, 0x00, 1, status}; // TL_OTRSP_PKT_TYPE
            (void)memory_write(buffer + 8, response, sizeof response);
        }
        break;
    }
    default:
        // Channel 4 is the memory manager's buffer release (TL_MM): CPU2 takes
        // the buffers back from pevt_free_buffer_queue; nothing else to do.
        break;
    }
    c1toc2sr_ &= ~bit;
    signal_cpu1_tx(channel);
}

void Stm32wb55::st_mac_deliver_notification() {
    if (mac_notification_outstanding_ || !st_mac_->has_notification())
        return;
    const std::uint32_t table = ref_table_entry(MacTableIndex);
    const std::uint32_t buffer = memory_u32(table + 4); // p_notack_buffer
    if (buffer < RamBase || buffer + 11 + 255 > RamBase + RamSize)
        return;
    const std::optional<StMacNotification> notification = st_mac_->pop_notification();
    if (!notification->data.empty())
        (void)memory_write(st_mac_->data_address(), notification->data.data(),
                           notification->data.size());
    // TL_EvtPacket_t: header, type, event code, payload length, payload
    // (the 16-bit MAC_802_15_4_Notification_t subEvtCode, then the structure).
    std::array<std::uint8_t, 3> stub{0x0c, 0xff,
                                     static_cast<std::uint8_t>(notification->payload.size())};
    (void)memory_write(buffer + 8, stub.data(), stub.size());
    (void)memory_write(buffer + 11, notification->payload.data(), notification->payload.size());
    mac_notification_outstanding_ = true;
    c2toc1sr_ |= 1u << 2;
    signal_cpu1_rx(3);
}

void Stm32wb55::st_mac_flush_trace(std::uint64_t now_ns) {
    if (cpu2_trace_name_.empty())
        return;
    for (const std::string &line : st_mac_->take_trace()) {
        char stamp[32];
        std::snprintf(stamp, sizeof stamp, "%llu.%06llu",
                      static_cast<unsigned long long>(now_ns / 1000000000ull),
                      static_cast<unsigned long long>((now_ns / 1000ull) % 1000000ull));
        std::cerr << "[" << stamp << "] " << cpu2_trace_name_ << " cpu2-mac: " << line << '\n';
    }
}

void Stm32wb55::initialize_mac_wireless_info() {
    const std::uint32_t info = ref_table_entry(0);
    if (info < RamBase || info + 32 > RamBase + RamSize)
        return;
    std::uint8_t *table = ram_ + (info - RamBase);
    // MB_SafeBootInfoTable_t, MB_FusInfoTable_t, MB_WirelessFwInfoTable_t
    // (mbox_def.h). Versions are major.minor.sub in bits 31..8: FUS 1.2.0
    // and MAC 802.15.4 1.24.0; INFO_STACK_TYPE_MAC = 0x40 (shci.h).
    put32(table + 0, 0x01000000u);
    put32(table + 4, 0x01020000u);
    put32(table + 8, 0);
    put32(table + 12, 0);
    put32(table + 16, 0x01180000u);
    put32(table + 20, (12u << 24) | (16u << 16) | 31u);
    put32(table + 24, 0x40u);
    put32(table + 28, 0);
}

std::uint32_t Stm32wb55::dma_read(std::uint32_t address) {
    const std::uint32_t offset = address - Dma1Base;
    if (offset == 0x00)
        return dma1_isr_;
    if (offset == 0x04)
        return 0;
    if (offset >= 0x08 && offset < 0x08 + 7 * 20) {
        const DmaChannel &channel = dma1_[(offset - 0x08) / 20];
        switch ((offset - 0x08) % 20) {
        case 0: return channel.ccr;
        case 4: return channel.cndtr;
        case 8: return channel.cpar;
        case 12: return channel.cmar;
        default: break;
        }
    }
    return registers_[address];
}

void Stm32wb55::dma_write(std::uint32_t address, std::uint32_t value) {
    const std::uint32_t offset = address - Dma1Base;
    if (offset == 0x00)
        return;
    if (offset == 0x04) {
        dma1_isr_ &= ~value; // IFCR
        dma_update_irq();
        return;
    }
    if (offset < 0x08 || offset >= 0x08 + 7 * 20) {
        registers_[address] = value;
        return;
    }
    const unsigned index = (offset - 0x08) / 20;
    DmaChannel &channel = dma1_[index];
    switch ((offset - 0x08) % 20) {
    case 0: {
        const bool was_enabled = channel.ccr & 1u;
        channel.ccr = value & 0x7fffu;
        // Only memory-to-USART1 transfers are modeled (the log of the
        // zigbee-rs Nucleo image): they complete at once, so the bytes reach
        // the UART sink when the channel is enabled.
        const bool to_usart = (channel.ccr & (1u << 4)) && !(channel.ccr & (1u << 14)) &&
                              channel.cpar == Usart1Tdr;
        if (!was_enabled && (channel.ccr & 1u) && to_usart) {
            const unsigned step = 1u << ((channel.ccr >> 10) & 3u);
            const bool increment = channel.ccr & (1u << 7);
            for (std::uint32_t i = 0; i != (channel.cndtr & 0xffffu); ++i) {
                std::uint8_t byte = 0;
                if (memory_read(channel.cmar + (increment ? i * step : 0), &byte, 1))
                    uart_tx_.push_back(byte);
            }
            channel.cndtr = 0;
            dma1_isr_ |= 0x7u << (4 * index); // GIF, TCIF, HTIF
        }
        dma_update_irq();
        break;
    }
    case 4:
        if (!(channel.ccr & 1u))
            channel.cndtr = value & 0xffffu;
        break;
    case 8: channel.cpar = value; break;
    case 12: channel.cmar = value; break;
    default: break;
    }
}

void Stm32wb55::dma_update_irq() {
    for (unsigned index = 0; index != dma1_.size(); ++index) {
        const std::uint32_t flags = (dma1_isr_ >> (4 * index)) & 0xfu;
        const std::uint32_t ccr = dma1_[index].ccr;
        const bool pending = ((flags & 2u) && (ccr & 2u)) || ((flags & 4u) && (ccr & 4u)) ||
                             ((flags & 8u) && (ccr & 8u));
        core_->set_irq_line(Dma1Channel1Irq + index, pending);
    }
}

void Stm32wb55::tim2_advance(std::uint64_t now_cycle) {
    if (!(tim2_cr1_ & 1u) || now_cycle <= tim2_epoch_cycle_)
        return;
    const std::uint64_t period = static_cast<std::uint64_t>(tim2_psc_active_) + 1;
    const std::uint64_t ticks = (now_cycle - tim2_epoch_cycle_) / period;
    if (ticks == 0)
        return;
    const std::uint64_t modulus = static_cast<std::uint64_t>(tim2_arr_) + 1;
    const std::uint64_t old = tim2_epoch_count_;
    const std::uint64_t total = old + ticks;
    for (unsigned i = 0; i != 4; ++i) {
        const std::uint64_t compare = tim2_ccr_[i];
        if (compare >= modulus)
            continue;
        // CCxIF: the counter reached CCRx somewhere in (old, total].
        const std::uint64_t first = compare > old ? compare : compare + modulus;
        if (first <= total)
            tim2_sr_ |= 2u << i;
    }
    if (total >= modulus) {
        tim2_sr_ |= 1u; // UIF: overflow update event, which also loads PSC
        tim2_psc_active_ = tim2_psc_;
    }
    tim2_epoch_count_ = total % modulus;
    tim2_epoch_cycle_ += ticks * period;
    tim2_update_irq();
}

std::uint32_t Stm32wb55::tim2_counter() const {
    return static_cast<std::uint32_t>(tim2_epoch_count_);
}

std::uint64_t Stm32wb55::tim2_next_event(std::uint64_t now_cycle) const {
    const std::uint32_t enabled = tim2_dier_ & 0x1fu;
    if (!(tim2_cr1_ & 1u) || enabled == 0)
        return Never;
    const std::uint64_t period = static_cast<std::uint64_t>(tim2_psc_active_) + 1;
    const std::uint64_t modulus = static_cast<std::uint64_t>(tim2_arr_) + 1;
    const std::uint64_t current = tim2_epoch_count_;
    std::uint64_t ticks = Never;
    if (enabled & 1u)
        ticks = modulus - current;
    for (unsigned i = 0; i != 4; ++i) {
        if (!(enabled & (2u << i)) || tim2_ccr_[i] >= modulus)
            continue;
        const std::uint64_t compare = tim2_ccr_[i];
        ticks = std::min(ticks, compare > current ? compare - current : compare + modulus - current);
    }
    if (ticks == Never)
        return Never;
    return std::max(now_cycle + 1, tim2_epoch_cycle_ + ticks * period);
}

void Stm32wb55::tim2_update_irq() {
    core_->set_irq_line(Tim2Irq, (tim2_sr_ & tim2_dier_ & 0x1fu) != 0);
}

std::uint32_t Stm32wb55::tim2_read(std::uint32_t offset) {
    switch (offset) {
    case 0x00: return tim2_cr1_;
    case 0x0c: return tim2_dier_;
    case 0x10: return tim2_sr_;
    case 0x14: return 0;
    case 0x24: return tim2_counter();
    case 0x28: return tim2_psc_;
    case 0x2c: return tim2_arr_;
    case 0x34: case 0x38: case 0x3c: case 0x40: return tim2_ccr_[(offset - 0x34) / 4];
    default: return registers_[Tim2Base + offset];
    }
}

void Stm32wb55::tim2_write(std::uint32_t offset, std::uint32_t value) {
    // RM0434 21: TIM2 is a 32-bit up-counter clocked at CK_INT (here the
    // 64 MHz CPU1 clock) divided by PSC + 1.
    const std::uint64_t now = core_->cycles();
    switch (offset) {
    case 0x00:
        if (!(tim2_cr1_ & 1u) && (value & 1u))
            tim2_epoch_cycle_ = now;
        tim2_cr1_ = value & 0x3ffu;
        break;
    case 0x0c: tim2_dier_ = value; break;
    case 0x10: tim2_sr_ &= value; break; // rc_w0
    case 0x14:
        if (value & 1u) {
            // UG reinitializes the counter and loads the prescaler; it sets
            // UIF unless URS limits update interrupts to overflows.
            tim2_epoch_count_ = 0;
            tim2_epoch_cycle_ = now;
            tim2_psc_active_ = tim2_psc_;
            if (!(tim2_cr1_ & (1u << 2)))
                tim2_sr_ |= 1u;
        }
        tim2_sr_ |= value & 0x1eu; // CCxG
        break;
    case 0x24:
        tim2_epoch_count_ = value % (static_cast<std::uint64_t>(tim2_arr_) + 1);
        tim2_epoch_cycle_ = now;
        break;
    case 0x28: tim2_psc_ = value & 0xffffu; break;
    case 0x2c: tim2_arr_ = value; break;
    case 0x34: case 0x38: case 0x3c: case 0x40: tim2_ccr_[(offset - 0x34) / 4] = value; break;
    default: registers_[Tim2Base + offset] = value; return;
    }
    tim2_update_irq();
    core_->request_service(now + 1);
}

} // namespace stm32wb55emu
