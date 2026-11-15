#include "sim/stm32wb55_mcu.hpp"

#include <cstdlib>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace sim {
namespace {

std::uint64_t stable_id(std::string_view name) {
    std::uint64_t value = 1469598103934665603ull;
    for (const unsigned char byte : name)
        value = (value ^ byte) * 1099511628211ull;
    return value;
}

} // namespace

Stm32wb55Mcu::Stm32wb55Mcu(std::string name) : Mcu(std::move(name)) {
    chip_.set_device_id(stable_id(this->name()));
    // MCUEMU_STM32WB_TRACE=1 prints the primitives of the built-in MAC model.
    if (const char *trace = std::getenv("MCUEMU_STM32WB_TRACE"); trace && *trace && *trace != '0')
        chip_.set_cpu2_trace(this->name());
}

void Stm32wb55Mcu::load_firmware(const std::string &path) {
    load_images({{path, 0}});
}

void Stm32wb55Mcu::load_images(const std::vector<FirmwareImage> &images) {
    if (images.empty())
        throw std::invalid_argument("stm32wb55 requires at least one firmware image");
    for (const FirmwareImage &entry : images) {
        std::ifstream input(entry.file, std::ios::binary);
        if (!input)
            throw std::runtime_error("cannot open firmware image: " + entry.file);
        std::vector<std::uint8_t> image((std::istreambuf_iterator<char>(input)),
                                        std::istreambuf_iterator<char>());
        if (entry.skip > image.size())
            throw std::runtime_error("firmware image " + entry.file + " is shorter than 'from'");
        image.erase(image.begin(), image.begin() + entry.skip);
        if (entry.size != 0) {
            if (entry.size > image.size())
                throw std::runtime_error("firmware image " + entry.file +
                                         " is shorter than 'from' + 'size'");
            image.resize(entry.size);
        }
        chip_.load_firmware(image, entry.offset);
    }
    chip_.reset();
}

void Stm32wb55Mcu::load_cpu2_images(const std::vector<FirmwareImage> &images) {
    for (const FirmwareImage &entry : images) {
        std::ifstream input(entry.file, std::ios::binary);
        if (!input)
            throw std::runtime_error("cannot open CPU2 firmware image: " + entry.file);
        std::vector<std::uint8_t> image((std::istreambuf_iterator<char>(input)),
                                        std::istreambuf_iterator<char>());
        if (entry.skip > image.size())
            throw std::runtime_error("CPU2 firmware image " + entry.file +
                                     " is shorter than 'from'");
        image.erase(image.begin(), image.begin() + entry.skip);
        if (entry.size != 0) {
            if (entry.size > image.size())
                throw std::runtime_error("CPU2 firmware image " + entry.file +
                                         " is shorter than 'from' + 'size'");
            image.resize(entry.size);
        }
        chip_.load_cpu2_firmware(image, entry.offset);
    }
    if (!images.empty() && !chip_.cpu2_has_vector_table())
        throw std::runtime_error(
            "CPU2 firmware has no Cortex-M vector table near its load offset; "
            "expected a raw executable image (ST wireless-stack .bin packages "
            "are encrypted for FUS and cannot be executed directly)");
}

void Stm32wb55Mcu::reset() {
    chip_.reset();
    state_ = McuState::Running;
    fault_.clear();
}

void Stm32wb55Mcu::set_start_offset(std::uint64_t offset_ns) {
    chip_.core().advance_to(chip_.core().ns_to_cycles(offset_ns));
}

McuState Stm32wb55Mcu::run_until(std::uint64_t target_ns) {
    if (state_ != McuState::Running)
        return state_;
    const std::uint64_t target = chip_.core().ns_to_cycles(target_ns);
    while (state_ == McuState::Running && chip_.core().cycles() < target) {
        const cortexm::RunResult result = chip_.core().run(target);
        switch (result.reason) {
        case cortexm::StopReason::Deadline:
        case cortexm::StopReason::StepLimit:
            break;
        case cortexm::StopReason::Halted:
            state_ = McuState::Halted;
            fault_ = result.message;
            break;
        case cortexm::StopReason::Fault:
            state_ = McuState::Fault;
            fault_ = result.message;
            break;
        case cortexm::StopReason::Reset:
            chip_.reset();
            break;
        }
    }
    if (state_ == McuState::Running) {
        const cortexm::RunResult result = chip_.run_cpu2_until(target_ns);
        switch (result.reason) {
        case cortexm::StopReason::Deadline:
        case cortexm::StopReason::StepLimit:
        case cortexm::StopReason::Reset:
            break;
        case cortexm::StopReason::Halted:
            state_ = McuState::Halted;
            fault_ = "CPU2: " + result.message;
            break;
        case cortexm::StopReason::Fault:
            state_ = McuState::Fault;
            fault_ = "CPU2: " + result.message;
            break;
        }
    }
    return state_;
}

void Stm32wb55Mcu::connect_pin(std::string_view pad, Pin &) {
    throw std::invalid_argument("stm32wb55 has no board pin model (cannot connect '" +
                                std::string(pad) + "')");
}

void Stm32wb55Mcu::attach_radio(mcuemu::VirtualAir &air, const std::string &node,
                                const std::vector<std::string> &protocols) {
    for (const std::string &protocol : protocols)
        if (protocol != "zigbee")
            throw std::invalid_argument("stm32wb55: '" + protocol +
                                        "' is not connected to the RCP (only zigbee)");
    chip_.connect_radio(air, node);
}

void Stm32wb55Mcu::read_memory(std::uint32_t address, std::uint8_t *out, std::size_t length) {
    if (!chip_.core().read_memory(address, out, length))
        throw std::runtime_error("address 0x" + std::to_string(address) + " is not mapped");
}

void Stm32wb55Mcu::write_memory(std::uint32_t address, const std::uint8_t *data,
                                std::size_t length) {
    if (!chip_.core().write_memory(address, data, length))
        throw std::runtime_error("address 0x" + std::to_string(address) + " is not mapped");
}

std::vector<std::uint8_t> Stm32wb55Mcu::uart_take_tx(unsigned index) {
    return index == 0 ? chip_.uart_take_tx() : std::vector<std::uint8_t>{};
}

} // namespace sim
