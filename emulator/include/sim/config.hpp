#pragma once

#include "sim/mcu.hpp"
#include "sim/net.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace sim {

// Invalid or unsupported configuration. what() starts with "line N: " when the
// problem is attributable to a place in the YAML document.
class ConfigError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// A radio medium ("air"): the propagation model shared by the devices that
// attach to it.
struct RadioMediumConfig {
    std::string id;
    std::string model{"ideal"}; // ideal | log_distance
    double reference_loss_db{40.0};
    double exponent{2.0};
    double shadowing_sigma_db{0.0};
    double noise_floor_dbm{-100.0};
    double sinr_threshold_db{6.0};
    int line{0};
};

// How an MCU or part sits on a medium.
struct RadioAttachConfig {
    std::string air;
    std::array<double, 3> position{0, 0, 0}; // meters
    double tx_power_dbm{0.0};
    double sensitivity_dbm{-97.0};
    std::vector<std::string> protocols{"zigbee"}; // zigbee, ble
    int line{0};
};

// Connects an endpoint to a sink (see sim/sink.hpp for the spec grammar).
struct AttachConfig {
    std::string endpoint; // device scope: relative to the device; else full name
    std::string sink;
    int line{0};
};

// SEGGER RTT control block of the firmware, read from target memory.
struct RttConfig {
    std::uint32_t address{0};
    unsigned channels{1}; // endpoints "rtt", "rtt1", ... for channels 0..n-1
    std::uint64_t interval_ns{100000};
    int line{0};
};

// Debugger emulation (cc2530f256 only): the CPU halts each time it reaches
// `pc`; on the N-th arrival the writes for `hit: N` are applied to XDATA in
// list order, then execution resumes. Mirrors a host that halts firmware at a
// checkpoint and loads mailbox/input data before continuing.
struct DebuggerWrite {
    unsigned hit{1}; // 1-based arrival count at the breakpoint
    std::uint16_t xdata{0};
    std::vector<std::uint8_t> bytes;
    int line{0};
};

// Checkpoint report: on arrivals 1..hits the halted CPU state is appended to
// `file` in the `mcuemu run --report-hits` format (before that arrival's
// writes are applied).
struct DebuggerReport {
    std::string file; // resolved against the YAML file's directory
    unsigned hits{1};
    bool state{true}; // dump all IDATA and SFRs
    std::vector<std::pair<std::uint16_t, std::uint32_t>> xdata; // (start, length)
    int line{0};
};

struct DebuggerConfig {
    std::uint16_t pc{0}; // logical CODE address
    std::vector<DebuggerWrite> writes;
    std::optional<DebuggerReport> report;
    int line{0};
};

// Flash fault injection and persistence (cc2530f256 only; docs/system-config.md
// "Flash faults, power cuts and persistence"). A power cut is either
// scheduled at world time `at_ns` or triggered `after_ns` into the `nth`
// matching flash operation (`op` erase|write|any, optionally on `page`).
struct PowerCutConfig {
    std::optional<std::uint64_t> at_ns;
    std::string op{"any"};
    std::uint64_t nth{1};
    std::optional<unsigned> page;
    std::uint64_t after_ns{0};
    bool reset{false}; // action: reset (RESET_N) instead of a power cut
    int line{0};
};

struct FlashConfig {
    std::string limits{"report"};       // report | nondeterministic
    std::string interrupted{"abandon"}; // abandon | partial | complete
    std::uint64_t seed{0};
    std::optional<std::uint8_t> sram_fill{std::uint8_t{0}}; // nullopt: random
    std::string persist; // resolved against the YAML file's directory
    std::string log;     // resolved against the YAML file's directory
    std::vector<PowerCutConfig> power_cuts;
    int line{0};
};

struct McuConfig {
    std::string id;
    std::string model;
    std::optional<std::string> variant;
    std::string firmware; // resolved against the YAML file's directory
    std::optional<std::string> efuse; // PHY6252: external 32-byte, four-block dump
    std::optional<std::string> rom; // PHY6252: external 96 KiB ROM or 128 KiB dump
    bool boot_from_rom{false}; // PHY6252: start at the external ROM reset vector
    std::optional<std::string> rf_calibration; // PHY6252: external four-result RF TP calibration
    // Every image to load; `firmware: path` gives one image at offset 0, a
    // list of {file, offset} places several (bootloader + application).
    std::vector<FirmwareImage> images;
    // stm32wb55: independent Cortex-M0+ image(s) for CPU2.
    std::vector<FirmwareImage> cpu2_images;
    // stm32wb55: built-in CPU2 model without cpu2_images ("thread_rcp", the
    // default, or "st_mac_802154").
    std::optional<std::string> cpu2_model;
    std::optional<std::uint32_t> clock_hz;
    std::optional<std::uint64_t> eui64; // efr32mg21: the EUI-64 of the device information page (default 0x0123456789ABCDEF)
    std::optional<std::uint32_t> supply_mv; // supply voltage (default 3300)
    std::optional<bool> flash_timing; // tlsr8258: flash and register-bus timing (default on)
    // tlsr8258: the 16 MHz system timer ignores a START this soon after a STOP, plus or
    // minus a random jitter (default off); see Tlsr8258::set_system_timer_start_window().
    std::optional<std::uint64_t> tick_start_settle_ns;
    std::optional<std::uint64_t> tick_start_jitter_ns;
    std::optional<std::uint64_t> tick_start_seed;
    std::uint64_t start_at_ns{0};
    std::optional<RadioAttachConfig> radio;
    std::optional<RttConfig> rtt;
    std::optional<DebuggerConfig> debugger;
    std::optional<FlashConfig> flash;
    int line{0};
};

struct NetConfig {
    std::string id;
    NetKind kind{NetKind::PushPull};
    Drive pull{Drive::HiZ}; // HiZ, PullUp or PullDown
    // Pad references. Device scope: "mcu.PAD". World scope: "device.mcu.PAD".
    std::vector<std::string> pins;
    int line{0};
};

struct LinkConfig {
    std::string type; // "uart"
    // Endpoint references: "mcu.uartN" (device scope) or "device.mcu.uartN".
    std::string a;
    std::string b;
    std::uint32_t baud{115200};
    std::uint64_t latency_ns{0};
    int line{0};
};

// External peripheral (sensor, thermistor, ...). `pins` maps part terminals to
// nets of the same device (or full world net names); `params` are scalars
// interpreted by the part type.
struct PartConfig {
    std::string id;
    std::string type;
    std::map<std::string, std::string> pins;
    std::map<std::string, std::string> params;
    std::string script; // type "script": file resolved against the YAML dir
    std::optional<RadioAttachConfig> radio;
    int line{0};
};

struct DeviceConfig {
    std::string id;
    std::vector<McuConfig> mcus;
    std::vector<PartConfig> parts;
    std::vector<AttachConfig> attach;
    std::vector<NetConfig> nets;
    std::vector<LinkConfig> links;
    int line{0};
};

// External stimulus at a given time: either drive a net from outside the
// devices, or set an endpoint (for example a part parameter).
struct EventConfig {
    std::uint64_t at_ns{0};
    std::string net; // full net name ("device.net" or a world-level net)
    Drive drive{Drive::HiZ};
    std::string endpoint; // set when this event sets an endpoint instead
    std::string value;    // scalar text: true/false, integer, number or string
    int line{0};
};

struct SystemConfig {
    std::uint64_t seed{1};
    std::optional<std::uint64_t> duration_ns;
    std::optional<std::uint64_t> quantum_ns;
    std::vector<RadioMediumConfig> radios;
    std::vector<DeviceConfig> devices;
    std::vector<NetConfig> nets;   // world-level, full pad references
    std::vector<LinkConfig> links; // world-level, full endpoint references
    std::vector<AttachConfig> attach; // world-level, full endpoint names
    std::vector<EventConfig> events;
};

// "10ns", "5us", "100ms", "2.5s", "0".
std::uint64_t parse_duration_ns(const std::string &text);
// "48000000", "48MHz", "32kHz", "1Hz".
std::uint32_t parse_frequency_hz(const std::string &text);

// Parses and structurally validates a system description. Relative firmware
// paths are resolved against `base_dir`. Keys that belong to planned but not
// yet implemented features (parts, radio, attach, script) are rejected with a
// clear message rather than silently ignored.
//
// A `devices` entry may be `{include: file.yaml, id?, params?}`: the file holds
// one `device:` (optionally with default `params:`); `${name}` scalars inside
// it are replaced by the parameters. `reader` returns the text of an included
// file given its resolved path (default: read from disk); a hosting service
// uses it to confine the files that may be read.
using IncludeReader = std::function<std::string(const std::string &path)>;
SystemConfig parse_system_config(const std::string &yaml_text,
                                 const std::string &base_dir = ".",
                                 const IncludeReader &reader = {});
SystemConfig load_system_config(const std::string &path);

} // namespace sim
