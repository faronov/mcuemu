#include "sim/config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <map>
#include <set>
#include <sstream>

namespace sim {

namespace {

// Name of the included file being parsed, for error messages.
thread_local std::string current_file;

struct FileScope {
    std::string saved;
    explicit FileScope(const std::string &file) : saved(current_file) { current_file = file; }
    ~FileScope() { current_file = saved; }
};

[[noreturn]] void fail(const YAML::Node &node, const std::string &message) {
    std::ostringstream out;
    if (!current_file.empty())
        out << current_file << ": ";
    if (node.Mark().line >= 0)
        out << "line " << node.Mark().line + 1 << ": ";
    out << message;
    throw ConfigError(out.str());
}

std::string scalar(const YAML::Node &node, const std::string &what) {
    if (!node.IsScalar())
        fail(node, what + " must be a scalar");
    return node.Scalar();
}

const YAML::Node &require_map(const YAML::Node &node, const std::string &what) {
    if (!node.IsMap())
        fail(node, what + " must be a mapping");
    return node;
}

const YAML::Node &require_seq(const YAML::Node &node, const std::string &what) {
    if (!node.IsSequence())
        fail(node, what + " must be a list");
    return node;
}

// Rejects keys that are neither allowed nor reserved for a later feature.
void check_keys(const YAML::Node &map, const std::string &what,
                std::initializer_list<const char *> allowed,
                std::initializer_list<const char *> reserved = {}) {
    for (const auto &entry : map) {
        const std::string key = entry.first.Scalar();
        bool ok = false;
        for (const char *name : allowed)
            ok |= key == name;
        if (ok)
            continue;
        for (const char *name : reserved)
            if (key == name)
                fail(entry.first, "'" + key + "' in " + what +
                                      " is planned but not supported yet");
        fail(entry.first, "unknown key '" + key + "' in " + what);
    }
}

std::string required_string(const YAML::Node &map, const char *key,
                            const std::string &what) {
    const YAML::Node value = map[key];
    if (!value)
        fail(map, what + " requires '" + key + "'");
    return scalar(value, std::string("'") + key + "'");
}

void check_id(const YAML::Node &node, const std::string &id,
              const std::string &what) {
    if (id.empty())
        fail(node, what + " id must not be empty");
    for (char c : id)
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' ||
              c == '-'))
            fail(node, what + " id '" + id +
                           "' may only contain letters, digits, '_' and '-'");
}

Drive parse_drive(const YAML::Node &node) {
    const std::string text = scalar(node, "drive");
    if (text == "low") return Drive::Low;
    if (text == "high") return Drive::High;
    if (text == "hiz") return Drive::HiZ;
    if (text == "pullup") return Drive::PullUp;
    if (text == "pulldown") return Drive::PullDown;
    fail(node, "drive must be low, high, hiz, pullup or pulldown, got '" + text +
                   "'");
}

std::uint64_t config_duration(const YAML::Node &node, const std::string &what) {
    try {
        return parse_duration_ns(scalar(node, what));
    } catch (const std::invalid_argument &error) {
        fail(node, what + ": " + error.what());
    }
}

std::uint32_t config_frequency(const YAML::Node &node, const std::string &what) {
    try {
        return parse_frequency_hz(scalar(node, what));
    } catch (const std::invalid_argument &error) {
        fail(node, what + ": " + error.what());
    }
}

std::uint64_t config_uint(const YAML::Node &node, const std::string &what) {
    const std::string text = scalar(node, what);
    if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos)
        fail(node, what + " must be a non-negative integer, got '" + text + "'");
    try {
        return std::stoull(text);
    } catch (const std::exception &) {
        fail(node, what + " is out of range: " + text);
    }
}

double config_number(const YAML::Node &node, const std::string &what) {
    const std::string text = scalar(node, what);
    std::size_t used = 0;
    double value = 0;
    try {
        value = std::stod(text, &used);
    } catch (const std::exception &) {
        used = 0;
    }
    if (text.empty() || used != text.size() || !std::isfinite(value))
        fail(node, what + " must be a number, got '" + text + "'");
    return value;
}

std::uint64_t config_integer(const YAML::Node &node, const std::string &what) {
    const std::string text = scalar(node, what);
    std::size_t used = 0;
    unsigned long long value = 0;
    try {
        value = std::stoull(text, &used, 0); // decimal or 0x hex
    } catch (const std::exception &) {
        used = 0;
    }
    if (text.empty() || used != text.size() || text[0] == '-')
        fail(node, what + " must be a non-negative integer, got '" + text + "'");
    return value;
}

AttachConfig parse_attach(const YAML::Node &node) {
    require_map(node, "attach entry");
    check_keys(node, "attach entry", {"endpoint", "sink"});
    AttachConfig attach;
    attach.line = node.Mark().line + 1;
    attach.endpoint = required_string(node, "endpoint", "attach entry");
    attach.sink = required_string(node, "sink", "attach entry");
    return attach;
}

RttConfig parse_rtt(const YAML::Node &node) {
    require_map(node, "rtt");
    check_keys(node, "rtt", {"address", "channels", "interval"});
    RttConfig rtt;
    rtt.line = node.Mark().line + 1;
    if (!node["address"])
        fail(node, "rtt requires 'address' (the _SEGGER_RTT control block)");
    const std::uint64_t address = config_integer(node["address"], "address");
    if (address > 0xffffffffull)
        fail(node["address"], "address must fit 32 bits");
    rtt.address = static_cast<std::uint32_t>(address);
    if (node["channels"]) {
        const std::uint64_t channels = config_integer(node["channels"], "channels");
        if (channels < 1 || channels > 16)
            fail(node["channels"], "channels must be 1..16");
        rtt.channels = static_cast<unsigned>(channels);
    }
    if (node["interval"]) {
        rtt.interval_ns = config_duration(node["interval"], "interval");
        if (rtt.interval_ns == 0)
            fail(node["interval"], "interval must be positive");
    }
    return rtt;
}

DebuggerReport parse_debugger_report(const YAML::Node &node, const std::string &base_dir) {
    require_map(node, "debugger report");
    check_keys(node, "debugger report", {"file", "hits", "state", "xdata"});
    DebuggerReport report;
    report.line = node.Mark().line + 1;
    std::filesystem::path path(required_string(node, "file", "debugger report"));
    if (path.is_relative())
        path = std::filesystem::path(base_dir) / path;
    report.file = path.lexically_normal().string();
    if (const YAML::Node hits = node["hits"]) {
        const std::uint64_t value = config_integer(hits, "hits");
        if (value < 1 || value > 0xffffffffull)
            fail(hits, "hits must be a positive arrival count");
        report.hits = static_cast<unsigned>(value);
    }
    if (const YAML::Node state = node["state"]) {
        const std::string text = scalar(state, "state");
        if (text != "true" && text != "false")
            fail(state, "state must be true or false");
        report.state = text == "true";
    }
    if (const YAML::Node xdata = node["xdata"]) {
        if (!xdata.IsSequence())
            fail(xdata, "xdata must be a list of {address, length}");
        for (const YAML::Node &entry : xdata) {
            require_map(entry, "debugger report xdata range");
            check_keys(entry, "debugger report xdata range", {"address", "length"});
            if (!entry["address"] || !entry["length"])
                fail(entry, "debugger report xdata range requires 'address' and 'length'");
            const std::uint64_t address = config_integer(entry["address"], "address");
            const std::uint64_t length = config_integer(entry["length"], "length");
            if (length == 0 || address > 0xffff || address + length > 0x10000)
                fail(entry, "xdata range must be non-empty and lie within the 64 KiB XDATA space");
            report.xdata.emplace_back(static_cast<std::uint16_t>(address),
                                      static_cast<std::uint32_t>(length));
        }
    }
    return report;
}

DebuggerConfig parse_debugger(const YAML::Node &node, const std::string &base_dir) {
    require_map(node, "debugger");
    check_keys(node, "debugger", {"pc", "writes", "report"});
    DebuggerConfig debugger;
    debugger.line = node.Mark().line + 1;
    if (!node["pc"])
        fail(node, "debugger requires 'pc' (the logical CODE address to halt at)");
    const std::uint64_t pc = config_integer(node["pc"], "pc");
    if (pc > 0xffff)
        fail(node["pc"], "pc must be a 16-bit logical CODE address");
    debugger.pc = static_cast<std::uint16_t>(pc);
    if (const YAML::Node report = node["report"])
        debugger.report = parse_debugger_report(report, base_dir);
    const YAML::Node writes = node["writes"];
    if (!writes && debugger.report)
        return debugger;
    if (!writes || !writes.IsSequence() || writes.size() == 0)
        fail(node, "debugger requires a non-empty 'writes' list (or a 'report')");
    for (const YAML::Node &entry : writes) {
        require_map(entry, "debugger write");
        check_keys(entry, "debugger write", {"hit", "xdata", "hex"});
        DebuggerWrite write;
        write.line = entry.Mark().line + 1;
        if (const YAML::Node hit = entry["hit"]) {
            const std::uint64_t value = config_integer(hit, "hit");
            if (value < 1 || value > 0xffffffffull)
                fail(hit, "hit must be a positive arrival count");
            write.hit = static_cast<unsigned>(value);
        }
        if (!entry["xdata"])
            fail(entry, "debugger write requires 'xdata'");
        const std::uint64_t address = config_integer(entry["xdata"], "xdata");
        std::string text = required_string(entry, "hex", "debugger write");
        std::erase_if(text, [](char c) { return c == ' ' || c == '_'; });
        if (text.empty() || text.size() % 2 != 0)
            fail(entry["hex"], "hex must be a non-empty even number of hex digits");
        for (std::size_t i = 0; i < text.size(); i += 2) {
            const auto digit = [&](char c) -> int {
                if (c >= '0' && c <= '9') return c - '0';
                if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                fail(entry["hex"], "hex contains a non-hex character");
            };
            write.bytes.push_back(
                static_cast<std::uint8_t>(digit(text[i]) << 4 | digit(text[i + 1])));
        }
        if (address > 0xffff || address + write.bytes.size() > 0x10000)
            fail(entry["xdata"], "xdata range must lie within the 64 KiB XDATA space");
        write.xdata = static_cast<std::uint16_t>(address);
        debugger.writes.push_back(std::move(write));
    }
    return debugger;
}

RadioAttachConfig parse_radio_attach(const YAML::Node &node) {
    require_map(node, "radio");
    check_keys(node, "radio",
               {"air", "position", "tx_power_dbm", "sensitivity_dbm", "protocols"});
    RadioAttachConfig radio;
    radio.line = node.Mark().line + 1;
    radio.air = required_string(node, "air", "radio");
    if (const YAML::Node position = node["position"]) {
        require_seq(position, "'position'");
        if (position.size() != 3)
            fail(position, "'position' must be [x, y, z] in meters");
        for (std::size_t i = 0; i < 3; ++i)
            radio.position[i] = config_number(position[i], "position");
    }
    if (node["tx_power_dbm"])
        radio.tx_power_dbm = config_number(node["tx_power_dbm"], "tx_power_dbm");
    if (node["sensitivity_dbm"])
        radio.sensitivity_dbm =
            config_number(node["sensitivity_dbm"], "sensitivity_dbm");
    if (const YAML::Node protocols = node["protocols"]) {
        radio.protocols.clear();
        for (const YAML::Node &protocol : require_seq(protocols, "'protocols'")) {
            const std::string name = scalar(protocol, "protocol");
            if (name != "zigbee" && name != "ble")
                fail(protocol, "protocol must be zigbee or ble, got '" + name + "'");
            radio.protocols.push_back(name);
        }
        if (radio.protocols.empty())
            fail(protocols, "'protocols' must not be empty");
    }
    return radio;
}

RadioMediumConfig parse_radio_medium(const YAML::Node &node) {
    require_map(node, "radio medium");
    check_keys(node, "radio medium",
               {"id", "model", "reference_loss_db", "path_loss_exponent",
                "shadowing_sigma_db", "noise_floor_dbm", "sinr_threshold_db"});
    RadioMediumConfig medium;
    medium.line = node.Mark().line + 1;
    medium.id = required_string(node, "id", "radio medium");
    check_id(node["id"], medium.id, "radio medium");
    if (node["model"]) {
        medium.model = scalar(node["model"], "model");
        if (medium.model != "ideal" && medium.model != "log_distance")
            fail(node["model"], "radio model must be ideal or log_distance, got '" +
                                    medium.model + "'");
    }
    if (node["reference_loss_db"])
        medium.reference_loss_db =
            config_number(node["reference_loss_db"], "reference_loss_db");
    if (node["path_loss_exponent"]) {
        medium.exponent = config_number(node["path_loss_exponent"], "path_loss_exponent");
        if (medium.exponent <= 0)
            fail(node["path_loss_exponent"], "path_loss_exponent must be positive");
    }
    if (node["shadowing_sigma_db"]) {
        medium.shadowing_sigma_db =
            config_number(node["shadowing_sigma_db"], "shadowing_sigma_db");
        if (medium.shadowing_sigma_db < 0)
            fail(node["shadowing_sigma_db"], "shadowing_sigma_db must not be negative");
    }
    if (node["noise_floor_dbm"])
        medium.noise_floor_dbm = config_number(node["noise_floor_dbm"], "noise_floor_dbm");
    if (node["sinr_threshold_db"])
        medium.sinr_threshold_db =
            config_number(node["sinr_threshold_db"], "sinr_threshold_db");
    return medium;
}

NetConfig parse_net(const YAML::Node &node) {
    require_map(node, "net");
    check_keys(node, "net", {"id", "kind", "pull", "pull_up", "pins"});
    NetConfig net;
    net.line = node.Mark().line + 1;
    net.id = required_string(node, "id", "net");
    check_id(node["id"], net.id, "net");
    if (const YAML::Node kind = node["kind"]) {
        const std::string text = scalar(kind, "kind");
        if (text == "push_pull")
            net.kind = NetKind::PushPull;
        else if (text == "open_drain")
            net.kind = NetKind::OpenDrain;
        else
            fail(kind, "net kind must be push_pull or open_drain, got '" +
                           text + "'");
    }
    if (node["pull"] && node["pull_up"])
        fail(node, "net '" + net.id + "' sets both 'pull' and 'pull_up'");
    if (const YAML::Node pull = node["pull"]) {
        const std::string text = scalar(pull, "pull");
        if (text == "up") net.pull = Drive::PullUp;
        else if (text == "down") net.pull = Drive::PullDown;
        else if (text != "none")
            fail(pull, "pull must be up, down or none, got '" + text + "'");
    }
    // `pull_up: { ohm: 4700, to: vdd }` is accepted; the resistance is not
    // modeled yet (the pull is a weak driver).
    if (node["pull_up"])
        net.pull = Drive::PullUp;
    if (const YAML::Node pins = node["pins"]) {
        for (const YAML::Node &pin : require_seq(pins, "'pins'"))
            net.pins.push_back(scalar(pin, "pin reference"));
    }
    return net;
}

LinkConfig parse_link(const YAML::Node &node) {
    require_map(node, "link");
    check_keys(node, "link", {"type", "a", "b", "baud", "latency"});
    LinkConfig link;
    link.line = node.Mark().line + 1;
    link.type = required_string(node, "type", "link");
    if (link.type != "uart")
        fail(node["type"], "link type '" + link.type +
                               "' is not supported (only 'uart')");
    link.a = required_string(node, "a", "link");
    link.b = required_string(node, "b", "link");
    if (const YAML::Node baud = node["baud"]) {
        const std::uint64_t value = config_uint(baud, "baud");
        if (value == 0 || value > 0xffffffffu)
            fail(baud, "baud out of range");
        link.baud = static_cast<std::uint32_t>(value);
    }
    if (const YAML::Node latency = node["latency"])
        link.latency_ns = config_duration(latency, "latency");
    return link;
}

PartConfig parse_part(const YAML::Node &node, const std::string &base_dir) {
    require_map(node, "part");
    check_keys(node, "part", {"id", "type", "pins", "params", "script", "radio"});
    PartConfig part;
    part.line = node.Mark().line + 1;
    part.id = required_string(node, "id", "part");
    check_id(node["id"], part.id, "part");
    part.type = required_string(node, "type", "part");
    if (const YAML::Node radio = node["radio"])
        part.radio = parse_radio_attach(radio);
    if (const YAML::Node script = node["script"]) {
        std::filesystem::path path(scalar(script, "'script'"));
        if (path.is_relative())
            path = std::filesystem::path(base_dir) / path;
        part.script = path.lexically_normal().string();
    }
    if (const YAML::Node pins = node["pins"]) {
        require_map(pins, "'pins'");
        for (const auto &entry : pins)
            part.pins[scalar(entry.first, "terminal name")] =
                scalar(entry.second, "net reference");
    }
    if (const YAML::Node params = node["params"]) {
        require_map(params, "'params'");
        for (const auto &entry : params)
            part.params[scalar(entry.first, "param name")] =
                scalar(entry.second, "param value");
    }
    return part;
}

PowerCutConfig parse_power_cut(const YAML::Node &node) {
    require_map(node, "power cut");
    check_keys(node, "power cut", {"at", "op", "nth", "page", "after", "action"});
    PowerCutConfig cut;
    cut.line = node.Mark().line + 1;
    if (const YAML::Node at = node["at"]) {
        for (const char *key : {"op", "nth", "page", "after", "action"})
            if (node[key])
                fail(node[key], std::string("a power cut at a time takes no '") + key +
                                    "' (that is a flash-event trigger)");
        cut.at_ns = config_duration(at, "at");
        return cut;
    }
    if (!node["op"])
        fail(node, "power cut requires 'at' (a time) or 'op' (erase, write or any)");
    cut.op = scalar(node["op"], "op");
    if (cut.op != "erase" && cut.op != "write" && cut.op != "any")
        fail(node["op"], "op must be erase, write or any, got '" + cut.op + "'");
    if (const YAML::Node nth = node["nth"]) {
        cut.nth = config_integer(nth, "nth");
        if (cut.nth == 0)
            fail(nth, "nth counts operations from 1");
    }
    if (const YAML::Node page = node["page"]) {
        const std::uint64_t value = config_integer(page, "page");
        if (value > 127)
            fail(page, "page must be 0-127 (128 pages of 2 KiB)");
        cut.page = static_cast<unsigned>(value);
    }
    if (const YAML::Node after = node["after"])
        cut.after_ns = config_duration(after, "after");
    if (const YAML::Node action = node["action"]) {
        const std::string text = scalar(action, "action");
        if (text != "power_cut" && text != "reset")
            fail(action, "action must be power_cut or reset, got '" + text + "'");
        cut.reset = text == "reset";
    }
    return cut;
}

FlashConfig parse_flash(const YAML::Node &node, const std::string &base_dir) {
    require_map(node, "flash");
    check_keys(node, "flash",
               {"limits", "interrupted", "seed", "sram_fill", "persist", "log", "power_cuts"});
    FlashConfig flash;
    flash.line = node.Mark().line + 1;
    auto resolve = [&](const YAML::Node &value, const char *what) {
        std::filesystem::path path(scalar(value, what));
        if (path.empty())
            fail(value, std::string(what) + " must name a file");
        if (path.is_relative())
            path = std::filesystem::path(base_dir) / path;
        return path.lexically_normal().string();
    };
    if (const YAML::Node limits = node["limits"]) {
        flash.limits = scalar(limits, "limits");
        if (flash.limits != "report" && flash.limits != "nondeterministic")
            fail(limits, "limits must be report or nondeterministic, got '" + flash.limits + "'");
    }
    if (const YAML::Node interrupted = node["interrupted"]) {
        flash.interrupted = scalar(interrupted, "interrupted");
        if (flash.interrupted != "abandon" && flash.interrupted != "partial" &&
            flash.interrupted != "complete")
            fail(interrupted, "interrupted must be abandon, partial or complete, got '" +
                                  flash.interrupted + "'");
    }
    if (const YAML::Node seed = node["seed"])
        flash.seed = config_integer(seed, "seed");
    if (const YAML::Node fill = node["sram_fill"]) {
        if (scalar(fill, "sram_fill") == "random") {
            flash.sram_fill = std::nullopt;
        } else {
            const std::uint64_t value = config_integer(fill, "sram_fill");
            if (value > 0xff)
                fail(fill, "sram_fill must be a byte (0-255) or random");
            flash.sram_fill = static_cast<std::uint8_t>(value);
        }
    }
    if (const YAML::Node persist = node["persist"])
        flash.persist = resolve(persist, "persist");
    if (const YAML::Node log = node["log"])
        flash.log = resolve(log, "log");
    if (!flash.persist.empty() && flash.persist == flash.log)
        fail(node, "persist and log must be different files");
    if (const YAML::Node cuts = node["power_cuts"]) {
        if (!cuts.IsSequence())
            fail(cuts, "power_cuts must be a list");
        for (const YAML::Node &entry : cuts)
            flash.power_cuts.push_back(parse_power_cut(entry));
    }
    return flash;
}

McuConfig parse_mcu(const YAML::Node &node, const std::string &base_dir) {
    require_map(node, "mcu");
    check_keys(node, "mcu",
               {"id", "model", "variant", "firmware", "cpu2_firmware", "cpu2_model", "efuse", "rom", "boot_from_rom", "rf_calibration", "clock_hz", "supply_mv",
                "start_at", "radio", "rtt", "flash_timing", "eui64",
                "tick_start_settle", "tick_start_jitter", "tick_start_seed", "debugger", "flash"});
    McuConfig mcu;
    mcu.line = node.Mark().line + 1;
    mcu.id = required_string(node, "id", "mcu");
    check_id(node["id"], mcu.id, "mcu");
    mcu.model = required_string(node, "model", "mcu");
    if (const YAML::Node variant = node["variant"])
        mcu.variant = scalar(variant, "variant");
    if (const YAML::Node eui = node["eui64"])
        mcu.eui64 = config_integer(eui, "'eui64'");
    const YAML::Node firmware = node["firmware"];
    if (!firmware)
        fail(node, "mcu requires 'firmware'");
    auto resolve = [&](const std::string &file) {
        std::filesystem::path path(file);
        if (path.is_relative())
            path = std::filesystem::path(base_dir) / path;
        return path.lexically_normal().string();
    };
    if (firmware.IsSequence()) {
        for (const YAML::Node &entry : firmware) {
            require_map(entry, "firmware image");
            check_keys(entry, "a firmware image", {"file", "offset", "from", "size"});
            FirmwareImage image;
            image.file = resolve(required_string(entry, "file", "firmware image"));
            if (const YAML::Node offset = entry["offset"]) {
                const std::uint64_t value = config_integer(offset, "'offset'");
                if (value > 0xffffffffull)
                    fail(offset, "offset is out of range");
                image.offset = static_cast<std::uint32_t>(value);
            }
            for (const char *key : {"from", "size"}) {
                const YAML::Node field = entry[key];
                if (!field)
                    continue;
                const std::uint64_t value = config_integer(field, std::string("'") + key + "'");
                if (value > 0xffffffffull)
                    fail(field, std::string("'") + key + "' is out of range");
                (key[0] == 'f' ? image.skip : image.size) = static_cast<std::uint32_t>(value);
            }
            mcu.images.push_back(image);
        }
        if (mcu.images.empty())
            fail(firmware, "'firmware' lists no images");
        mcu.firmware = mcu.images.front().file;
    } else {
        mcu.firmware = resolve(scalar(firmware, "'firmware'"));
        mcu.images.push_back({mcu.firmware, 0});
    }
    if (const YAML::Node cpu2_firmware = node["cpu2_firmware"]) {
        if (mcu.model != "stm32wb55")
            fail(cpu2_firmware, "'cpu2_firmware' is only supported by stm32wb55");
        auto add_cpu2_image = [&](const YAML::Node &entry) {
            FirmwareImage image;
            if (entry.IsScalar()) {
                image.file = resolve(scalar(entry, "cpu2_firmware"));
            } else {
                require_map(entry, "CPU2 firmware image");
                check_keys(entry, "a CPU2 firmware image", {"file", "offset", "from", "size"});
                image.file = resolve(required_string(entry, "file", "CPU2 firmware image"));
                if (const YAML::Node offset = entry["offset"]) {
                    const std::uint64_t value = config_integer(offset, "CPU2 image 'offset'");
                    if (value > 0xffffffffull)
                        fail(offset, "offset is out of range");
                    image.offset = static_cast<std::uint32_t>(value);
                }
                for (const char *key : {"from", "size"}) {
                    const YAML::Node field = entry[key];
                    if (!field)
                        continue;
                    const std::uint64_t value = config_integer(field, std::string("'") + key + "'");
                    if (value > 0xffffffffull)
                        fail(field, std::string("'") + key + "' is out of range");
                    (key[0] == 'f' ? image.skip : image.size) = static_cast<std::uint32_t>(value);
                }
            }
            mcu.cpu2_images.push_back(std::move(image));
        };
        if (cpu2_firmware.IsSequence()) {
            for (const YAML::Node &entry : cpu2_firmware)
                add_cpu2_image(entry);
        } else {
            add_cpu2_image(cpu2_firmware);
        }
        if (mcu.cpu2_images.empty())
            fail(cpu2_firmware, "'cpu2_firmware' lists no images");
    }
    if (const YAML::Node cpu2_model = node["cpu2_model"]) {
        if (mcu.model != "stm32wb55")
            fail(cpu2_model, "'cpu2_model' is only supported by stm32wb55");
        mcu.cpu2_model = scalar(cpu2_model, "'cpu2_model'");
        if (*mcu.cpu2_model != "thread_rcp" && *mcu.cpu2_model != "st_mac_802154")
            fail(cpu2_model, "'cpu2_model' must be thread_rcp or st_mac_802154");
        if (!mcu.cpu2_images.empty())
            fail(cpu2_model, "'cpu2_model' selects a built-in CPU2 model; remove 'cpu2_firmware'");
    }
    if (const YAML::Node efuse = node["efuse"]) {
        if (mcu.model != "phy6252")
            fail(efuse, "'efuse' is only supported by phy6252");
        mcu.efuse = resolve(scalar(efuse, "'efuse'"));
    }
    if (const YAML::Node rom = node["rom"]) {
        if (mcu.model != "phy6252")
            fail(rom, "'rom' is only supported by phy6252");
        mcu.rom = resolve(scalar(rom, "'rom'"));
    }
    if (const YAML::Node boot_from_rom = node["boot_from_rom"]) {
        if (mcu.model != "phy6252")
            fail(boot_from_rom, "'boot_from_rom' is only supported by phy6252");
        const std::string text = scalar(boot_from_rom, "'boot_from_rom'");
        if (text != "true" && text != "false")
            fail(boot_from_rom, "boot_from_rom must be true or false");
        mcu.boot_from_rom = text == "true";
        if (mcu.boot_from_rom && !mcu.rom)
            fail(boot_from_rom, "'boot_from_rom: true' requires 'rom'");
    }
    if (const YAML::Node calibration = node["rf_calibration"]) {
        if (mcu.model != "phy6252")
            fail(calibration, "'rf_calibration' is only supported by phy6252");
        mcu.rf_calibration = resolve(scalar(calibration, "'rf_calibration'"));
    }
    if (const YAML::Node clock = node["clock_hz"])
        mcu.clock_hz = config_frequency(clock, "clock_hz");
    if (const YAML::Node supply = node["supply_mv"]) {
        const std::uint64_t value = config_uint(supply, "supply_mv");
        if (value < 1000 || value > 5500)
            fail(supply, "supply_mv must be between 1000 and 5500");
        mcu.supply_mv = static_cast<std::uint32_t>(value);
    }
    if (const YAML::Node timing = node["flash_timing"]) {
        const std::string text = timing.Scalar();
        if (text != "true" && text != "false")
            fail(timing, "flash_timing must be true or false");
        mcu.flash_timing = text == "true";
    }
    if (const YAML::Node settle = node["tick_start_settle"])
        mcu.tick_start_settle_ns = config_duration(settle, "tick_start_settle");
    if (const YAML::Node jitter = node["tick_start_jitter"])
        mcu.tick_start_jitter_ns = config_duration(jitter, "tick_start_jitter");
    if (const YAML::Node seed = node["tick_start_seed"])
        mcu.tick_start_seed = config_uint(seed, "tick_start_seed");
    if (mcu.tick_start_jitter_ns &&
        (!mcu.tick_start_settle_ns || *mcu.tick_start_jitter_ns > *mcu.tick_start_settle_ns))
        fail(node["tick_start_jitter"],
             "tick_start_jitter needs tick_start_settle and may not exceed it");
    if (const YAML::Node start = node["start_at"])
        mcu.start_at_ns = config_duration(start, "start_at");
    if (const YAML::Node radio = node["radio"])
        mcu.radio = parse_radio_attach(radio);
    if (const YAML::Node rtt = node["rtt"])
        mcu.rtt = parse_rtt(rtt);
    if (const YAML::Node debugger = node["debugger"])
        mcu.debugger = parse_debugger(debugger, base_dir);
    if (const YAML::Node flash = node["flash"])
        mcu.flash = parse_flash(flash, base_dir);
    return mcu;
}

DeviceConfig parse_device(const YAML::Node &node, const std::string &base_dir) {
    require_map(node, "device");
    check_keys(node, "device", {"id", "mcus", "nets", "links", "parts", "attach"});
    DeviceConfig device;
    device.line = node.Mark().line + 1;
    device.id = required_string(node, "id", "device");
    check_id(node["id"], device.id, "device");
    const YAML::Node mcus = node["mcus"];
    if (!mcus)
        fail(node, "device '" + device.id + "' requires 'mcus'");
    for (const YAML::Node &mcu : require_seq(mcus, "'mcus'"))
        device.mcus.push_back(parse_mcu(mcu, base_dir));
    if (device.mcus.empty())
        fail(mcus, "device '" + device.id + "' has no MCUs");
    if (const YAML::Node nets = node["nets"])
        for (const YAML::Node &net : require_seq(nets, "'nets'"))
            device.nets.push_back(parse_net(net));
    if (const YAML::Node links = node["links"])
        for (const YAML::Node &link : require_seq(links, "'links'"))
            device.links.push_back(parse_link(link));
    if (const YAML::Node parts = node["parts"])
        for (const YAML::Node &part : require_seq(parts, "'parts'"))
            device.parts.push_back(parse_part(part, base_dir));
    if (const YAML::Node attach = node["attach"])
        for (const YAML::Node &entry : require_seq(attach, "'attach'"))
            device.attach.push_back(parse_attach(entry));
    return device;
}

EventConfig parse_event(const YAML::Node &node) {
    require_map(node, "event");
    check_keys(node, "event", {"at", "net", "drive", "endpoint", "set"},
               {"target"});
    EventConfig event;
    event.line = node.Mark().line + 1;
    if (!node["at"])
        fail(node, "event requires 'at'");
    event.at_ns = config_duration(node["at"], "at");
    const bool drives_net = node["net"] || node["drive"];
    const bool sets_endpoint = node["endpoint"] || node["set"];
    if (drives_net == sets_endpoint)
        fail(node, "event needs either 'net' + 'drive' or 'endpoint' + 'set'");
    if (drives_net) {
        event.net = required_string(node, "net", "event");
        if (!node["drive"])
            fail(node, "event requires 'drive'");
        event.drive = parse_drive(node["drive"]);
    } else {
        event.endpoint = required_string(node, "endpoint", "event");
        if (!node["set"])
            fail(node, "event requires 'set'");
        event.value = scalar(node["set"], "set");
    }
    return event;
}

} // namespace

std::uint64_t parse_duration_ns(const std::string &text) {
    if (text == "0")
        return 0;
    struct Unit {
        const char *suffix;
        double scale;
    };
    static const Unit units[] = {{"ns", 1.0}, {"us", 1e3}, {"ms", 1e6}, {"s", 1e9}};
    for (const Unit &unit : units) {
        const std::string suffix = unit.suffix;
        if (text.size() <= suffix.size() ||
            text.compare(text.size() - suffix.size(), suffix.size(), suffix) != 0)
            continue;
        const std::string number = text.substr(0, text.size() - suffix.size());
        if (number.find_first_not_of("0123456789.") != std::string::npos ||
            std::count(number.begin(), number.end(), '.') > 1 || number == ".")
            break;
        const double value = std::stod(number) * unit.scale;
        if (value >= 1.8e19)
            throw std::invalid_argument("duration is too large: " + text);
        return static_cast<std::uint64_t>(std::llround(value));
    }
    throw std::invalid_argument("invalid duration '" + text +
                                "' (use e.g. 100ns, 5us, 10ms, 2.5s)");
}

std::uint32_t parse_frequency_hz(const std::string &text) {
    struct Unit {
        const char *suffix;
        double scale;
    };
    static const Unit units[] = {
        {"MHz", 1e6}, {"kHz", 1e3}, {"Hz", 1.0}, {"", 1.0}};
    for (const Unit &unit : units) {
        const std::string suffix = unit.suffix;
        if (text.size() <= suffix.size() ||
            text.compare(text.size() - suffix.size(), suffix.size(), suffix) != 0)
            continue;
        const std::string number = text.substr(0, text.size() - suffix.size());
        if (number.find_first_not_of("0123456789.") != std::string::npos ||
            std::count(number.begin(), number.end(), '.') > 1 || number == ".")
            continue;
        const double value = std::stod(number) * unit.scale;
        if (value < 1.0 || value > 4294967295.0)
            throw std::invalid_argument("frequency out of range: " + text);
        return static_cast<std::uint32_t>(std::llround(value));
    }
    throw std::invalid_argument("invalid frequency '" + text +
                                "' (use e.g. 48000000, 48MHz, 32kHz)");
}

namespace {

constexpr std::size_t MaxIncludeBytes = 1u << 20;

std::string read_include_file(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw ConfigError("cannot open included file: " + path);
    std::ostringstream text;
    text << input.rdbuf();
    return text.str();
}

// Replaces ${name} in every scalar below `node`, in place. A scalar that is
// exactly one reference takes the parameter's whole value (so lists and
// numbers keep their type); otherwise the parameter's text is spliced in.
void substitute(YAML::Node node, const std::map<std::string, YAML::Node> &params) {
    if (node.IsSequence()) {
        for (std::size_t i = 0; i < node.size(); ++i) {
            YAML::Node item = node[i];
            substitute(item, params);
        }
        return;
    }
    if (node.IsMap()) {
        for (auto entry : node)
            substitute(entry.second, params);
        return;
    }
    if (!node.IsScalar())
        return;
    const std::string text = node.Scalar();
    if (text.find("${") == std::string::npos)
        return;
    auto lookup = [&](const std::string &name) -> const YAML::Node & {
        const auto found = params.find(name);
        if (found == params.end())
            fail(node, "unknown parameter '" + name + "'");
        return found->second;
    };
    if (text.size() > 3 && text.compare(0, 2, "${") == 0 && text.back() == '}' &&
        text.find('}') == text.size() - 1) {
        const YAML::Node &value = lookup(text.substr(2, text.size() - 3));
        node = YAML::Clone(value);
        return;
    }
    std::string result;
    for (std::size_t pos = 0; pos < text.size();) {
        const std::size_t open = text.find("${", pos);
        if (open == std::string::npos) {
            result += text.substr(pos);
            break;
        }
        const std::size_t close = text.find('}', open);
        if (close == std::string::npos)
            fail(node, "unterminated '${' in '" + text + "'");
        result += text.substr(pos, open - pos);
        const YAML::Node &value = lookup(text.substr(open + 2, close - open - 2));
        if (!value.IsScalar())
            fail(node, "parameter '" + text.substr(open + 2, close - open - 2) +
                           "' is not a scalar and cannot be part of a string");
        result += value.Scalar();
        pos = close + 1;
    }
    node = result;
}

// Loads `{include, id?, params?}` and returns the parsed device.
DeviceConfig parse_include(const YAML::Node &entry, const std::string &base_dir,
                           const IncludeReader &reader) {
    check_keys(entry, "an include", {"include", "id", "params"});
    const std::string relative = scalar(entry["include"], "'include'");
    const std::filesystem::path resolved =
        (std::filesystem::path(base_dir) / relative).lexically_normal();
    std::string text;
    try {
        text = reader ? reader(resolved.string()) : read_include_file(resolved.string());
    } catch (const ConfigError &) {
        throw;
    } catch (const std::exception &error) {
        fail(entry["include"], error.what());
    }
    if (text.size() > MaxIncludeBytes)
        fail(entry["include"], "included file '" + relative + "' is larger than 1 MiB");

    const FileScope scope(relative);
    YAML::Node document;
    try {
        document = YAML::Load(text);
    } catch (const YAML::Exception &error) {
        std::ostringstream out;
        out << relative << ": line " << error.mark.line + 1 << ": " << error.msg;
        throw ConfigError(out.str());
    }
    if (!document.IsMap())
        throw ConfigError(relative + ": the document must be a mapping");
    check_keys(document, "a device file", {"version", "params", "device"});
    if (!document["version"] || scalar(document["version"], "'version'") != "1")
        fail(document, "a device file needs 'version: 1'");
    if (!document["device"])
        fail(document, "a device file needs 'device'");

    std::map<std::string, YAML::Node> params;
    if (const YAML::Node defaults = document["params"])
        for (const auto &item : require_map(defaults, "'params'"))
            params[item.first.Scalar()] = item.second;
    // The including document names the values; reject names the file lacks so
    // that a typo does not silently fall back to a default.
    std::set<std::string> known;
    for (const auto &item : params)
        known.insert(item.first);
    {
        const FileScope outer(""); // errors below are in the including file
        if (const YAML::Node given = entry["params"])
            for (const auto &item : require_map(given, "'params'")) {
                if (!known.count(item.first.Scalar()))
                    fail(item.first, "unknown parameter '" + item.first.Scalar() +
                                         "' for " + relative);
                params[item.first.Scalar()] = item.second;
            }
    }
    YAML::Node device = document["device"];
    require_map(device, "'device'");
    substitute(device, params);
    if (entry["id"])
        device["id"] = scalar(entry["id"], "'id'");
    DeviceConfig parsed = parse_device(
        device, std::filesystem::path(resolved).parent_path().string());
    parsed.line = entry.Mark().line + 1; // duplicates are reported at the include
    return parsed;
}

} // namespace

SystemConfig parse_system_config(const std::string &yaml_text,
                                 const std::string &base_dir,
                                 const IncludeReader &reader) {
    YAML::Node root;
    try {
        root = YAML::Load(yaml_text);
    } catch (const YAML::Exception &error) {
        std::ostringstream out;
        out << "line " << error.mark.line + 1 << ": " << error.msg;
        throw ConfigError(out.str());
    }
    if (!root.IsMap())
        throw ConfigError("the document must be a mapping");
    check_keys(root, "the document",
               {"version", "world", "devices", "nets", "links", "scenario",
                "attach"});

    const YAML::Node version = root["version"];
    if (!version)
        throw ConfigError("'version' is required (currently 1)");
    if (scalar(version, "'version'") != "1")
        fail(version, "unsupported version '" + version.Scalar() +
                          "' (only 1)");

    SystemConfig config;
    if (const YAML::Node world = root["world"]) {
        require_map(world, "'world'");
        check_keys(world, "'world'", {"seed", "duration", "quantum", "radio"});
        if (world["seed"])
            config.seed = config_uint(world["seed"], "seed");
        if (world["duration"])
            config.duration_ns = config_duration(world["duration"], "duration");
        if (const YAML::Node radios = world["radio"])
            for (const YAML::Node &radio : require_seq(radios, "'radio'"))
                config.radios.push_back(parse_radio_medium(radio));
        if (world["quantum"]) {
            config.quantum_ns = config_duration(world["quantum"], "quantum");
            if (*config.quantum_ns == 0)
                fail(world["quantum"], "quantum must be positive");
        }
    }

    const YAML::Node devices = root["devices"];
    if (!devices)
        throw ConfigError("'devices' is required");
    for (const YAML::Node &device : require_seq(devices, "'devices'")) {
        if (device.IsMap() && device["include"])
            config.devices.push_back(parse_include(device, base_dir, reader));
        else
            config.devices.push_back(parse_device(device, base_dir));
    }
    if (config.devices.empty())
        fail(devices, "'devices' is empty");
    if (const YAML::Node nets = root["nets"])
        for (const YAML::Node &net : require_seq(nets, "'nets'"))
            config.nets.push_back(parse_net(net));
    if (const YAML::Node links = root["links"])
        for (const YAML::Node &link : require_seq(links, "'links'"))
            config.links.push_back(parse_link(link));
    if (const YAML::Node attach = root["attach"])
        for (const YAML::Node &entry : require_seq(attach, "'attach'"))
            config.attach.push_back(parse_attach(entry));
    if (const YAML::Node scenario = root["scenario"]) {
        require_map(scenario, "'scenario'");
        check_keys(scenario, "'scenario'", {"events"}, {"script"});
        if (const YAML::Node events = scenario["events"])
            for (const YAML::Node &event : require_seq(events, "'events'"))
                config.events.push_back(parse_event(event));
    }

    std::set<std::string> radio_ids;
    for (const RadioMediumConfig &medium : config.radios)
        if (!radio_ids.insert(medium.id).second)
            throw ConfigError("line " + std::to_string(medium.line) +
                              ": duplicate radio medium id '" + medium.id + "'");
    // Identifier uniqueness is a property of the document, not of the chips.
    std::set<std::string> device_ids;
    for (const DeviceConfig &device : config.devices) {
        if (!device_ids.insert(device.id).second)
            throw ConfigError("line " + std::to_string(device.line) +
                              ": duplicate device id '" + device.id + "'");
        std::set<std::string> mcu_ids;
        for (const McuConfig &mcu : device.mcus)
            if (!mcu_ids.insert(mcu.id).second)
                throw ConfigError("line " + std::to_string(mcu.line) +
                                  ": duplicate mcu id '" + mcu.id +
                                  "' in device '" + device.id + "'");
        std::set<std::string> part_ids;
        for (const PartConfig &part : device.parts)
            if (!part_ids.insert(part.id).second || mcu_ids.count(part.id))
                throw ConfigError("line " + std::to_string(part.line) +
                                  ": duplicate part id '" + part.id +
                                  "' in device '" + device.id + "'");
    }
    return config;
}

SystemConfig load_system_config(const std::string &path) {
    std::ifstream input(path);
    if (!input)
        throw ConfigError("cannot open configuration: " + path);
    std::ostringstream text;
    text << input.rdbuf();
    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    return parse_system_config(text.str(),
                               parent.empty() ? "." : parent.string());
}

} // namespace sim
