#include "sim/system.hpp"

#include "sim/bl702_mcu.hpp"
#include "sim/ca51f253l3_mcu.hpp"
#include "sim/cc2530f256_mcu.hpp"
#include "sim/efr32mg21_mcu.hpp"
#include "sim/esp32_mcu.hpp"
#include "sim/nrf52840_mcu.hpp"
#include "sim/phy6252_mcu.hpp"
#include "sim/stm32wb55_mcu.hpp"
#include "sim/tlsr8258_mcu.hpp"
#include "sim/tlsr826x_mcu.hpp"

#include <functional>
#include <tuple>
#include <algorithm>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iostream>
#include <iterator>
#include <set>
#include <sstream>

namespace sim {

namespace {
std::uint32_t phy6252_random_seed(std::uint64_t world_seed,
                                 const std::string &instance_name) {
    std::uint64_t hash = 1469598103934665603ull ^ world_seed;
    for (unsigned char byte : instance_name)
        hash = (hash ^ byte) * 1099511628211ull;
    const std::uint32_t seed = static_cast<std::uint32_t>(hash ^ (hash >> 32));
    return seed == 0 ? 0x6d2b79f5u : seed;
}
} // namespace

void McuRegistry::add(std::string model, Factory factory) {
    factories_[std::move(model)] = std::move(factory);
}

const McuRegistry::Factory *McuRegistry::find(const std::string &model) const {
    const auto it = factories_.find(model);
    return it == factories_.end() ? nullptr : &it->second;
}

std::vector<std::string> McuRegistry::models() const {
    std::vector<std::string> names;
    for (const auto &entry : factories_)
        names.push_back(entry.first);
    return names;
}

const McuRegistry &McuRegistry::builtin() {
    static const McuRegistry registry = [] {
        McuRegistry r;
        r.add("tlsr8258", [](const McuConfig &config, const std::string &name) {
            mcuemu::Tlsr8258Model model = mcuemu::Tlsr8258Model::F512;
            if (config.variant) {
                try {
                    model = mcuemu::parse_model(*config.variant);
                } catch (const std::invalid_argument &) {
                    throw std::invalid_argument(
                        "unknown tlsr8258 variant '" + *config.variant +
                        "' (use F512 or F1K)");
                }
            }
            auto mcu = std::make_unique<Tlsr8258Mcu>(
                name, model, config.clock_hz.value_or(48000000));
            if (config.supply_mv)
                mcu->device().set_supply_mv(*config.supply_mv);
            mcu->cpu().set_flash_timing(config.flash_timing.value_or(true));
            if (config.tick_start_settle_ns)
                mcu->device().set_system_timer_start_window(
                    *config.tick_start_settle_ns, config.tick_start_jitter_ns.value_or(0),
                    // Each device draws its own sequence.
                    config.tick_start_seed.value_or(1) ^ std::hash<std::string>{}(name));
            return mcu;
        });
        for (const char *chip : {"tlsr8269", "tlsr8267"}) {
            r.add(chip, [chip](const McuConfig &config, const std::string &name) {
                if (config.variant)
                    throw std::invalid_argument(std::string(chip) + " has no variants");
                auto mcu = std::make_unique<Tlsr826xMcu>(
                    name, mcuemu::parse_tlsr826x_model(chip),
                    config.clock_hz.value_or(mcuemu::Tlsr826x::DefaultClockHz));
                // INFERRED: the 826x has the same instruction cache in front of the flash
                // (cstartup/link script reserve its tag RAM), so the 8258 costs are used.
                mcu->cpu().set_flash_timing(config.flash_timing.value_or(true));
                return mcu;
            });
        }
        r.add("ca51f253l3", [](const McuConfig &config, const std::string &name) {
            if (config.variant)
                throw std::invalid_argument(
                    "ca51f253l3 has no variants; remove 'variant'");
            if (config.supply_mv)
                throw std::invalid_argument(
                    "ca51f253l3 does not model its supply; remove 'supply_mv'");
            if (config.flash_timing)
                throw std::invalid_argument("flash_timing is only modelled for tlsr8258");
            return std::make_unique<Ca51F253L3Mcu>(
                name, config.clock_hz.value_or(24000000));
        });
        r.add("cc2530f256", [](const McuConfig &config, const std::string &name) {
            if (config.variant)
                throw std::invalid_argument(
                    "cc2530f256 has no variants; remove 'variant'");
            // SWRS081B: all supply pins at the same voltage (AVDD5 = DVDD),
            // operating range 2 V - 3.6 V; default matches the nets' 3300 mV.
            const std::uint32_t supply = config.supply_mv.value_or(3300);
            if (supply < 2000 || supply > 3600)
                throw std::invalid_argument(
                    "cc2530f256 supply_mv must be between 2000 and 3600 (SWRS081B operating range)");
            auto mcu = std::make_unique<Cc2530F256Mcu>(
                name, config.clock_hz.value_or(32000000));
            mcu->device().set_vdd_mv(supply);
            if (config.debugger) {
                std::vector<Cc2530F256Mcu::DebuggerWrite> writes;
                for (const DebuggerWrite &write : config.debugger->writes)
                    writes.push_back({write.hit, write.xdata, write.bytes});
                std::optional<Cc2530F256Mcu::DebuggerReport> report;
                if (const auto &r = config.debugger->report)
                    report = Cc2530F256Mcu::DebuggerReport{r->file, r->hits, {r->state, r->xdata}};
                mcu->set_debugger(config.debugger->pc, std::move(writes), std::move(report));
            }
            if (config.flash) {
                const FlashConfig &flash = *config.flash;
                Cc2530F256Mcu::FlashSetup setup;
                setup.fault.limit_policy = flash.limits == "nondeterministic"
                                               ? mcs51emu::Cc2530FlashLimitPolicy::Nondeterministic
                                               : mcs51emu::Cc2530FlashLimitPolicy::Report;
                setup.fault.interrupted =
                    flash.interrupted == "partial"    ? mcs51emu::Cc2530FlashInterrupted::Partial
                    : flash.interrupted == "complete" ? mcs51emu::Cc2530FlashInterrupted::Complete
                                                      : mcs51emu::Cc2530FlashInterrupted::Abandon;
                setup.fault.seed = flash.seed;
                setup.fault.sram_fill = flash.sram_fill;
                for (const PowerCutConfig &cut : flash.power_cuts) {
                    if (cut.at_ns) {
                        // World time; the chip counts cycles from its power-on.
                        if (*cut.at_ns < config.start_at_ns)
                            throw std::invalid_argument(
                                "a power cut at " + std::to_string(*cut.at_ns) +
                                " ns comes before the MCU starts (start_at)");
                        setup.cut_cycles.push_back(
                            mcu->ns_to_cycles(*cut.at_ns - config.start_at_ns));
                        continue;
                    }
                    mcs51emu::Cc2530FlashTrigger trigger;
                    trigger.op = cut.op == "erase"   ? mcs51emu::Cc2530FlashOpFilter::Erase
                                 : cut.op == "write" ? mcs51emu::Cc2530FlashOpFilter::Write
                                                     : mcs51emu::Cc2530FlashOpFilter::Any;
                    trigger.nth = cut.nth;
                    trigger.page = cut.page;
                    trigger.delay_cycles = mcu->ns_to_cycles(cut.after_ns);
                    trigger.action = cut.reset ? mcs51emu::Cc2530FaultAction::Reset
                                               : mcs51emu::Cc2530FaultAction::PowerCut;
                    setup.triggers.push_back(trigger);
                }
                setup.persist = flash.persist;
                setup.log = flash.log;
                mcu->set_flash(std::move(setup));
            }
            return mcu;
        });
        r.add("nrf52840", [](const McuConfig &config, const std::string &name) {
            if (config.variant)
                throw std::invalid_argument("nrf52840 has no variants; remove 'variant'");
            if (config.clock_hz)
                throw std::invalid_argument(
                    "nrf52840 runs at a fixed 64 MHz; remove 'clock_hz'");
            if (config.flash_timing)
                throw std::invalid_argument("flash_timing is only modelled for tlsr8258");
            auto mcu = std::make_unique<Nrf52840Mcu>(name);
            if (config.supply_mv)
                mcu->chip().set_supply_mv(*config.supply_mv);
            return mcu;
        });
        r.add("efr32mg21", [](const McuConfig &config, const std::string &name) {
            if (config.variant)
                throw std::invalid_argument("efr32mg21 has no variants; remove 'variant'");
            if (config.clock_hz)
                throw std::invalid_argument(
                    "efr32mg21 runs at a fixed 38.4 MHz; remove 'clock_hz'");
            if (config.supply_mv)
                throw std::invalid_argument(
                    "efr32mg21 does not model its supply; remove 'supply_mv'");
            if (config.flash_timing)
                throw std::invalid_argument("flash_timing is only modelled for tlsr8258");
            auto mcu = std::make_unique<Efr32mg21Mcu>(name);
            if (config.eui64)
                mcu->chip().set_device_id(*config.eui64);
            return mcu;
        });
        r.add("stm32wb55", [](const McuConfig &config, const std::string &name) {
            if (config.variant)
                throw std::invalid_argument("stm32wb55 has no variants; remove 'variant'");
            if (config.clock_hz)
                throw std::invalid_argument("stm32wb55 runs at a fixed 64 MHz; remove 'clock_hz'");
            if (config.supply_mv)
                throw std::invalid_argument("stm32wb55 does not model its supply; remove 'supply_mv'");
            if (config.flash_timing)
                throw std::invalid_argument("flash_timing is only modelled for tlsr8258");
            auto mcu = std::make_unique<Stm32wb55Mcu>(name);
            if (config.eui64)
                mcu->chip().set_device_id(*config.eui64);
            if (config.cpu2_model && *config.cpu2_model == "st_mac_802154")
                mcu->chip().set_cpu2_model(stm32wb55emu::Stm32wb55::Cpu2Model::StMac802154);
            return mcu;
        });
        for (const auto &[model, variant, mhz] :
             {std::tuple{"esp32c6", esp32emu::Variant::C6, "160"}, std::tuple{"esp32h2", esp32emu::Variant::H2, "96"}}) {
            r.add(model, [model = std::string(model), variant, mhz = std::string(mhz)](const McuConfig &config,
                                                                                         const std::string &name) {
                if (config.variant)
                    throw std::invalid_argument(model + " has no variants; remove 'variant'");
                if (config.clock_hz)
                    throw std::invalid_argument(model + " runs at a fixed " + mhz + " MHz; remove 'clock_hz'");
                if (config.supply_mv)
                    throw std::invalid_argument(model + " does not model its supply; remove 'supply_mv'");
                if (config.flash_timing)
                    throw std::invalid_argument("flash_timing is only modelled for tlsr8258");
                return std::make_unique<Esp32Mcu>(name, variant);
            });
        }
        r.add("bl702", [](const McuConfig &config, const std::string &name) {
            if (config.variant)
                throw std::invalid_argument("bl702 has no variants; remove 'variant'");
            if (config.clock_hz)
                throw std::invalid_argument("bl702 runs at a fixed 32 MHz; remove 'clock_hz'");
            if (config.supply_mv)
                throw std::invalid_argument("bl702 does not model its supply; remove 'supply_mv'");
            if (config.flash_timing)
                throw std::invalid_argument("flash_timing is only modelled for tlsr8258");
            return std::make_unique<Bl702Mcu>(name);
        });
        r.add("phy6252", [](const McuConfig &config, const std::string &name) {
            if (config.variant || config.clock_hz || config.supply_mv || config.flash_timing)
                throw std::invalid_argument("phy6252 does not accept variant, clock_hz, supply_mv or flash_timing");
            auto mcu = std::make_unique<Phy6252Mcu>(name);
            if (config.rom) {
                std::ifstream input(*config.rom, std::ios::binary);
                if (!input)
                    throw std::invalid_argument("cannot open PHY6252 ROM dump: " + *config.rom);
                const std::vector<std::uint8_t> dump(
                    (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
                mcu->chip().load_rom_dump(dump);
            }
            mcu->chip().set_boot_from_rom(config.boot_from_rom);
            if (config.efuse) {
                std::ifstream input(*config.efuse, std::ios::binary);
                if (!input)
                    throw std::invalid_argument("cannot open eFuse dump: " + *config.efuse);
                const std::vector<std::uint8_t> dump(
                    (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
                mcu->chip().load_efuse_dump(dump);
            }
            if (config.rf_calibration) {
                std::ifstream input(*config.rf_calibration, std::ios::binary);
                if (!input)
                    throw std::invalid_argument("cannot open RF calibration dump: " + *config.rf_calibration);
                const std::vector<std::uint8_t> dump(
                    (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
                mcu->chip().load_rf_calibration_dump(dump);
            }
            return mcu;
        });
        return r;
    }();
    return registry;
}

void System::drive_net(const std::string &name, Drive drive) {
    Net &net = world_.net(name);
    Pin *&pin = scenario_pins_[name];
    if (!pin)
        pin = &net.attach("external");
    pin->set_drive(drive, world_.now());
}

System::System() = default;
System::~System() = default;

namespace {

[[noreturn]] void fail_at(int line, const std::string &message) {
    throw ConfigError("line " + std::to_string(line) + ": " + message);
}

// Splits "a.b.c" at '.'; pad names such as "P0.1" are handled by the caller
// through the fixed prefix length (device.mcu.PAD keeps the remainder whole).
std::vector<std::string> split_ref(const std::string &ref, std::size_t max_parts) {
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (parts.size() + 1 < max_parts) {
        const std::size_t dot = ref.find('.', start);
        if (dot == std::string::npos)
            break;
        parts.push_back(ref.substr(start, dot - start));
        start = dot + 1;
    }
    parts.push_back(ref.substr(start));
    return parts;
}

struct Scope {
    const std::string *device; // null at world scope
};

// Resolves "mcu.X" (device scope) or "device.mcu.X" (world scope) into the
// MCU and the trailing name X.
std::pair<Mcu *, std::string> resolve_mcu_ref(World &world, const Scope &scope,
                                              const std::string &ref, int line) {
    const std::size_t prefix = scope.device ? 2 : 3;
    const std::vector<std::string> parts = split_ref(ref, prefix);
    if (parts.size() != prefix || parts.back().empty())
        fail_at(line, "reference '" + ref + "' must look like " +
                          (scope.device ? "mcu.NAME" : "device.mcu.NAME"));
    const std::string mcu_name =
        scope.device ? *scope.device + "." + parts[0] : parts[0] + "." + parts[1];
    try {
        return {&world.mcu(mcu_name), parts.back()};
    } catch (const std::out_of_range &) {
        fail_at(line, "unknown mcu '" + mcu_name + "' in reference '" + ref + "'");
    }
}

void build_net(World &world, const Scope &scope, const NetConfig &config) {
    const std::string name =
        scope.device ? *scope.device + "." + config.id : config.id;
    Net *net = nullptr;
    try {
        net = &world.add_net(name, config.kind);
    } catch (const std::invalid_argument &) {
        fail_at(config.line, "duplicate net '" + name + "'");
    }
    if (config.pull != Drive::HiZ)
        net->set_pull(config.pull);
    for (const std::string &ref : config.pins) {
        auto [mcu, pad] = resolve_mcu_ref(world, scope, ref, config.line);
        try {
            world.connect(*mcu, pad, *net);
        } catch (const std::invalid_argument &error) {
            fail_at(config.line, "net '" + name + "': " + error.what());
        }
    }
}

unsigned parse_uart_index(const std::string &name, int line,
                          const std::string &ref) {
    if (name.size() < 5 || name.compare(0, 4, "uart") != 0 ||
        name.find_first_not_of("0123456789", 4) != std::string::npos)
        fail_at(line, "'" + ref + "' is not a UART (expected ...uartN)");
    return static_cast<unsigned>(std::stoul(name.substr(4)));
}

} // namespace

namespace {

mcuemu::RadioNodeConfig node_config(const RadioAttachConfig &radio) {
    mcuemu::RadioNodeConfig node;
    node.x = radio.position[0];
    node.y = radio.position[1];
    node.z = radio.position[2];
    node.tx_power_dbm = radio.tx_power_dbm;
    node.sensitivity_dbm = radio.sensitivity_dbm;
    return node;
}

Value parse_scalar_value(const std::string &text) {
    if (text == "true")
        return Value{true};
    if (text == "false")
        return Value{false};
    std::size_t used = 0;
    try {
        const long long integer = std::stoll(text, &used, 0);
        if (used == text.size())
            return Value{static_cast<std::int64_t>(integer)};
    } catch (const std::exception &) {
    }
    try {
        const double number = std::stod(text, &used);
        if (used == text.size())
            return Value{number};
    } catch (const std::exception &) {
    }
    return Value{text};
}

void build_part(World &world, std::vector<std::unique_ptr<Part>> &parts,
                const std::string &device_id, const PartConfig &config,
                const PartRegistry &registry, std::uint64_t seed) {
    const PartRegistry::Factory *factory = registry.find(config.type);
    if (!factory) {
        std::ostringstream types;
        for (const std::string &type : registry.types())
            types << (types.tellp() ? ", " : "") << type;
        fail_at(config.line, "unknown part type '" + config.type + "' (known: " +
                                 types.str() + ")");
    }
    const std::string name = device_id + "." + config.id;
    std::set<std::string> used_terminals;
    auto pin_provider = [&](const std::string &terminal) -> Pin & {
        const auto it = config.pins.find(terminal);
        if (it == config.pins.end())
            throw std::invalid_argument("terminal '" + terminal +
                                        "' is not wired (add it under 'pins')");
        used_terminals.insert(terminal);
        // A net of the same device wins over a world-level net of that name.
        const std::string local = device_id + "." + it->second;
        Net *net = nullptr;
        for (const std::string &candidate : {local, it->second}) {
            try {
                net = &world.net(candidate);
                break;
            } catch (const std::out_of_range &) {
            }
        }
        if (!net)
            throw std::invalid_argument("terminal '" + terminal +
                                        "' refers to unknown net '" + it->second + "'");
        return net->attach(name + "." + terminal);
    };
    std::vector<std::string> terminals;
    for (const auto &entry : config.pins)
        terminals.push_back(entry.first);
    if (!config.script.empty() && config.type != "script")
        fail_at(config.line, "part '" + name + "': only type 'script' takes 'script'");
    if (config.type == "script" && config.script.empty())
        fail_at(config.line, "part '" + name + "': type 'script' requires 'script'");
    PartContext context(name, world.scheduler(), config.params, pin_provider,
                        terminals, config.script, seed);
    if (config.radio) {
        mcuemu::VirtualAir *air = nullptr;
        try {
            air = &world.radio(config.radio->air);
        } catch (const std::out_of_range &) {
            fail_at(config.radio->line, "part '" + name + "': unknown radio medium '" +
                                            config.radio->air + "'");
        }
        air->set_node(name, node_config(*config.radio));
        context.set_radio(air, name, config.radio->protocols);
    }
    std::unique_ptr<Part> part;
    try {
        part = (*factory)(context);
        context.params().finish();
        for (const auto &entry : config.pins)
            if (!used_terminals.count(entry.first))
                throw std::invalid_argument("part type '" + config.type +
                                            "' has no terminal '" + entry.first + "'");
        World *w = &world;
        part->register_endpoints(world.endpoints(),
                                 [w] { return w->scheduler().now(); });
    } catch (const std::exception &error) {
        fail_at(config.line, "part '" + name + "': " + error.what());
    }
    parts.push_back(std::move(part));
}

} // namespace

std::unique_ptr<System> build_system(const SystemConfig &config,
                                     const McuRegistry &registry,
                                     const PartRegistry &part_registry) {
    auto system = std::make_unique<System>();
    World &world = system->world_;
    system->duration_ns_ = config.duration_ns;
    if (config.quantum_ns)
        world.set_quantum_ns(*config.quantum_ns);

    for (const RadioMediumConfig &medium : config.radios) {
        mcuemu::VirtualAir *air = nullptr;
        try {
            air = &world.add_radio(medium.id);
        } catch (const std::invalid_argument &error) {
            fail_at(medium.line, error.what());
        }
        mcuemu::Propagation propagation;
        propagation.model = medium.model == "ideal"
                                ? mcuemu::Propagation::Model::Ideal
                                : mcuemu::Propagation::Model::LogDistance;
        propagation.reference_loss_db = medium.reference_loss_db;
        propagation.exponent = medium.exponent;
        propagation.shadowing_sigma_db = medium.shadowing_sigma_db;
        propagation.noise_floor_dbm = medium.noise_floor_dbm;
        propagation.sinr_threshold_db = medium.sinr_threshold_db;
        propagation.seed = config.seed;
        air->set_propagation(propagation);
    }

    // MCUs first, so nets and links can refer to any of them.
    for (const DeviceConfig &device : config.devices) {
        for (const McuConfig &mcu_config : device.mcus) {
            const McuRegistry::Factory *factory = registry.find(mcu_config.model);
            if (!factory) {
                std::ostringstream models;
                for (const std::string &model : registry.models())
                    models << (models.tellp() ? ", " : "") << model;
                fail_at(mcu_config.line, "unknown mcu model '" +
                                             mcu_config.model + "' (known: " +
                                             models.str() + ")");
            }
            if (mcu_config.model != "tlsr8258" &&
                (mcu_config.tick_start_settle_ns || mcu_config.tick_start_jitter_ns ||
                 mcu_config.tick_start_seed))
                fail_at(mcu_config.line, "tick_start_* is only modelled for tlsr8258");
            const std::string name = device.id + "." + mcu_config.id;
            if (mcu_config.debugger && mcu_config.model != "cc2530f256")
                fail_at(mcu_config.debugger->line,
                        "mcu '" + name + "': 'debugger' is only supported for cc2530f256");
            if (mcu_config.flash && mcu_config.model != "cc2530f256")
                fail_at(mcu_config.flash->line,
                        "mcu '" + name + "': 'flash' is only supported for cc2530f256");
            std::unique_ptr<Mcu> mcu;
            try {
                mcu = (*factory)(mcu_config, name);
                if (auto *phy6252 = dynamic_cast<Phy6252Mcu *>(mcu.get()))
                    phy6252->chip().set_random_seed(
                        phy6252_random_seed(config.seed, name));
                mcu->load_images(mcu_config.images);
                if (auto *stm32wb55 = dynamic_cast<Stm32wb55Mcu *>(mcu.get()))
                    stm32wb55->load_cpu2_images(mcu_config.cpu2_images);
            } catch (const std::exception &error) {
                fail_at(mcu_config.line, "mcu '" + name + "': " + error.what());
            }
            if (mcu_config.radio) {
                mcuemu::VirtualAir *air = nullptr;
                try {
                    air = &world.radio(mcu_config.radio->air);
                } catch (const std::out_of_range &) {
                    fail_at(mcu_config.radio->line,
                            "mcu '" + name + "': unknown radio medium '" +
                                mcu_config.radio->air + "'");
                }
                try {
                    air->set_node(name, node_config(*mcu_config.radio));
                    mcu->attach_radio(*air, name, mcu_config.radio->protocols);
                } catch (const std::exception &error) {
                    fail_at(mcu_config.radio->line,
                            "mcu '" + name + "': " + error.what());
                }
            }
            Mcu &added = world.add_mcu(std::move(mcu), mcu_config.start_at_ns);
            if (mcu_config.flash) {
                // Only with `flash:`, so other systems keep their endpoint set.
                World *owner = &world;
                world.endpoints().add(
                    name + ".power",
                    std::make_unique<Cc2530PowerEndpoint>(
                        name + ".power", static_cast<Cc2530F256Mcu &>(added),
                        [owner] { return owner->now(); }));
            }
            try {
                for (unsigned i = 0; i < added.uart_count(); ++i) {
                    const std::string uart_name = name + ".uart" + std::to_string(i);
                    auto uart = std::make_unique<UartEndpoint>(uart_name, added, i);
                    UartEndpoint &bytes = *uart;
                    system->uarts_.push_back(uart.get());
                    world.endpoints().add(uart_name, std::move(uart));
                    if (added.uart_frames(i)) {
                        auto frames = std::make_unique<UartFramesEndpoint>(
                            uart_name + ".frames", added, i, bytes);
                        system->uart_frames_.push_back(frames.get());
                        world.endpoints().add(uart_name + ".frames", std::move(frames));
                    }
                }
                if (mcu_config.rtt) {
                    std::uint8_t probe = 0;
                    added.read_memory(mcu_config.rtt->address, &probe, 1);
                    for (unsigned ch = 0; ch < mcu_config.rtt->channels; ++ch) {
                        const std::string endpoint_name =
                            name + (ch == 0 ? ".rtt" : ".rtt" + std::to_string(ch));
                        auto rtt = std::make_unique<RttEndpoint>(
                            endpoint_name, added, mcu_config.rtt->address, ch,
                            mcu_config.rtt->interval_ns);
                        system->rtts_.push_back(rtt.get());
                        world.endpoints().add(endpoint_name, std::move(rtt));
                    }
                }
            } catch (const std::exception &error) {
                fail_at(mcu_config.rtt ? mcu_config.rtt->line : mcu_config.line,
                        "mcu '" + name + "': " + error.what());
            }
        }
    }

    for (const DeviceConfig &device : config.devices) {
        const Scope scope{&device.id};
        for (const NetConfig &net : device.nets)
            build_net(world, scope, net);
    }
    for (const NetConfig &net : config.nets)
        build_net(world, Scope{nullptr}, net);

    for (const DeviceConfig &device : config.devices)
        for (const PartConfig &part : device.parts)
            build_part(world, system->parts_, device.id, part, part_registry,
                       config.seed);

    auto build_link = [&](const Scope &scope, const LinkConfig &link) {
        auto [mcu_a, name_a] = resolve_mcu_ref(world, scope, link.a, link.line);
        auto [mcu_b, name_b] = resolve_mcu_ref(world, scope, link.b, link.line);
        const unsigned index_a = parse_uart_index(name_a, link.line, link.a);
        const unsigned index_b = parse_uart_index(name_b, link.line, link.b);
        if (index_a >= mcu_a->uart_count())
            fail_at(link.line, "'" + link.a + "': " + mcu_a->name() + " has " +
                                   std::to_string(mcu_a->uart_count()) +
                                   " UART(s)");
        if (index_b >= mcu_b->uart_count())
            fail_at(link.line, "'" + link.b + "': " + mcu_b->name() + " has " +
                                   std::to_string(mcu_b->uart_count()) +
                                   " UART(s)");
        if (mcu_a == mcu_b && index_a == index_b)
            fail_at(link.line, "link connects " + link.a + " to itself");
        auto uart_name = [](const Mcu *mcu, unsigned index) {
            return mcu->name() + ".uart" + std::to_string(index);
        };
        auto *end_a = dynamic_cast<UartEndpoint *>(
            world.endpoints().find(uart_name(mcu_a, index_a)));
        auto *end_b = dynamic_cast<UartEndpoint *>(
            world.endpoints().find(uart_name(mcu_b, index_b)));
        if (!end_a->can_receive() && !end_b->can_receive())
            fail_at(link.line, "neither " + link.a + " nor " + link.b +
                                   " can receive UART data");
        if (!end_a->can_receive() || !end_b->can_receive())
            system->warnings_.push_back(
                "line " + std::to_string(link.line) + ": link " + link.a +
                " <-> " + link.b + " is one-way (one end cannot receive)");
        // Bytes one side transmits arrive at the other after the latency; the
        // destination queues them until its FIFO has room.
        Scheduler *scheduler = &world.scheduler();
        const std::uint64_t latency = link.latency_ns;
        auto forward = [scheduler, latency](Endpoint *from, Endpoint *to, bool receives) {
            if (!receives)
                return;
            from->subscribe([scheduler, latency, to](const EndpointEvent &event) {
                const Value data = event.value;
                scheduler->at(event.time_ns + latency, [to, data] { to->set(data); });
            });
        };
        // When both ends model frames, the frames (9th bit) travel instead
        // of the bytes; plain frames are received exactly like bytes.
        auto *frames_a = dynamic_cast<UartFramesEndpoint *>(
            world.endpoints().find(uart_name(mcu_a, index_a) + ".frames"));
        auto *frames_b = dynamic_cast<UartFramesEndpoint *>(
            world.endpoints().find(uart_name(mcu_b, index_b) + ".frames"));
        if (frames_a && frames_b) {
            forward(frames_a, frames_b, end_b->can_receive());
            forward(frames_b, frames_a, end_a->can_receive());
        } else {
            forward(end_a, end_b, end_b->can_receive());
            forward(end_b, end_a, end_a->can_receive());
        }
    };
    for (const DeviceConfig &device : config.devices)
        for (const LinkConfig &link : device.links)
            build_link(Scope{&device.id}, link);
    for (const LinkConfig &link : config.links)
        build_link(Scope{nullptr}, link);

    for (const EventConfig &event : config.events) {
        World *w = &world;
        if (!event.endpoint.empty()) {
            Endpoint *endpoint = world.endpoints().find(event.endpoint);
            if (!endpoint)
                fail_at(event.line, "unknown endpoint '" + event.endpoint +
                                        "' in event");
            const Value value = parse_scalar_value(event.value);
            // Frame tokens are checked now so a typo names its line.
            if (const auto *text = std::get_if<std::string>(&value);
                text && dynamic_cast<UartFramesEndpoint *>(endpoint)) {
                try {
                    parse_uart_frames(*text);
                } catch (const std::invalid_argument &error) {
                    fail_at(event.line, error.what());
                }
            }
            const int line = event.line;
            const std::string name = event.endpoint;
            world.scheduler().at(event.at_ns, [endpoint, value, line, name] {
                try {
                    endpoint->set(value);
                } catch (const std::exception &error) {
                    // A bad value only shows up at run time; report it and
                    // keep the simulation going.
                    std::cerr << "warning: line " << line << ": setting '"
                              << name << "' failed: " << error.what() << '\n';
                }
            });
            continue;
        }
        Net *net = nullptr;
        try {
            net = &world.net(event.net);
        } catch (const std::out_of_range &) {
            fail_at(event.line, "unknown net '" + event.net + "' in event");
        }
        Pin *&pin = system->scenario_pins_[event.net];
        if (!pin)
            pin = &net->attach("scenario");
        const Drive drive = event.drive;
        world.scheduler().at(event.at_ns, [pin, drive, w] {
            pin->set_drive(drive, w->scheduler().now());
        });
    }
    // Endpoints that need regular service: UART FIFOs, RTT buffers and the
    // sinks' sockets (every 16th slice).
    System *self = system.get();
    world.add_poller([self](std::uint64_t now) {
        for (UartEndpoint *uart : self->uarts_)
            uart->poll(now);
        for (UartFramesEndpoint *frames : self->uart_frames_)
            frames->poll(now);
        for (RttEndpoint *rtt : self->rtts_)
            rtt->poll(now);
        if (++self->session_ticks_ % 16 == 0)
            self->session_.poll();
    });

    auto apply_attach = [&](const std::string *device, const AttachConfig &attach) {
        std::string endpoint = attach.endpoint;
        if (device && world.endpoints().find(*device + "." + endpoint))
            endpoint = *device + "." + endpoint;
        try {
            system->session_.attach(endpoint, attach.sink);
        } catch (const std::out_of_range &) {
            fail_at(attach.line, "unknown endpoint '" + attach.endpoint + "'");
        } catch (const std::exception &error) {
            fail_at(attach.line, "attach '" + attach.endpoint + "': " + error.what());
        }
    };
    for (const DeviceConfig &device : config.devices)
        for (const AttachConfig &attach : device.attach)
            apply_attach(&device.id, attach);
    for (const AttachConfig &attach : config.attach)
        apply_attach(nullptr, attach);
    return system;
}

} // namespace sim
