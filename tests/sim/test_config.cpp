#include "../test_support.hpp"

#include "sim/config.hpp"
#include "sim/simulation.hpp"
#include "sim/system.hpp"
#include "sim/tlsr8258_mcu.hpp"

#include <filesystem>
#include <fstream>
#include <string>

namespace mcuemu_tests {

namespace {

// Writes an idle-loop TC32 image next to the test binary's temp dir.
std::string idle_image() {
    static const std::string path = [] {
        const auto file =
            std::filesystem::temp_directory_path() / "simcore-test-idle.bin";
        std::ofstream out(file, std::ios::binary);
        out.put(static_cast<char>(0xfe));
        out.put(static_cast<char>(0x87)); // b .
        return file.string();
    }();
    return path;
}

std::string with_image(std::string yaml) {
    for (std::size_t at; (at = yaml.find("IMG")) != std::string::npos;)
        yaml.replace(at, 3, idle_image());
    return yaml;
}

template <typename Fn> std::string error_of(Fn &&fn) {
    try {
        fn();
    } catch (const sim::ConfigError &error) {
        return error.what();
    } catch (const sim::SimulationError &error) {
        return error.what();
    }
    return {};
}

bool contains(const std::string &text, const std::string &part) {
    return text.find(part) != std::string::npos;
}

const char *two_mcu_yaml = R"(version: 1
world: { duration: 1ms, quantum: 2us }
devices:
  - id: node
    mcus:
      - { id: main, model: tlsr8258, variant: F1K, firmware: IMG, clock_hz: 32MHz }
      - { id: aux, model: tlsr8258, firmware: IMG, start_at: 100us }
    nets:
      - { id: line, kind: open_drain, pull: up, pins: [main.PA0, aux.PB1] }
    links:
      - { type: uart, a: main.uart0, b: aux.uart0, latency: 10us }
scenario:
  events:
    - { at: 200us, net: node.line, drive: low }
)";

} // namespace

void test_config_parsing() {
    const sim::SystemConfig config = sim::parse_system_config(
        with_image(two_mcu_yaml), "/base");
    CHECK(config.duration_ns == 1000000u);
    CHECK(config.quantum_ns == 2000u);
    CHECK(config.devices.size() == 1);
    const sim::DeviceConfig &device = config.devices[0];
    CHECK(device.mcus.size() == 2);
    CHECK(device.mcus[0].variant == "F1K");
    CHECK(device.mcus[0].clock_hz == 32000000u);
    CHECK(device.mcus[1].start_at_ns == 100000u);
    CHECK(device.nets.size() == 1);
    CHECK(device.nets[0].kind == sim::NetKind::OpenDrain);
    CHECK(device.nets[0].pull == sim::Drive::PullUp);
    CHECK(device.nets[0].pins.size() == 2);
    CHECK(device.links.size() == 1 && device.links[0].latency_ns == 10000u);
    CHECK(config.events.size() == 1 && config.events[0].drive == sim::Drive::Low);

    // Relative firmware paths resolve against the base directory.
    const sim::SystemConfig relative = sim::parse_system_config(
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: tlsr8258, firmware: fw/a.bin}\n",
        "/base/dir");
    CHECK(std::filesystem::path(relative.devices[0].mcus[0].firmware).generic_string() ==
          "/base/dir/fw/a.bin");
    const sim::SystemConfig efuse_config = sim::parse_system_config(
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: phy6252, firmware: fw/relay.bin, efuse: data/chip.efuse}\n",
        "/base/dir");
    CHECK(std::filesystem::path(*efuse_config.devices[0].mcus[0].efuse).generic_string() ==
          "/base/dir/data/chip.efuse");
    const sim::SystemConfig rom_config = sim::parse_system_config(
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: phy6252, firmware: fw/relay.bin, rom: data/rom.bin}\n",
        "/base/dir");
    CHECK(std::filesystem::path(*rom_config.devices[0].mcus[0].rom).generic_string() ==
          "/base/dir/data/rom.bin");
    const sim::SystemConfig rom_boot_config = sim::parse_system_config(
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: phy6252, firmware: fw/relay.bin, rom: data/rom.bin, boot_from_rom: true}\n",
        "/base/dir");
    CHECK(rom_boot_config.devices[0].mcus[0].boot_from_rom);
    CHECK(error_of([&] {
        sim::parse_system_config(
            "version: 1\ndevices:\n  - id: d\n    mcus:\n"
            "      - {id: m, model: phy6252, firmware: fw/relay.bin, boot_from_rom: true}\n",
            "/base/dir");
    }).find("requires 'rom'") != std::string::npos);

    CHECK(sim::parse_duration_ns("2.5s") == 2500000000ull);
    CHECK(sim::parse_duration_ns("750ns") == 750u);
    CHECK(sim::parse_duration_ns("0") == 0u);
    CHECK(sim::parse_frequency_hz("48MHz") == 48000000u);
    CHECK(sim::parse_frequency_hz("32kHz") == 32000u);
    CHECK(sim::parse_frequency_hz("24000000") == 24000000u);
}

void test_config_errors() {
    auto parse = [](const std::string &text) {
        return error_of([&] { sim::parse_system_config(text); });
    };
    CHECK(contains(parse("devices: []\n"), "'version' is required"));
    CHECK(contains(parse("version: 2\ndevices: []\n"), "unsupported version"));
    CHECK(contains(parse("version: 1\n"), "'devices' is required"));
    CHECK(contains(parse("version: 1\ndevices: [\n"), "line"));
    const std::string head = "version: 1\ndevices:\n  - id: d\n    mcus:\n";
    // Unknown key points at its line.
    const std::string typo =
        parse(head + "      - {id: m, model: tlsr8258, firmware: f, clok_hz: 1}\n");
    CHECK(contains(typo, "line 5") && contains(typo, "unknown key 'clok_hz'"));
    // Planned features are rejected explicitly, not ignored.
    CHECK(contains(parse(head + "      - {id: m, model: tlsr8258, firmware: f}\n"
                                "scenario:\n  script: x.lua\n"),
                   "planned but not supported yet"));
    CHECK(contains(parse(head + "      - {id: m, model: x, firmware: f, radio: {}}\n"),
                   "requires 'air'"));
    CHECK(contains(parse(head + "      - {id: m, model: tlsr8258}\n"),
                   "requires 'firmware'"));
    CHECK(contains(parse(head + "      - {id: m, model: tlsr8258, firmware: f, efuse: e}\n"),
                   "'efuse' is only supported by phy6252"));
    CHECK(contains(parse(head + "      - {id: m, model: tlsr8258, firmware: f, rom: r}\n"),
                   "'rom' is only supported by phy6252"));
    CHECK(contains(parse(head + "      - {id: 'a b', model: m, firmware: f}\n"),
                   "may only contain"));
    CHECK(contains(parse(head + "      - {id: m, model: t, firmware: f, start_at: 5}\n"),
                   "invalid duration"));
    CHECK(contains(parse(head + "      - {id: m, model: t, firmware: f}\n"
                                "      - {id: m, model: t, firmware: f}\n"),
                   "duplicate mcu id"));
    CHECK(contains(parse("version: 1\ndevices:\n  - {id: d, mcus: [{id: m, model: t, firmware: f}]}\n"
                         "  - {id: d, mcus: [{id: m, model: t, firmware: f}]}\n"),
                   "duplicate device id"));
    CHECK(contains(parse(head + "      - {id: m, model: t, firmware: f}\n"
                                "    nets:\n      - {id: n, kind: weird}\n"),
                   "push_pull or open_drain"));
    CHECK(contains(parse(head + "      - {id: m, model: t, firmware: f}\n"
                                "    links:\n      - {type: spi, a: x, b: y}\n"),
                   "not supported (only 'uart')"));
}

void test_system_build_and_run() {
    const sim::SystemConfig config = sim::parse_system_config(with_image(two_mcu_yaml));
    const auto system = sim::build_system(config);
    sim::World &world = system->world();
    CHECK(system->warnings().empty());

    auto &main_mcu = static_cast<sim::Tlsr8258Mcu &>(world.mcu("node.main"));
    auto &aux = static_cast<sim::Tlsr8258Mcu &>(world.mcu("node.aux"));
    CHECK(aux.time_ns() == 100000u); // start_at applied
    sim::Net &line = world.net("node.line");
    CHECK(line.level() == sim::Level::High); // open-drain with pull-up

    world.run_until(*system->duration_ns());
    // The scenario pulled the line low at 200 us and never released it.
    CHECK(line.level() == sim::Level::Low);
    CHECK((main_mcu.device().read8(mcuemu::Tlsr8258::GpioBase) & 1u) == 0u);
    CHECK((aux.device().read8(mcuemu::Tlsr8258::GpioBase + 8) & 2u) == 0u);
    CHECK(main_mcu.state() == sim::McuState::Running);
    CHECK(world.now() == 1000000u);
}

void test_system_build_errors() {
    auto build = [](const std::string &text) {
        return error_of([&] {
            sim::build_system(sim::parse_system_config(with_image(text)));
        });
    };
    const std::string head =
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: tlsr8258, firmware: IMG}\n";
    CHECK(contains(build("version: 1\ndevices:\n  - id: d\n    mcus:\n"
                         "      - {id: m, model: nope, firmware: IMG}\n"),
                   "unknown mcu model 'nope' (known: bl702, ca51f253l3, cc2530f256, efr32mg21, esp32c6, esp32h2, nrf52840, phy6252, stm32wb55, tlsr8258, tlsr8267, tlsr8269)"));
    CHECK(contains(build("version: 1\ndevices:\n  - id: d\n    mcus:\n"
                         "      - {id: m, model: tlsr8258, cpu2_model: st_mac_802154, firmware: IMG}\n"),
                   "'cpu2_model' is only supported by stm32wb55"));
    CHECK(contains(build("version: 1\ndevices:\n  - id: d\n    mcus:\n"
                         "      - {id: m, model: stm32wb55, cpu2_model: zigbee, firmware: IMG}\n"),
                   "'cpu2_model' must be thread_rcp or st_mac_802154"));
    CHECK(contains(build("version: 1\ndevices:\n  - id: d\n    mcus:\n"
                         "      - {id: m, model: tlsr8258, firmware: /no/such.bin}\n"),
                   "cannot open firmware image"));
    CHECK(contains(build("version: 1\ndevices:\n  - id: d\n    mcus:\n"
                         "      - {id: m, model: tlsr8258, variant: X, firmware: IMG}\n"),
                   "unknown tlsr8258 variant"));
    CHECK(contains(build(head + "    nets:\n      - {id: n, pins: [x.PA0]}\n"),
                   "unknown mcu 'd.x'"));
    CHECK(contains(build(head + "    nets:\n      - {id: n, pins: [m.PZ9]}\n"),
                   "no pad named 'PZ9'"));
    CHECK(contains(build(head + "    nets:\n      - {id: a, pins: [m.PA0]}\n"
                                "      - {id: b, pins: [m.PA0]}\n"),
                   "already connected"));
    CHECK(contains(build(head + "    nets:\n      - {id: a}\n      - {id: a}\n"),
                   "duplicate net 'd.a'"));
    CHECK(contains(build(head + "    links:\n      - {type: uart, a: m.uart0, b: m.uart0}\n"),
                   "to itself"));
    CHECK(contains(build(head + "    links:\n      - {type: uart, a: m.uart0, b: m.uart3}\n"),
                   "has 1 UART(s)"));
    CHECK(contains(build(head + "    links:\n      - {type: uart, a: m.uart0, b: m.serial}\n"),
                   "is not a UART"));
    CHECK(contains(build(head + "scenario:\n  events:\n"
                                "    - {at: 1us, net: nowhere, drive: low}\n"),
                   "unknown net 'nowhere'"));
    // World-level nets use full references and may span devices.
    CHECK(build("version: 1\ndevices:\n"
                "  - {id: a, mcus: [{id: m, model: tlsr8258, firmware: IMG}]}\n"
                "  - {id: b, mcus: [{id: m, model: tlsr8258, firmware: IMG}]}\n"
                "nets:\n  - {id: shared, pins: [a.m.PA0, b.m.PA0]}\n").empty());
}

void test_uart_link_between_mcus() {
    const auto system = sim::build_system(sim::parse_system_config(with_image(
        "version: 1\ndevices:\n"
        "  - id: d\n    mcus:\n"
        "      - {id: a, model: tlsr8258, firmware: IMG}\n"
        "      - {id: b, model: tlsr8258, firmware: IMG}\n"
        "      - {id: c, model: ca51f253l3, firmware: IMG}\n")));
    sim::World &world = system->world();
    (void)world;
    // CA51F253L3 cannot receive UART data yet: a link to it is one-way.
    const std::string yaml =
        "version: 1\ndevices:\n"
        "  - id: d\n    mcus:\n"
        "      - {id: a, model: tlsr8258, firmware: IMG}\n"
        "      - {id: b, model: tlsr8258, firmware: IMG}\n"
        "    links:\n      - {type: uart, a: a.uart0, b: b.uart0, latency: 5us}\n";
    const auto linked = sim::build_system(sim::parse_system_config(with_image(yaml)));
    sim::World &linked_world = linked->world();
    CHECK(linked->warnings().empty());

    auto &a = static_cast<sim::Tlsr8258Mcu &>(linked_world.mcu("d.a"));
    auto &b = static_cast<sim::Tlsr8258Mcu &>(linked_world.mcu("d.b"));
    // Firmware-free transmit: enable the UART and write a byte to A's TX FIFO.
    a.device().write8(mcuemu::Tlsr8258::UartControl0, 0x4f);
    a.device().write8(mcuemu::Tlsr8258::UartData, 0x5a);
    linked_world.run_until(2000000); // frame time at default baud << 2 ms
    // Byte arrived in B's RX FIFO (low nibble of the buffer count).
    CHECK((b.device().read8(mcuemu::Tlsr8258::UartBufferCount) & 0x0fu) == 1);
    CHECK(b.device().read8(mcuemu::Tlsr8258::UartData + 3) == 0x5a);
    CHECK(a.uart_take_tx(0).empty()); // the link consumed A's output
}

} // namespace mcuemu_tests

namespace mcuemu_tests {

void test_config_include() {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "simcore-include-tests";
    fs::remove_all(dir);
    fs::create_directories(dir / "devices");
    auto write = [&](const char *name, const std::string &text) {
        std::ofstream(dir / name) << text;
    };
    write("devices/dev.yaml",
          "version: 1\n"
          "params: {pos: [0, 0, 0], fw: a.bin, pin: PB1, tx: 0}\n"
          "device:\n"
          "  id: dev\n"
          "  mcus:\n"
          "    - {id: m, model: tlsr8258, firmware: \"${fw}\", radio: {air: air0, position: \"${pos}\", tx_power_dbm: \"${tx}\"}}\n"
          "  nets:\n"
          "    - {id: n, pins: [\"m.${pin}\"]}\n");
    const std::string world =
        "version: 1\nworld: {radio: [{id: air0, model: log_distance}]}\ndevices:\n"
        "  - include: devices/dev.yaml\n    id: one\n    params: {pos: [1, 2, 3], tx: 4}\n"
        "  - include: devices/dev.yaml\n    id: two\n    params: {pin: PB2, fw: b.bin}\n";
    const sim::SystemConfig config = sim::parse_system_config(world, dir.string());
    CHECK(config.devices.size() == 2);
    CHECK(config.devices[0].id == "one" && config.devices[1].id == "two");
    const sim::McuConfig &one = config.devices[0].mcus[0], &two = config.devices[1].mcus[0];
    // Typed substitution (list and number), and paths relative to the included file.
    CHECK(one.radio && one.radio->position == (std::array<double, 3>{1, 2, 3}));
    CHECK(one.radio->tx_power_dbm == 4);
    CHECK(two.radio->position == (std::array<double, 3>{0, 0, 0}));
    CHECK(one.firmware == (dir / "devices/a.bin").lexically_normal().string());
    CHECK(two.firmware == (dir / "devices/b.bin").lexically_normal().string());
    CHECK(config.devices[0].nets[0].pins[0] == "m.PB1" && config.devices[1].nets[0].pins[0] == "m.PB2");

    auto parse = [&](const std::string &text) {
        return error_of([&] { sim::parse_system_config(text, dir.string()); });
    };
    const std::string head = "version: 1\nworld: {radio: [{id: air0}]}\ndevices:\n";
    // Two copies without an id override collide.
    CHECK(contains(parse(head + "  - {include: devices/dev.yaml}\n  - {include: devices/dev.yaml}\n"),
                   "duplicate device id 'dev'"));
    // Misspelled parameter, unknown key, missing file.
    CHECK(contains(parse(head + "  - {include: devices/dev.yaml, params: {posn: [1,1,1]}}\n"),
                   "unknown parameter 'posn'"));
    CHECK(contains(parse(head + "  - {include: devices/dev.yaml, mcus: []}\n"), "unknown key 'mcus'"));
    CHECK(contains(parse(head + "  - {include: devices/missing.yaml}\n"), "cannot open"));
    // Errors inside the file name it and its own line.
    write("devices/bad.yaml", "version: 1\ndevice:\n  id: d\n  mcus:\n    - {id: m, modl: t, firmware: f}\n");
    const std::string bad = parse(head + "  - {include: devices/bad.yaml}\n");
    CHECK(contains(bad, "devices/bad.yaml: line 5") && contains(bad, "unknown key 'modl'"));
    write("devices/ref.yaml", "version: 1\ndevice:\n  id: d\n  mcus:\n    - {id: m, model: t, firmware: \"${nope}\"}\n");
    CHECK(contains(parse(head + "  - {include: devices/ref.yaml}\n"), "unknown parameter 'nope'"));
    write("devices/part.yaml",
          "version: 1\nparams: {list: [1, 2]}\ndevice:\n  id: d\n  mcus:\n    - {id: m, model: t, firmware: \"x${list}\"}\n");
    CHECK(contains(parse(head + "  - {include: devices/part.yaml}\n"), "not a scalar"));

    // The hosting sandbox confines includes as it does every other file.
    fs::create_directories(dir / "root");
    std::ofstream(dir / "outside.yaml") << "version: 1\ndevice: {id: d, mcus: []}\n";
    fs::create_symlink(dir / "outside.yaml", dir / "root" / "link.yaml");
    sim::Simulation sandbox((dir / "root").string());
    const std::string sandbox_head = "version: 1\nworld: {radio: [{id: air0}]}\ndevices:\n";
    auto load = [&](const std::string &include) {
        return error_of([&] { sandbox.load(sandbox_head + "  - {include: " + include + "}\n"); });
    };
    CHECK(contains(load("../outside.yaml"), "outside the allowed directory"));
    CHECK(contains(load("link.yaml"), "outside the allowed directory"));
    CHECK(contains(load("/etc/passwd"), "outside the allowed directory"));
    const std::string rom_outside = error_of([&] {
        sandbox.load(sandbox_head +
                     "  - id: d\n    mcus:\n"
                     "      - {id: m, model: phy6252, firmware: fw.bin, rom: ../outside.yaml}\n");
    });
    CHECK(contains(rom_outside, "outside the allowed directory"));
}

} // namespace mcuemu_tests

namespace mcuemu_tests {

// The shipped examples must stay loadable: parse the world with its
// insertions and check the wiring (firmware files are not needed to parse).
void test_shipped_examples() {
    const std::string dir = std::string(MCUEMU_SOURCE_DIR) + "/examples";
    const sim::SystemConfig config = sim::load_system_config(dir + "/world.yaml");
    CHECK(config.devices.size() == 3);
    CHECK(config.devices[0].id == "coordinator");
    const sim::DeviceConfig &thermostat = config.devices[1];
    CHECK(thermostat.id == "thermostat" && thermostat.mcus.size() == 2);
    CHECK(thermostat.mcus[0].model == "tlsr8258" && thermostat.mcus[1].model == "ca51f253l3");
    CHECK(thermostat.mcus[0].images.size() == 2);
    CHECK(thermostat.mcus[0].images[1].offset == 0x9ad0 && thermostat.mcus[0].images[1].skip == 0x1ad0);
    CHECK(thermostat.links.size() == 1 && thermostat.links[0].a == "main.uart1");
    CHECK(thermostat.mcus[0].radio && thermostat.mcus[0].radio->position[0] == 4);
    const sim::DeviceConfig &plug = config.devices[2];
    CHECK(plug.id == "plug" && plug.parts.size() == 4 && plug.nets.size() == 6);
    CHECK(plug.mcus[0].radio && plug.mcus[0].radio->position[1] == 5);
    bool found_power = false;
    for (const sim::PartConfig &part : plug.parts)
        if (part.id == "meter") {
            found_power = part.params.at("power_w") == "60";
            CHECK(part.script.find("bl0937.lua") != std::string::npos);
        }
    CHECK(found_power);
    CHECK(config.events.size() == 4 && config.attach.size() == 2);

    // The nRF52840 board and its example world.
    const sim::SystemConfig nrf = sim::load_system_config(dir + "/nrf52840_promicro.yaml");
    CHECK(nrf.devices.size() == 1 && nrf.devices[0].mcus.size() == 1);
    CHECK(nrf.devices[0].mcus[0].model == "nrf52840");
    CHECK(nrf.devices[0].nets.size() == 4 && nrf.devices[0].parts.size() == 3);

    // Both devices are valid on their own too, with the defaults.
    for (const char *file : {"/devices/tuya_plug.yaml", "/devices/tuya_thermostat.yaml"}) {
        const std::string name = std::string("version: 1\nworld: {radio: [{id: air0}]}\ndevices:\n"
                                             "  - include: devices") + (file + 8) + "\n";
        CHECK(sim::parse_system_config(name, dir).devices.size() == 1);
    }
}

} // namespace mcuemu_tests
