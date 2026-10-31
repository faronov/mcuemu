#include "../test_support.hpp"

#include "mcs51emu/run_command.hpp"
#include "sim/cc2530f256_mcu.hpp"
#include "sim/config.hpp"
#include "sim/endpoint.hpp"
#include "sim/net.hpp"
#include "sim/system.hpp"
#include "sim/world.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using sim::Cc2530F256Mcu;

const std::filesystem::path &scratch() {
    static const std::filesystem::path path = [] {
        auto dir = std::filesystem::temp_directory_path() / "simcore-cc2530-tests";
        std::filesystem::create_directories(dir);
        return dir;
    }();
    return path;
}

// Physical flash contents keyed by physical address.
using Image = std::map<std::uint32_t, std::vector<std::uint8_t>>;

std::string write_hex(const std::string &name, const Image &image) {
    std::ostringstream text;
    auto record = [&](std::uint8_t type, std::uint16_t address,
                      const std::vector<std::uint8_t> &data) {
        std::vector<std::uint8_t> bytes{static_cast<std::uint8_t>(data.size()),
                                        static_cast<std::uint8_t>(address >> 8),
                                        static_cast<std::uint8_t>(address), type};
        bytes.insert(bytes.end(), data.begin(), data.end());
        unsigned sum = 0;
        text << ':';
        for (std::uint8_t byte : bytes) {
            sum += byte;
            text << "0123456789ABCDEF"[byte >> 4] << "0123456789ABCDEF"[byte & 15];
        }
        const std::uint8_t check = static_cast<std::uint8_t>(-static_cast<int>(sum));
        text << "0123456789ABCDEF"[check >> 4] << "0123456789ABCDEF"[check & 15] << '\n';
    };
    for (const auto &[address, data] : image) {
        record(4, 0, {static_cast<std::uint8_t>(address >> 24),
                      static_cast<std::uint8_t>(address >> 16)});
        record(0, static_cast<std::uint16_t>(address), data);
    }
    record(1, 0, {});
    const auto path = scratch() / name;
    std::ofstream(path) << text.str();
    return path.generic_string();
}

// Common code calls bank 3 through FMAP, stores the result at XDATA 0x0200,
// then spins at 0x004d.
std::string banked_image() {
    return write_hex("banked.hex",
                     {{0x00000, {0x02, 0x00, 0x40}},                   // LJMP 0x0040
                      {0x00040,
                       {0x75, 0x81, 0x60,                               // MOV SP,#0x60
                        0x75, 0x9f, 0x03,                               // MOV FMAP,#3
                        0x12, 0x80, 0x00,                               // LCALL 0x8000
                        0x90, 0x02, 0x00,                               // MOV DPTR,#0x0200
                        0xf0,                                           // MOVX @DPTR,A
                        0x80, 0xfe}},                                   // SJMP $
                      {0x18000, {0x74, 0x33, 0x7f, 0x77, 0x22}}});     // A=0x33 R7=0x77 RET
}

std::string with_firmware(const std::string &mcu_fields) {
    return "version: 1\ndevices:\n  - id: d\n    mcus:\n      - {id: m, model: cc2530f256, " +
           mcu_fields + "}\n";
}

std::string error_of_build(const std::string &yaml) {
    try {
        sim::build_system(sim::parse_system_config(yaml));
    } catch (const std::exception &error) {
        return error.what();
    }
    return {};
}

bool contains(const std::string &text, const std::string &part) {
    return text.find(part) != std::string::npos;
}

std::array<std::uint8_t, Cc2530F256Mcu::CpuSnapshotSize> snapshot(sim::Mcu &mcu) {
    std::array<std::uint8_t, Cc2530F256Mcu::CpuSnapshotSize> block{};
    mcu.read_memory(Cc2530F256Mcu::CpuSnapshotWindow, block.data(), block.size());
    return block;
}

std::uint64_t le(const std::uint8_t *bytes, unsigned size) {
    std::uint64_t value = 0;
    for (unsigned i = size; i-- > 0;)
        value = (value << 8) | bytes[i];
    return value;
}

std::string raw_image(const std::string &name, std::initializer_list<int> bytes) {
    const auto path = scratch() / name;
    std::ofstream out(path, std::ios::binary);
    for (int byte : bytes)
        out.put(static_cast<char>(byte));
    return path.generic_string();
}

template <typename Fn> bool throws_invalid(Fn &&fn) {
    try {
        fn();
    } catch (const std::invalid_argument &) {
        return true;
    }
    return false;
}

template <typename Fn> bool throws_runtime(Fn &&fn) {
    try {
        fn();
    } catch (const std::runtime_error &) {
        return true;
    }
    return false;
}


// Mailbox poller: INC 0x32 at the breakpoint 0x0046 counts passes; a nonzero
// XDATA 0x0100 is added to IDATA 0x30, counted in 0x31 and cleared.
std::string mailbox_image() {
    return write_hex("mailbox.hex",
                     {{0x00000, {0x02, 0x00, 0x40}},          // LJMP 0x0040
                      {0x00040,
                       {0x75, 0x81, 0x60,                      // MOV SP,#0x60
                        0x90, 0x01, 0x00,                      // MOV DPTR,#0x0100
                        0x05, 0x32,                            // 0x46: INC 0x32
                        0xe0,                                  // MOVX A,@DPTR
                        0x60, 0xfb,                            // JZ 0x0046
                        0x25, 0x30,                            // ADD A,0x30
                        0xf5, 0x30,                            // MOV 0x30,A
                        0x05, 0x31,                            // INC 0x31
                        0xe4,                                  // CLR A
                        0xf0,                                  // MOVX @DPTR,A
                        0x80, 0xf1}}});                        // SJMP 0x0046
}

std::uint8_t idata(sim::Mcu &mcu, std::uint8_t address) {
    std::uint8_t value = 0;
    mcu.read_memory(Cc2530F256Mcu::IdataWindow + address, &value, 1);
    return value;
}

std::uint8_t xdata(sim::Mcu &mcu, std::uint16_t address) {
    std::uint8_t value = 0;
    mcu.read_memory(Cc2530F256Mcu::XdataWindow + address, &value, 1);
    return value;
}
} // namespace

namespace mcuemu_tests {

void test_cc2530_system_runs_banked_firmware() {
    const auto system = sim::build_system(
        sim::parse_system_config(with_firmware("firmware: \"" + banked_image() + "\"")));
    sim::World &world = system->world();
    sim::Mcu &mcu = world.mcu("d.m");
    CHECK(mcu.model() == "cc2530f256");
    CHECK(mcu.time_ns() == 0);
    world.run_until(100000); // 100 us
    CHECK(mcu.state() == sim::McuState::Halted);
    CHECK(contains(mcu.fault_message(), "idle self-loop at pc=0x004d fmap=3"));

    const auto cpu = snapshot(mcu);
    CHECK(le(&cpu[Cc2530F256Mcu::CpuSnapshotPc], 2) == 0x004d);
    CHECK(cpu[Cc2530F256Mcu::CpuSnapshotFmap] == 3);
    CHECK(cpu[Cc2530F256Mcu::CpuSnapshotA] == 0x33);
    CHECK(cpu[Cc2530F256Mcu::CpuSnapshotSp] == 0x60);
    CHECK(cpu[Cc2530F256Mcu::CpuSnapshotDph] == 0x02 && cpu[Cc2530F256Mcu::CpuSnapshotDpl] == 0);
    CHECK(cpu[Cc2530F256Mcu::CpuSnapshotR0 + 7] == 0x77);
    CHECK(cpu[Cc2530F256Mcu::CpuSnapshotFlags] == 4); // idle loop, not sleeping, not in ISR
    CHECK(le(&cpu[Cc2530F256Mcu::CpuSnapshotPhysicalPc], 4) == 0x004d);
    CHECK(le(&cpu[Cc2530F256Mcu::CpuSnapshotInterrupts], 4) == 0);
    const std::uint64_t cycles = le(&cpu[Cc2530F256Mcu::CpuSnapshotCycles], 8);
    // Halted at the first world quantum that ended in the self-loop, so the
    // core stopped before 100 us (3200 cycles at the default 32 MHz).
    CHECK(cycles > 8 && cycles <= 3200 && mcu.time_ns() <= 100000);
    CHECK(le(&cpu[Cc2530F256Mcu::CpuSnapshotInstructions], 8) > 8);
    // DPTR1/DPS and the interrupt-controller fields are idle; the rest is reserved.
    CHECK(cpu[Cc2530F256Mcu::CpuSnapshotDpl1] == 0 && cpu[Cc2530F256Mcu::CpuSnapshotDph1] == 0);
    CHECK(cpu[Cc2530F256Mcu::CpuSnapshotDps] == 0);
    CHECK(cpu[Cc2530F256Mcu::CpuSnapshotIrqLevels] == 0);
    CHECK(le(&cpu[Cc2530F256Mcu::CpuSnapshotPostedVector], 2) == 0);
    CHECK(cpu[Cc2530F256Mcu::CpuSnapshotPostedLevel] == 0);
    CHECK(cpu[Cc2530F256Mcu::CpuSnapshotIrqInhibit] == 0);
    for (unsigned i = 0x34; i < Cc2530F256Mcu::CpuSnapshotSize; ++i)
        CHECK(cpu[i] == 0);

    std::uint8_t byte = 0;
    mcu.read_memory(Cc2530F256Mcu::XdataWindow + 0x0200, &byte, 1);
    CHECK(byte == 0x33);
    // CODE view follows FMAP; the flash window is physical.
    std::array<std::uint8_t, 2> code{};
    mcu.read_memory(Cc2530F256Mcu::CodeWindow + 0x8000, code.data(), 2);
    CHECK(code[0] == 0x74 && code[1] == 0x33);
    mcu.read_memory(Cc2530F256Mcu::FlashWindow + 0x18000, code.data(), 2);
    CHECK(code[0] == 0x74 && code[1] == 0x33);
    mcu.read_memory(Cc2530F256Mcu::FlashWindow + 0x08000, code.data(), 2);
    CHECK(code[0] == 0xff && code[1] == 0xff); // erased bank 1
    // IDATA is the top of SRAM; SFR window reads FMAP and SP.
    mcu.read_memory(Cc2530F256Mcu::IdataWindow + 0x07, &byte, 1);
    CHECK(byte == 0x77); // R7 of bank 0
    std::uint8_t aliased = 0;
    mcu.read_memory(Cc2530F256Mcu::XdataWindow + 0x1f07, &aliased, 1);
    CHECK(aliased == 0x77);
    mcu.read_memory(Cc2530F256Mcu::SfrWindow + 0x9f, &byte, 1);
    CHECK(byte == 3);
    mcu.read_memory(Cc2530F256Mcu::SfrWindow + 0x81, &byte, 1);
    CHECK(byte == 0x60);
    // XREG chip ID through XDATA.
    mcu.read_memory(Cc2530F256Mcu::XdataWindow + 0x624a, &byte, 1);
    CHECK(byte == 0xa5);

    // Unmapped windows and ranges crossing a window end are rejected whole.
    std::array<std::uint8_t, 2> untouched{0x11, 0x22};
    CHECK(throws_runtime([&] { mcu.read_memory(0x00050000, untouched.data(), 1); }));
    CHECK(throws_runtime(
        [&] { mcu.read_memory(Cc2530F256Mcu::IdataWindow + 0xff, untouched.data(), 2); }));
    CHECK(throws_runtime([&] {
        mcu.read_memory(Cc2530F256Mcu::SfrWindow + 0x7f, untouched.data(), 1);
    }));
    CHECK(throws_runtime([&] {
        mcu.read_memory(Cc2530F256Mcu::FlashWindow + 0x3ffff, untouched.data(), 2);
    }));
    CHECK(untouched[0] == 0x11 && untouched[1] == 0x22);

    // Writes reach SRAM/IDATA only.
    const std::uint8_t value = 0x5c;
    mcu.write_memory(Cc2530F256Mcu::XdataWindow + 0x1234, &value, 1);
    mcu.read_memory(Cc2530F256Mcu::XdataWindow + 0x1234, &byte, 1);
    CHECK(byte == 0x5c);
    mcu.write_memory(Cc2530F256Mcu::IdataWindow + 0x30, &value, 1);
    mcu.read_memory(Cc2530F256Mcu::XdataWindow + 0x1f30, &byte, 1);
    CHECK(byte == 0x5c);
    CHECK(throws_runtime(
        [&] { mcu.write_memory(Cc2530F256Mcu::XdataWindow + 0x2000, &value, 1); }));
    CHECK(throws_runtime(
        [&] { mcu.write_memory(Cc2530F256Mcu::XdataWindow + 0x1fff, code.data(), 2); }));
    CHECK(throws_runtime([&] { mcu.write_memory(Cc2530F256Mcu::FlashWindow, &value, 1); }));
    CHECK(throws_runtime([&] { mcu.write_memory(Cc2530F256Mcu::SfrWindow + 0x9f, &value, 1); }));

    // A halted MCU stays where it stopped while the world moves on.
    world.run_until(200000);
    CHECK(mcu.state() == sim::McuState::Halted);
    CHECK(le(&snapshot(mcu)[Cc2530F256Mcu::CpuSnapshotCycles], 8) == cycles);

    // reset() is a power-on reset: SRAM cleared, the program runs again.
    mcu.reset();
    CHECK(mcu.state() == sim::McuState::Running && mcu.fault_message().empty());
    mcu.read_memory(Cc2530F256Mcu::XdataWindow + 0x0200, &byte, 1);
    CHECK(byte == 0);
    CHECK(snapshot(mcu)[Cc2530F256Mcu::CpuSnapshotFmap] == 1);
}

void test_cc2530_config_errors_and_pins() {
    const std::string image = banked_image();
    CHECK(contains(error_of_build(with_firmware("firmware: \"" + image + "\", variant: F128")),
                   "cc2530f256 has no variants"));
    // SWRS081B operating supply 2 V - 3.6 V (AVDD5 = DVDD).
    CHECK(contains(error_of_build(with_firmware("firmware: \"" + image + "\", supply_mv: 1999")),
                   "SWRS081B operating range"));
    CHECK(contains(error_of_build(with_firmware("firmware: \"" + image + "\", supply_mv: 3601")),
                   "SWRS081B operating range"));
    CHECK(contains(error_of_build(with_firmware("firmware: \"/no/such.hex\"")),
                   "cannot open"));
    const auto elf = raw_image("image.elf", {0x7f, 'E', 'L', 'F', 1, 1, 1, 0});
    CHECK(contains(error_of_build(with_firmware("firmware: \"" + elf + "\"")),
                   "llvm-objcopy -O ihex"));
    Cc2530F256Mcu mcu("c");
    // SWRU191F 7: P0.0-P0.7, P1.0-P1.7, P2.0-P2.4.
    const std::vector<std::string> pads = mcu.pin_names();
    CHECK(pads.size() == 21 && pads.front() == "P0.0" && pads.back() == "P2.4");
    const auto rejects = [&](std::string_view pad, sim::Net &net) {
        try {
            mcu.connect_pin(pad, net.attach("c." + std::string(pad)));
        } catch (const std::invalid_argument &) {
            return true;
        }
        return false;
    };
    sim::Net n0("n0"), n1("n1"), n2("n2"), n3("n3");
    CHECK(!rejects("P0.0", n0));
    CHECK(rejects("P0.0", n1)); // already wired
    CHECK(rejects("P2.5", n2)); // P2 has five pads
    CHECK(rejects("P3.0", n3));
    CHECK(mcu.uart_count() == 2);
    CHECK(mcu.uart_take_tx(0).empty());
    // Only the 802.15.4 radio exists, and only on the 32 MHz time base.
    mcuemu::VirtualAir air;
    bool ble_rejected = false;
    try {
        mcu.attach_radio(air, "c", {"ble"});
    } catch (const std::invalid_argument &) {
        ble_rejected = true;
    }
    CHECK(ble_rejected);
    Cc2530F256Mcu slow("slow", 16000000);
    bool slow_rejected = false;
    try {
        slow.attach_radio(air, "slow", {"zigbee"});
    } catch (const std::invalid_argument &) {
        slow_rejected = true;
    }
    CHECK(slow_rejected);
}

void test_cc2530_fault_and_sleep_states() {
    {
        // DPS = 1 selects DPTR1 (SWRU191F 2.3.1) instead of faulting.
        Cc2530F256Mcu mcu("dps");
        mcu.load_firmware(raw_image("dps.bin", {0x75, 0x92, 0x01,   // MOV DPS,#1
                                                0x90, 0x12, 0x34,   // MOV DPTR,#0x1234
                                                0x80, 0xfe}));
        CHECK(mcu.run_until(10000) == sim::McuState::Halted);
        CHECK(contains(mcu.fault_message(), "idle self-loop at pc=0x0006"));
        const auto cpu = snapshot(mcu);
        CHECK(cpu[Cc2530F256Mcu::CpuSnapshotDps] == 1);
        CHECK(cpu[Cc2530F256Mcu::CpuSnapshotDpl1] == 0x34);
        CHECK(cpu[Cc2530F256Mcu::CpuSnapshotDph1] == 0x12);
        CHECK(cpu[Cc2530F256Mcu::CpuSnapshotDpl] == 0 && cpu[Cc2530F256Mcu::CpuSnapshotDph] == 0);
    }
    {
        Cc2530F256Mcu mcu("bad");
        mcu.load_firmware(raw_image("a5.bin", {0x00, 0xa5}));
        CHECK(mcu.run_until(10000) == sim::McuState::Fault);
        CHECK(contains(mcu.fault_message(), "0x0001"));
        const std::uint64_t at = mcu.time_ns();
        CHECK(mcu.run_until(20000) == sim::McuState::Fault);
        CHECK(mcu.time_ns() == at); // a faulted core does not advance
    }
    {
        Cc2530F256Mcu mcu("sleep");
        // PCON.IDLE with no interrupt source: sleeps for good.
        mcu.load_firmware(raw_image("sleep.bin", {0x75, 0x87, 0x01, 0x80, 0xfe}));
        CHECK(mcu.run_until(10000) == sim::McuState::Sleeping);
        CHECK(contains(mcu.fault_message(), "PCON power mode"));
        CHECK((snapshot(mcu)[Cc2530F256Mcu::CpuSnapshotFlags] & 1) == 1);
    }
}

void test_cc2530_time_offset_and_quanta() {
    // A busy counter loop never halts: INC 0x30 / SJMP back.
    const std::string busy = raw_image("busy.bin", {0x05, 0x30, 0x80, 0xfc});
    Cc2530F256Mcu whole("whole");
    whole.load_firmware(busy);
    CHECK(whole.run_until(50000) == sim::McuState::Running);
    CHECK(whole.time_ns() >= 50000 && whole.time_ns() < 50000 + 200);

    // Same firmware started 100 us late and run in 1 us quanta: identical
    // instruction stream relative to its own power-on.
    Cc2530F256Mcu late("late");
    late.load_firmware(busy);
    late.set_start_offset(100000);
    CHECK(late.time_ns() == 100000);
    CHECK(late.run_until(90000) == sim::McuState::Running);
    CHECK(late.time_ns() == 100000);
    for (std::uint64_t t = 101000; t <= 150000; t += 1000)
        late.run_until(t);
    CHECK(late.time_ns() >= 150000 && late.time_ns() < 150000 + 200);
    const auto a = snapshot(whole);
    const auto b = snapshot(late);
    CHECK(le(&a[Cc2530F256Mcu::CpuSnapshotCycles], 8) ==
          le(&b[Cc2530F256Mcu::CpuSnapshotCycles], 8));
    CHECK(le(&a[Cc2530F256Mcu::CpuSnapshotInstructions], 8) ==
          le(&b[Cc2530F256Mcu::CpuSnapshotInstructions], 8));
    std::uint8_t ca = 0, cb = 0;
    whole.read_memory(Cc2530F256Mcu::IdataWindow + 0x30, &ca, 1);
    late.read_memory(Cc2530F256Mcu::IdataWindow + 0x30, &cb, 1);
    CHECK(ca == cb && ca != 0);

    // A slower clock gives fewer cycles in the same time.
    Cc2530F256Mcu slow("slow", 16000000);
    slow.load_firmware(busy);
    slow.run_until(50000);
    const std::uint64_t slow_cycles = le(&snapshot(slow)[Cc2530F256Mcu::CpuSnapshotCycles], 8);
    CHECK(slow_cycles >= 800 && slow_cycles < 800 + 4);
}

void test_cc2530_debugger_writes() {
    const std::vector<Cc2530F256Mcu::DebuggerWrite> writes{
        {3, 0x0100, {0x5a}},
        {4, 0x0100, {0xa5}},
        {4, 0x0200, {0x01, 0x02}},
        {200, 0x0300, {0xee}}};
    for (const std::uint64_t quantum : {std::uint64_t{0}, std::uint64_t{20}, std::uint64_t{97}}) {
        Cc2530F256Mcu mcu("dbg");
        mcu.load_firmware(mailbox_image());
        mcu.set_debugger(0x0046, writes);
        bool consistent = true;
        if (quantum == 0) {
            CHECK(mcu.run_until(100000) == sim::McuState::Running);
        } else {
            // Quanta that end at arbitrary instruction boundaries, some of
            // them exactly at the breakpoint: every arrival counted once.
            for (std::uint64_t t = quantum; t <= 100000; t += quantum) {
                CHECK(mcu.run_until(t) == sim::McuState::Running);
                if (mcu.debugger_arrivals() >= 200)
                    continue; // breakpoint removed, passes keep counting
                const unsigned passes = idata(mcu, 0x32);
                const unsigned arrivals = mcu.debugger_arrivals();
                const bool at_break = mcu.device().pc() == 0x0046;
                if (!(arrivals == passes || (at_break && arrivals == passes + 1)))
                    consistent = false;
            }
        }
        CHECK(consistent);
        CHECK(mcu.debugger_arrivals() == 200);
        // Hit 3 delivered 0x5a, consumed before arrival 4 delivered 0xa5.
        CHECK(idata(mcu, 0x30) == 0xff);
        CHECK(idata(mcu, 0x31) == 2);
        CHECK(xdata(mcu, 0x0200) == 0x01 && xdata(mcu, 0x0201) == 0x02);
        CHECK(xdata(mcu, 0x0300) == 0xee);
        // After the last write the breakpoint is gone and the loop runs free.
        CHECK(idata(mcu, 0x32) != 200);
    }
    {
        Cc2530F256Mcu mcu("dbg");
        mcu.load_firmware(mailbox_image());
        mcu.set_debugger(0x0046, writes);
        mcu.run_until(100000);
        mcu.reset(); // power-on reset re-arms the debugger
        CHECK(mcu.debugger_arrivals() == 0);
        mcu.run_until(100000);
        CHECK(mcu.debugger_arrivals() == 200 && idata(mcu, 0x31) == 2);
    }
    {
        // A power cut inside a run (breakpoint already removed) restarts
        // the count: every write is delivered again to the cleared SRAM.
        Cc2530F256Mcu mcu("dbg");
        mcu.load_firmware(mailbox_image());
        mcu.set_debugger(0x0046, writes);
        mcu.run_until(100000);
        CHECK(mcu.debugger_arrivals() == 200);
        mcu.device().schedule_power_cut(mcu.device().registers().cycles + 100);
        mcu.run_until(300000);
        CHECK(mcu.device().diagnostics().power_cuts == 1);
        CHECK(mcu.debugger_arrivals() == 200 && idata(mcu, 0x31) == 2);
        CHECK(xdata(mcu, 0x0300) == 0xee);
    }
    {
        Cc2530F256Mcu mcu("dbg");
        CHECK(throws_invalid([&] { mcu.set_debugger(0x0046, {}); }));
        CHECK(throws_invalid([&] { mcu.set_debugger(0x0046, {{0, 0x0100, {1}}}); }));
        CHECK(throws_invalid([&] { mcu.set_debugger(0x0046, {{1, 0x1fff, {1, 2}}}); }));
    }
}

void test_cc2530_debugger_config() {
    const std::string image = mailbox_image();
    auto system = sim::build_system(sim::parse_system_config(with_firmware(
        "firmware: \"" + image +
        "\", debugger: {pc: 0x0046, writes: [{hit: 3, xdata: 0x0100, hex: \"5a\"}, "
        "{hit: 4, xdata: 0x0100, hex: \"a5\"}, {hit: 4, xdata: 0x200, hex: \"01 02\"}]}")));
    system->world().run_until(100000);
    sim::Mcu &mcu = system->world().mcu("d.m");
    CHECK(idata(mcu, 0x30) == 0xff && idata(mcu, 0x31) == 2);
    CHECK(xdata(mcu, 0x0200) == 0x01 && xdata(mcu, 0x0201) == 0x02);
    CHECK(dynamic_cast<Cc2530F256Mcu &>(mcu).debugger_arrivals() == 4);

    const auto with_debugger = [&](const std::string &debugger) {
        return error_of_build(with_firmware("firmware: \"" + image + "\", debugger: " + debugger));
    };
    CHECK(contains(with_debugger("{writes: [{xdata: 0, hex: \"00\"}]}"), "requires 'pc'"));
    CHECK(contains(with_debugger("{pc: 0x10000, writes: [{xdata: 0, hex: \"00\"}]}"),
                   "16-bit"));
    CHECK(contains(with_debugger("{pc: 0x46, writes: []}"), "non-empty 'writes'"));
    CHECK(contains(with_debugger("{pc: 0x46, writes: [{hit: 0, xdata: 0, hex: \"00\"}]}"),
                   "positive arrival count"));
    CHECK(contains(with_debugger("{pc: 0x46, writes: [{xdata: 0, hex: \"0\"}]}"),
                   "even number"));
    CHECK(contains(with_debugger("{pc: 0x46, writes: [{xdata: 0, hex: \"zz\"}]}"),
                   "non-hex"));
    CHECK(contains(with_debugger("{pc: 0x46, writes: [{xdata: 0x2000, hex: \"00\"}]}"),
                   "leaves XDATA SRAM"));
    CHECK(contains(with_debugger("{pc: 0x46, writes: [{xdata: 0, hex: \"00\", idata: 1}]}"),
                   "idata"));
    CHECK(contains(error_of_build("version: 1\ndevices:\n  - id: d\n    mcus:\n      - {id: m, "
                                  "model: ca51f253l3, firmware: \"" + image +
                                  "\", debugger: {pc: 0, writes: [{xdata: 0, hex: \"00\"}]}}\n"),
                   "only supported for cc2530f256"));
}

// The `report` hook writes exactly what `mcuemu run --report-hits` prints at
// the same arrivals (state before that arrival's writes), independent of how
// the run is split into quanta, and the breakpoint stays armed until the last
// reported arrival.
void test_cc2530_debugger_report() {
    const std::string image = mailbox_image();
    const auto read_text = [](const std::filesystem::path &path) {
        std::ifstream in(path);
        std::ostringstream text;
        text << in.rdbuf();
        return text.str();
    };
    // Reference: the CLI with the same breakpoint, writes and dumps.
    std::string cli;
    {
        std::vector<std::string> args{"mcuemu", "run", "--model", "CC2530F256", "--firmware",
                                      image, "--steps", "100000", "--state", "--until-pc",
                                      "0x0046", "--hits", "5", "--report-hits",
                                      "--write-xdata", "3:0x0100:5a",
                                      "--dump-xdata", "0x0100:4", "--dump-xdata", "0x01fe:20"};
        std::vector<char *> argv;
        for (std::string &arg : args)
            argv.push_back(arg.data());
        std::ostringstream captured;
        std::streambuf *saved = std::cout.rdbuf(captured.rdbuf());
        const int rc = mcs51emu::run_command(static_cast<int>(argv.size()), argv.data());
        std::cout.rdbuf(saved);
        CHECK(rc == 0);
        const std::string out = captured.str();
        cli = out.substr(0, out.find("model: CC2530F256\n"));
    }
    CHECK(cli.find("hit: 1 instructions=3 ") == 0);
    CHECK(cli.find("end-hit: 5\n") != std::string::npos);
    // Hit 3 shows the mailbox before its write, hit 4 after the firmware
    // consumed it (ADD/MOV/INC/CLR/MOVX/SJMP: 3 + 2*3 + 9 instructions).
    CHECK(cli.find("hit: 4 instructions=18 ") != std::string::npos);
    CHECK(cli.find("xdata 0x0100: 00 00 00 00\n") != std::string::npos);
    CHECK(cli.find("idata 0x0030: 5a 01 03") != std::string::npos);

    for (const std::uint64_t quantum : {std::uint64_t{0}, std::uint64_t{20}, std::uint64_t{97}}) {
        const std::filesystem::path file = scratch() / ("report-" + std::to_string(quantum) + ".txt");
        Cc2530F256Mcu mcu("dbg");
        mcu.load_firmware(image);
        mcu.set_debugger(0x0046, {{3, 0x0100, {0x5a}}},
                         Cc2530F256Mcu::DebuggerReport{
                             file.string(), 5, {true, {{0x0100, 4}, {0x01fe, 20}}}});
        if (quantum == 0) {
            mcu.run_until(100000);
        } else {
            for (std::uint64_t t = quantum; t <= 100000; t += quantum)
                mcu.run_until(t);
        }
        CHECK(read_text(file) == cli);
        // Removed after the last reported arrival; the loop keeps running.
        CHECK(mcu.debugger_arrivals() == 5);
        CHECK(idata(mcu, 0x32) > 5);
    }
    {
        // Report only, state off, XDATA only: no writes needed.
        const std::filesystem::path file = scratch() / "report-only.txt";
        Cc2530F256Mcu mcu("dbg");
        mcu.load_firmware(image);
        mcu.set_debugger(0x0046, {}, Cc2530F256Mcu::DebuggerReport{file.string(), 2, {false, {{0x0100, 1}}}});
        mcu.run_until(100000);
        CHECK(mcu.debugger_arrivals() == 2);
        const std::string text = read_text(file);
        CHECK(text.find("hit: 1 instructions=3 ") == 0);
        CHECK(text.find("hit: 2 instructions=6 ") != std::string::npos);
        CHECK(text.find("idata") == std::string::npos && text.find("sfr") == std::string::npos);
        CHECK(text.find("xdata 0x0100: 00\nend-hit: 2\n") != std::string::npos);
        // A power-on reset re-arms the breakpoint and appends a new count.
        mcu.reset();
        mcu.run_until(100000);
        const std::string again = read_text(file);
        CHECK(again.size() == 2 * text.size() && again.substr(text.size()) == text);
    }
    {
        Cc2530F256Mcu mcu("dbg");
        CHECK(throws_invalid([&] {
            mcu.set_debugger(0x0046, {}, Cc2530F256Mcu::DebuggerReport{(scratch() / "x").string(), 0, {}});
        }));
        CHECK(throws_invalid([&] {
            mcu.set_debugger(0x0046, {}, Cc2530F256Mcu::DebuggerReport{
                                             (scratch() / "x").string(), 1, {true, {{0xfff0, 0x20}}}});
        }));
        CHECK(throws_invalid([&] {
            mcu.set_debugger(0x0046, {}, Cc2530F256Mcu::DebuggerReport{
                                             (scratch() / "no-such-dir" / "x").string(), 1, {}});
        }));
    }

    // Configuration: the file is resolved against the YAML directory.
    const std::filesystem::path file = scratch() / "report-config.txt";
    std::filesystem::remove(file);
    auto system = sim::build_system(sim::parse_system_config(
        with_firmware("firmware: \"" + image +
                      "\", debugger: {pc: 0x0046, writes: [{hit: 3, xdata: 0x0100, hex: \"5a\"}], "
                      "report: {file: report-config.txt, hits: 5, "
                      "xdata: [{address: 0x0100, length: 4}, {address: 0x1fe, length: 20}]}}"),
        scratch().string()));
    system->world().run_until(100000);
    CHECK(read_text(file) == cli);
    CHECK(dynamic_cast<Cc2530F256Mcu &>(system->world().mcu("d.m")).debugger_arrivals() == 5);

    const auto with_debugger = [&](const std::string &debugger) {
        return error_of_build(with_firmware("firmware: \"" + image + "\", debugger: " + debugger));
    };
    const std::string out = (scratch() / "r.txt").string();
    CHECK(error_of_build(with_firmware("firmware: \"" + image +
                                       "\", debugger: {pc: 0x46, report: {file: \"" + out +
                                       "\"}}")).empty());
    CHECK(contains(with_debugger("{pc: 0x46, report: {hits: 1}}"), "file"));
    CHECK(contains(with_debugger("{pc: 0x46, report: {file: \"" + out + "\", hits: 0}}"),
                   "positive arrival count"));
    CHECK(contains(with_debugger("{pc: 0x46, report: {file: \"" + out + "\", state: maybe}}"),
                   "true or false"));
    CHECK(contains(with_debugger("{pc: 0x46, report: {file: \"" + out +
                                 "\", xdata: [{address: 0xfff0, length: 0x20}]}}"),
                   "64 KiB XDATA"));
    CHECK(contains(with_debugger("{pc: 0x46, report: {file: \"" + out +
                                 "\", xdata: [{address: 0}]}}"),
                   "'address' and 'length'"));
    CHECK(contains(with_debugger("{pc: 0x46, report: {file: \"" + out + "\", every: 1}}"),
                   "every"));
    CHECK(contains(with_debugger("{pc: 0x46, writes: [], report: {file: \"" + out + "\"}}"),
                   "non-empty 'writes'"));
    CHECK(contains(with_debugger("{pc: 0x46, report: {file: \"/nonexistent-dir/r.txt\"}}"),
                   "cannot create report file"));
}

// The adapter with the real USART: a host byte reaches U0DBUF one frame after
// it is queued and the firmware's echo leaves one frame after the write.
// Expected timing comes from SWRU191F 17.4 equation 3: BAUD_M = 216, BAUD_E =
// 11 at 32 MHz is 115203 baud, so a 10-bit frame (start, 8 data, 1 stop with
// the reset U0UCR) lasts 86.80 us; a byte is given to the host when its frame
// ends (U0CSR.TX_BYTE).
void test_cc2530_uart_host_link() {
    Cc2530F256Mcu mcu("uart");
    mcu.load_firmware(raw_image(
        "uart.bin", {0x75, 0x81, 0x60,       // 0x00 MOV SP,#0x60
                     0x75, 0xc6, 0x80,       // 0x03 MOV CLKCONCMD,#0x80 (32 MHz XOSC)
                     0xe5, 0x9e,             // 0x06 MOV A,CLKCONSTA
                     0xb4, 0x80, 0xfb,       // 0x08 CJNE A,#0x80,0x06
                     0x75, 0xf1, 0x00,       // 0x0b MOV PERCFG,#0 (USART0 alt 1)
                     0x43, 0xf3, 0x0c,       // 0x0e ORL P0SEL,#0x0c (P0_2 RX, P0_3 TX)
                     0x75, 0xc2, 0xd8,       // 0x11 MOV U0BAUD,#216
                     0x75, 0xc5, 0x0b,       // 0x14 MOV U0GCR,#11
                     0x75, 0x86, 0xc0,       // 0x17 MOV U0CSR,#0xc0 (UART, RE)
                     0x75, 0xc1, 0x55,       // 0x1a MOV U0DBUF,#0x55
                     0xe5, 0x86,             // 0x1d MOV A,U0CSR
                     0x30, 0xe2, 0xfb,       // 0x1f JNB ACC.2,0x1d (RX_BYTE)
                     0xe5, 0xc1,             // 0x22 MOV A,U0DBUF
                     0x90, 0x01, 0x00,       // 0x24 MOV DPTR,#0x0100
                     0xf0,                   // 0x27 MOVX @DPTR,A
                     0x04,                   // 0x28 INC A
                     0xf5, 0xc1,             // 0x29 MOV U0DBUF,A
                     0x80, 0xfe}));          // 0x2b SJMP $
    // The receiver is off at reset: the byte is refused, not queued.
    CHECK(!mcu.uart_rx(0, 0x11));
    const auto pc = [&] { return le(&snapshot(mcu)[Cc2530F256Mcu::CpuSnapshotPc], 2); };
    std::uint64_t t = 0;
    while (pc() < 0x1d && t < 2000000)
        mcu.run_until(t += 250);
    const std::uint64_t written = t; // U0DBUF written within the last 250 ns
    CHECK(written < 2000000);
    std::uint64_t sent = 0;
    while (sent == 0 && t < written + 200000) {
        mcu.run_until(t += 250);
        const auto bytes = mcu.uart_take_tx(0);
        if (!bytes.empty()) {
            CHECK(bytes.size() == 1 && bytes[0] == 0x55);
            sent = t;
        }
    }
    CHECK(sent - written >= 86500 && sent - written <= 87300);
    CHECK(mcu.uart_take_tx(1).empty());

    const std::uint64_t queued = t;
    CHECK(mcu.uart_rx(0, 0xa5));
    mcu.run_until(queued + 80000); // still receiving
    CHECK(xdata(mcu, 0x0100) == 0);
    mcu.run_until(queued + 95000);
    CHECK(xdata(mcu, 0x0100) == 0xa5);
    CHECK(mcu.uart_take_tx(0).empty()); // echo still in flight
    mcu.run_until(queued + 190000);
    const auto echo = mcu.uart_take_tx(0);
    CHECK(echo.size() == 1 && echo[0] == 0xa6);
    CHECK(mcu.state() == sim::McuState::Running); // a used UART can still wake it
}

// System level: YAML nets and parts on CC2530 pads with the real I/O and ADC.
// Mailbox at XDATA 0x0080: 1 = P0 -> 0x0081, 2 = convert AIN0 (internal 1.15 V
// reference, 512 decimation) -> ADCL/ADCH at 0x0082/0x0083, 3 = CPL P1_0,
// 4 = P2INP.PDUP0 (port 0 pull-down). P1_0 is an output from boot.
void test_cc2530_system_gpio_adc_parts() {
    const std::string image = raw_image(
        "gpio_adc.bin",
        {0x75, 0x81, 0x60,              // 0x00 MOV SP,#0x60
         0x75, 0xc6, 0x80,              // 0x03 MOV CLKCONCMD,#0x80
         0xe5, 0x9e,                    // 0x06 MOV A,CLKCONSTA
         0xb4, 0x80, 0xfb,              // 0x08 CJNE A,#0x80,0x06
         0x75, 0xf2, 0x01,              // 0x0b MOV APCFG,#0x01
         0x43, 0xfe, 0x01,              // 0x0e ORL P1DIR,#0x01
         0x90, 0x00, 0x80,              // 0x11 MOV DPTR,#0x0080
         0xe0,                          // 0x14 MOVX A,@DPTR
         0x60, 0xfa,                    // 0x15 JZ 0x11
         0xb4, 0x01, 0x06,              // 0x17 CJNE A,#1,0x20
         0xe5, 0x80,                    // 0x1a MOV A,P0
         0xa3,                          // 0x1c INC DPTR
         0xf0,                          // 0x1d MOVX @DPTR,A
         0x80, 0x24,                    // 0x1e SJMP 0x44
         0xb4, 0x02, 0x14,              // 0x20 CJNE A,#2,0x37
         0x75, 0xb6, 0x30,              // 0x23 MOV ADCCON3,#0x30
         0xe5, 0xb4,                    // 0x26 MOV A,ADCCON1
         0x30, 0xe7, 0xfb,              // 0x28 JNB ACC.7,0x26 (EOC)
         0x90, 0x00, 0x82,              // 0x2b MOV DPTR,#0x0082
         0xe5, 0xba,                    // 0x2e MOV A,ADCL
         0xf0,                          // 0x30 MOVX @DPTR,A
         0xa3,                          // 0x31 INC DPTR
         0xe5, 0xbb,                    // 0x32 MOV A,ADCH
         0xf0,                          // 0x34 MOVX @DPTR,A
         0x80, 0x0d,                    // 0x35 SJMP 0x44
         0xb4, 0x03, 0x04,              // 0x37 CJNE A,#3,0x3e
         0xb2, 0x90,                    // 0x3a CPL P1.0
         0x80, 0x06,                    // 0x3c SJMP 0x44
         0xb4, 0x04, 0x03,              // 0x3e CJNE A,#4,0x44
         0x43, 0xf7, 0x20,              // 0x41 ORL P2INP,#0x20
         0x90, 0x00, 0x80,              // 0x44 MOV DPTR,#0x0080
         0xe4,                          // 0x47 CLR A
         0xf0,                          // 0x48 MOVX @DPTR,A
         0x80, 0xc6});                  // 0x49 SJMP 0x11
    auto system = sim::build_system(sim::parse_system_config(
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: cc2530f256, firmware: \"" + image + "\"}\n"
        "    nets:\n"
        "      - {id: led, pins: [m.P1.0]}\n"
        "      - {id: key, pins: [m.P0.1]}\n"
        "      - {id: open, pins: [m.P0.3]}\n"
        "      - {id: adc, pins: [m.P0.0]}\n"
        "    parts:\n"
        "      - {id: lamp, type: led, pins: {in: led}}\n"
        "      - {id: btn, type: button, pins: {out: key}}\n"
        "      - {id: ntc, type: ntc, pins: {out: adc}, params: {supply_mv: 1150}}\n"));
    sim::World &world = system->world();
    sim::Mcu &mcu = world.mcu("d.m");
    const auto command = [&](std::uint8_t cmd, std::uint64_t run_ns) {
        mcu.write_memory(Cc2530F256Mcu::XdataWindow + 0x0080, &cmd, 1);
        world.run_for(run_ns);
        CHECK(xdata(mcu, 0x0080) == 0); // acknowledged
    };
    const auto bool_of = [&](const std::string &endpoint) {
        return std::get<bool>(world.endpoints().at(endpoint).get());
    };
    // P1_0 has no pull (SWRU191F 7.3): the net floats until P1DIR makes it an
    // output with the reset latch 1.
    CHECK(world.net("d.led").level() == sim::Level::Floating);
    world.run_until(50000);
    CHECK(mcu.state() == sim::McuState::Running);
    CHECK(world.net("d.led").level() == sim::Level::High && bool_of("d.lamp.on"));
    // Inputs reset with pull-ups (P0INP = 0, P2INP.PDUP0 = 0): a released button
    // and an otherwise open net read high and the pads pull their nets up.
    CHECK(world.net("d.key").level() == sim::Level::High);
    CHECK(world.net("d.open").level() == sim::Level::High);
    command(1, 10000);
    CHECK((xdata(mcu, 0x0081) & 0xfe) == 0xfe);
    world.endpoints().at("d.btn.pressed").set(sim::Value{1.0});
    CHECK(world.net("d.key").level() == sim::Level::Low);
    command(1, 10000);
    CHECK((xdata(mcu, 0x0081) & 0xfe) == 0xfc);
    world.endpoints().at("d.btn.pressed").set(sim::Value{0.0});
    command(4, 10000);
    CHECK(world.net("d.key").level() == sim::Level::Low); // pad pulls down now
    CHECK(world.net("d.open").level() == sim::Level::Low);
    command(1, 10000);
    CHECK((xdata(mcu, 0x0081) & 0xfe) == 0x00);
    command(3, 10000);
    CHECK(world.net("d.led").level() == sim::Level::Low && !bool_of("d.lamp.on"));

    // ADC: 12 bits from 512 decimation, two's complement MSB-aligned in
    // ADCH:ADCL (SWRU191F 12.2.5); code = floor(V * 2048 / 1.15 V). One
    // conversion is 132 us (12.2.7), so 400 us covers it.
    const auto expect_code = [&](std::uint32_t mv) {
        const std::int64_t code =
            std::min<std::int64_t>(std::int64_t{mv} * 1000 * 2048 / 1150000, 2047);
        return static_cast<std::uint16_t>(code << 4);
    };
    const auto adc_result = [&] {
        return static_cast<std::uint16_t>(xdata(mcu, 0x0083) << 8 | xdata(mcu, 0x0082));
    };
    CHECK(world.net("d.adc").millivolts() == 575); // R_ntc == R_series at 25 C
    command(2, 400000);
    CHECK(adc_result() == 0x4000 && adc_result() == expect_code(575));
    world.endpoints().at("d.ntc.temp_c").set(sim::Value{60.0});
    const std::uint32_t hot = world.net("d.adc").millivolts();
    CHECK(hot > 100 && hot < 575);
    command(2, 400000);
    CHECK(adc_result() == expect_code(hot));
    world.endpoints().at("d.ntc.temp_c").set(sim::Value{-40.0});
    CHECK(world.net("d.adc").millivolts() > 1100);
    command(2, 400000);
    CHECK(adc_result() == expect_code(world.net("d.adc").millivolts()));
    CHECK(mcu.state() == sim::McuState::Running);
}

// The MCU's `supply_mv` is AVDD5 (SWRS081B: all supply pins at the same
// voltage). One conversion of channel 15 = AVDD5/3 against the internal
// 1.15 V reference, 512 decimation (SWRU191F 12.2.1, 12.2.3, 12.2.10):
// code = floor(AVDD5 / 3 * 2048 / 1.15 V), result at XDATA 0x0082/0x0083.
void test_cc2530_system_supply_mv() {
    const std::string image = raw_image(
        "supply.bin",
        {0x75, 0x81, 0x60,              // 0x00 MOV SP,#0x60
         0x75, 0xc6, 0x80,              // 0x03 MOV CLKCONCMD,#0x80
         0xe5, 0x9e,                    // 0x06 MOV A,CLKCONSTA
         0xb4, 0x80, 0xfb,              // 0x08 CJNE A,#0x80,0x06
         0x75, 0xb6, 0x3f,              // 0x0b MOV ADCCON3,#0x3F
         0xe5, 0xb4,                    // 0x0e MOV A,ADCCON1
         0x30, 0xe7, 0xfb,              // 0x10 JNB ACC.7,0x0e (EOC)
         0x90, 0x00, 0x82,              // 0x13 MOV DPTR,#0x0082
         0xe5, 0xba,                    // 0x16 MOV A,ADCL
         0xf0,                          // 0x18 MOVX @DPTR,A
         0xa3,                          // 0x19 INC DPTR
         0xe5, 0xbb,                    // 0x1a MOV A,ADCH
         0xf0,                          // 0x1c MOVX @DPTR,A
         0x80, 0xfe});                  // 0x1d SJMP 0x1d
    const auto vdd3_code = [&](const std::string &extra) {
        auto system = sim::build_system(sim::parse_system_config(
            "version: 1\ndevices:\n  - id: d\n    mcus:\n"
            "      - {id: m, model: cc2530f256, firmware: \"" + image + "\"" + extra + "}\n"));
        sim::World &world = system->world();
        world.run_until(1'000'000);
        sim::Mcu &mcu = world.mcu("d.m");
        CHECK(mcu.state() == sim::McuState::Halted && contains(mcu.fault_message(), "idle self-loop at pc=0x001d"));
        return static_cast<std::uint16_t>(xdata(mcu, 0x0083) << 8 | xdata(mcu, 0x0082));
    };
    const auto expect = [](std::uint32_t supply_mv) {
        const std::int64_t code =
            std::min<std::int64_t>(std::int64_t{supply_mv} * 1000 / 3 * 2048 / 1150000, 2047);
        return static_cast<std::uint16_t>(code << 4);
    };
    // Default: 3300 mV, the same supply as nets and parts.
    CHECK(vdd3_code("") == expect(3300) && expect(3300) == (1958 << 4));
    CHECK(vdd3_code(", supply_mv: 3000") == expect(3000) && expect(3000) == (1780 << 4));
    CHECK(vdd3_code(", supply_mv: 2000") == expect(2000));
}

// USART1 SPI master, alternative 1 (SWRU191F Table 7-1: MO P0_4, MI P0_5,
// C P0_3; P2DIR.PRIP0 = 01 gives USART1 port 0 priority), with a net wiring
// MO to MI as in firmware/emut/emut_usart.yaml. 17.2.1: MOSI shifts out
// while MISO shifts in, so every received byte equals the sent byte. Bytes
// i ^ 0x5a (i = 0-7: CPOL 0, CPHA 0, MSB first; i = 8-15: CPOL 1, CPHA 1,
// LSB first; BAUD_M 0, BAUD_E 17 = 4 MHz SCK, 17.4) are stored at XDATA
// 0x0100 + i after URX1IF (TCON.7).
void test_cc2530_system_spi_loopback() {
    const std::string image = raw_image(
        "spi_loop.bin",
        {0x75, 0x81, 0x60,              // 0x00 MOV SP,#0x60
         0x75, 0xc6, 0x80,              // 0x03 MOV CLKCONCMD,#0x80
         0xe5, 0x9e,                    // 0x06 MOV A,CLKCONSTA
         0xb4, 0x80, 0xfb,              // 0x08 CJNE A,#0x80,0x06
         0x75, 0xff, 0x40,              // 0x0b MOV P2DIR,#0x40 (PRIP0 = 01)
         0x75, 0xf3, 0x38,              // 0x0e MOV P0SEL,#0x38
         0x75, 0xfa, 0x00,              // 0x11 MOV U1BAUD,#0x00
         0x75, 0xfc, 0x31,              // 0x14 MOV U1GCR,#0x31
         0x90, 0x01, 0x00,              // 0x17 MOV DPTR,#0x0100
         0x7f, 0x00,                    // 0x1a MOV R7,#0
         0xef,                          // 0x1c MOV A,R7
         0x64, 0x5a,                    // 0x1d XRL A,#0x5a
         0xf5, 0xf9,                    // 0x1f MOV U1DBUF,A
         0x30, 0x8f, 0xfd,              // 0x21 JNB TCON.7,0x21 (URX1IF)
         0xc2, 0x8f,                    // 0x24 CLR TCON.7
         0xe5, 0xf9,                    // 0x26 MOV A,U1DBUF
         0xf0,                          // 0x28 MOVX @DPTR,A
         0xa3,                          // 0x29 INC DPTR
         0x0f,                          // 0x2a INC R7
         0xbf, 0x08, 0xee,              // 0x2b CJNE R7,#8,0x1c
         0x75, 0xfc, 0xd1,              // 0x2e MOV U1GCR,#0xd1
         0xef,                          // 0x31 MOV A,R7
         0x64, 0x5a,                    // 0x32 XRL A,#0x5a
         0xf5, 0xf9,                    // 0x34 MOV U1DBUF,A
         0x30, 0x8f, 0xfd,              // 0x36 JNB TCON.7,0x36
         0xc2, 0x8f,                    // 0x39 CLR TCON.7
         0xe5, 0xf9,                    // 0x3b MOV A,U1DBUF
         0xf0,                          // 0x3d MOVX @DPTR,A
         0xa3,                          // 0x3e INC DPTR
         0x0f,                          // 0x3f INC R7
         0xbf, 0x10, 0xee,              // 0x40 CJNE R7,#16,0x31
         0x80, 0xfe});                  // 0x43 SJMP 0x43
    auto system = sim::build_system(sim::parse_system_config(
        "version: 1\ndevices:\n  - id: d\n    mcus:\n"
        "      - {id: m, model: cc2530f256, firmware: \"" + image + "\"}\n"
        "    nets:\n"
        "      - {id: spi_loop, pins: [m.P0.4, m.P0.5]}\n"));
    sim::World &world = system->world();
    sim::Mcu &mcu = world.mcu("d.m");
    world.run_until(1'000'000);
    // A net is attached, so the self-loop does not halt the core.
    CHECK(mcu.state() == sim::McuState::Running);
    CHECK(le(&snapshot(mcu)[Cc2530F256Mcu::CpuSnapshotPc], 2) == 0x0043);
    for (unsigned i = 0; i < 16; ++i)
        CHECK(xdata(mcu, static_cast<std::uint16_t>(0x0100 + i)) == (i ^ 0x5a));
}

// SWRU191F 3.1/3.2 through the nets: a debugger on the RESET_N, P2.2 (DC)
// and P2.1 (DD) nets enters debug mode, reads the status and executes an
// instruction; the adapter's host port and its errors.
void test_cc2530_debug_if_pads() {
    Cc2530F256Mcu mcu("dbgif");
    mcu.load_firmware(raw_image("dbgif.bin", {0x75, 0x81, 0x60,   // MOV SP,#0x60
                                              0x80, 0xfe}));      // SJMP $
    sim::Net reset("reset"), dc("dc"), dd("dd");
    mcu.connect_pin("RESET_N", reset.attach("dbgif.RESET_N"));
    mcu.connect_pin("P2.2", dc.attach("dbgif.P2.2"));
    mcu.connect_pin("P2.1", dd.attach("dbgif.P2.1"));
    sim::Net spare("spare");
    CHECK(throws_invalid([&] { mcu.connect_pin("RESET_N", spare.attach("again")); }));
    CHECK(mcu.pin_names().size() == 21); // RESET_N is not a GPIO pad
    sim::Pin &rst = reset.attach("probe.reset");
    sim::Pin &clk = dc.attach("probe.dc");
    sim::Pin &data = dd.attach("probe.dd");

    // DC at 1 MHz: 500 ns per half period.
    std::uint64_t t = 1000;
    const auto drive = [&](sim::Pin &pin, sim::Drive level) {
        CHECK(mcu.run_until(t) == sim::McuState::Running);
        pin.set_drive(level, t);
        t += 500;
    };
    const auto pulse = [&] {
        drive(clk, sim::Drive::High);
        drive(clk, sim::Drive::Low);
    };
    rst.set_drive(sim::Drive::High, 0);
    drive(clk, sim::Drive::Low);
    drive(rst, sim::Drive::Low);
    CHECK(throws_runtime([&] { mcu.debug_if_enter(); })); // RESET_N held low
    pulse();
    pulse();
    drive(rst, sim::Drive::High);
    CHECK(mcu.device().debug_if_active() && mcu.device().debug_if_halted());

    const auto command = [&](std::vector<std::uint8_t> bytes) {
        for (const std::uint8_t byte : bytes)
            for (int bit = 7; bit >= 0; --bit) {
                drive(data, (byte >> bit) & 1u ? sim::Drive::High : sim::Drive::Low);
                pulse();
            }
        drive(data, sim::Drive::HiZ);
        unsigned rounds = 0;
        while (dd.level() != sim::Level::Low && rounds++ < 100)
            for (unsigned i = 0; i < 8; ++i)
                pulse();
        std::vector<std::uint8_t> response;
        for (unsigned i = 0; i < mcs51emu::Cc2530F256::debug_if_output_length(bytes.front()); ++i) {
            std::uint8_t value = 0;
            for (unsigned bit = 0; bit < 8; ++bit) {
                drive(clk, sim::Drive::High);
                value = static_cast<std::uint8_t>(value << 1 | (dd.level() == sim::Level::High));
                drive(clk, sim::Drive::Low);
            }
            response.push_back(value);
        }
        CHECK(dd.level() == sim::Level::Floating); // DD released after the response
        return response;
    };
    CHECK(command({0x30}) == std::vector<std::uint8_t>{0x22});       // READ_STATUS
    CHECK(command({0x68}).at(0) == 0xa5);                            // GET_CHIP_ID
    CHECK(command({0x52, 0x74, 0x3c}) == std::vector<std::uint8_t>{0x3c}); // MOV A,#0x3c
    CHECK(command({0x58}) == std::vector<std::uint8_t>{0x3c});       // STEP_INSTR
    CHECK(command({0x28}) == (std::vector<std::uint8_t>{0x00, 0x03})); // GET_PC

    // The host port of the adapter, on the same chip.
    mcu.debug_if_submit({0x30});
    CHECK(!mcu.debug_if_take_response());
    CHECK(mcu.run_until(mcu.debug_if_due_ns()) == sim::McuState::Running);
    const auto response = mcu.debug_if_take_response();
    CHECK(response && response->accepted && response->bytes == std::vector<std::uint8_t>{0x22});
    // A RESET_N pulse without DC edges: the firmware starts.
    drive(rst, sim::Drive::Low);
    drive(rst, sim::Drive::High);
    CHECK(!mcu.device().debug_if_active());
    CHECK(mcu.run_until(t + 10000) == sim::McuState::Running);
    CHECK(mcu.device().pc() == 0x0003);
    mcu.debug_if_enter();
    CHECK(mcu.device().debug_if_halted() && mcu.device().pc() == 0x0000);
    mcu.debug_if_reset_pulse();
    CHECK(!mcu.device().debug_if_active());
}

} // namespace mcuemu_tests
