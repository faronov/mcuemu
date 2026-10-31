// CC2530F256 SoC model tests. Every check executes real MCS-51 machine code
// on the Unicorn core; the programs are hand-encoded with the mnemonic beside
// each instruction.
#include "mcs51emu/cc2530f256.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <optional>
#include <utility>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using mcs51emu::Cc2530F256;
using mcs51emu::Cc2530RunResult;
using mcs51emu::Cc2530PowerMode;

unsigned failures = 0;

#define CHECK(condition)                                                       \
    do {                                                                       \
        if (!(condition)) {                                                    \
            std::cerr << __FILE__ << ':' << __LINE__                           \
                      << ": check failed: " #condition << '\n';                \
            ++failures;                                                       \
        }                                                                      \
    } while (false)

constexpr std::uint8_t SP = 0x81, PCON = 0x87, TCON = 0x88, MPAGE = 0x93,
                       S1CON = 0x9b, IEN2 = 0x9a, SLEEPSTA = 0x9d,
                       CLKCONSTA = 0x9e, FMAP = 0x9f, P2 = 0xa0, IEN0 = 0xa8,
                       IP0 = 0xa9, IEN1 = 0xb8, IP1 = 0xb9, SLEEPCMD = 0xbe,
                       IRCON = 0xc0, CLKCONCMD = 0xc6, MEMCTR = 0xc7,
                       IRCON2 = 0xe8;

// Physical flash image under construction.
struct Flash {
    std::vector<std::uint8_t> bytes =
        std::vector<std::uint8_t>(Cc2530F256::FlashSize, 0xff);
    std::uint32_t cursor = 0;

    // Common CODE (bank 0, logical == physical).
    Flash &at(std::uint32_t physical) {
        cursor = physical;
        return *this;
    }
    // Logical upper-window address `logical` of FMAP bank `bank`.
    Flash &bank(unsigned bank, std::uint16_t logical) {
        cursor = bank * 0x8000u + (logical - 0x8000u);
        return *this;
    }
    Flash &op(std::initializer_list<std::uint8_t> code) {
        for (const std::uint8_t byte : code)
            bytes.at(cursor++) = byte;
        return *this;
    }
    // Logical address of the cursor when it is in the common area.
    std::uint16_t here() const { return static_cast<std::uint16_t>(cursor); }
};

void load(Cc2530F256 &device, const Flash &flash) { device.load_flash(0, flash.bytes); }

std::uint8_t hi(std::uint16_t value) { return static_cast<std::uint8_t>(value >> 8); }
std::uint8_t lo(std::uint16_t value) { return static_cast<std::uint8_t>(value); }

// Everything a program can observe: registers, counters, SFRs, all SRAM.
std::vector<std::uint8_t> snapshot(const Cc2530F256 &device) {
    const auto regs = device.registers();
    std::vector<std::uint8_t> state{lo(regs.pc), hi(regs.pc), regs.a, regs.b, regs.psw,
                                    regs.sp, regs.dpl, regs.dph, regs.fmap, regs.memctr,
                                    regs.mpage, regs.dpl1, regs.dph1, regs.dps,
                                    regs.interrupt_levels, regs.interrupt_inhibit,
                                    lo(regs.posted_vector), hi(regs.posted_vector),
                                    regs.posted_level,
                                    static_cast<std::uint8_t>(regs.posted_source)};
    for (unsigned shift = 0; shift < 64; shift += 8) {
        state.push_back(static_cast<std::uint8_t>(regs.cycles >> shift));
        state.push_back(static_cast<std::uint8_t>(regs.instructions >> shift));
    }
    for (unsigned address = 0x80; address < 0x100; ++address)
        state.push_back(device.read_sfr(static_cast<std::uint8_t>(address)));
    for (unsigned address = 0; address < Cc2530F256::SramSize; ++address)
        state.push_back(device.peek_xdata(static_cast<std::uint16_t>(address)));
    return state;
}

std::filesystem::path temp_file(const char *name) {
    return std::filesystem::temp_directory_path() / (std::string("mcuemu-cc2530-") + name);
}

std::string hex_record(std::uint8_t type, std::uint16_t offset,
                       const std::vector<std::uint8_t> &data) {
    std::vector<std::uint8_t> record{static_cast<std::uint8_t>(data.size()), hi(offset),
                                     lo(offset), type};
    record.insert(record.end(), data.begin(), data.end());
    unsigned sum = 0;
    for (const std::uint8_t byte : record)
        sum += byte;
    record.push_back(static_cast<std::uint8_t>(-sum));
    std::ostringstream text;
    text << ':' << std::hex << std::uppercase;
    for (const std::uint8_t byte : record)
        text << (byte < 16 ? "0" : "") << unsigned(byte);
    return text.str();
}

// --------------------------------------------------------------------------

void test_reset_state() {
    Cc2530F256 device;
    CHECK(device.pc() == 0);
    CHECK(device.read_sfr(SP) == 0x07);
    CHECK(device.fmap() == 1);
    CHECK(device.memctr() == 0);
    CHECK(device.read_sfr(MPAGE) == 0);
    CHECK(device.read_sfr(0x80) == 0xff && device.read_sfr(0x90) == 0xff &&
          device.read_sfr(P2) == 0x1f);
    CHECK(device.read_sfr(TCON) == 0x05);
    CHECK(device.read_sfr(CLKCONCMD) == 0xc9 && device.read_sfr(CLKCONSTA) == 0xc9);
    CHECK(device.read_sfr(SLEEPCMD) == 0x04);
    CHECK((device.read_sfr(SLEEPSTA) & 0x18) == 0x00); // RST = power-on
    CHECK(device.peek_xdata(0x624a) == 0xa5);          // CHIPID
    CHECK(device.peek_xdata(0x6276) == 0x44);          // CHIPINFO0: 256 KiB
    CHECK(device.peek_xdata(0x6277) == 0x07);          // CHIPINFO1: 8 KiB SRAM
    CHECK(device.code_physical(0x8000) == 0x8000u);    // FMAP = 1
    CHECK(device.read_code(0x8000) == 0xff);
    // SWRU191F 2.3.1 / Table 2-1: DPTR0, DPTR1 and DPS reset to 0.
    CHECK(device.read_sfr(0x82) == 0 && device.read_sfr(0x83) == 0);
    CHECK(device.read_sfr(0x84) == 0 && device.read_sfr(0x85) == 0);
    CHECK(device.read_sfr(0x92) == 0);
}

void test_all_fmap_banks_and_common_area() {
    Flash flash;
    flash.at(0x0000).op({0x75, SP, 0x30}); // MOV SP,#0x30
    for (std::uint8_t bank = 0; bank < 8; ++bank) {
        flash.op({0x75, FMAP, bank,                       // MOV FMAP,#bank
                  0x12, 0xf0, 0x00,                       // LCALL 0xF000
                  0xf5, static_cast<std::uint8_t>(0x40 + bank)}); // MOV 0x40+bank,A
    }
    const std::uint16_t end = flash.here();
    flash.op({0x80, 0xfe}); // SJMP $
    for (std::uint8_t bank = 0; bank < 8; ++bank)
        flash.bank(bank, 0xf000).op({0x74, static_cast<std::uint8_t>(0x10 + bank), // MOV A,#
                                     0x22});                                       // RET

    Cc2530F256 device;
    load(device, flash);
    const std::vector<std::uint8_t> common{device.read_code(0), device.read_code(1),
                                           device.read_code(2)};
    const Cc2530RunResult run = device.run(200);
    CHECK(run.error.empty());
    CHECK(run.pc == end);
    CHECK(device.idle_forever());
    for (std::uint8_t bank = 0; bank < 8; ++bank)
        CHECK(device.read_idata(static_cast<std::uint8_t>(0x40 + bank)) == 0x10 + bank);
    CHECK(device.fmap() == 7);
    CHECK(device.read_sfr(SP) == 0x30);
    CHECK(device.code_physical(0xf000) == 7u * 0x8000u + 0x7000u);
    CHECK(device.code_physical(0x7fff) == 0x7fffu);
    CHECK((std::vector<std::uint8_t>{device.read_code(0), device.read_code(1),
                                     device.read_code(2)}) == common);
    CHECK(device.diagnostics().fmap_writes == 8);
}

// The LLVM/SDCC CC2530 bank thunk: PUSH FMAP; MOV FMAP,#n; LCALL f; POP FMAP;
// RET, executed from bank-1 code, plus an FMAP write that redirects the very
// next fetch while executing inside the banked window.
Flash banked_call_program() {
    Flash flash;
    flash.at(0x0000).op({0x75, SP, 0x30,       // MOV SP,#0x30
                         0x75, FMAP, 0x01,     // MOV FMAP,#1
                         0x12, 0x80, 0x00,     // LCALL 0x8000 (bank 1)
                         0xf5, 0x50,           // MOV 0x50,A
                         0x85, FMAP, 0x51,     // MOV 0x51,FMAP
                         0x85, SP, 0x52,       // MOV 0x52,SP
                         0x02, 0x90, 0x00});   // LJMP 0x9000 (bank 1)
    flash.at(0x0100).op({0xc0, FMAP,           // PUSH FMAP
                         0x75, FMAP, 0x02,     // MOV FMAP,#2
                         0x12, 0x80, 0x00,     // LCALL 0x8000 (bank 2)
                         0xd0, FMAP,           // POP FMAP
                         0x22});               // RET
    flash.bank(1, 0x8000).op({0x12, 0x01, 0x00, // LCALL thunk
                              0x24, 0x01,       // ADD A,#1
                              0x22});           // RET
    flash.bank(2, 0x8000).op({0x74, 0x40,       // MOV A,#0x40
                              0x85, FMAP, 0x53, // MOV 0x53,FMAP
                              0x22});           // RET
    flash.bank(1, 0x9000).op({0x75, FMAP, 0x03, // MOV FMAP,#3: next fetch is bank 3
                              0x74, 0x11,       // (bank 1, never executed)
                              0x80, 0xfe});
    flash.bank(3, 0x9003).op({0x74, 0x33,       // MOV A,#0x33
                              0xf5, 0x54,       // MOV 0x54,A
                              0x80, 0xfe});     // SJMP $ at 0x9007
    return flash;
}

void test_cross_bank_thunk_and_switch_while_banked() {
    Cc2530F256 device;
    load(device, banked_call_program());
    const Cc2530RunResult run = device.run(100);
    CHECK(run.error.empty());
    CHECK(run.pc == 0x9007);
    CHECK(run.fmap == 3);
    CHECK(device.read_idata(0x50) == 0x41);
    CHECK(device.read_idata(0x51) == 1);
    CHECK(device.read_idata(0x52) == 0x30); // stack balanced
    CHECK(device.read_idata(0x53) == 2);
    CHECK(device.read_idata(0x54) == 0x33);
    CHECK(device.code_physical(0x9007) == 3u * 0x8000u + 0x1007u);
    CHECK(device.idle_forever());

    std::ostringstream trace;
    Cc2530F256 traced;
    load(traced, banked_call_program());
    traced.set_trace(&trace);
    const Cc2530RunResult traced_run = traced.run(100);
    CHECK(traced_run.error.empty());
    CHECK(trace.str().find("pc=0x8000 fmap=2 phys=0x10000  74 40") != std::string::npos);
    CHECK(trace.str().find("pc=0x9003 fmap=3 phys=0x19003  74 33") != std::string::npos);
}

void test_xbank_is_independent_of_fmap() {
    Flash flash;
    flash.at(0x0000).op({0x75, FMAP, 0x01,     // MOV FMAP,#1
                         0x75, MEMCTR, 0x05,   // MOV MEMCTR,#5 (XBANK 5)
                         0x90, 0x80, 0x10,     // MOV DPTR,#0x8010
                         0xe0,                 // MOVX A,@DPTR -> flash bank 5
                         0xf5, 0x40,           // MOV 0x40,A
                         0xe4,                 // CLR A
                         0x93,                 // MOVC A,@A+DPTR -> CODE bank 1
                         0xf5, 0x41,           // MOV 0x41,A
                         0x74, 0x99,           // MOV A,#0x99
                         0xf0,                 // MOVX @DPTR,A (flash: ignored)
                         0xe0,                 // MOVX A,@DPTR
                         0xf5, 0x42,           // MOV 0x42,A
                         0x75, MEMCTR, 0xf6,   // MOV MEMCTR,#0xF6 (bits 7:4 read 0)
                         0xe0,                 // MOVX A,@DPTR -> flash bank 6
                         0xf5, 0x43,           // MOV 0x43,A
                         0x80, 0xfe});
    flash.at(1u * 0x8000u + 0x10).op({0x11});
    flash.at(5u * 0x8000u + 0x10).op({0x55});
    flash.at(6u * 0x8000u + 0x10).op({0x66});

    Cc2530F256 device;
    load(device, flash);
    const Cc2530RunResult run = device.run(100);
    CHECK(run.error.empty());
    CHECK(device.read_idata(0x40) == 0x55);
    CHECK(device.read_idata(0x41) == 0x11);
    CHECK(device.read_idata(0x42) == 0x55);
    CHECK(device.read_idata(0x43) == 0x66);
    CHECK(device.memctr() == 0x06);
    CHECK(device.fmap() == 1);
    CHECK(device.diagnostics().read_only_xdata_writes == 1);
    CHECK(device.diagnostics().memctr_writes == 2);
}

void test_sram_aliases_mpage_and_sfr_alias() {
    Flash flash;
    flash.at(0x0000).op({0x75, 0x30, 0x5a,     // MOV 0x30,#0x5A
                         0x90, 0x1f, 0x30,     // MOV DPTR,#0x1F30 (IDATA alias)
                         0xe0,                 // MOVX A,@DPTR
                         0xf5, 0x40,           // MOV 0x40,A
                         0x74, 0xc3,           // MOV A,#0xC3
                         0xa3,                 // INC DPTR
                         0xf0,                 // MOVX @DPTR,A -> IDATA 0x31
                         0x85, 0x31, 0x41,     // MOV 0x41,0x31
                         0x75, P2, 0x00,       // MOV P2,#0 (must not page MOVX @Ri)
                         0x75, MPAGE, 0x1f,    // MOV MPAGE,#0x1F
                         0x78, 0x30,           // MOV R0,#0x30
                         0xe2,                 // MOVX A,@R0 -> XDATA 0x1F30
                         0xf5, 0x42,           // MOV 0x42,A
                         0x75, MPAGE, 0x01,    // MOV MPAGE,#1
                         0x74, 0x77,           // MOV A,#0x77
                         0xf2,                 // MOVX @R0,A -> XDATA 0x0130
                         0x90, 0x70, 0x9f,     // MOV DPTR,#0x709F (FMAP alias)
                         0x74, 0x04,           // MOV A,#4
                         0xf0,                 // MOVX @DPTR,A -> FMAP = 4
                         0x90, 0x70, 0xe0,     // MOV DPTR,#0x70E0 (ACC: CPU-internal)
                         0xe0,                 // MOVX A,@DPTR -> 0
                         0xf5, 0x43,           // MOV 0x43,A
                         0x90, 0x70, 0x90,     // MOV DPTR,#0x7090 (P1 alias)
                         0xe0,                 // MOVX A,@DPTR -> 0xFF
                         0xf5, 0x44,           // MOV 0x44,A
                         0x74, 0x12,           // MOV A,#0x12
                         0xf0,                 // MOVX @DPTR,A (read-only alias)
                         0x80, 0xfe});

    Cc2530F256 device;
    load(device, flash);
    const Cc2530RunResult run = device.run(100);
    CHECK(run.error.empty());
    CHECK(device.read_idata(0x40) == 0x5a);
    CHECK(device.read_idata(0x41) == 0xc3);
    CHECK(device.read_idata(0x42) == 0x5a);
    CHECK(device.peek_xdata(0x0130) == 0x77);
    CHECK(device.peek_xdata(0x0030) == 0x00);
    CHECK(device.peek_xdata(0x1f00) == 0x30); // R0 of bank 0
    CHECK(device.fmap() == 4);
    CHECK(device.read_idata(0x43) == 0x00);
    CHECK(device.read_idata(0x44) == 0xff);
    CHECK(device.read_sfr(0x90) == 0xff);
    CHECK(device.diagnostics().read_only_xdata_writes == 1);
    CHECK(device.diagnostics().unmapped_xdata_reads == 1);
    CHECK(device.diagnostics().fmap_writes == 1);
}

void test_xmap_executes_from_sram_and_restores_flash() {
    Flash flash;
    flash.at(0x0000).op({0x75, SP, 0x30,       // MOV SP,#0x30
                         0x90, 0x00, 0x00,     // MOV DPTR,#0
                         0x74, 0x74, 0xf0,     // SRAM[0] = 0x74 (MOV A,#)
                         0xa3, 0x74, 0x77, 0xf0, // SRAM[1] = 0x77
                         0xa3, 0x74, 0x22, 0xf0, // SRAM[2] = 0x22 (RET)
                         0x75, MEMCTR, 0x08,   // MOV MEMCTR,#0x08 (XMAP)
                         0x12, 0x80, 0x00,     // LCALL 0x8000 -> SRAM
                         0xf5, 0x40,           // MOV 0x40,A
                         0x90, 0x00, 0x01,     // MOV DPTR,#1
                         0x74, 0x78, 0xf0,     // patch SRAM[1] = 0x78 while mapped
                         0x12, 0x80, 0x00,     // LCALL 0x8000 -> SRAM again
                         0xf5, 0x41,           // MOV 0x41,A
                         0x12, 0xa0, 0x00,     // LCALL 0xA000 -> flash bank 1
                         0xf5, 0x42,           // MOV 0x42,A
                         0x75, MEMCTR, 0x00,   // MOV MEMCTR,#0
                         0x12, 0x80, 0x00,     // LCALL 0x8000 -> flash bank 1
                         0xf5, 0x43,           // MOV 0x43,A
                         0x80, 0xfe});
    flash.bank(1, 0x8000).op({0x74, 0x81, 0x22});
    flash.bank(1, 0xa000).op({0x74, 0xa1, 0x22});

    Cc2530F256 device;
    load(device, flash);
    device.set_breakpoint(0x8000);
    Cc2530RunResult run = device.run(100);
    CHECK(run.break_hit && run.pc == 0x8000);
    CHECK(device.memctr() == 0x08);
    CHECK(!device.code_physical(0x8000));
    CHECK(device.code_physical(0xa000) == 0xa000u);
    CHECK(device.read_code(0x8000) == 0x74 && device.read_code(0x8001) == 0x77);
    device.set_breakpoint(std::nullopt);
    run = device.run(100);
    CHECK(run.error.empty());
    CHECK(device.read_idata(0x40) == 0x77);
    CHECK(device.read_idata(0x41) == 0x78);
    CHECK(device.read_idata(0x42) == 0xa1);
    CHECK(device.read_idata(0x43) == 0x81);
    CHECK(device.read_sfr(SP) == 0x30);
    CHECK(device.idle_forever());
}

// T1 interrupt (vector 0x4B) taken while executing bank 3; the ISR records
// FMAP, calls into bank 5 and restores FMAP before RETI.
Flash banked_irq_program() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});   // LJMP 0x0100
    flash.at(0x004b).op({0x02, 0x02, 0x00});   // T1 vector: LJMP 0x0200
    flash.at(0x0100).op({0x75, SP, 0x30,       // MOV SP,#0x30
                         0x75, FMAP, 0x03,     // MOV FMAP,#3
                         0x02, 0x80, 0x00});   // LJMP 0x8000 (bank 3)
    flash.at(0x0200).op({0x85, FMAP, 0x61,     // MOV 0x61,FMAP
                         0x85, SP, 0x62,       // MOV 0x62,SP
                         0x05, 0x63,           // INC 0x63
                         0x85, 0x64, 0x67,     // MOV 0x67,0x64
                         0x75, FMAP, 0x05,     // MOV FMAP,#5
                         0x12, 0x80, 0x00,     // LCALL 0x8000 (bank 5)
                         0x75, FMAP, 0x03,     // MOV FMAP,#3
                         0x32});               // RETI
    flash.bank(5, 0x8000).op({0x74, 0x55, 0xf5, 0x65, 0x22}); // MOV A,#0x55; MOV 0x65,A; RET
    flash.bank(3, 0x8000).op({0x75, IEN1, 0x02,   // MOV IEN1,#0x02 (T1IE)
                              0xd2, 0xaf,         // SETB EA
                              0x43, IRCON, 0x02,  // ORL IRCON,#0x02 (T1IF)
                              0x00,               // NOP
                              0x75, 0x64, 0xaa,   // MOV 0x64,#0xAA
                              0x85, SP, 0x66,     // MOV 0x66,SP
                              0x80, 0xfe});       // SJMP $ at 0x800F
    return flash;
}

void test_interrupt_in_banked_code_and_reti() {
    Cc2530F256 device;
    load(device, banked_irq_program());
    const Cc2530RunResult run = device.run(200);
    CHECK(run.error.empty());
    CHECK(run.pc == 0x800f);
    CHECK(run.fmap == 3);
    CHECK(device.read_idata(0x61) == 3);    // interrupted bank
    CHECK(device.read_idata(0x62) == 0x32); // return address pushed
    CHECK(device.read_idata(0x63) == 1);    // taken exactly once
    CHECK(device.read_idata(0x65) == 0x55); // banked call from the ISR
    CHECK(device.read_idata(0x67) == 0x00); // before the post-NOP instruction
    CHECK(device.read_idata(0x64) == 0xaa); // resumed after RETI
    CHECK(device.read_idata(0x66) == 0x30); // stack balanced
    CHECK((device.read_sfr(IRCON) & 0x02) == 0); // T1IF cleared by hardware
    CHECK(device.diagnostics().interrupts_taken == 1);
    CHECK(device.idle_forever());
}

// Six simultaneous requests; each ISR appends its source number at @R1.
Flash priority_program(std::uint8_t ip0, std::uint8_t ip1) {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});                         // LJMP 0x0100
    flash.at(0x0003).op({0x77, 0, 0x09, 0x32});                      // RFERR (HW clear)
    flash.at(0x000b).op({0x77, 1, 0x09, 0x32});                      // ADC (HW clear)
    flash.at(0x0033).op({0x77, 6, 0x09, 0x53, IRCON2, 0xfe, 0x32});  // P2INT: clear P2IF
    flash.at(0x0043).op({0x77, 8, 0x09, 0x53, IRCON, 0xfe, 0x32});   // DMA: clear DMAIF
    flash.at(0x004b).op({0x77, 9, 0x09, 0x32});                      // T1 (HW clear)
    flash.at(0x0083).op({0x77, 16, 0x09, 0x53, S1CON, 0xfc, 0x32});  // RF: clear RFIF
    flash.at(0x0100).op({0x75, SP, 0x30,          // MOV SP,#0x30
                         0x79, 0x40,              // MOV R1,#0x40
                         0x75, IP0, ip0,          // MOV IP0,#ip0
                         0x75, IP1, ip1,          // MOV IP1,#ip1
                         0x75, IEN0, 0x03,        // RFERRIE | ADCIE, EA = 0
                         0x75, IEN1, 0x03,        // DMAIE | T1IE
                         0x75, IEN2, 0x03,        // RFIE | P2IE
                         0x43, TCON, 0x22,        // RFERRIF | ADCIF
                         0x43, IRCON, 0x03,       // DMAIF | T1IF
                         0x43, IRCON2, 0x01,      // P2IF
                         0x43, S1CON, 0x01,       // RFIF_0
                         0xd2, 0xaf,              // SETB EA
                         0x00,                    // NOP
                         0x80, 0xfe});            // SJMP $ at 0x0123
    return flash;
}

std::vector<std::uint8_t> interrupt_order(std::uint8_t ip0, std::uint8_t ip1, bool trace) {
    Cc2530F256 device;
    load(device, priority_program(ip0, ip1));
    std::ostringstream sink;
    if (trace)
        device.set_trace(&sink);
    const Cc2530RunResult run = device.run(400);
    CHECK(run.error.empty());
    CHECK(run.pc == 0x0123);
    CHECK(device.diagnostics().interrupts_taken == 6);
    CHECK((device.read_sfr(TCON) & 0x22) == 0);
    CHECK((device.read_sfr(IRCON) & 0x03) == 0);
    CHECK(device.read_sfr(SP) == 0x30);
    std::vector<std::uint8_t> order;
    for (std::uint8_t address = 0x40; address < device.read_idata(0x01); ++address)
        order.push_back(device.read_idata(address));
    return order;
}

void test_interrupt_priority_groups_and_polling_order() {
    // SWRU191F Table 2-8 polling sequence at one level:
    // RFERR, RF, DMA, ADC, T1, ..., P2INT.
    const std::vector<std::uint8_t> same_level{0, 16, 8, 1, 9, 6};
    CHECK(interrupt_order(0x00, 0x00, false) == same_level);
    CHECK(interrupt_order(0x00, 0x00, true) == same_level);
    // IPG1 (ADC, T1, P2INT) at level 3 first, then IPG0 at level 0.
    const std::vector<std::uint8_t> group1_high{1, 9, 6, 0, 16, 8};
    CHECK(interrupt_order(0x02, 0x02, false) == group1_high);
    CHECK(interrupt_order(0x02, 0x02, true) == group1_high);
    // IPG0 level 1 still beats IPG1 level 0 ... and level 2 beats level 1.
    CHECK(interrupt_order(0x01, 0x00, false) == same_level);
    CHECK(interrupt_order(0x01, 0x02, false) == group1_high);
}

void test_global_enable_gates_and_retracts_requests() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});              // LJMP 0x0100
    flash.at(0x0043).op({0x02, 0x03, 0x00});              // DMA vector: LJMP 0x0300
    flash.at(0x004b).op({0x85, 0x70, 0x72,                // T1: MOV 0x72,0x70
                         0x85, 0x74, 0x76,                //     MOV 0x76,0x74
                         0x05, 0x77, 0x32});              //     INC 0x77; RETI
    flash.at(0x0100).op({0x75, SP, 0x30,                  // MOV SP,#0x30
                         0x75, IEN1, 0x03,                // DMAIE | T1IE
                         0xd2, 0xaf,                      // SETB EA
                         0xc2, 0xaf,                      // CLR EA
                         0x43, IRCON, 0x02,               // T1IF with EA = 0
                         0x00,                            // NOP
                         0x75, 0x70, 0x01,                // MOV 0x70,#1
                         0xd2, 0xaf,                      // SETB EA -> T1
                         0x00,                            // NOP
                         0x75, 0x71, 0x01,                // MOV 0x71,#1
                         0x43, IRCON, 0x01,               // DMAIF -> DMA ISR
                         0x00,                            // NOP
                         0x75, 0x74, 0x01,                // MOV 0x74,#1 (T1 pending, EA = 0)
                         0xd2, 0xaf,                      // SETB EA -> T1 again
                         0x00,                            // NOP
                         0x75, 0x75, 0x01,                // MOV 0x75,#1
                         0x80, 0xfe});
    // DMA ISR: raise T1IF while in service (posted, not taken), clear DMAIF
    // and EA, then RETI: the posted T1 request must be withdrawn.
    flash.at(0x0300).op({0x43, IRCON, 0x02,               // ORL IRCON,#0x02
                         0x53, IRCON, 0xfe,               // ANL IRCON,#0xFE
                         0xc2, 0xaf,                      // CLR EA
                         0x32});                          // RETI

    for (const bool trace : {false, true}) {
        Cc2530F256 device;
        load(device, flash);
        std::ostringstream sink;
        if (trace)
            device.set_trace(&sink);
        const Cc2530RunResult run = device.run(400);
        CHECK(run.error.empty());
        CHECK(device.idle_forever());
        CHECK(device.read_idata(0x77) == 2);   // T1 taken twice
        CHECK(device.read_idata(0x72) == 1);   // only after EA was set again
        CHECK(device.read_idata(0x76) == 1);   // second time: after 0x74 was written
        CHECK(device.read_idata(0x75) == 1);
        CHECK(device.diagnostics().interrupts_taken == 3);
    }
}

void test_reset_during_isr_returns_to_power_on_interrupt_state() {
    Flash flash;
    flash.at(0x0000).op({0x75, SP, 0x30,       // MOV SP,#0x30
                         0x05, 0x50,           // INC 0x50
                         0x75, IEN1, 0x02,     // T1IE
                         0xd2, 0xaf,           // SETB EA
                         0x43, IRCON, 0x02,    // T1IF
                         0x00,                 // NOP
                         0x80, 0xfe});
    flash.at(0x004b).op({0x02, 0x02, 0x00});   // LJMP 0x0200
    flash.at(0x0200).op({0x80, 0xfe});         // ISR never returns

    Cc2530F256 device;
    load(device, flash);
    Cc2530RunResult run = device.run(100);
    CHECK(run.error.empty() && run.pc == 0x0200);
    CHECK(device.read_idata(0x50) == 1);
    device.reset();
    CHECK(device.pc() == 0);
    CHECK((device.read_sfr(SLEEPSTA) & 0x18) == 0x08); // RST = external reset
    CHECK(device.read_sfr(SP) == 0x07);
    CHECK(device.read_idata(0x50) == 1); // SRAM kept
    run = device.run(100);
    CHECK(run.error.empty() && run.pc == 0x0200); // the ISR is entered again
    CHECK(device.read_idata(0x50) == 2);
    CHECK(device.diagnostics().interrupts_taken == 2);
    device.power_on_reset();
    CHECK(device.read_idata(0x50) == 0);
    CHECK((device.read_sfr(SLEEPSTA) & 0x18) == 0x00);
    CHECK(device.registers().cycles == 0 && device.registers().instructions == 0);
}

void test_pcon_idle_sleeps_until_an_interrupt() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x004b).op({0x05, 0x60, 0x32});   // T1: INC 0x60; RETI
    flash.at(0x0100).op({0x75, SP, 0x30,       // MOV SP,#0x30
                         0x75, IEN1, 0x02,     // T1IE
                         0xd2, 0xaf,           // SETB EA
                         0x75, SLEEPCMD, 0xfc, // MOV SLEEPCMD,#0xFC: IDLE (bits 6:3 read 0)
                         0x75, PCON, 0x01,     // MOV PCON,#1 -> power mode
                         0x05, 0x61,           // INC 0x61 at 0x010E
                         0x80, 0xfe});

    Cc2530F256 device;
    load(device, flash);
    Cc2530RunResult run = device.run(100);
    CHECK(run.error.empty());
    CHECK(run.sleeping);
    CHECK(run.pc == 0x010e);
    CHECK(device.read_sfr(PCON) == 0);       // IDLE reads 0
    CHECK(device.read_sfr(SLEEPCMD) == 0x84);
    CHECK(device.power_mode() == Cc2530PowerMode::Idle);
    CHECK(device.read_idata(0x61) == 0);
    run = device.run(100);
    CHECK(run.sleeping && run.instructions == 0);
    device.write_sfr(IRCON, 0x02);           // host raises T1IF
    run = device.run(100);
    CHECK(run.error.empty() && !run.sleeping);
    CHECK(device.power_mode() == Cc2530PowerMode::Active);
    CHECK(device.read_idata(0x60) == 1);
    CHECK(device.read_idata(0x61) == 1);
    CHECK(device.idle_forever());
}

// SWRU191F 4.1: PM1 ends on reset, an I/O port interrupt or the sleep timer,
// not on other enabled interrupts.
void test_pm1_wakes_only_on_port_or_sleep_timer() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x002b).op({0x05, 0x62, 0x53, IRCON, 0x7f, 0x32}); // ST: INC 0x62; clear STIF; RETI
    flash.at(0x004b).op({0x05, 0x60, 0x32});                    // T1: INC 0x60; RETI
    flash.at(0x0100).op({0x75, SP, 0x30,       // MOV SP,#0x30
                         0x75, IEN1, 0x02,     // T1IE
                         0x75, IEN0, 0xa0,     // EA, STIE
                         0x75, SLEEPCMD, 0xfd, // MODE = PM1
                         0x75, PCON, 0x01,     // MOV PCON,#1 -> PM1
                         0x05, 0x61,           // INC 0x61
                         0x80, 0xfe});

    Cc2530F256 device;
    load(device, flash);
    // SWRU191F 11.2: with STIE = 1 the default sleep timer compare (0xFFFFFF)
    // would wake PM1 after 2^24 - 1 LF periods, so bound the first run.
    Cc2530RunResult run = device.run(100, 1000);
    CHECK(run.error.empty() && run.sleeping);
    CHECK(device.read_sfr(SLEEPCMD) == 0x85);
    CHECK(device.power_mode() == Cc2530PowerMode::Pm1);
    device.write_sfr(IRCON, 0x02);           // T1IF: no PM1 wake source
    run = device.run(100, device.registers().cycles + 1000);
    CHECK(run.error.empty() && run.sleeping && run.instructions == 0);
    CHECK(device.read_idata(0x60) == 0);
    device.write_sfr(IRCON, 0x82);           // STIF wakes; T1 is taken too
    run = device.run(100);
    CHECK(run.error.empty() && !run.sleeping);
    CHECK(device.power_mode() == Cc2530PowerMode::Active);
    CHECK(device.read_idata(0x62) == 1);
    CHECK(device.read_idata(0x60) == 1);
    CHECK(device.read_idata(0x61) == 1);
    CHECK(device.diagnostics().power_mode_entries_by_mode[2] == 1);
    CHECK(device.diagnostics().power_mode_cycles_by_mode[2] >= 1000);
}

void test_clock_and_status_registers() {
    Flash flash;
    flash.at(0x0000).op({0x75, CLKCONCMD, 0x80,   // 32 MHz XOSC, 32 MHz
                         0x85, CLKCONSTA, 0x40,
                         0x75, CLKCONCMD, 0xc0,   // RC OSC: TICKSPD/CLKSPD >= 16 MHz
                         0x85, CLKCONSTA, 0x41,
                         0x75, CLKCONSTA, 0x00,   // read-only
                         0x85, CLKCONSTA, 0x42,
                         0x75, SLEEPSTA, 0xff,    // read-only
                         0x85, SLEEPSTA, 0x43,
                         0x75, FMAP, 0xff,        // bits 7:3 read 0
                         0x85, FMAP, 0x44,
                         0x75, FMAP, 0x01,
                         0x80, 0xfe});
    Cc2530F256 device;
    load(device, flash);
    const Cc2530RunResult run = device.run(100);
    CHECK(run.error.empty());
    CHECK(device.read_idata(0x40) == 0x80);
    CHECK(device.read_idata(0x41) == 0xc9);
    CHECK(device.read_idata(0x42) == 0xc9);
    CHECK(device.read_idata(0x43) == 0x60); // only XOSC_STB and HFRC_STB are set
    CHECK(device.read_idata(0x44) == 0x07);
    CHECK(device.diagnostics().read_only_sfr_writes == 2);
}

void test_xreg_chip_id_info_page_and_unmapped_xdata() {
    Flash flash;
    flash.at(0x0000).op({0x90, 0x62, 0x4a,               // DPTR = CHIPID
                         0xe0, 0xf5, 0x40,               // -> 0x40
                         0x74, 0x00, 0xf0,               // write ignored
                         0xe0, 0xf5, 0x41,               // -> 0x41
                         0x90, 0x62, 0x76,               // DPTR = CHIPINFO0
                         0xe0, 0xf5, 0x42,
                         0xa3, 0xe0, 0xf5, 0x43,         // CHIPINFO1
                         0x90, 0x61, 0xd8,               // unmodeled XREG (reserved, SWRU191F Table 23-5)
                         0x74, 0x3c, 0xf0,
                         0xe0, 0xf5, 0x44,
                         0x90, 0x50, 0x00,               // unmapped XDATA
                         0x74, 0x11, 0xf0,
                         0xe0, 0xf5, 0x45,
                         0x90, 0x78, 0x0c,               // info page IEEE address LSB
                         0xe0, 0xf5, 0x46,
                         0x74, 0x00, 0xf0,               // info page: read-only
                         0x80, 0xfe});
    Cc2530F256 device;
    load(device, flash);
    device.set_ieee_address(0x00124b0001020304ull);
    const Cc2530RunResult run = device.run(100);
    CHECK(run.error.empty());
    CHECK(device.read_idata(0x40) == 0xa5);
    CHECK(device.read_idata(0x41) == 0xa5);
    CHECK(device.read_idata(0x42) == 0x44);
    CHECK(device.read_idata(0x43) == 0x07);
    CHECK(device.read_idata(0x44) == 0x3c);
    CHECK(device.read_idata(0x45) == 0x00);
    CHECK(device.read_idata(0x46) == 0x04);
    CHECK(device.peek_xdata(0x7813) == 0x00); // IEEE MSB
    CHECK(device.peek_xdata(0x7812) == 0x12);
    const auto &diag = device.diagnostics();
    CHECK(diag.read_only_xdata_writes == 2);
    CHECK(diag.unmapped_xdata_writes == 1);
    CHECK(diag.unmapped_xdata_reads == 1);
    CHECK(diag.unmodeled_xreg_writes.at(0x61d8) == 1);
    CHECK(diag.unmodeled_xreg_reads.at(0x61d8) == 1);
    CHECK(!diag.unmodeled_xreg_reads.count(0x624a));
}

void test_unsupported_behavior_stops_with_an_error() {
    {
        // EMU-04A: DPS selects DPTR1 (SWRU191F 2.3.1); it is no longer an
        // unsupported-feature stop.
        Flash flash;
        flash.at(0).op({0x75, 0x92, 0x00,   // MOV DPS,#0: DPTR0
                        0x75, 0x92, 0x01,   // MOV DPS,#1: DPTR1
                        0x00, 0x80, 0xfe});
        Cc2530F256 device;
        load(device, flash);
        const Cc2530RunResult run = device.run(100);
        CHECK(run.error.empty());
        CHECK(run.pc == 7);
        CHECK(run.step_limit_reached);
        CHECK(device.read_sfr(0x92) == 0x01);
    }
    {
        Flash flash;
        flash.at(0).op({0x00, 0xa5, 0x00}); // NOP; reserved opcode 0xA5
        Cc2530F256 device;
        load(device, flash);
        const Cc2530RunResult run = device.run(100);
        CHECK(run.error.find("UC_ERR_INSN_INVALID") != std::string::npos);
        CHECK(run.pc == 1);
        // The core counts the attempted (faulting) instruction.
        CHECK(run.instructions == 2);
    }
}

// Fast interpretation and the per-instruction trace hook must produce the
// same machine state at every stop.
void check_equivalence(const Flash &flash, std::size_t steps) {
    Cc2530F256 fast;
    Cc2530F256 traced;
    Cc2530F256 fast_step;
    Cc2530F256 traced_step;
    for (Cc2530F256 *device : {&fast, &traced, &fast_step, &traced_step})
        load(*device, flash);
    std::ostringstream sink;
    traced.set_trace(&sink);
    traced_step.set_trace(&sink);
    fast.run(steps);
    traced.run(steps);
    CHECK(snapshot(fast) == snapshot(traced));
    bool lockstep = true;
    for (std::size_t i = 0; i < steps; ++i) {
        const Cc2530RunResult a = fast_step.run(1);
        const Cc2530RunResult b = traced_step.run(1);
        lockstep = lockstep && a.instructions == b.instructions && a.pc == b.pc &&
                   a.cycles == b.cycles && a.error == b.error &&
                   snapshot(fast_step) == snapshot(traced_step);
    }
    CHECK(lockstep);
    CHECK(snapshot(fast) == snapshot(fast_step));
    CHECK(fast.diagnostics().interrupts_taken == traced_step.diagnostics().interrupts_taken);

    for (const std::uint64_t cycle_limit : {1ull, 7ull, 23ull, 50ull, 101ull}) {
        Cc2530F256 a;
        Cc2530F256 b;
        load(a, flash);
        load(b, flash);
        b.set_trace(&sink);
        const Cc2530RunResult ra = a.run(steps, cycle_limit);
        const Cc2530RunResult rb = b.run(steps, cycle_limit);
        CHECK(ra.instructions == rb.instructions && ra.cycles == rb.cycles &&
              ra.pc == rb.pc);
        CHECK(ra.cycles >= cycle_limit || ra.instructions == steps || !ra.error.empty() ||
              ra.sleeping);
        CHECK(snapshot(a) == snapshot(b));
    }
}

void test_trace_and_fast_execution_are_equivalent() {
    check_equivalence(banked_call_program(), 40);
    check_equivalence(banked_irq_program(), 60);
    check_equivalence(priority_program(0x00, 0x00), 80);
    check_equivalence(priority_program(0x02, 0x02), 80);
}

void test_breakpoint_semantics_match_in_both_modes() {
    for (const bool trace : {false, true}) {
        Cc2530F256 device;
        load(device, banked_irq_program());
        std::ostringstream sink;
        if (trace)
            device.set_trace(&sink);
        // The ISR body: stop before its first instruction executes.
        device.set_breakpoint(0x0200);
        Cc2530RunResult run = device.run(200);
        CHECK(run.break_hit && run.error.empty() && run.pc == 0x0200);
        CHECK(device.read_idata(0x63) == 0);
        CHECK((device.read_sfr(IRCON) & 0x02) == 0); // flag cleared on vectoring
        const std::size_t first = run.instructions;
        // Resuming executes the breakpoint instruction.
        run = device.run(1);
        CHECK(!run.break_hit && run.instructions == 1 && run.pc == 0x0203);
        // A breakpoint in a bank-3 address stops on the return path.
        device.set_breakpoint(0x8009);
        run = device.run(200);
        CHECK(run.break_hit && run.pc == 0x8009 && run.fmap == 3);
        CHECK(device.read_idata(0x63) == 1 && device.read_idata(0x64) == 0);
        CHECK(first > 0);
    }
}

// ---- EMU-04A golden vectors: dual DPTR ------------------------------------

// --------------------------------------------------------------------------
// EMU-04A golden vectors. Every expected value below is derived by hand from
// SWRU191F (2.3.1 data pointers; 2.5 interrupts, Tables 2-5..2-8) and the
// Intel MCS-51 RET/RETI definitions, never from emulator output. Checks
// marked MODEL-DEFINED pin a choice the references leave open
// (emu04a-cc2530-cpu-design.md 5.2, docs/cc2530.md).

constexpr std::uint8_t DPL0 = 0x82, DPH0 = 0x83, DPL1 = 0x84, DPH1 = 0x85, DPS = 0x92,
                       S0CON = 0x98;

// A program and the address of its final SJMP $.
struct Program {
    Flash flash;
    std::uint16_t end{};
};

// Interrupt programs append markers at @R1 (bank 0 R1 = IDATA 0x01) from 0x40.
std::vector<std::uint8_t> irq_log(const Cc2530F256 &device) {
    std::vector<std::uint8_t> log;
    for (unsigned address = 0x40; address < device.read_idata(0x01); ++address)
        log.push_back(device.read_idata(static_cast<std::uint8_t>(address)));
    return log;
}

// 64 iterations; each stores through DPTR0, toggles DPS with INC DPS (bits
// 7:1 read 0, so 1 + 1 = 2 reads back as 0), stores through DPTR1 and
// toggles back: 128 pointer switches.
Program dptr_alternation_program() {
    Program p;
    p.flash.at(0x0000).op({0x75, SP, 0x30,       // MOV SP,#0x30
                           0x75, DPS, 0x00,      // MOV DPS,#0
                           0x90, 0x01, 0x00,     // MOV DPTR,#0x0100 (DPTR0)
                           0x75, DPS, 0x01,      // MOV DPS,#1
                           0x90, 0x03, 0x00,     // MOV DPTR,#0x0300 (DPTR1)
                           0x75, DPS, 0x00,      // MOV DPS,#0
                           0x7f, 0x40,           // MOV R7,#64
                           0x7e, 0x00});         // MOV R6,#0
    p.flash.at(0x0016).op({0xee,                 // loop: MOV A,R6
                           0x64, 0x5a,           // XRL A,#0x5A
                           0xf0,                 // MOVX @DPTR,A (DPTR0)
                           0xa3,                 // INC DPTR
                           0x05, DPS,            // INC DPS: 0 -> 1
                           0xee,                 // MOV A,R6
                           0xf4,                 // CPL A
                           0xf0,                 // MOVX @DPTR,A (DPTR1)
                           0xa3,                 // INC DPTR
                           0x05, DPS,            // INC DPS: 1 -> 2, reads 0
                           0x0e,                 // INC R6
                           0xdf, 0xf0,           // DJNZ R7,loop (0x0024 -> 0x0016)
                           0x75, DPS, 0x01,      // MOV DPS,#1
                           0x90, 0x03, 0x05,     // MOV DPTR,#0x0305 (DPTR1)
                           0xe0,                 // MOVX A,@DPTR: ~5 = 0xFA
                           0xf5, 0x40,           // MOV 0x40,A
                           0x75, DPS, 0x00,      // MOV DPS,#0
                           0xe0,                 // MOVX A,@DPTR: DPTR0 = 0x0140 (guard)
                           0xf5, 0x41,           // MOV 0x41,A
                           0x85, DPL0, 0x42,     // MOV 0x42,DPL0
                           0x85, DPH0, 0x43,     // MOV 0x43,DPH0
                           0x85, DPL1, 0x44,     // MOV 0x44,DPL1
                           0x85, DPH1, 0x45,     // MOV 0x45,DPH1
                           0x85, DPS, 0x46,      // MOV 0x46,DPS
                           0x75, DPS, 0xff,      // MOV DPS,#0xFF
                           0x85, DPS, 0x47,      // MOV 0x47,DPS: 0x01
                           0xe0,                 // MOVX A,@DPTR: DPTR1 -> 0xFA
                           0xf5, 0x48,           // MOV 0x48,A
                           0x75, DPS, 0xfe,      // MOV DPS,#0xFE: bit 0 = 0
                           0x85, DPS, 0x49,      // MOV 0x49,DPS: 0x00
                           0xe0,                 // MOVX A,@DPTR: DPTR0 -> guard
                           0xf5, 0x4a});         // MOV 0x4A,A
    p.end = p.flash.here();
    p.flash.op({0x80, 0xfe});                    // SJMP $
    return p;
}

void test_dual_dptr_alternation_movx_and_dps() {
    const Program program = dptr_alternation_program();
    for (const bool trace : {false, true}) {
        Cc2530F256 device;
        load(device, program.flash);
        for (const std::uint16_t guard : {0x00ff, 0x0140, 0x02ff, 0x0340})
            device.write_sram(guard, 0xee);
        std::ostringstream sink;
        if (trace)
            device.set_trace(&sink);
        const Cc2530RunResult run = device.run(2000);
        CHECK(run.error.empty());
        CHECK(run.pc == program.end && device.idle_forever());
        bool stored = true;
        for (unsigned i = 0; i < 64; ++i) {
            stored = stored &&
                     device.peek_xdata(static_cast<std::uint16_t>(0x0100 + i)) == (i ^ 0x5a) &&
                     device.peek_xdata(static_cast<std::uint16_t>(0x0300 + i)) ==
                         static_cast<std::uint8_t>(~i);
        }
        CHECK(stored);
        for (const std::uint16_t guard : {0x00ff, 0x0140, 0x02ff, 0x0340})
            CHECK(device.peek_xdata(guard) == 0xee);
        CHECK(device.read_idata(0x40) == 0xfa);
        CHECK(device.read_idata(0x41) == 0xee);
        CHECK(device.read_idata(0x42) == 0x40 && device.read_idata(0x43) == 0x01); // DPTR0
        CHECK(device.read_idata(0x44) == 0x05 && device.read_idata(0x45) == 0x03); // DPTR1
        CHECK(device.read_idata(0x46) == 0x00);
        CHECK(device.read_idata(0x47) == 0x01); // DPS bits 7:1 read 0
        CHECK(device.read_idata(0x48) == 0xfa);
        CHECK(device.read_idata(0x49) == 0x00);
        CHECK(device.read_idata(0x4a) == 0xee);
        const auto regs = device.registers();
        CHECK(regs.dpl == 0x40 && regs.dph == 0x01);
        CHECK(regs.dpl1 == 0x05 && regs.dph1 == 0x03);
        CHECK(regs.dps == 0x00 && device.read_sfr(DPS) == 0x00);
        CHECK(device.read_sfr(DPL1) == 0x05 && device.read_sfr(DPH1) == 0x03);
        CHECK(device.diagnostics().interrupts_taken == 0);
    }
}

// MOVC, JMP @A+DPTR and INC DPTR through DPTR1; direct SFR access to the
// inactive pointer; MOVC through DPTR1 into an FMAP bank; MOVX through DPTR1
// to an XREG.
Program dptr_code_program() {
    Program p;
    p.flash.at(0x0000).op({0x75, SP, 0x30,       // MOV SP,#0x30
                           0x90, 0x05, 0x00,     // MOV DPTR,#0x0500 (DPTR0: DPS = 0)
                           0x75, DPS, 0x01,      // MOV DPS,#1
                           0x90, 0x04, 0x00,     // MOV DPTR,#0x0400 (DPTR1)
                           0x74, 0x03,           // MOV A,#3
                           0x93,                 // MOVC A,@A+DPTR (DPTR1): 0xA3
                           0xf5, 0x40,           // MOV 0x40,A
                           0x75, DPS, 0x00,      // MOV DPS,#0
                           0x74, 0x03,           // MOV A,#3
                           0x93,                 // MOVC A,@A+DPTR (DPTR0): 0xB3
                           0xf5, 0x41,           // MOV 0x41,A
                           0x75, DPL1, 0xff,     // MOV DPL1,#0xFF (inactive pointer)
                           0x75, DPH1, 0x12,     // MOV DPH1,#0x12
                           0x75, DPS, 0x01,      // MOV DPS,#1
                           0xa3,                 // INC DPTR: 0x12FF -> 0x1300
                           0x85, DPL1, 0x42,     // MOV 0x42,DPL1
                           0x85, DPH1, 0x43,     // MOV 0x43,DPH1
                           0x75, DPL1, 0xff,     // MOV DPL1,#0xFF
                           0x75, DPH1, 0xff,     // MOV DPH1,#0xFF
                           0xa3,                 // INC DPTR: 0xFFFF -> 0x0000
                           0x85, DPL1, 0x44,     // MOV 0x44,DPL1
                           0x85, DPH1, 0x45,     // MOV 0x45,DPH1
                           0x85, DPL0, 0x46,     // MOV 0x46,DPL0 (untouched)
                           0x85, DPH0, 0x47,     // MOV 0x47,DPH0
                           0x90, 0x06, 0x00,     // MOV DPTR,#0x0600 (DPTR1)
                           0x74, 0x04,           // MOV A,#4
                           0x73});               // JMP @A+DPTR -> 0x0604
    p.flash.at(0x0400).op({0xa0, 0xa1, 0xa2, 0xa3, 0xa4});
    p.flash.at(0x0500).op({0xb0, 0xb1, 0xb2, 0xb3,
                           0x75, 0x48, 0xee,     // 0x0504 (DPTR0 + 4): MOV 0x48,#0xEE
                           0x80, 0xfe});
    p.flash.at(0x0600).op({0x02, 0x06, 0x10});   // 0x0600 (offset lost): LJMP 0x0610
    p.flash.at(0x0604).op({0x75, 0x48, 0xa4,     // MOV 0x48,#0xA4
                           0x02, 0x07, 0x00});   // LJMP 0x0700
    p.flash.at(0x0610).op({0x75, 0x48, 0xe1, 0x80, 0xfe});
    p.flash.at(0x0700).op({0x85, DPL0, 0x49,     // MOV 0x49,DPL0
                           0x85, DPH0, 0x4a,     // MOV 0x4A,DPH0
                           0x85, DPL1, 0x4b,     // MOV 0x4B,DPL1
                           0x85, DPH1, 0x4c,     // MOV 0x4C,DPH1
                           0xc0, DPL1,           // PUSH DPL1
                           0xc0, DPH1,           // PUSH DPH1
                           0x90, 0xab, 0xcd,     // MOV DPTR,#0xABCD (DPTR1)
                           0xd0, DPH1,           // POP DPH1
                           0xd0, DPL1,           // POP DPL1: DPTR1 = 0x0600
                           0xe4,                 // CLR A
                           0x93,                 // MOVC A,@A+DPTR: CODE[0x0600] = 0x02
                           0xf5, 0x4d,           // MOV 0x4D,A
                           0x75, FMAP, 0x02,     // MOV FMAP,#2
                           0x90, 0x80, 0x00,     // MOV DPTR,#0x8000 (DPTR1)
                           0xe4,                 // CLR A
                           0x93,                 // MOVC A,@A+DPTR: bank 2 byte 0x5B
                           0xf5, 0x4e,           // MOV 0x4E,A
                           0x90, 0x62, 0x4a,     // MOV DPTR,#0x624A (CHIPID)
                           0xe0,                 // MOVX A,@DPTR: 0xA5
                           0xf5, 0x4f,           // MOV 0x4F,A
                           0x85, SP, 0x50});     // MOV 0x50,SP
    p.end = p.flash.here();
    p.flash.op({0x80, 0xfe});
    p.flash.bank(2, 0x8000).op({0x5b});
    return p;
}

void test_dual_dptr_movc_jmp_inc_and_sfr_access() {
    const Program program = dptr_code_program();
    for (const bool trace : {false, true}) {
        Cc2530F256 device;
        load(device, program.flash);
        std::ostringstream sink;
        if (trace)
            device.set_trace(&sink);
        const Cc2530RunResult run = device.run(200);
        CHECK(run.error.empty());
        CHECK(run.pc == program.end && run.fmap == 2);
        const std::vector<std::uint8_t> expected{0xa3, 0xb3, 0x00, 0x13, 0x00, 0x00, 0x00, 0x05,
                                                 0xa4, 0x00, 0x05, 0x00, 0x06, 0x02, 0x5b, 0xa5,
                                                 0x30};
        std::vector<std::uint8_t> observed;
        for (unsigned address = 0x40; address <= 0x50; ++address)
            observed.push_back(device.read_idata(static_cast<std::uint8_t>(address)));
        CHECK(observed == expected);
        const auto regs = device.registers();
        CHECK(regs.dpl == 0x00 && regs.dph == 0x05);   // DPTR0 = 0x0500 since 0x0003
        CHECK(regs.dpl1 == 0x4a && regs.dph1 == 0x62); // DPTR1 = 0x624A
        CHECK(regs.dps == 0x01);
    }
}

void test_dual_dptr_reset_state() {
    Flash flash;
    flash.at(0x0000).op({0x90, 0x56, 0x78,       // MOV DPTR,#0x5678 (DPS = 0 at reset)
                         0x75, DPS, 0x01,        // MOV DPS,#1
                         0x90, 0x12, 0x34,       // MOV DPTR,#0x1234 (DPTR1)
                         0x80, 0xfe});           // SJMP $
    Cc2530F256 device;
    CHECK(device.read_sfr(DPS) == 0 && device.read_sfr(DPL1) == 0 &&
          device.read_sfr(DPH1) == 0);
    load(device, flash);
    CHECK(device.run(10).error.empty());
    auto regs = device.registers();
    CHECK(regs.dpl == 0x78 && regs.dph == 0x56 && regs.dpl1 == 0x34 && regs.dph1 == 0x12 &&
          regs.dps == 1);
    device.reset(); // SWRU191F 2.3.1: DPL0/DPH0/DPL1/DPH1/DPS reset to 0x00
    regs = device.registers();
    CHECK(regs.dpl == 0 && regs.dph == 0 && regs.dpl1 == 0 && regs.dph1 == 0 && regs.dps == 0);
    for (const std::uint8_t sfr : {DPL0, DPH0, DPL1, DPH1, DPS})
        CHECK(device.read_sfr(sfr) == 0);
    // After reset DPTR instructions use DPTR0 again.
    device.set_breakpoint(0x0003);
    const Cc2530RunResult run = device.run(10);
    CHECK(run.break_hit && run.pc == 0x0003);
    regs = device.registers();
    CHECK(regs.dpl == 0x78 && regs.dph == 0x56 && regs.dpl1 == 0 && regs.dph1 == 0);
    device.set_breakpoint(std::nullopt);
    CHECK(device.run(10).error.empty());
    device.power_on_reset();
    regs = device.registers();
    CHECK(regs.dpl == 0 && regs.dph == 0 && regs.dpl1 == 0 && regs.dph1 == 0 && regs.dps == 0);
}

// DPTR1 into the XBANK window (MOVX, flash bank MEMCTR.XBANK) and the XMAP
// window (MOVC, CODE 0x8000 = SRAM 0x0000).
void test_dual_dptr_xbank_and_xmap() {
    Flash flash;
    flash.at(0x0000).op({0x75, MEMCTR, 0x05,     // 0000 XBANK 5
                         0x75, DPS, 0x01,        // 0003
                         0x90, 0x80, 0x10,       // 0006 DPTR1 = 0x8010
                         0xe0,                   // 0009 MOVX A,@DPTR: flash bank 5 + 0x10
                         0xf5, 0x40,             // 000A
                         0x90, 0x00, 0x20,       // 000C DPTR1 = 0x0020 (SRAM)
                         0x74, 0xc3,             // 000F
                         0xf0,                   // 0011 MOVX @DPTR,A
                         0x75, MEMCTR, 0x08,     // 0012 XMAP, XBANK 0
                         0x90, 0x80, 0x20,       // 0015 DPTR1 = 0x8020
                         0xe4,                   // 0018 CLR A
                         0x93,                   // 0019 MOVC A,@A+DPTR: SRAM 0x0020
                         0xf5, 0x41,             // 001A
                         0x75, DPS, 0x00,        // 001C
                         0x85, DPL0, 0x42,       // 001F
                         0x85, DPH0, 0x43,       // 0022
                         0x80, 0xfe});           // 0025
    flash.at(5u * 0x8000u + 0x10).op({0x55});
    for (const bool trace : {false, true}) {
        Cc2530F256 device;
        load(device, flash);
        std::ostringstream sink;
        if (trace)
            device.set_trace(&sink);
        const Cc2530RunResult run = device.run(100);
        CHECK(run.error.empty() && run.pc == 0x0025);
        CHECK(device.read_idata(0x40) == 0x55);
        CHECK(device.read_idata(0x41) == 0xc3);
        CHECK(device.read_idata(0x42) == 0x00 && device.read_idata(0x43) == 0x00);
        CHECK(device.peek_xdata(0x0020) == 0xc3);
        const auto regs = device.registers();
        CHECK(regs.dpl1 == 0x20 && regs.dph1 == 0x80 && regs.dps == 0);
    }
}

// ---- EMU-04A golden vectors: interrupt priorities, nesting and RETI -------


// Prints `context` with a failed check inside a loop over configurations.
#define CHECK_IN(condition, context)                                           \
    do {                                                                       \
        if (!(condition)) {                                                    \
            std::cerr << __FILE__ << ':' << __LINE__ << " [" << (context)      \
                      << "]: check failed: " #condition << '\n';               \
            ++failures;                                                       \
        }                                                                      \
    } while (false)

std::string mode_name(bool trace) { return trace ? "trace" : "fast"; }

// One run(1) step: the PC after it and, where >= 0, SP, the in-progress
// level mask and the post-RETI inhibit.
struct Step {
    std::uint16_t pc;
    int sp = -1;
    int levels = -1;
    int inhibit = -1;
};

bool follow_steps(Cc2530F256 &device, const std::vector<Step> &steps,
                  const std::string &name, std::size_t first_step = 1) {
    bool ok = true;
    for (std::size_t i = 0; i < steps.size(); ++i) {
        const Cc2530RunResult run = device.run(1);
        const auto regs = device.registers();
        const Step &step = steps[i];
        const bool match = run.error.empty() && run.instructions == 1 && run.pc == step.pc &&
                           (step.sp < 0 || regs.sp == step.sp) &&
                           (step.levels < 0 || regs.interrupt_levels == step.levels) &&
                           (step.inhibit < 0 || regs.interrupt_inhibit == (step.inhibit != 0));
        if (!match) {
            std::cerr << name << " step " << first_step + i << ": expected pc=" << std::hex
                      << step.pc << " got pc=" << run.pc << " sp=" << unsigned(regs.sp)
                      << " levels=" << unsigned(regs.interrupt_levels)
                      << " inhibit=" << regs.interrupt_inhibit << std::dec
                      << " instructions=" << run.instructions << " error=" << run.error << '\n';
            ok = false;
        }
    }
    return ok;
}

// T1 (IPG1) at level `t1_level`, P0INT (IPG5) at level `p0_level`
// (SWRU191F Tables 2-6/2-7: level of group g = IP1.g:IP0.g). The T1 ISR
// raises P0IF in its body; P0INT may preempt only at a higher level (2.5.2).
Program nesting_pair_program(unsigned t1_level, unsigned p0_level) {
    const auto ip0 = static_cast<std::uint8_t>(((t1_level & 1u) << 1) | ((p0_level & 1u) << 5));
    const auto ip1 = static_cast<std::uint8_t>(((t1_level >> 1) << 1) | ((p0_level >> 1) << 5));
    Program p;
    p.flash.at(0x0000).op({0x02, 0x01, 0x00});   // LJMP 0x0100
    p.flash.at(0x004b).op({0x02, 0x02, 0x00});   // T1 vector: LJMP 0x0200
    p.flash.at(0x006b).op({0x02, 0x03, 0x00});   // P0INT vector: LJMP 0x0300
    p.flash.at(0x0100).op({0x75, SP, 0x30,       // 0100 MOV SP,#0x30
                           0x79, 0x40,           // 0103 MOV R1,#0x40
                           0x75, IP0, ip0,       // 0105 MOV IP0,#ip0
                           0x75, IP1, ip1,       // 0108 MOV IP1,#ip1
                           0x75, IEN1, 0x22,     // 010B MOV IEN1,#0x22 (P0IE | T1IE)
                           0xd2, 0xaf,           // 010E SETB EA
                           0x43, IRCON, 0x02,    // 0110 ORL IRCON,#0x02 (T1IF)
                           0x89, 0x70,           // 0113 MOV 0x70,R1
                           0x89, 0x71});         // 0115 MOV 0x71,R1
    p.end = p.flash.here();                      // 0117
    p.flash.op({0x80, 0xfe});
    p.flash.at(0x0200).op({0x77, 0x11,           // 0200 MOV @R1,#0x11
                           0x09,                 // 0202 INC R1
                           0x85, SP, 0x60,       // 0203 MOV 0x60,SP
                           0x43, IRCON, 0x20,    // 0206 ORL IRCON,#0x20 (P0IF)
                           0x77, 0x12,           // 0209 MOV @R1,#0x12
                           0x09,                 // 020B INC R1
                           0x32});               // 020C RETI
    p.flash.at(0x0300).op({0x77, 0xe0,           // 0300 MOV @R1,#0xE0
                           0x09,                 // 0302 INC R1
                           0x85, SP, 0x61,       // 0303 MOV 0x61,SP
                           0x53, IRCON, 0xdf,    // 0306 ANL IRCON,#0xDF (P0IF is SW-cleared)
                           0x77, 0xe1,           // 0309 MOV @R1,#0xE1
                           0x09,                 // 030B INC R1
                           0x32});               // 030C RETI
    return p;
}

void test_nesting_for_every_level_pair() {
    for (unsigned low = 0; low < 4; ++low) {
        for (unsigned high = 0; high < 4; ++high) {
            const Program program = nesting_pair_program(low, high);
            const bool nested = high > low;
            for (const bool trace : {false, true}) {
                const std::string context = "T1 level " + std::to_string(low) +
                                            ", P0INT level " + std::to_string(high) + ", " +
                                            mode_name(trace);
                Cc2530F256 device;
                load(device, program.flash);
                std::ostringstream sink;
                if (trace)
                    device.set_trace(&sink);
                const Cc2530RunResult run = device.run(200);
                CHECK_IN(run.error.empty() && run.pc == program.end, context);
                CHECK_IN(device.idle_forever(), context);
                const std::vector<std::uint8_t> log = irq_log(device);
                const std::vector<std::uint8_t> nested_log{0x11, 0xe0, 0xe1, 0x12};
                const std::vector<std::uint8_t> serial_log{0x11, 0x12, 0xe0, 0xe1};
                CHECK_IN(log == (nested ? nested_log : serial_log), context);
                CHECK_IN(device.read_idata(0x60) == 0x32, context);
                // Nested: P0INT entered on top of T1. Otherwise after T1's RETI
                // plus the one post-RETI instruction (MOV 0x70,R1).
                CHECK_IN(device.read_idata(0x61) == (nested ? 0x34 : 0x32), context);
                CHECK_IN(device.read_idata(0x70) == (nested ? 0x44 : 0x42), context);
                CHECK_IN(device.read_idata(0x71) == 0x44, context);
                if (nested) {
                    // Main return 0x0113 below T1's return 0x0209.
                    CHECK_IN(device.read_idata(0x31) == 0x13 && device.read_idata(0x32) == 0x01,
                             context);
                    CHECK_IN(device.read_idata(0x33) == 0x09 && device.read_idata(0x34) == 0x02,
                             context);
                } else {
                    CHECK_IN(device.read_idata(0x31) == 0x15 && device.read_idata(0x32) == 0x01,
                             context);
                }
                const auto regs = device.registers();
                CHECK_IN(regs.sp == 0x30 && regs.interrupt_levels == 0 &&
                             !regs.interrupt_inhibit && regs.posted_vector == 0,
                         context);
                CHECK_IN(device.diagnostics().interrupts_taken == 2, context);
                CHECK_IN((device.read_sfr(IRCON) & 0x22) == 0, context);
            }
        }
    }
}

const std::vector<Step> &nesting_prologue_steps() {
    static const std::vector<Step> steps{
        {0x0100, 0x07, 0, 0}, // LJMP
        {0x0103, 0x30},       // MOV SP
        {0x0105},             // MOV R1
        {0x0108},             // MOV IP0
        {0x010b},             // MOV IP1
        {0x010e},             // MOV IEN1
        {0x0110},             // SETB EA
        {0x0113, 0x30, 0, 0}, // ORL IRCON: T1IF set
        {0x004b, 0x32, 0x01, 0}, // boundary: T1 (level 0) entered
        {0x0200},             // LJMP
        {0x0202},             // MOV @R1,#0x11
        {0x0203},             // INC R1
        {0x0206},             // MOV 0x60,SP
        {0x0209},             // ORL IRCON,#0x20: P0IF set
    };
    return steps;
}

void test_golden_step_traces_nested_and_serial() {
    // Level 3 over level 0: P0INT preempts at the first boundary.
    const std::vector<Step> nested{
        {0x006b, 0x34, 0x09, 0}, // P0INT entered, return 0x0209 pushed
        {0x0300}, {0x0302}, {0x0303}, {0x0306}, {0x0309}, {0x030b}, {0x030c},
        {0x0209, 0x32, 0x01, 1}, // RETI: back in T1, level 3 released, inhibit armed
        {0x020b, -1, 0x01, 0},   // MOV @R1,#0x12 executed exactly once
        {0x020c},                // INC R1
        {0x0113, 0x30, 0x00, 1}, // RETI to main
        {0x0115, -1, 0x00, 0},   // MOV 0x70,R1
        {0x0117}, {0x0117}};
    // Level 0 over level 0: P0INT waits for T1's RETI and the instruction after it.
    const std::vector<Step> serial{
        {0x020b, -1, 0x01, 0},   // MOV @R1,#0x12: no preemption
        {0x020c},
        {0x0113, 0x30, 0x00, 1}, // RETI with P0INT pending: inhibit armed
        {0x0115, -1, 0x00, 0},   // the post-RETI instruction MOV 0x70,R1
        {0x006b, 0x32, 0x01, 0}, // then P0INT, return 0x0115 pushed
        {0x0300}, {0x0302}, {0x0303}, {0x0306}, {0x0309}, {0x030b}, {0x030c},
        {0x0115, 0x30, 0x00, 1},
        {0x0117, -1, 0x00, 0},   // MOV 0x71,R1
        {0x0117}};
    for (const bool trace : {false, true}) {
        for (const bool preempt : {true, false}) {
            const std::string name = std::string(preempt ? "nested" : "serial") + " " +
                                     mode_name(trace);
            Cc2530F256 device;
            load(device, nesting_pair_program(0, preempt ? 3 : 0).flash);
            std::ostringstream sink;
            if (trace)
                device.set_trace(&sink);
            CHECK_IN(follow_steps(device, nesting_prologue_steps(), name), name);
            const auto regs = device.registers();
            CHECK_IN(regs.posted_vector == 0x006b && regs.posted_source == 13 &&
                         regs.posted_level == (preempt ? 3 : 0),
                     name);
            CHECK_IN(follow_steps(device, preempt ? nested : serial, name, 15), name);
            CHECK_IN(device.idle_forever() && device.diagnostics().interrupts_taken == 2, name);
            if (trace) {
                const std::string text = sink.str();
                CHECK_IN(text.find("pc=0x0113 fmap=1 phys=0x00113  irq -> 0x004b T1 level=0") !=
                             std::string::npos,
                         name);
                CHECK_IN(text.find(preempt
                                       ? "pc=0x0209 fmap=1 phys=0x00209  irq -> 0x006b P0INT level=3"
                                       : "pc=0x0115 fmap=1 phys=0x00115  irq -> 0x006b P0INT level=0") !=
                             std::string::npos,
                         name);
                // The instruction after RETI is traced as executed, not as an entry.
                CHECK_IN(text.find(preempt ? "pc=0x0209 fmap=1 phys=0x00209  77 12"
                                           : "pc=0x0113 fmap=1 phys=0x00113  89 70") !=
                             std::string::npos,
                         name);
            }
        }
    }
}

// DMA (IPG0) level 0, T2 (IPG2) level 1, T3 (IPG3) level 2, P0INT (IPG5)
// level 3: IP0 = 0x24, IP1 = 0x28. With `chain` each ISR raises the next
// higher source; otherwise main raises all four at once.
Program four_level_program(bool chain) {
    const std::uint8_t main_flags = chain ? 0x01 : 0x2d;
    const std::uint8_t raise_t2 = chain ? 0x04 : 0x00;
    const std::uint8_t raise_t3 = chain ? 0x08 : 0x00;
    const std::uint8_t raise_p0 = chain ? 0x20 : 0x00;
    Program p;
    p.flash.at(0x0000).op({0x02, 0x01, 0x00});
    p.flash.at(0x0043).op({0x02, 0x02, 0x00});   // DMA: LJMP 0x0200
    p.flash.at(0x0053).op({0x02, 0x02, 0x10});   // T2: LJMP 0x0210
    p.flash.at(0x005b).op({0x02, 0x02, 0x20});   // T3: LJMP 0x0220
    p.flash.at(0x006b).op({0x02, 0x02, 0x30});   // P0INT: LJMP 0x0230
    p.flash.at(0x0100).op({0x75, SP, 0x30,       // 0100 MOV SP,#0x30
                           0x79, 0x40,           // 0103 MOV R1,#0x40
                           0x75, IP0, 0x24,      // 0105 IPG2, IPG5
                           0x75, IP1, 0x28,      // 0108 IPG3, IPG5
                           0x75, IEN1, 0x2d,     // 010B DMAIE T2IE T3IE P0IE
                           0xd2, 0xaf,           // 010E SETB EA
                           0x43, IRCON, main_flags, // 0110
                           0x89, 0x70,           // 0113 MOV 0x70,R1
                           0x89, 0x71,           // 0115
                           0x89, 0x72,           // 0117
                           0x89, 0x73});         // 0119
    p.end = p.flash.here();                      // 011B
    p.flash.op({0x80, 0xfe});
    p.flash.at(0x0200).op({0x77, 0xd0, 0x09,     // 0200 log D0
                           0x85, SP, 0x60,       // 0203
                           0x43, IRCON, raise_t2, // 0206
                           0x77, 0xd1, 0x09,     // 0209 log D1
                           0x53, IRCON, 0xfe,    // 020C clear DMAIF
                           0x32});               // 020F RETI
    p.flash.at(0x0210).op({0x77, 0x20, 0x09,     // 0210
                           0x85, SP, 0x61,       // 0213
                           0x43, IRCON, raise_t3, // 0216
                           0x77, 0x21, 0x09,     // 0219
                           0x32});               // 021C (T2IF: hardware clear)
    p.flash.at(0x0220).op({0x77, 0x30, 0x09,     // 0220
                           0x85, SP, 0x62,       // 0223
                           0x43, IRCON, raise_p0, // 0226
                           0x77, 0x31, 0x09,     // 0229
                           0x32});               // 022C
    p.flash.at(0x0230).op({0x77, 0xf0, 0x09,     // 0230
                           0x85, SP, 0x63,       // 0233
                           0x53, IRCON, 0xdf,    // 0236 clear P0IF
                           0x77, 0xf1, 0x09,     // 0239
                           0x32});               // 023C
    return p;
}

void test_four_level_nesting_and_simultaneous_levels() {
    for (const bool trace : {false, true}) {
        const std::string mode = mode_name(trace);
        {
            const Program program = four_level_program(true);
            Cc2530F256 device;
            load(device, program.flash);
            std::ostringstream sink;
            if (trace)
                device.set_trace(&sink);
            device.set_breakpoint(0x0230);
            Cc2530RunResult run = device.run(200);
            CHECK_IN(run.break_hit && run.pc == 0x0230, mode);
            auto regs = device.registers();
            CHECK_IN(regs.interrupt_levels == 0x0f && regs.sp == 0x38, mode);
            const std::vector<std::uint8_t> stack{0x13, 0x01, 0x09, 0x02,
                                                  0x19, 0x02, 0x29, 0x02};
            std::vector<std::uint8_t> observed;
            for (unsigned address = 0x31; address <= 0x38; ++address)
                observed.push_back(device.read_idata(static_cast<std::uint8_t>(address)));
            CHECK_IN(observed == stack, mode);
            device.set_breakpoint(std::nullopt);
            run = device.run(200);
            CHECK_IN(run.error.empty() && run.pc == program.end && device.idle_forever(), mode);
            const std::vector<std::uint8_t> expected{0xd0, 0x20, 0x30, 0xf0,
                                                     0xf1, 0x31, 0x21, 0xd1};
            CHECK_IN(irq_log(device) == expected, mode);
            CHECK_IN(device.read_idata(0x60) == 0x32 && device.read_idata(0x61) == 0x34 &&
                         device.read_idata(0x62) == 0x36 && device.read_idata(0x63) == 0x38,
                     mode);
            for (unsigned address = 0x70; address <= 0x73; ++address)
                CHECK_IN(device.read_idata(static_cast<std::uint8_t>(address)) == 0x48, mode);
            regs = device.registers();
            CHECK_IN(regs.sp == 0x30 && regs.interrupt_levels == 0, mode);
            CHECK_IN(device.diagnostics().interrupts_taken == 4, mode);
        }
        {
            // Four simultaneous requests at four levels: highest first, and
            // one main instruction after each RETI.
            const Program program = four_level_program(false);
            Cc2530F256 device;
            load(device, program.flash);
            std::ostringstream sink;
            if (trace)
                device.set_trace(&sink);
            const Cc2530RunResult run = device.run(200);
            CHECK_IN(run.error.empty() && run.pc == program.end && device.idle_forever(), mode);
            const std::vector<std::uint8_t> expected{0xf0, 0xf1, 0x30, 0x31,
                                                     0x20, 0x21, 0xd0, 0xd1};
            CHECK_IN(irq_log(device) == expected, mode);
            CHECK_IN(device.read_idata(0x70) == 0x42 && device.read_idata(0x71) == 0x44 &&
                         device.read_idata(0x72) == 0x46 && device.read_idata(0x73) == 0x48,
                     mode);
            for (unsigned address = 0x60; address <= 0x63; ++address)
                CHECK_IN(device.read_idata(static_cast<std::uint8_t>(address)) == 0x32, mode);
            CHECK_IN(device.diagnostics().interrupts_taken == 4, mode);
            CHECK_IN(device.read_sfr(IRCON) == 0, mode);
        }
    }
}

// SWRU191F Table 2-5, written out independently of the model's table:
// interrupt flag register and mask of source n (vector 0x03 + 8n).
struct FlagBit {
    std::uint8_t sfr;
    std::uint8_t mask;
};
const FlagBit SourceFlags[18] = {
    {TCON, 0x02},   {TCON, 0x20},   {TCON, 0x08},   {TCON, 0x80},   // RFERR ADC URX0 URX1
    {S0CON, 0x03},  {IRCON, 0x80},  {IRCON2, 0x01}, {IRCON2, 0x02}, // ENC ST P2INT UTX0
    {IRCON, 0x01},  {IRCON, 0x02},  {IRCON, 0x04},  {IRCON, 0x08},  // DMA T1 T2 T3
    {IRCON, 0x10},  {IRCON, 0x20},  {IRCON2, 0x04}, {IRCON2, 0x08}, // T4 P0INT UTX1 P1INT
    {S1CON, 0x03},  {IRCON2, 0x10}};                                // RF WDT

// All 18 sources enabled and pending before EA is set. Each ISR sits in its
// own 8-byte vector slot, logs its source number and clears its flag.
Program all_sources_program(std::uint8_t ip0, std::uint8_t ip1) {
    Program p;
    p.flash.at(0x0000).op({0x02, 0x01, 0x00});
    for (unsigned n = 0; n < 18; ++n) {
        const auto clear = static_cast<std::uint8_t>(~SourceFlags[n].mask);
        p.flash.at(0x03 + 8 * n)
            .op({0x77, static_cast<std::uint8_t>(n), // MOV @R1,#n
                 0x09,                               // INC R1
                 0x53, SourceFlags[n].sfr, clear,    // ANL flag,#~mask
                 0x32});                             // RETI
    }
    p.flash.at(0x0100).op({0x75, SP, 0x30,       // MOV SP,#0x30
                           0x79, 0x40,           // MOV R1,#0x40
                           0x75, IP0, ip0,
                           0x75, IP1, ip1,
                           0x75, IEN0, 0x3f,     // six IEN0 sources, EA = 0
                           0x75, IEN1, 0x3f,
                           0x75, IEN2, 0x3f,
                           0x43, TCON, 0xaa,     // RFERRIF ADCIF URX0IF URX1IF
                           0x43, S0CON, 0x03,    // ENCIF
                           0x43, IRCON, 0xbf,    // DMA T1 T2 T3 T4 P0 ST
                           0x43, IRCON2, 0x1f,   // P2 UTX0 UTX1 P1 WDT
                           0x43, S1CON, 0x03,    // RFIF
                           0xd2, 0xaf,           // SETB EA
                           0x00});               // NOP
    p.end = p.flash.here();
    p.flash.op({0x80, 0xfe});
    return p;
}

// Expected service order: by level (highest first), then Table 2-8.
std::vector<std::uint8_t> expected_order(std::uint8_t ip0, std::uint8_t ip1) {
    static const std::uint8_t polling[18] = {0, 16, 8, 1, 9, 2, 10, 3, 11,
                                             4, 12, 5, 13, 6, 7, 14, 15, 17};
    static const std::uint8_t group[18] = {0, 1, 2, 3, 4, 5, 1, 2, 0,
                                           1, 2, 3, 4, 5, 3, 4, 0, 5}; // Table 2-7
    std::vector<std::uint8_t> order;
    for (int level = 3; level >= 0; --level)
        for (const std::uint8_t source : polling) {
            const unsigned g = group[source];
            if (static_cast<int>((((ip1 >> g) & 1u) << 1) | ((ip0 >> g) & 1u)) == level)
                order.push_back(source);
        }
    return order;
}

void test_all_eighteen_sources_priority_and_polling() {
    // Hand-written expectations for the configurations (SWRU191F 2.5.3).
    const std::vector<std::uint8_t> table_2_8{0, 16, 8, 1, 9, 2, 10, 3, 11,
                                              4, 12, 5, 13, 6, 7, 14, 15, 17};
    const std::vector<std::uint8_t> mixed_a{2, 10, 7, 3, 11, 14, 0, 16, 8,
                                            4, 12, 15, 1, 9, 5, 13, 6, 17};
    const std::vector<std::uint8_t> mixed_b{5, 13, 17, 4, 12, 15, 1, 9, 3,
                                            11, 6, 14, 0, 16, 8, 2, 10, 7};
    CHECK(expected_order(0x00, 0x00) == table_2_8);
    CHECK(expected_order(0x3f, 0x3f) == table_2_8);
    CHECK(expected_order(0x15, 0x0c) == mixed_a); // IPG2 L3, IPG3 L2, IPG0/4 L1
    CHECK(expected_order(0x2a, 0x30) == mixed_b); // IPG5 L3, IPG4 L2, IPG1/3 L1
    struct Config {
        std::uint8_t ip0, ip1;
        const std::vector<std::uint8_t> *order;
    };
    for (const Config config : {Config{0x00, 0x00, &table_2_8}, Config{0x3f, 0x3f, &table_2_8},
                                Config{0x15, 0x0c, &mixed_a}, Config{0x2a, 0x30, &mixed_b},
                                Config{0x3f, 0x00, &table_2_8}, Config{0x00, 0x3f, &table_2_8}}) {
        const Program program = all_sources_program(config.ip0, config.ip1);
        for (const bool trace : {false, true}) {
            std::ostringstream context;
            context << "IP0=" << std::hex << unsigned(config.ip0) << " IP1=" << unsigned(config.ip1)
                    << ' ' << mode_name(trace);
            Cc2530F256 device;
            load(device, program.flash);
            std::ostringstream sink;
            if (trace)
                device.set_trace(&sink);
            const Cc2530RunResult run = device.run(400);
            // SWRU191F 11.2: STIE = 1 and the default sleep timer compare
            // (0xFFFFFF) is still ahead, so the loop is idle only without STIE.
            CHECK_IN(run.error.empty() && run.pc == program.end && !device.idle_forever(),
                     context.str());
            device.write_sfr(IEN0, static_cast<std::uint8_t>(device.read_sfr(IEN0) & ~0x20u));
            CHECK_IN(device.idle_forever(), context.str());
            CHECK_IN(irq_log(device) == *config.order, context.str());
            CHECK_IN(device.diagnostics().interrupts_taken == 18, context.str());
            CHECK_IN((device.read_sfr(TCON) & 0xaa) == 0 && (device.read_sfr(S0CON) & 3) == 0 &&
                         (device.read_sfr(IRCON) & 0xbf) == 0 &&
                         (device.read_sfr(IRCON2) & 0x1f) == 0 &&
                         (device.read_sfr(S1CON) & 3) == 0,
                     context.str());
            CHECK_IN(device.read_sfr(SP) == 0x30 && device.registers().interrupt_levels == 0,
                     context.str());
        }
    }
}

// MODEL-DEFINED (design 5.2 item 2): an enable or priority write takes
// effect at the next instruction boundary; no one-instruction block after
// IEN*/IP* writes. I10a (MODEL-DEFINED): the level of a service in progress
// is the level at entry.
void test_enable_and_priority_writes_model_defined() {
    for (const bool trace : {false, true}) {
        const std::string mode = mode_name(trace);
        {
            // (a) T1IF pending with T1IE = 0; enabling it is taken at once.
            Flash flash;
            flash.at(0x0000).op({0x02, 0x01, 0x00});
            flash.at(0x004b).op({0x85, 0x70, 0x71,   // MOV 0x71,0x70
                                 0x05, 0x72,         // INC 0x72
                                 0x32});             // RETI
            flash.at(0x0100).op({0x75, SP, 0x30,     // 0100
                                 0xd2, 0xaf,         // 0103 SETB EA
                                 0x43, IRCON, 0x02,  // 0105 T1IF, T1IE = 0
                                 0x75, 0x70, 0x01,   // 0108 MOV 0x70,#1
                                 0x75, IEN1, 0x02,   // 010B MOV IEN1,#0x02
                                 0x75, 0x70, 0x02,   // 010E MOV 0x70,#2
                                 0x80, 0xfe});       // 0111
            Cc2530F256 device;
            load(device, flash);
            std::ostringstream sink;
            if (trace)
                device.set_trace(&sink);
            CHECK_IN(follow_steps(device,
                                  {{0x0100}, {0x0103}, {0x0105}, {0x0108}, {0x010b},
                                   {0x010e}, {0x004b, 0x32, 0x01, 0}},
                                  "enable while pending " + mode),
                     mode);
            CHECK_IN(device.run(100).error.empty() && device.idle_forever(), mode);
            CHECK_IN(device.read_idata(0x71) == 1 && device.read_idata(0x72) == 1 &&
                         device.read_idata(0x70) == 2,
                     mode);
        }
        {
            // (b) P0INT pending at level 0 inside the level-0 T1 ISR; raising
            // IPG5 to level 2 lets it preempt at the next boundary.
            Flash flash;
            flash.at(0x0000).op({0x02, 0x01, 0x00});
            flash.at(0x004b).op({0x02, 0x02, 0x00});
            flash.at(0x006b).op({0x02, 0x03, 0x00});
            flash.at(0x0100).op({0x75, SP, 0x30, 0x79, 0x40,
                                 0x75, IEN1, 0x22,   // P0IE | T1IE, all level 0
                                 0xd2, 0xaf,
                                 0x43, IRCON, 0x02,  // T1IF
                                 0x89, 0x70,         // MOV 0x70,R1
                                 0x80, 0xfe});
            flash.at(0x0200).op({0x77, 0x11, 0x09,
                                 0x43, IRCON, 0x20,  // P0IF: same level, waits
                                 0x77, 0x12, 0x09,
                                 0x75, IP1, 0x20,    // IPG5 -> level 2
                                 0x77, 0x13, 0x09,
                                 0x32});
            flash.at(0x0300).op({0x77, 0xe0, 0x09, 0x53, IRCON, 0xdf, 0x77, 0xe1, 0x09, 0x32});
            Cc2530F256 device;
            load(device, flash);
            std::ostringstream sink;
            if (trace)
                device.set_trace(&sink);
            CHECK_IN(device.run(200).error.empty() && device.idle_forever(), mode);
            const std::vector<std::uint8_t> expected{0x11, 0x12, 0xe0, 0xe1, 0x13};
            CHECK_IN(irq_log(device) == expected, mode);
            CHECK_IN(device.read_idata(0x70) == 0x45 && device.registers().interrupt_levels == 0,
                     mode);
        }
        {
            // (c) T1 entered at level 2 lowers its own group to 0 and puts
            // IPG5 at level 1: the service stays a level-2 service, so P0INT
            // waits for its RETI and the post-RETI instruction.
            Flash flash;
            flash.at(0x0000).op({0x02, 0x01, 0x00});
            flash.at(0x004b).op({0x02, 0x02, 0x00});
            flash.at(0x006b).op({0x02, 0x03, 0x00});
            flash.at(0x0100).op({0x75, SP, 0x30, 0x79, 0x40,
                                 0x75, IP1, 0x02,    // IPG1 (T1) -> level 2
                                 0x75, IEN1, 0x22,
                                 0xd2, 0xaf,
                                 0x43, IRCON, 0x02,
                                 0x89, 0x70,
                                 0x89, 0x71,
                                 0x80, 0xfe});
            flash.at(0x0200).op({0x77, 0x11, 0x09,
                                 0x75, IP1, 0x00,    // IPG1 -> level 0
                                 0x75, IP0, 0x20,    // IPG5 -> level 1
                                 0x43, IRCON, 0x20,  // P0IF
                                 0x77, 0x12, 0x09,
                                 0x32});
            flash.at(0x0300).op({0x77, 0xe0, 0x09, 0x53, IRCON, 0xdf, 0x77, 0xe1, 0x09, 0x32});
            Cc2530F256 device;
            load(device, flash);
            std::ostringstream sink;
            if (trace)
                device.set_trace(&sink);
            device.set_breakpoint(0x020c); // after ORL IRCON,#0x20 in the T1 ISR
            Cc2530RunResult run = device.run(200);
            CHECK_IN(run.break_hit && run.pc == 0x020c, mode);
            const auto regs = device.registers();
            CHECK_IN(regs.interrupt_levels == 0x04 && regs.posted_vector == 0x006b &&
                         regs.posted_level == 1,
                     mode);
            device.set_breakpoint(std::nullopt);
            CHECK_IN(device.run(200).error.empty() && device.idle_forever(), mode);
            const std::vector<std::uint8_t> expected{0x11, 0x12, 0xe0, 0xe1};
            CHECK_IN(irq_log(device) == expected, mode);
            CHECK_IN(device.read_idata(0x70) == 0x42 && device.read_idata(0x71) == 0x44, mode);
        }
    }
}

// A software-cleared flag left set re-triggers after RETI, but only after
// exactly one instruction of the interrupted program (INTEL-ISA RETI rule).
void test_retrigger_after_reti_executes_one_instruction() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x006b).op({0x02, 0x03, 0x00});
    flash.at(0x0100).op({0x75, SP, 0x30,         // 0100
                         0x79, 0x40,             // 0103
                         0x75, IEN1, 0x20,       // 0105 P0IE
                         0xd2, 0xaf,             // 0108
                         0x43, IRCON, 0x20,      // 010A P0IF
                         0x05, 0x70,             // 010D INC 0x70
                         0x05, 0x70,             // 010F
                         0x05, 0x70,             // 0111
                         0x80, 0xfe});           // 0113
    flash.at(0x0300).op({0xa7, 0x70,             // 0300 MOV @R1,0x70
                         0x09,                   // 0302 INC R1
                         0xe9,                   // 0303 MOV A,R1
                         0xb4, 0x43, 0x03,       // 0304 CJNE A,#0x43,0x030A
                         0x53, IRCON, 0xdf,      // 0307 clear P0IF on the third entry
                         0x32});                 // 030A RETI
    for (const bool trace : {false, true}) {
        const std::string mode = mode_name(trace);
        Cc2530F256 device;
        load(device, flash);
        std::ostringstream sink;
        if (trace)
            device.set_trace(&sink);
        const Cc2530RunResult run = device.run(200);
        CHECK_IN(run.error.empty() && run.pc == 0x0113 && device.idle_forever(), mode);
        const std::vector<std::uint8_t> expected{0x00, 0x01, 0x02};
        CHECK_IN(irq_log(device) == expected, mode);
        CHECK_IN(device.read_idata(0x70) == 3, mode);
        CHECK_IN(device.diagnostics().interrupts_taken == 3, mode);
    }
}

// RETI -> RETI: the post-RETI instruction is itself a RETI, which arms the
// rule again, so one main instruction runs before the pending P2INT.
void test_reti_chain_rearms_the_post_reti_rule() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x0033).op({0x77, 0x66, 0x09,       // P2INT (IPG1, level 0)
                         0x53, IRCON2, 0xfe,
                         0x32});
    flash.at(0x004b).op({0x02, 0x02, 0x00});
    flash.at(0x006b).op({0x02, 0x03, 0x00});
    flash.at(0x0100).op({0x75, SP, 0x30,         // 0100
                         0x79, 0x40,             // 0103
                         0x75, IP0, 0x20,        // 0105 IPG5 -> level 3
                         0x75, IP1, 0x20,        // 0108
                         0x75, IEN1, 0x22,       // 010B
                         0x75, IEN2, 0x02,       // 010E P2IE
                         0xd2, 0xaf,             // 0111
                         0x43, IRCON, 0x02,      // 0113 T1IF
                         0x89, 0x70,             // 0116
                         0x89, 0x71,             // 0118
                         0x80, 0xfe});           // 011A
    flash.at(0x0200).op({0x77, 0x11, 0x09,       // 0200 T1 (level 0)
                         0x43, IRCON, 0x20,      // 0203 P0IF -> preempts
                         0x32});                 // 0206 RETI
    flash.at(0x0300).op({0x77, 0xe0, 0x09,       // 0300 P0INT (level 3)
                         0x53, IRCON, 0xdf,      // 0303
                         0x43, IRCON2, 0x01,     // 0306 P2IF: level 0, blocked
                         0x77, 0xe1, 0x09,       // 0309
                         0x32});                 // 030C RETI
    for (const bool trace : {false, true}) {
        const std::string mode = mode_name(trace);
        Cc2530F256 device;
        load(device, flash);
        std::ostringstream sink;
        if (trace)
            device.set_trace(&sink);
        device.set_breakpoint(0x030c);
        Cc2530RunResult run = device.run(200);
        CHECK_IN(run.break_hit && device.registers().interrupt_levels == 0x09, mode);
        device.set_breakpoint(std::nullopt);
        CHECK_IN(follow_steps(device,
                              {{0x0206, 0x32, 0x01, 1},  // RETI: into T1 at its RETI
                               {0x0116, 0x30, 0x00, 1},  // T1's RETI: armed again
                               {0x0118, 0x30, 0x00, 0},  // MOV 0x70,R1
                               {0x0033, 0x32, 0x01, 0}}, // P2INT
                              "RETI chain " + mode),
                 mode);
        run = device.run(100);
        CHECK_IN(run.error.empty() && device.idle_forever(), mode);
        const std::vector<std::uint8_t> expected{0x11, 0xe0, 0xe1, 0x66};
        CHECK_IN(irq_log(device) == expected, mode);
        CHECK_IN(device.read_idata(0x70) == 0x43 && device.read_idata(0x71) == 0x44, mode);
        CHECK_IN(device.diagnostics().interrupts_taken == 3, mode);
    }
}

// A level-3 request raised while the post-RETI inhibit is armed still waits
// for that one instruction ("at least one more instruction", INTEL-ISA).
void test_higher_level_request_during_post_reti_instruction() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x004b).op({0x77, 0x11, 0x09, 0x32}); // T1 (level 0): RETI at 0x004E
    flash.at(0x006b).op({0x77, 0xe0, 0x09, 0x53, IRCON, 0xdf, 0x32}); // P0INT (level 3)
    flash.at(0x0100).op({0x75, SP, 0x30,         // 0100
                         0x79, 0x40,             // 0103
                         0x75, IP0, 0x20,        // 0105
                         0x75, IP1, 0x20,        // 0108
                         0x75, IEN1, 0x22,       // 010B
                         0xd2, 0xaf,             // 010E
                         0x43, IRCON, 0x02,      // 0110
                         0x89, 0x70,             // 0113
                         0x89, 0x71,             // 0115
                         0x80, 0xfe});           // 0117
    for (const bool trace : {false, true}) {
        for (const bool stepped : {true, false}) {
            const std::string mode = mode_name(trace) + (stepped ? " stepped" : " breakpoint");
            Cc2530F256 device;
            load(device, flash);
            std::ostringstream sink;
            if (trace)
                device.set_trace(&sink);
            device.set_breakpoint(0x004e); // T1's RETI
            CHECK_IN(device.run(100).break_hit, mode);
            if (stepped) {
                device.set_breakpoint(std::nullopt);
                const Cc2530RunResult reti = device.run(1);
                CHECK_IN(reti.pc == 0x0113 && reti.instructions == 1, mode);
            } else {
                // Stop on the return address: the RETI has executed and the
                // inhibit is armed.
                device.set_breakpoint(0x0113);
                const Cc2530RunResult reti = device.run(100);
                CHECK_IN(reti.break_hit && reti.instructions == 1, mode);
                device.set_breakpoint(std::nullopt);
            }
            CHECK_IN(device.pc() == 0x0113 && device.registers().interrupt_inhibit, mode);
            device.raise_interrupt_flag_for_test(13); // P0IF; no peripheral involved
            auto regs = device.registers();
            CHECK_IN(regs.posted_vector == 0x006b && regs.posted_level == 3 &&
                         regs.interrupt_levels == 0 && regs.interrupt_inhibit,
                     mode);
            CHECK_IN(!device.idle_forever(), mode);
            if (stepped) {
                CHECK_IN(follow_steps(device, {{0x0115, 0x30, 0x00, 0}, {0x006b, 0x32, 0x08, 0}},
                                      "inhibit " + mode),
                         mode);
            }
            CHECK_IN(device.run(100).error.empty() && device.idle_forever(), mode);
            const std::vector<std::uint8_t> expected{0x11, 0xe0};
            CHECK_IN(irq_log(device) == expected, mode);
            CHECK_IN(device.read_idata(0x70) == 0x41 && device.read_idata(0x71) == 0x42, mode);
            CHECK_IN(device.diagnostics().test_injected_flags == 1, mode);
            CHECK_IN(device.diagnostics().interrupts_taken == 2, mode);
            if (trace)
                CHECK_IN(sink.str().find("pc=0x0113 fmap=1 phys=0x00113  89 70") !=
                                 std::string::npos &&
                             sink.str().find("pc=0x0115 fmap=1 phys=0x00115  irq -> 0x006b "
                                             "P0INT level=3") != std::string::npos,
                         mode);
        }
    }
}

// RET does not release the in-progress level ("the interrupt control system
// thinking an interrupt was still in progress", Intel); RETI without an ISR
// in progress returns like RET and leaves the levels clear.
void test_ret_versus_reti() {
    Flash flash;
    flash.at(0x0000).op({0x02, 0x01, 0x00});
    flash.at(0x004b).op({0x77, 0x11, 0x09, 0x22}); // T1 (level 0) ends with RET
    flash.at(0x006b).op({0x77, 0xe0, 0x09, 0x53, IRCON, 0xdf, 0x32}); // P0INT (level 3)
    flash.at(0x0100).op({0x75, SP, 0x30,         // 0100
                         0x79, 0x40,             // 0103
                         0x75, IP0, 0x20,        // 0105
                         0x75, IP1, 0x20,        // 0108
                         0x75, IEN1, 0x22,       // 010B
                         0xd2, 0xaf,             // 010E
                         0x43, IRCON, 0x02,      // 0110 T1IF
                         0x89, 0x70,             // 0113
                         0x43, IRCON, 0x02,      // 0115 T1IF again: level 0 still in progress
                         0x89, 0x71,             // 0118
                         0x43, IRCON, 0x20,      // 011A P0IF: level 3 preempts
                         0x89, 0x72,             // 011D
                         0x80, 0xfe});           // 011F
    for (const bool trace : {false, true}) {
        const std::string mode = mode_name(trace);
        Cc2530F256 device;
        load(device, flash);
        std::ostringstream sink;
        if (trace)
            device.set_trace(&sink);
        const Cc2530RunResult run = device.run(200);
        CHECK_IN(run.error.empty() && run.pc == 0x011f, mode);
        const std::vector<std::uint8_t> expected{0x11, 0xe0};
        CHECK_IN(irq_log(device) == expected, mode);
        CHECK_IN(device.read_idata(0x70) == 0x41 && device.read_idata(0x71) == 0x41 &&
                     device.read_idata(0x72) == 0x42,
                 mode);
        const auto regs = device.registers();
        CHECK_IN(regs.interrupt_levels == 0x01 && regs.sp == 0x30, mode);
        CHECK_IN(regs.posted_vector == 0x004b && regs.posted_level == 0, mode);
        CHECK_IN(device.in_interrupt() && device.idle_forever(), mode); // T1 never deliverable
        CHECK_IN(device.diagnostics().interrupts_taken == 2, mode);
    }
    // RETI reached through LCALL with no interrupt in progress.
    Flash call;
    call.at(0x0000).op({0x75, SP, 0x30,          // 0000
                        0x12, 0x02, 0x00,        // 0003 LCALL 0x0200
                        0x75, 0x40, 0x01,        // 0006
                        0x80, 0xfe});            // 0009
    call.at(0x0200).op({0x32});                  // RETI
    for (const bool trace : {false, true}) {
        const std::string mode = mode_name(trace);
        Cc2530F256 device;
        load(device, call);
        std::ostringstream sink;
        if (trace)
            device.set_trace(&sink);
        CHECK_IN(follow_steps(device,
                              {{0x0003, 0x30, 0, 0}, {0x0200, 0x32, 0, 0},
                               {0x0006, 0x30, 0, 1}, {0x0009, 0x30, 0, 0}},
                              "RETI without ISR " + mode),
                 mode);
        CHECK_IN(device.read_idata(0x40) == 1 && !device.in_interrupt(), mode);
    }
}

// T1 interrupts bank-3 code; its ISR calls into bank 5, where P0INT (level
// 3) preempts it; the P0INT ISR calls into bank 6. Every return restores
// the bank the interrupted code was running in.
Program banked_nested_program() {
    Program p;
    p.flash.at(0x0000).op({0x02, 0x01, 0x00});
    p.flash.at(0x004b).op({0x02, 0x02, 0x00});
    p.flash.at(0x006b).op({0x02, 0x03, 0x00});
    p.flash.at(0x0100).op({0x75, SP, 0x30,       // 0100
                           0x79, 0x40,           // 0103
                           0x75, IP0, 0x20,      // 0105 IPG5 -> level 3
                           0x75, IP1, 0x20,      // 0108
                           0x75, FMAP, 0x03,     // 010B
                           0x02, 0x80, 0x00});   // 010E LJMP 0x8000 (bank 3)
    p.flash.bank(3, 0x8000).op({0x75, IEN1, 0x22, // 8000
                                0xd2, 0xaf,       // 8003
                                0x43, IRCON, 0x02, // 8005 T1IF
                                0x77, 0x33,       // 8008
                                0x09,             // 800A
                                0x80, 0xfe});     // 800B
    p.flash.at(0x0200).op({0x77, 0x11, 0x09,     // 0200
                           0x85, FMAP, 0x60,     // 0203 MOV 0x60,FMAP
                           0x75, FMAP, 0x05,     // 0206
                           0x12, 0x80, 0x00,     // 0209 LCALL 0x8000 (bank 5)
                           0x85, 0x60, FMAP,     // 020C MOV FMAP,0x60
                           0x77, 0x12, 0x09,     // 020F
                           0x32});               // 0212
    p.flash.bank(5, 0x8000).op({0x77, 0x55, 0x09, // 8000
                                0x43, IRCON, 0x20, // 8003 P0IF
                                0x77, 0x56, 0x09, // 8006
                                0x22});           // 8009 RET
    p.flash.at(0x0300).op({0x77, 0xe0, 0x09,     // 0300
                           0x85, FMAP, 0x61,     // 0303
                           0x75, FMAP, 0x06,     // 0306
                           0x12, 0x80, 0x00,     // 0309 LCALL 0x8000 (bank 6)
                           0x85, 0x61, FMAP,     // 030C
                           0x53, IRCON, 0xdf,    // 030F
                           0x77, 0xe1, 0x09,     // 0312
                           0x32});               // 0315
    p.flash.bank(6, 0x8000).op({0x77, 0x66, 0x09, 0x22});
    p.end = 0x800b;
    return p;
}

void test_nested_interrupts_in_banked_code() {
    const Program program = banked_nested_program();
    for (const bool trace : {false, true}) {
        const std::string mode = mode_name(trace);
        Cc2530F256 device;
        load(device, program.flash);
        std::ostringstream sink;
        if (trace)
            device.set_trace(&sink);
        device.set_breakpoint(0x0300);
        Cc2530RunResult run = device.run(200);
        CHECK_IN(run.break_hit && run.fmap == 5, mode);
        auto regs = device.registers();
        CHECK_IN(regs.sp == 0x36 && regs.interrupt_levels == 0x09, mode);
        CHECK_IN(device.read_idata(0x35) == 0x06 && device.read_idata(0x36) == 0x80, mode);
        device.set_breakpoint(std::nullopt);
        run = device.run(200);
        CHECK_IN(run.error.empty() && run.pc == program.end && run.fmap == 3, mode);
        CHECK_IN(device.idle_forever(), mode);
        const std::vector<std::uint8_t> expected{0x11, 0x55, 0xe0, 0x66,
                                                 0xe1, 0x56, 0x12, 0x33};
        CHECK_IN(irq_log(device) == expected, mode);
        CHECK_IN(device.read_idata(0x60) == 3 && device.read_idata(0x61) == 5, mode);
        regs = device.registers();
        CHECK_IN(regs.sp == 0x30 && regs.interrupt_levels == 0, mode);
        CHECK_IN(device.diagnostics().interrupts_taken == 2, mode);
    }
}

void test_interrupt_controller_reset_and_reserved_bits() {
    // Reset in the middle of four nested ISRs, then again with the inhibit armed.
    const Program program = four_level_program(true);
    for (const bool trace : {false, true}) {
        const std::string mode = mode_name(trace);
        Cc2530F256 device;
        load(device, program.flash);
        std::ostringstream sink;
        if (trace)
            device.set_trace(&sink);
        device.set_breakpoint(0x0230);
        CHECK_IN(device.run(200).break_hit && device.registers().interrupt_levels == 0x0f, mode);
        device.reset();
        auto regs = device.registers();
        CHECK_IN(regs.interrupt_levels == 0 && !regs.interrupt_inhibit && regs.posted_vector == 0 &&
                     regs.posted_source == -1 && regs.sp == 0x07,
                 mode);
        for (const std::uint8_t sfr : {IEN0, IEN1, IEN2, IP0, IP1, IRCON, IRCON2})
            CHECK_IN(device.read_sfr(sfr) == 0, mode);
        // TCON.IT1 and TCON.IT0 reset to 1 (SWRU191F TCON); all flags clear.
        CHECK_IN(device.read_sfr(TCON) == 0x05, mode);
        // Same breakpoint, same nesting again.
        CHECK_IN(device.run(200).break_hit && device.registers().interrupt_levels == 0x0f, mode);
        device.set_breakpoint(0x0229); // T3 resumes after P0INT's RETI: inhibit armed
        CHECK_IN(device.run(200).break_hit, mode);
        regs = device.registers();
        CHECK_IN(regs.interrupt_inhibit && regs.interrupt_levels == 0x07, mode);
        device.reset();
        regs = device.registers();
        CHECK_IN(!regs.interrupt_inhibit && regs.interrupt_levels == 0, mode);
        device.set_breakpoint(std::nullopt);
        CHECK_IN(device.run(400).error.empty() && device.idle_forever(), mode);
        const std::vector<std::uint8_t> expected{0xd0, 0x20, 0x30, 0xf0,
                                                 0xf1, 0x31, 0x21, 0xd1};
        CHECK_IN(irq_log(device) == expected, mode);
    }
    // SWRU191F: IEN0 bit 6 and IEN1/IEN2 bits 7:6 read 0; IP0/IP1 bits 7:6 are R/W.
    Flash flash;
    flash.at(0x0000).op({0x75, IEN0, 0xff, 0x85, IEN0, 0x40,
                         0x75, IEN1, 0xff, 0x85, IEN1, 0x41,
                         0x75, IEN2, 0xff, 0x85, IEN2, 0x42,
                         0x75, IP0, 0xff, 0x85, IP0, 0x43,
                         0x75, IP1, 0xff, 0x85, IP1, 0x44,
                         0x80, 0xfe});
    Cc2530F256 device;
    load(device, flash);
    // SWRU191F 11.2: EA = STIE = 1 with the default sleep timer compare ahead.
    CHECK(device.run(100).error.empty() && !device.idle_forever());
    CHECK(device.read_idata(0x40) == 0xbf);
    CHECK(device.read_idata(0x41) == 0x3f);
    CHECK(device.read_idata(0x42) == 0x3f);
    CHECK(device.read_idata(0x43) == 0xff);
    CHECK(device.read_idata(0x44) == 0xff);
    CHECK(device.diagnostics().interrupts_taken == 0);
}

void test_interrupt_injection_is_bounded() {
    Flash flash;
    flash.at(0x0000).op({0x80, 0xfe});
    Cc2530F256 device;
    load(device, flash);
    CHECK(device.run(10).error.empty());
    const std::vector<std::uint8_t> before = snapshot(device);
    bool rejected = false;
    try {
        device.raise_interrupt_flag_for_test(18);
    } catch (const std::out_of_range &) {
        rejected = true;
    }
    CHECK(rejected);
    CHECK(snapshot(device) == before);
    CHECK(device.diagnostics().test_injected_flags == 0);
    device.raise_interrupt_flag_for_test(9); // T1: IRCON bit 1, nothing else
    std::vector<std::uint8_t> after = snapshot(device);
    std::size_t differing = 0;
    for (std::size_t i = 0; i < before.size(); ++i)
        differing += before[i] != after[i];
    CHECK(differing == 1);
    CHECK(device.read_sfr(IRCON) == 0x02);
    CHECK(device.diagnostics().test_injected_flags == 1);
    // EA = 0: nothing is posted or taken.
    CHECK(device.registers().posted_vector == 0);
    CHECK(device.run(10).error.empty() && device.diagnostics().interrupts_taken == 0);
}

// Fast interpretation, per-instruction tracing and single stepping agree on
// every EMU-04A golden program, including the nested-priority state.
void test_emu04a_trace_and_fast_equivalence() {
    check_equivalence(dptr_alternation_program().flash, 1500);
    check_equivalence(dptr_code_program().flash, 200);
    for (const auto &[low, high] : {std::pair{0u, 3u}, std::pair{0u, 0u},
                                    std::pair{1u, 2u}, std::pair{3u, 3u},
                                    std::pair{2u, 1u}})
        check_equivalence(nesting_pair_program(low, high).flash, 200);
    check_equivalence(four_level_program(true).flash, 300);
    check_equivalence(four_level_program(false).flash, 300);
    check_equivalence(all_sources_program(0x00, 0x00).flash, 400);
    check_equivalence(all_sources_program(0x15, 0x0c).flash, 400);
    check_equivalence(all_sources_program(0x3f, 0x00).flash, 400);
    check_equivalence(banked_nested_program().flash, 300);
}

void test_intel_hex_with_extended_addresses() {
    const auto path = temp_file("banked.hex");
    {
        std::ofstream out(path);
        out << hex_record(0x00, 0x0000,
                          {0x75, FMAP, 0x02, 0x12, 0x80, 0x00, 0xf5, 0x40, 0x80, 0xfe})
            << '\n'
            << hex_record(0x04, 0x0000, {0x00, 0x01}) << '\n' // 0x10000: bank 2
            << hex_record(0x00, 0x0000, {0x74, 0x42, 0x22}) << '\n'
            << hex_record(0x01, 0x0000, {}) << '\n';
    }
    Cc2530F256 device;
    device.load_firmware(path.string());
    CHECK(device.read_flash(0x10000) == 0x74);
    CHECK(device.diagnostics().load_warnings.empty());
    const Cc2530RunResult run = device.run(100);
    CHECK(run.error.empty());
    CHECK(device.read_idata(0x40) == 0x42);

    // llvm-objcopy emits an extended *segment* address record (type 02) below
    // 1 MiB: :020000021000EC selects base 0x1000 << 4 = 0x10000 (bank 2).
    {
        std::ofstream out(path);
        out << hex_record(0x00, 0x0000,
                          {0x75, FMAP, 0x02, 0x12, 0x80, 0x00, 0xf5, 0x40, 0x80, 0xfe})
            << '\n'
            << hex_record(0x02, 0x0000, {0x10, 0x00}) << '\n'
            << hex_record(0x00, 0x0000, {0x74, 0x43, 0x22}) << '\n'
            << hex_record(0x01, 0x0000, {}) << '\n';
    }
    device.load_firmware(path.string());
    CHECK(device.read_flash(0x10000) == 0x74 && device.read_flash(0x0000) == 0x75);
    CHECK(device.diagnostics().load_warnings.empty());
    CHECK(device.run(100).error.empty());
    CHECK(device.read_idata(0x40) == 0x43);
    {
        std::ofstream out(path);
        out << hex_record(0x00, 0x0000, {0x80, 0xfe}) << '\n'
            << hex_record(0x04, 0x0000, {0x00, 0x03}) << '\n'
            << hex_record(0x00, 0xe800, {0x00}) << '\n' // NV page 125
            << hex_record(0x00, 0xfff0, {0x00}) << '\n' // lock bits
            << hex_record(0x01, 0x0000, {}) << '\n';
    }
    device.load_firmware(path.string());
    CHECK(device.diagnostics().load_warnings.size() == 2);

    {
        std::ofstream out(path);
        out << hex_record(0x04, 0x0000, {0x00, 0x04}) << '\n' // 0x40000
            << hex_record(0x00, 0x0000, {0x00}) << '\n'
            << hex_record(0x01, 0x0000, {}) << '\n';
    }
    bool rejected = false;
    try {
        device.load_firmware(path.string());
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    CHECK(rejected);

    {
        std::ofstream out(path, std::ios::binary);
        out << "\x7f" "ELF" << std::string(60, '\0');
    }
    std::string message;
    try {
        device.load_firmware(path.string());
    } catch (const std::runtime_error &error) {
        message = error.what();
    }
    CHECK(message.find("llvm-objcopy -O ihex") != std::string::npos);
    std::filesystem::remove(path);
}

} // namespace

int main() {
    test_reset_state();
    test_all_fmap_banks_and_common_area();
    test_cross_bank_thunk_and_switch_while_banked();
    test_xbank_is_independent_of_fmap();
    test_sram_aliases_mpage_and_sfr_alias();
    test_xmap_executes_from_sram_and_restores_flash();
    test_interrupt_in_banked_code_and_reti();
    test_interrupt_priority_groups_and_polling_order();
    test_global_enable_gates_and_retracts_requests();
    test_reset_during_isr_returns_to_power_on_interrupt_state();
    test_pcon_idle_sleeps_until_an_interrupt();
    test_pm1_wakes_only_on_port_or_sleep_timer();
    test_clock_and_status_registers();
    test_xreg_chip_id_info_page_and_unmapped_xdata();
    test_unsupported_behavior_stops_with_an_error();
    test_trace_and_fast_execution_are_equivalent();
    test_breakpoint_semantics_match_in_both_modes();
    test_dual_dptr_alternation_movx_and_dps();
    test_dual_dptr_movc_jmp_inc_and_sfr_access();
    test_dual_dptr_reset_state();
    test_dual_dptr_xbank_and_xmap();
    test_nesting_for_every_level_pair();
    test_golden_step_traces_nested_and_serial();
    test_four_level_nesting_and_simultaneous_levels();
    test_all_eighteen_sources_priority_and_polling();
    test_enable_and_priority_writes_model_defined();
    test_retrigger_after_reti_executes_one_instruction();
    test_reti_chain_rearms_the_post_reti_rule();
    test_higher_level_request_during_post_reti_instruction();
    test_ret_versus_reti();
    test_nested_interrupts_in_banked_code();
    test_interrupt_controller_reset_and_reserved_bits();
    test_interrupt_injection_is_bounded();
    test_emu04a_trace_and_fast_equivalence();
    test_intel_hex_with_extended_addresses();
    if (failures != 0) {
        std::cerr << failures << " check(s) failed\n";
        return 1;
    }
    std::cout << "cc2530emu tests passed\n";
    return 0;
}
