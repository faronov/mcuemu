#include "mcs51emu/cc2530f256.hpp"

#include "cc2530_aes.hpp"
#include "cc2530_analog.hpp"
#include "cc2530_debug.hpp"
#include "cc2530_dma.hpp"
#include "cc2530_flash.hpp"
#include "cc2530_io.hpp"
#include "cc2530_mactimer.hpp"
#include "cc2530_radio.hpp"
#include "cc2530_soc.hpp"
#include "cc2530_timers.hpp"
#include "cc2530_usart.hpp"

#include "unicorn/mcs51.h"
#include "unicorn/unicorn.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace mcs51emu {
namespace {

constexpr std::uint64_t XdataBase = UC_MCS51_XDATA_BASE;
constexpr std::uint64_t IdataBase = UC_MCS51_IDATA_BASE;
constexpr std::uint64_t SfrBase = UC_MCS51_SFR_BASE;
constexpr std::size_t PageSize = 0x1000; // Unicorn MCS-51 TARGET_PAGE_BITS 12
constexpr std::size_t CodeSpace = 0x10000;
constexpr std::uint16_t XdataMmioStart = 0x2000; // first byte above the SRAM
constexpr std::uint32_t NoBreak = 0x10000;

// SFR addresses (SWRU191F Table 2-1).
constexpr std::uint8_t SfrP0 = 0x80;
constexpr std::uint8_t SfrSp = 0x81;
constexpr std::uint8_t SfrPcon = 0x87;
constexpr std::uint8_t SfrTcon = 0x88;
constexpr std::uint8_t SfrP1 = 0x90;
constexpr std::uint8_t SfrDpl1 = 0x84;
constexpr std::uint8_t SfrDph1 = 0x85;
constexpr std::uint8_t SfrDps = 0x92;
constexpr std::uint8_t SfrMpage = 0x93;
constexpr std::uint8_t SfrS0con = 0x98;
constexpr std::uint8_t SfrIen2 = 0x9a;
constexpr std::uint8_t SfrS1con = 0x9b;
constexpr std::uint8_t SfrSleepsta = 0x9d;
constexpr std::uint8_t SfrClkconsta = 0x9e;
constexpr std::uint8_t SfrFmap = 0x9f;
constexpr std::uint8_t SfrP2 = 0xa0;
constexpr std::uint8_t SfrIen0 = 0xa8;
constexpr std::uint8_t SfrIp0 = 0xa9;
constexpr std::uint8_t SfrIen1 = 0xb8;
constexpr std::uint8_t SfrIp1 = 0xb9;
constexpr std::uint8_t SfrSleepcmd = 0xbe;
constexpr std::uint8_t SfrIrcon = 0xc0;
constexpr std::uint8_t SfrClkconcmd = 0xc6;
constexpr std::uint8_t SfrMemctr = 0xc7;
constexpr std::uint8_t SfrIrcon2 = 0xe8;

constexpr std::uint8_t SfrDpl0 = 0x82;
constexpr std::uint8_t SfrDph0 = 0x83;
constexpr std::uint8_t SfrPsw = 0xd0;
constexpr std::uint8_t SfrAcc = 0xe0;
constexpr std::uint8_t SfrB = 0xf0;

constexpr std::uint8_t MemctrXmap = 0x08;
constexpr std::uint8_t MemctrXbank = 0x07;
constexpr std::uint8_t Ien0Ea = 0x80;
constexpr std::uint8_t ClkconOsc = 0x40;
constexpr std::uint8_t ClkconOsc32k = 0x80;
constexpr std::uint8_t CldEn = 0x01;
// SWRU191F 5.2: maximum clock-loss reset time-outs (0.25 ms and 0.5 ms) in
// 32 MHz cycles.
constexpr std::uint64_t CldXoscTimeout = 8000;
constexpr std::uint64_t Cld32kTimeout = 16000;

// XREGs with modeled content (SWRU191F 12.x chip identification).
constexpr std::uint16_t XregStart = 0x6000;
constexpr std::uint16_t XregEnd = 0x6400;
constexpr std::uint16_t XregChver = 0x6249;
constexpr std::uint16_t XregChipid = 0x624a;
constexpr std::uint16_t XregChipinfo0 = 0x6276;
constexpr std::uint16_t XregChipinfo1 = 0x6277;
constexpr std::uint16_t XregCld = 0x6290; // clock-loss detector enable (SWRU191F 4.4)
constexpr std::uint8_t ChipidCc2530 = 0xa5;
constexpr std::uint8_t Chipinfo0Flash256k = 0x44; // FLASHSIZE=100, bit 2 reads 1
constexpr std::uint8_t Chipinfo1Sram8k = 0x07;    // SRAMSIZE = 8 KiB - 1

constexpr std::uint16_t SfrAliasStart = 0x7080;
constexpr std::uint16_t SfrAliasEnd = 0x7100;

// The 8051-internal SFRs (gray in SWRU191F Table 2-1). They are not mapped
// into the XDATA SFR alias, except that P0-P2 are readable there.
constexpr bool cpu_internal_sfr(std::uint8_t address) {
    switch (address) {
    case 0x80: case 0x81: case 0x82: case 0x83: case 0x84: case 0x85:
    case 0x87: case 0x88: case 0x90: case 0x92: case 0x98: case 0x9a:
    case 0x9b: case 0xa0: case 0xa8: case 0xa9: case 0xb8: case 0xb9:
    case 0xc0: case 0xd0: case 0xe0: case 0xe8: case 0xf0:
        return true;
    default:
        return false;
    }
}

// SFRs the core itself implements (CPU, memory, power, clock and interrupt
// controller); peripherals cannot claim them.
constexpr bool core_sfr(std::uint8_t address) {
    if (address == SfrP0 || address == SfrP1 || address == SfrP2)
        return false; // CPU-internal, but the I/O controller owns the ports
    if (cpu_internal_sfr(address))
        return true;
    switch (address) {
    case SfrMpage: case SfrSleepsta: case SfrClkconsta: case SfrFmap:
    case SfrSleepcmd: case SfrClkconcmd: case SfrMemctr:
        return true;
    default:
        return false;
    }
}

// The 18 interrupt sources of SWRU191F Table 2-5; vector = 0x03 + 8 * n.
struct IrqSource {
    const char *name;
    std::uint8_t flag_sfr;
    std::uint8_t flag_mask;
    std::uint8_t enable_sfr;
    std::uint8_t enable_mask;
    std::uint8_t group; // priority group IPG0..IPG5 (Table 2-7)
    bool hardware_clear; // flag cleared when the CPU vectors to the ISR
};

constexpr std::array<IrqSource, 18> IrqSources{{
    {"RFERR", SfrTcon, 0x02, SfrIen0, 0x01, 0, true},
    {"ADC", SfrTcon, 0x20, SfrIen0, 0x02, 1, true},
    {"URX0", SfrTcon, 0x08, SfrIen0, 0x04, 2, true},
    {"URX1", SfrTcon, 0x80, SfrIen0, 0x08, 3, true},
    {"ENC", SfrS0con, 0x03, SfrIen0, 0x10, 4, false},
    {"ST", SfrIrcon, 0x80, SfrIen0, 0x20, 5, false},
    {"P2INT", SfrIrcon2, 0x01, SfrIen2, 0x02, 1, false},
    {"UTX0", SfrIrcon2, 0x02, SfrIen2, 0x04, 2, false},
    {"DMA", SfrIrcon, 0x01, SfrIen1, 0x01, 0, false},
    {"T1", SfrIrcon, 0x02, SfrIen1, 0x02, 1, true},
    {"T2", SfrIrcon, 0x04, SfrIen1, 0x04, 2, true},
    {"T3", SfrIrcon, 0x08, SfrIen1, 0x08, 3, true},
    {"T4", SfrIrcon, 0x10, SfrIen1, 0x10, 4, true},
    {"P0INT", SfrIrcon, 0x20, SfrIen1, 0x20, 5, false},
    {"UTX1", SfrIrcon2, 0x04, SfrIen2, 0x08, 3, false},
    {"P1INT", SfrIrcon2, 0x08, SfrIen2, 0x10, 4, false},
    {"RF", SfrS1con, 0x03, SfrIen2, 0x01, 0, false},
    {"WDT", SfrIrcon2, 0x10, SfrIen2, 0x20, 5, false},
}};

// SWRU191F 2.5.1, IRCON bit 6: "Must be written 0. Writing a 1 always
// enables the interrupt source." Figure 2-4 ORs ircon.6 with STIF into the
// sleep timer (ST, interrupt 5) request ahead of STIE/EA, and ircon.6 is not
// one of the shaded flags the hardware clears on ISR entry, so a 1 requests
// the ST interrupt until software writes it 0.
constexpr unsigned SourceSt = 5;
constexpr std::uint8_t IrconStRequest = 0x40;

// The source's request input: its flag bits, plus ircon.6 for ST.
bool flag_requested(const std::array<std::uint8_t, 256> &sfr, unsigned source) {
    const IrqSource &irq = IrqSources[source];
    const std::uint8_t mask =
        source == SourceSt ? static_cast<std::uint8_t>(irq.flag_mask | IrconStRequest)
                           : irq.flag_mask;
    return (sfr[irq.flag_sfr] & mask) != 0;
}

// Interrupt polling sequence within one priority level (Table 2-8).
constexpr std::array<std::uint8_t, 18> PollingOrder{
    0, 16, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 7, 14, 15, 17};

constexpr std::uint16_t irq_vector(int source) {
    return static_cast<std::uint16_t>(0x03 + 8 * source);
}

std::uint8_t hex_byte(std::string_view text, std::size_t offset,
                      const std::string &path) {
    if (offset + 2 > text.size())
        throw std::runtime_error(path + ": truncated Intel HEX record");
    unsigned value = 0;
    for (std::size_t i = offset; i < offset + 2; ++i) {
        const int ch = std::toupper(static_cast<unsigned char>(text[i]));
        const int digit = std::isdigit(ch) ? ch - '0'
                          : ch >= 'A' && ch <= 'F' ? ch - 'A' + 10
                                                   : -1;
        if (digit < 0)
            throw std::runtime_error(path + ": invalid Intel HEX digit");
        value = (value << 4) | static_cast<unsigned>(digit);
    }
    return static_cast<std::uint8_t>(value);
}

using FlashImage = Cc2530FlashImage;

FlashImage load_intel_hex(const std::string &path) {
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("cannot open firmware image: " + path);
    FlashImage image{std::vector<std::uint8_t>(Cc2530F256::FlashSize, 0xff),
                     std::vector<bool>(Cc2530F256::FlashSize, false)};
    std::string line;
    std::uint32_t upper_address = 0;
    bool eof_seen = false;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        while (!line.empty() && (line.back() == '\r' || line.back() == ' '))
            line.pop_back();
        if (line.empty())
            continue;
        const std::string where = " at line " + std::to_string(line_number);
        if (eof_seen)
            throw std::runtime_error(path + ": data follows Intel HEX EOF record");
        if (line[0] != ':' || line.size() < 11 || (line.size() & 1u) == 0)
            throw std::runtime_error(path + ": malformed Intel HEX record" + where);
        const std::string_view record(line.data() + 1, line.size() - 1);
        const std::uint8_t count = hex_byte(record, 0, path);
        const std::uint8_t address_hi = hex_byte(record, 2, path);
        const std::uint8_t address_lo = hex_byte(record, 4, path);
        const std::uint8_t type = hex_byte(record, 6, path);
        if (record.size() != 10 + static_cast<std::size_t>(count) * 2)
            throw std::runtime_error(path + ": Intel HEX byte count mismatch" + where);
        unsigned sum = count + address_hi + address_lo + type;
        std::vector<std::uint8_t> data(count);
        for (std::size_t i = 0; i < count; ++i) {
            data[i] = hex_byte(record, 8 + i * 2, path);
            sum += data[i];
        }
        sum += hex_byte(record, 8 + count * 2u, path);
        if ((sum & 0xffu) != 0)
            throw std::runtime_error(path + ": Intel HEX checksum mismatch" + where);
        const std::uint32_t offset = (static_cast<std::uint32_t>(address_hi) << 8) |
                                     address_lo;
        switch (type) {
        case 0x00: {
            const std::uint64_t absolute = static_cast<std::uint64_t>(upper_address) + offset;
            if (absolute + count > Cc2530F256::FlashSize)
                throw std::runtime_error(
                    path + ": data beyond the CC2530F256 256 KiB flash" + where +
                    " (CODE images must use physical flash addresses)");
            for (std::size_t i = 0; i < count; ++i) {
                image.bytes[absolute + i] = data[i];
                image.programmed[absolute + i] = true;
            }
            break;
        }
        case 0x01:
            if (count != 0 || offset != 0)
                throw std::runtime_error(path + ": malformed Intel HEX EOF record");
            eof_seen = true;
            break;
        case 0x02:
            if (count != 2 || offset != 0)
                throw std::runtime_error(path + ": malformed segment address record");
            upper_address = ((static_cast<std::uint32_t>(data[0]) << 8) | data[1]) << 4;
            break;
        case 0x04:
            if (count != 2 || offset != 0)
                throw std::runtime_error(path + ": malformed linear address record");
            upper_address = ((static_cast<std::uint32_t>(data[0]) << 8) | data[1]) << 16;
            break;
        case 0x03: case 0x05:
            break; // start address: the CC2530 always resets to CODE 0x0000
        default:
            throw std::runtime_error(path + ": unsupported Intel HEX record type " +
                                     std::to_string(type));
        }
    }
    if (!eof_seen)
        throw std::runtime_error(path + ": missing Intel HEX EOF record");
    return image;
}

FlashImage load_firmware_file(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw std::runtime_error("cannot open firmware image: " + path);
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)),
                                    std::istreambuf_iterator<char>());
    if (bytes.empty())
        throw std::runtime_error("firmware image is empty: " + path);
    if (bytes.size() >= 4 && bytes[0] == 0x7f && bytes[1] == 'E' && bytes[2] == 'L' &&
        bytes[3] == 'F')
        throw std::runtime_error(
            path + ": ELF images are not loaded directly; convert with "
                   "'llvm-objcopy -O ihex' so banked sections land at their "
                   "physical load addresses");
    const auto first = std::find_if_not(bytes.begin(), bytes.end(), [](std::uint8_t ch) {
        return std::isspace(static_cast<unsigned char>(ch));
    });
    if (first != bytes.end() && *first == ':')
        return load_intel_hex(path);
    if (bytes.size() > Cc2530F256::FlashSize)
        throw std::runtime_error(path + ": raw image exceeds the 256 KiB flash");
    FlashImage image{bytes, std::vector<bool>(Cc2530F256::FlashSize, false)};
    for (std::size_t i = 0; i < bytes.size(); ++i)
        image.programmed[i] = bytes[i] != 0xff;
    image.bytes.resize(Cc2530F256::FlashSize, 0xff);
    return image;
}

std::string hex(unsigned value, int width) {
    std::ostringstream text;
    text << "0x" << std::hex << std::setw(width) << std::setfill('0') << value;
    return text.str();
}

} // namespace

class Cc2530F256::BusImpl final : public cc2530::Bus {
public:
    explicit BusImpl(Cc2530F256 &chip) : c_(chip) {}

    void claim_sfr(std::uint8_t sfr, cc2530::Peripheral *owner, bool computed_read) override {
        if (sfr < 0x80 || core_sfr(sfr) || c_.sfr_owner_[sfr])
            throw std::logic_error("SFR " + hex(sfr, 2) + " cannot be claimed by " +
                                   owner->name());
        c_.sfr_owner_[sfr] = owner;
        c_.sfr_special_write_[sfr] = 1;
        if (computed_read)
            c_.sfr_special_read_[sfr] = 1;
    }
    void claim_xreg(std::uint16_t address, cc2530::Peripheral *owner) override {
        if (address < XregStart || address >= XregEnd || address == XregChver ||
            address == XregChipid || address == XregChipinfo0 || address == XregChipinfo1 ||
            address == XregCld || c_.xreg_owner_[address - XregStart])
            throw std::logic_error("XREG " + hex(address, 4) + " cannot be claimed by " +
                                   owner->name());
        c_.xreg_owner_[address - XregStart] = owner;
    }

    std::uint64_t now() const override { return c_.engine_cycles(); }
    std::uint64_t hf_now() const override { return c_.hf_now(); }
    std::uint64_t hf_deadline(std::uint64_t hf) const override {
        if (hf == cc2530::Never || hf_stopped())
            return cc2530::Never;
        const std::uint64_t current = c_.hf_now();
        const std::uint64_t at = c_.engine_cycles();
        return hf <= current ? at : at + (hf - current);
    }
    std::uint64_t lf_now() const override { return c_.lf_now(); }
    std::uint64_t lf_deadline(std::uint64_t lf) const override {
        if (lf == cc2530::Never || c_.lf_frozen_since_) // PM3 or a stopped 32 kHz clock
            return cc2530::Never;
        const std::uint64_t current = c_.lf_now();
        const std::uint64_t at = c_.engine_cycles();
        return lf <= current ? at : at + (lf - current);
    }
    cc2530::PowerMode power_mode() const override { return c_.power_mode_; }
    std::uint8_t clkconsta() const override { return c_.sfr_[SfrClkconsta]; }

    std::uint8_t &storage(cc2530::Reg reg) override { return slot(reg); }
    std::uint8_t storage(cc2530::Reg reg) const override {
        return const_cast<BusImpl *>(this)->slot(reg);
    }
    bool rmw_access() const override {
        std::uint8_t value = 0;
        uc_reg_read(reinterpret_cast<uc_engine *>(c_.uc_), UC_MCS51_REG_RMW_ACCESS, &value);
        return value != 0;
    }

    void raise_irq(cc2530::Irq source, std::uint8_t mask) override {
        const IrqSource &irq = IrqSources[static_cast<unsigned>(source)];
        const std::uint8_t bits = mask == 0 ? irq.flag_mask : mask;
        if ((bits & ~irq.flag_mask) != 0)
            throw std::logic_error(std::string("invalid flag mask for ") + irq.name);
        c_.set_interrupt_flags(static_cast<unsigned>(source), bits);
    }
    bool irq_flag(cc2530::Irq source) const override {
        const IrqSource &irq = IrqSources[static_cast<unsigned>(source)];
        return (c_.sfr_[irq.flag_sfr] & irq.flag_mask) != 0;
    }
    void clear_irq(cc2530::Irq source, std::uint8_t mask) override {
        const IrqSource &irq = IrqSources[static_cast<unsigned>(source)];
        const std::uint8_t bits = mask == 0 ? irq.flag_mask : mask;
        if ((bits & ~irq.flag_mask) != 0)
            throw std::logic_error(std::string("invalid flag mask for ") + irq.name);
        c_.sfr_[irq.flag_sfr] = static_cast<std::uint8_t>(c_.sfr_[irq.flag_sfr] & ~bits);
        c_.update_interrupts();
    }
    void dma_trigger(unsigned trigger) override {
        if (c_.soc_->dma)
            c_.soc_->dma->trigger(trigger);
    }
    void reschedule() override { c_.peripheral_accessed(); }

    std::uint8_t xdata_read(std::uint16_t address) override {
        if (address < SramSize)
            return c_.sram_[address];
        return c_.xdata_read(address);
    }
    void xdata_write(std::uint16_t address, std::uint8_t value) override {
        c_.xdata_write(address, value);
    }
    std::uint8_t xdata_peek(std::uint16_t address) const override {
        return c_.xdata_value(address);
    }
    std::vector<std::uint8_t> &flash() override { return c_.flash_; }
    void flash_changed(std::uint32_t begin, std::uint32_t end) override {
        c_.refresh_flash(begin, end);
    }
    bool cpu_executing_from_flash() const override {
        // A CPU halted by the debug interface (or in reset) fetches nothing;
        // a RESUME/STEP_INSTR re-checks (FlashController::cpu_resumed).
        return !c_.cpu_held() && c_.code_physical(c_.pc()).has_value();
    }

    void stall_cpu_until(std::uint64_t until) override {
        if (until <= c_.stall_until_)
            return;
        c_.stall_until_ = until;
        ++c_.diagnostics_.cpu_stalls;
        uc_emu_stop(reinterpret_cast<uc_engine *>(c_.uc_));
    }
    void request_reset(cc2530::ResetCause cause) override {
        if (!c_.pending_reset_)
            c_.pending_reset_ = cause;
        uc_emu_stop(reinterpret_cast<uc_engine *>(c_.uc_));
    }
    void fault(std::string message) override { c_.request_stop(std::move(message)); }
    void note_unmodeled(cc2530::Reg reg, bool write) override {
        ++(write ? c_.diagnostics_.unmodeled_feature_writes
                 : c_.diagnostics_.unmodeled_feature_reads)[reg];
    }
    bool reset_held() const override { return c_.reset_held_; }
    bool sleep_requested() const override { return c_.sleep_requested_; }
    std::uint16_t cpu_pc() const override { return c_.pc(); }
    bool engine_running() const override { return c_.engine_running_; }
    std::uint8_t debug_execute(const std::uint8_t *code, unsigned length) override {
        return c_.debug_execute(code, length);
    }
    void wake_for_debug_halt() override {
        if (c_.sleep_requested_)
            c_.wake_from_power_mode();
    }
    void set_timers_suspended(bool suspended) override {
        if (suspended == c_.timers_suspended_)
            return;
        // Settle Timer 1/3/4 on the old time base first.
        const std::uint64_t now = c_.engine_cycles();
        static_cast<cc2530::Peripheral *>(c_.soc_->timer1)->service(now);
        static_cast<cc2530::Peripheral *>(c_.soc_->timer34)->service(now);
        const std::uint64_t hf = c_.hf_now();
        if (suspended)
            c_.timers_suspended_since_ = hf;
        else
            c_.timers_suspended_total_ += hf - c_.timers_suspended_since_;
        c_.timers_suspended_ = suspended;
        reschedule();
    }
    bool timers_suspended() const override { return c_.timers_suspended_; }
    std::uint64_t timer_hf_now() const override {
        return (c_.timers_suspended_ ? c_.timers_suspended_since_ : c_.hf_now()) -
               c_.timers_suspended_total_;
    }
    std::uint64_t timer_hf_deadline(std::uint64_t hf) const override {
        if (hf == cc2530::Never || c_.timers_suspended_)
            return cc2530::Never;
        return hf_deadline(hf + c_.timers_suspended_total_);
    }
    cc2530::Soc &soc() override { return *c_.soc_; }

private:
    bool hf_stopped() const {
        return c_.power_mode_ == Cc2530PowerMode::Pm1 || c_.power_mode_ == Cc2530PowerMode::Pm2 ||
               c_.power_mode_ == Cc2530PowerMode::Pm3;
    }
    std::uint8_t &slot(cc2530::Reg reg) {
        if (cc2530::is_sfr(reg))
            return c_.sfr_[cc2530::sfr_of(reg)];
        if (reg >= XregStart && reg < XregEnd)
            return c_.xreg_[reg - XregStart];
        throw std::logic_error("storage(): " + hex(reg, 4) + " is not a register");
    }

    Cc2530F256 &c_;
};

unsigned Cc2530F256::instruction_size(std::uint8_t opcode) noexcept {
    // Intel MCS-51 instruction lengths. (The Unicorn translator's own table
    // only sizes translation blocks and differs for C6/C7, E8-EF, F8-FF and
    // 92; execution decodes the bytes itself.)
    if ((opcode & 0x1fu) == 0x01u || (opcode & 0x1fu) == 0x11u)
        return 2; // AJMP, ACALL
    if ((opcode & 0xf8u) == 0x78u || (opcode & 0xf8u) == 0x88u ||
        (opcode & 0xf8u) == 0xa8u || (opcode & 0xf8u) == 0xd8u)
        return 2; // MOV Rn,#d; MOV dir,Rn; MOV Rn,dir; DJNZ Rn,rel
    if ((opcode & 0xf0u) == 0xb0u && opcode >= 0xb4u)
        return 3; // CJNE
    switch (opcode) {
    case 0x02: case 0x10: case 0x12: case 0x20: case 0x30: case 0x43:
    case 0x53: case 0x63: case 0x75: case 0x85: case 0x90: case 0xd5:
        return 3;
    case 0x05: case 0x15: case 0x24: case 0x25: case 0x34: case 0x35:
    case 0x40: case 0x42: case 0x44: case 0x45: case 0x50: case 0x52:
    case 0x54: case 0x55: case 0x60: case 0x62: case 0x64: case 0x65:
    case 0x70: case 0x72: case 0x74: case 0x76: case 0x77: case 0x80:
    case 0x82: case 0x86: case 0x87: case 0x92: case 0x94: case 0x95:
    case 0xa0: case 0xa2: case 0xa6: case 0xa7: case 0xb0: case 0xb2:
    case 0xc0: case 0xc2: case 0xc5: case 0xd0: case 0xd2: case 0xe5:
    case 0xf5:
        return 2;
    default:
        return 1;
    }
}

Cc2530F256::Cc2530F256()
    : flash_(FlashSize, 0xff), views_(BankCount * CodeSpace, 0xff),
      xmap_view_(CodeSpace, 0xff) {
    sram_ = xmap_view_.data() + 0x8000;
    idata_ = sram_ + IdataXdataAlias;
    uc_engine *engine = nullptr;
    const uc_err open_error = uc_open(UC_ARCH_MCS51, UC_MODE_LITTLE_ENDIAN, &engine);
    if (open_error != UC_ERR_OK)
        throw std::runtime_error(std::string("uc_open(MCS-51): ") +
                                 uc_strerror(open_error));
    uc_ = reinterpret_cast<uc_struct *>(engine);
    try {
        // The interpreter fetches through uc_mcs51_fast_memory.code, which
        // install_code_view() repoints at the view selected by FMAP/XMAP. The
        // soft-MMU CODE mapping only serves the translator's per-instruction
        // block sizing, which does not affect execution.
        check_error(uc_mem_map_ptr(engine, 0, CodeSpace, UC_PROT_READ | UC_PROT_EXEC,
                                   views_.data() + CodeSpace),
                    "map CODE");
        check_error(uc_mem_map_ptr(engine, XdataBase, SramSize, UC_PROT_ALL, sram_),
                    "map SRAM into XDATA");
        check_error(uc_mmio_map(engine, XdataBase + XdataMmioStart,
                                CodeSpace - XdataMmioStart, xdata_mmio_read, this,
                                xdata_mmio_write, this),
                    "map XDATA registers, info page and XBANK");
        // DATA/IDATA is the top 256 bytes of the SRAM. Unicorn maps whole
        // pages, the CPU only addresses the first 256 bytes of this one.
        check_error(uc_mem_map_ptr(engine, IdataBase, PageSize, UC_PROT_ALL, idata_),
                    "map IDATA onto SRAM");
        check_error(uc_mmio_map(engine, SfrBase, PageSize, sfr_mmio_read, this,
                                sfr_mmio_write, this),
                    "map SFR");
        for (const std::uint8_t address :
             {SfrFmap, SfrMemctr, SfrDps, SfrPcon, SfrClkconcmd, SfrClkconsta,
              SfrSleepsta, SfrSleepcmd, SfrTcon, SfrS0con, SfrS1con, SfrIrcon,
              SfrIrcon2, SfrIen0, SfrIen1, SfrIen2, SfrIp0, SfrIp1})
            sfr_special_write_[address] = 1;
        const std::uint8_t page_sfr = SfrMpage;
        check_error(uc_reg_write(engine, UC_MCS51_REG_XDATA_PAGE_SFR, &page_sfr),
                    "select MPAGE for MOVX @Ri");
        // IEN0.1/TCON.5 are the CC2530 ADC interrupt, which has a priority
        // group and polling position; the model posts it like every source.
        const std::uint8_t core_timer0 = 0;
        check_error(uc_reg_write(engine, UC_MCS51_REG_TIMER0_INTERRUPT, &core_timer0),
                    "disable the core's classic Timer 0 interrupt");
        // SWRU191F 2.3.1: DPS bit 0 selects DPTR1 (DPH1:DPL1) for every
        // instruction that uses the data pointer.
        for (const auto &[reg, sfr] : {std::pair{UC_MCS51_REG_DPS_SFR, SfrDps},
                                       std::pair{UC_MCS51_REG_DPL1_SFR, SfrDpl1},
                                       std::pair{UC_MCS51_REG_DPH1_SFR, SfrDph1}})
            check_error(uc_reg_write(engine, reg, &sfr), "configure DPTR1");
        // 8051 RETI: one more instruction executes before any interrupt.
        const std::uint8_t post_reti_rule = 1;
        check_error(uc_reg_write(engine, UC_MCS51_REG_POST_RETI_RULE, &post_reti_rule),
                    "enable the post-RETI instruction rule");
        // SWRU191F Table 2-3 per-opcode clock counts and the 2.5.2 interrupt
        // entry (1 detect + 6 LCALL cycles); see "Timing contract" above.
        const std::uint8_t cycle_model = UC_MCS51_CYCLE_MODEL_CC253X;
        check_error(uc_reg_write(engine, UC_MCS51_REG_CYCLE_MODEL, &cycle_model),
                    "select the CC253x cycle model");
        // SLEEPSTA.CLK32K follows the 32 kHz clock.
        sfr_special_read_[SfrSleepsta] = 1;
        create_peripherals();
        info_page_.fill(0xff);
        rebuild_views();
        power_on_reset();
    } catch (...) {
        uc_close(engine);
        uc_ = nullptr;
        throw;
    }
}

void Cc2530F256::create_peripherals() {
    bus_ = std::make_unique<BusImpl>(*this);
    soc_ = std::make_unique<cc2530::Soc>();
    cc2530::Bus &bus = *bus_;
    const auto add = [this]<class T>(std::unique_ptr<T> peripheral) {
        T *raw = peripheral.get();
        peripherals_.push_back(std::move(peripheral));
        return raw;
    };
    soc_->io = add(std::make_unique<cc2530::Io>(bus));
    soc_->sleep_timer = add(std::make_unique<cc2530::SleepTimer>(bus));
    soc_->watchdog = add(std::make_unique<cc2530::Watchdog>(bus));
    soc_->timer1 = add(std::make_unique<cc2530::Timer1>(bus));
    soc_->timer34 = add(std::make_unique<cc2530::Timer34>(bus));
    soc_->mac_timer = add(std::make_unique<cc2530::MacTimer>(bus));
    soc_->adc = add(std::make_unique<cc2530::Adc>(bus));
    soc_->comparator = add(std::make_unique<cc2530::AnalogComparator>(bus));
    soc_->opamp = add(std::make_unique<cc2530::OpAmp>(bus));
    soc_->usart[0] = add(std::make_unique<cc2530::Usart>(bus, 0));
    soc_->usart[1] = add(std::make_unique<cc2530::Usart>(bus, 1));
    soc_->dma = add(std::make_unique<cc2530::Dma>(bus));
    soc_->flash = add(std::make_unique<cc2530::FlashController>(bus));
    soc_->flash->bind(diagnostics_,
                      [this](Cc2530FaultAction action) { request_power_fault(action); });
    soc_->aes = add(std::make_unique<cc2530::Aes>(bus));
    soc_->radio = add(std::make_unique<cc2530::Radio>(bus));
    // Last: its reset reads the pads and DBGLOCK after the others reset.
    soc_->debug = add(std::make_unique<cc2530::Debug>(bus));
}

Cc2530F256::~Cc2530F256() {
    if (uc_)
        uc_close(reinterpret_cast<uc_engine *>(uc_));
}

void Cc2530F256::check_error(int error, const char *operation) const {
    const auto result = static_cast<uc_err>(error);
    if (result != UC_ERR_OK)
        throw std::runtime_error(std::string("Unicorn ") + operation + ": " +
                                 uc_strerror(result));
}

void Cc2530F256::load_firmware(const std::string &path) {
    const FlashImage image = load_firmware_file(path);
    diagnostics_ = {};
    soc_->flash->flash_replaced();
    flash_ = image.bytes;
    bool reserved = false;
    bool lock = false;
    for (std::uint32_t address = ReservedFlashStart; address < FlashSize; ++address) {
        if (!image.programmed[address])
            continue;
        (address >= LockBitsStart ? lock : reserved) = true;
    }
    if (reserved)
        diagnostics_.load_warnings.push_back(
            "image programs flash at or above " + hex(ReservedFlashStart, 5) +
            " (cc2530-zigbee NV pages 125-126 / lock page 127)");
    if (lock)
        diagnostics_.load_warnings.push_back(
            "image programs the lock bits at " + hex(LockBitsStart, 5) +
            "; PAGELOCK bits are enforced by the flash controller, the debug-lock bit (DBGLOCK) "
            "by the debug interface from the next reset");
    rebuild_views();
    power_on_reset();
}

Cc2530FlashImage Cc2530F256::read_firmware_image(const std::string &path) {
    return load_firmware_file(path);
}

void Cc2530F256::load_flash(std::uint32_t address, const std::vector<std::uint8_t> &image) {
    if (address > FlashSize || image.size() > FlashSize - address)
        throw std::out_of_range("image exceeds the 256 KiB flash");
    diagnostics_ = {};
    soc_->flash->flash_replaced();
    std::fill(flash_.begin(), flash_.end(), 0xff);
    std::copy(image.begin(), image.end(), flash_.begin() + address);
    rebuild_views();
    power_on_reset();
}

void Cc2530F256::rebuild_views() {
    for (unsigned bank = 0; bank < BankCount; ++bank) {
        std::uint8_t *view = views_.data() + bank * CodeSpace;
        std::copy_n(flash_.begin(), BankSize, view);
        std::copy_n(flash_.begin() + bank * BankSize, BankSize, view + BankSize);
    }
    std::copy_n(flash_.begin(), BankSize, xmap_view_.begin());
}

void Cc2530F256::power_on_reset() {
    // The supply went away: a flash operation in flight gets its interrupted
    // outcome while virtual time still shows the cut.
    soc_->flash->interrupt_operation(Cc2530FlashCut::PowerCut, Cc2530ResetCause::PowerOn);
    // SRAM content is undefined at power-on; the model fills it with the
    // configured byte (0 by default) or seeded bytes so runs are
    // deterministic. Virtual time restarts first so peripherals reset at
    // cycle 0.
    power_on_sram_fill();
    power_mode_ = requested_mode_ = Cc2530PowerMode::Active; // time restarts below
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    const std::uint64_t zero = 0;
    check_error(uc_reg_write(engine, UC_MCS51_REG_CYCLES, &zero), "reset cycles");
    check_error(uc_reg_write(engine, UC_MCS51_REG_INSTRUCTIONS, &zero),
                "reset instruction counter");
    hf_stopped_ = 0;
    lf_stopped_ = 0;
    soc_->flash->time_restarted();
    lf_frozen_since_.reset();
    // Model-defined: a power-on reset (a new board, as far as the model
    // knows) brings back every oscillator stop_oscillator() stopped.
    osc_stop_at_.fill(UINT64_MAX);
    osc_stops_applied_ = 0;
    cld_since_ = 0;
    timers_suspended_ = false;
    timers_suspended_since_ = 0;
    timers_suspended_total_ = 0;
    reset_core(Cc2530ResetCause::PowerOn);
}

void Cc2530F256::reset() {
    reset_core(Cc2530ResetCause::External);
}

bool Cc2530F256::apply_pending_reset() {
    if (!pending_reset_)
        return false;
    if (pending_power_cut_) {
        power_cut();
        return true;
    }
    const Cc2530ResetCause cause = *pending_reset_;
    reset_core(cause);
    return true;
}

void Cc2530F256::reset_core(Cc2530ResetCause cause, bool count) {
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    // A flash operation in flight ends with the configured interrupted
    // outcome (no-op after power_cut(), which ended it already).
    soc_->flash->interrupt_operation(Cc2530FlashCut::Reset, cause);
    if (soc_->debug)
        soc_->debug->before_reset(cause);
    const std::uint64_t now = engine_cycles();
    // A reset in PM1-PM3 ends the power mode: close the clock-domain books.
    if (power_mode_ != Cc2530PowerMode::Active) {
        const std::uint64_t elapsed = now - power_mode_since_;
        diagnostics_.power_mode_cycles_by_mode[static_cast<unsigned>(power_mode_)] += elapsed;
        if (power_mode_ != Cc2530PowerMode::Idle)
            hf_stopped_ += elapsed;
    }
    power_mode_ = requested_mode_ = Cc2530PowerMode::Active;
    pending_reset_.reset();
    pending_power_cut_ = false;
    stall_until_ = 0;
    if (count)
        ++diagnostics_.resets_by_cause[static_cast<unsigned>(cause)];
    const auto reset_cause = static_cast<std::uint8_t>(cause);
    // Register reset values (SWRU191F): storage is zeroed, the core sets its
    // own registers, then every peripheral writes its reset values.
    sfr_.fill(0);
    xreg_.fill(0);
    sfr_[SfrP0] = sfr_[SfrP1] = sfr_[SfrP2] = 0xff;
    sfr_[SfrSp] = 0x07;
    sfr_[SfrTcon] = 0x05; // IT0 = IT1 = 1
    sfr_[SfrFmap] = 0x01;
    sfr_[SfrClkconcmd] = sfr_[SfrClkconsta] = 0xc9;
    // SWRU191F 5.2: "After a reset, the internal RC oscillators are used";
    // the 32 kHz domain runs again unless the 32 kHz RCOSC is stopped.
    refresh_lf_domain(now);
    sfr_[SfrSleepcmd] = 0x04;
    sfr_[SfrSleepsta] = static_cast<std::uint8_t>((reset_cause & 3u) << 3);
    xreg_[XregChipid - XregStart] = ChipidCc2530;
    xreg_[XregChipinfo0 - XregStart] = Chipinfo0Flash256k;
    xreg_[XregChipinfo1 - XregStart] = Chipinfo1Sram8k;
    posted_vector_ = 0;
    posted_level_ = 0;
    posted_source_ = -1;
    sleep_requested_ = false;
    events_.clear();
    stop_error_.clear();
    install_code_view();
    const std::uint16_t pc = 0;
    const std::uint16_t vector = 0;
    const std::uint8_t zero = 0;
    check_error(uc_reg_write(engine, UC_MCS51_REG_PC, &pc), "reset PC");
    check_error(uc_reg_write(engine, UC_MCS51_REG_IRQ_VECTOR, &vector),
                "reset pending interrupt");
    check_error(uc_reg_write(engine, UC_MCS51_REG_IRQ_LEVEL, &zero),
                "reset pending interrupt level");
    // A reset during (nested) ISRs returns the controller to power-on state.
    check_error(uc_reg_write(engine, UC_MCS51_REG_INTERRUPT_LEVELS, &zero),
                "reset in-progress interrupt levels");
    check_error(uc_reg_write(engine, UC_MCS51_REG_INTERRUPT_INHIBIT, &zero),
                "reset post-RETI inhibit");
    for (const auto &peripheral : peripherals_)
        peripheral->reset(cause);
    update_interrupts();
    if (reset_listener_)
        reset_listener_(cause);
}

void Cc2530F256::install_code_view() {
    std::uint8_t *code = nullptr;
    const unsigned bank = sfr_[SfrFmap] & 0x07u;
    if (sfr_[SfrMemctr] & MemctrXmap) {
        // SRAM stays in place at 0x8000-0x9fff; the rest of the upper half
        // still shows the FMAP bank.
        const std::uint8_t *source = views_.data() + bank * CodeSpace;
        std::copy(source + BankSize + SramSize, source + CodeSpace,
                  xmap_view_.begin() + BankSize + SramSize);
        code = xmap_view_.data();
    } else {
        code = views_.data() + bank * CodeSpace;
    }
    const uc_mcs51_fast_memory fast{code, idata_, sfr_.data(), sfr_special_read_.data(),
                                    sfr_special_write_.data()};
    const uc_mcs51_fast_memory *fast_pointer = &fast;
    check_error(uc_reg_write(reinterpret_cast<uc_engine *>(uc_),
                             UC_MCS51_REG_FAST_MEMORY, &fast_pointer),
                "install CODE view");
}

std::uint64_t Cc2530F256::sfr_mmio_read(uc_struct *, std::uint64_t offset,
                                        unsigned size, void *user_data) {
    auto *self = static_cast<Cc2530F256 *>(user_data);
    std::uint64_t value = 0;
    for (unsigned i = 0; i < size; ++i) {
        const std::uint64_t address = offset + i;
        const std::uint8_t byte =
            address >= 0x80 && address < self->sfr_.size()
                ? self->sfr_read(static_cast<std::uint8_t>(address))
                : 0;
        value |= static_cast<std::uint64_t>(byte) << (i * 8u);
    }
    return value;
}

// CPU/DMA read of an SFR with side effects.
std::uint8_t Cc2530F256::sfr_read(std::uint8_t address) {
    if (cc2530::Peripheral *owner = sfr_owner_[address]) {
        if (!sfr_special_read_[address])
            return sfr_[address];
        const std::uint8_t value = owner->read(cc2530::sfr_reg(address));
        peripheral_accessed();
        return value;
    }
    if (address == SfrSleepsta)
        return sleepsta_value();
    return sfr_[address];
}

std::uint8_t Cc2530F256::sfr_peek(std::uint8_t address) const {
    if (const cc2530::Peripheral *owner = sfr_owner_[address])
        return owner->peek(cc2530::sfr_reg(address));
    if (address == SfrSleepsta)
        return sleepsta_value();
    return sfr_[address];
}

std::uint8_t Cc2530F256::read_sfr(std::uint8_t address) const {
    return address < 0x80 ? idata_[address] : sfr_peek(address);
}

std::uint8_t Cc2530F256::sleepsta_value() const {
    // RST (bits 4:3) is latched at reset. CLK32K (bit 0) is the 32 kHz clock
    // level: 65536 half periods per second, i.e. one per 15625/32 cycles of
    // the 32 MHz time base, starting low at cycle 0 (phase is a model
    // choice). OSC32K_CALDIS (bit 7) shows SLEEPCMD.OSC32K_CALDIS while the
    // 32 kHz RC oscillator is selected (CLKCONSTA.OSC32K = 1), the model's
    // reading of "not set ... before the chip has been run on the 32-kHz RC
    // oscillator".
    std::uint8_t value = sfr_[SfrSleepsta] & 0x18u;
    const std::uint64_t lf = lf_now();
    const std::uint64_t half_periods = lf / 15625 * 32 + lf % 15625 * 32 / 15625;
    value |= static_cast<std::uint8_t>(half_periods & 1u);
    if (sfr_[SfrClkconsta] & 0x80u)
        value |= sfr_[SfrSleepcmd] & 0x80u;
    return value;
}

void Cc2530F256::peripheral_accessed() {
    if (!running_fast_)
        return;
    const std::uint64_t deadline = next_deadline();
    if (deadline < fast_event_stop_) {
        fast_event_stop_ = deadline;
        check_error(uc_reg_write(reinterpret_cast<uc_engine *>(uc_), UC_MCS51_REG_EVENT_CYCLES,
                                 &fast_event_stop_),
                    "lower the event stop");
    }
}

std::uint64_t Cc2530F256::next_deadline() const {
    std::uint64_t deadline = events_.empty() ? UINT64_MAX : events_.front().at;
    deadline = std::min({deadline, next_oscillator_stop(), clock_loss_deadline()});
    for (const auto &peripheral : peripherals_)
        deadline = std::min(deadline, peripheral->deadline());
    return deadline;
}

void Cc2530F256::refresh_flash(std::uint32_t begin, std::uint32_t end) {
    end = std::min<std::uint32_t>(end, FlashSize);
    for (std::uint32_t address = begin; address < end; ++address) {
        const std::uint8_t value = flash_[address];
        if (address < BankSize) {
            for (unsigned bank = 0; bank < BankCount; ++bank)
                views_[bank * CodeSpace + address] = value;
            xmap_view_[address] = value;
            views_[BankSize + address] = value; // bank 0 also fills view 0's upper half
        } else {
            const std::uint32_t bank = address / BankSize;
            views_[bank * CodeSpace + BankSize + address % BankSize] = value;
        }
    }
    install_code_view(); // the XMAP view copies the upper half
}

void Cc2530F256::sfr_mmio_write(uc_struct *, std::uint64_t offset, unsigned size,
                                std::uint64_t value, void *user_data) {
    auto *self = static_cast<Cc2530F256 *>(user_data);
    for (unsigned i = 0; i < size; ++i) {
        const std::uint64_t address = offset + i;
        if (address >= 0x80 && address < self->sfr_.size())
            self->sfr_write(static_cast<std::uint8_t>(address),
                            static_cast<std::uint8_t>(value >> (i * 8u)));
    }
}

std::uint64_t Cc2530F256::xdata_mmio_read(uc_struct *, std::uint64_t offset,
                                          unsigned size, void *user_data) {
    auto *self = static_cast<Cc2530F256 *>(user_data);
    std::uint64_t value = 0;
    for (unsigned i = 0; i < size; ++i) {
        const auto address = static_cast<std::uint16_t>(XdataMmioStart + offset + i);
        value |= static_cast<std::uint64_t>(self->xdata_read(address)) << (i * 8u);
    }
    return value;
}

void Cc2530F256::xdata_mmio_write(uc_struct *, std::uint64_t offset, unsigned size,
                                  std::uint64_t value, void *user_data) {
    auto *self = static_cast<Cc2530F256 *>(user_data);
    for (unsigned i = 0; i < size; ++i)
        self->xdata_write(static_cast<std::uint16_t>(XdataMmioStart + offset + i),
                          static_cast<std::uint8_t>(value >> (i * 8u)));
}

std::uint8_t Cc2530F256::xdata_value(std::uint16_t address) const {
    if (address < SramSize)
        return sram_[address];
    if (address >= XregStart && address < XregEnd) {
        if (const cc2530::Peripheral *owner = xreg_owner_[address - XregStart])
            return owner->peek(address);
        return xreg_[address - XregStart];
    }
    if (address >= SfrAliasStart && address < SfrAliasEnd) {
        const auto sfr = static_cast<std::uint8_t>(address);
        if (!cpu_internal_sfr(sfr) || sfr == SfrP0 || sfr == SfrP1 || sfr == SfrP2)
            return sfr_peek(sfr);
        return 0;
    }
    if (address >= InfoPageStart && address < 0x8000)
        return info_page_[address - InfoPageStart];
    if (address >= 0x8000)
        return flash_[(sfr_[SfrMemctr] & MemctrXbank) * BankSize + (address - 0x8000u)];
    return 0; // unimplemented areas read as zero
}

std::uint8_t Cc2530F256::xdata_read(std::uint16_t address) {
    if (address >= XregStart && address < XregEnd) {
        if (cc2530::Peripheral *owner = xreg_owner_[address - XregStart]) {
            const std::uint8_t value = owner->read(address);
            peripheral_accessed();
            return value;
        }
        if (address != XregChver && address != XregChipid && address != XregChipinfo0 &&
            address != XregChipinfo1 && address != XregCld)
            ++diagnostics_.unmodeled_xreg_reads[address];
    } else if (address >= SfrAliasStart && address < SfrAliasEnd) {
        const auto sfr = static_cast<std::uint8_t>(address);
        if (cpu_internal_sfr(sfr) && sfr != SfrP0 && sfr != SfrP1 && sfr != SfrP2)
            ++diagnostics_.unmapped_xdata_reads;
        else
            return sfr_read(sfr);
    } else if (address >= SramSize && address < InfoPageStart) {
        ++diagnostics_.unmapped_xdata_reads;
    }
    return xdata_value(address);
}

void Cc2530F256::xdata_write(std::uint16_t address, std::uint8_t value) {
    if (address < SramSize) {
        sram_[address] = value;
    } else if (address >= XregStart && address < XregEnd) {
        if (address == XregChver || address == XregChipid || address == XregChipinfo0 ||
            address == XregChipinfo1) {
            ++diagnostics_.read_only_xdata_writes;
            return;
        }
        if (cc2530::Peripheral *owner = xreg_owner_[address - XregStart]) {
            owner->write(address, value);
            peripheral_accessed();
            return;
        }
        if (address == XregCld) {
            // SWRU191F 5.2: CLD.EN (bit 0) enables the clock-loss detector;
            // bits 7:1 are reserved and read 0. Monitoring starts at the
            // write that enables it (clock_loss_deadline()).
            const bool was_enabled = (xreg_[address - XregStart] & CldEn) != 0;
            xreg_[address - XregStart] = value & CldEn;
            if (!was_enabled && (value & CldEn))
                cld_since_ = engine_cycles();
            peripheral_accessed();
            return;
        }
        ++diagnostics_.unmodeled_xreg_writes[address];
        xreg_[address - XregStart] = value;
    } else if (address >= SfrAliasStart && address < SfrAliasEnd) {
        const auto sfr = static_cast<std::uint8_t>(address);
        if (sfr == SfrP0 || sfr == SfrP1 || sfr == SfrP2)
            ++diagnostics_.read_only_xdata_writes;
        else if (cpu_internal_sfr(sfr))
            ++diagnostics_.unmapped_xdata_writes;
        else
            sfr_write(sfr, value);
    } else if (address >= InfoPageStart) {
        ++diagnostics_.read_only_xdata_writes; // info page and XBANK flash
    } else {
        ++diagnostics_.unmapped_xdata_writes;
    }
}

void Cc2530F256::sfr_write(std::uint8_t address, std::uint8_t value) {
    switch (address) {
    case SfrFmap:
        ++diagnostics_.fmap_writes;
        sfr_[address] = value & 0x07u; // every bank is valid on the F256
        install_code_view();
        break;
    case SfrMemctr: {
        ++diagnostics_.memctr_writes;
        const std::uint8_t old = sfr_[address];
        sfr_[address] = value & 0x0fu;
        if ((old ^ sfr_[address]) & MemctrXmap)
            install_code_view();
        break;
    }
    case SfrDps:
        sfr_[address] = value & 0x01u; // bits 7:1 reserved, read 0
        break;
    case SfrPcon:
        sfr_[address] = value & 0xfeu; // IDLE always reads 0
        if (value & 0x01u)
            enter_power_mode();
        break;
    case SfrClkconcmd: {
        const std::uint8_t previous = sfr_[address];
        sfr_[address] = value;
        // A new oscillator selection is what the clock-loss detector watches
        // from now on (clock_loss_deadline()).
        if ((previous ^ value) & (ClkconOsc32k | ClkconOsc)) {
            cld_since_ = engine_cycles();
            peripheral_accessed();
        }
        set_clock_status(clock_status(value, (sfr_[SfrClkconsta] & ClkconOsc) != 0));
        break;
    }
    case SfrClkconsta:
    case SfrSleepsta:
        ++diagnostics_.read_only_sfr_writes;
        break;
    case SfrSleepcmd:
        sfr_[address] = value & 0x87u;
        break;
    case SfrIen0:
        sfr_[address] = value & 0xbfu; // bit 6 reserved, read 0
        update_interrupts();
        break;
    case SfrIen1: case SfrIen2:
        sfr_[address] = value & 0x3fu; // bits 7:6 reserved, read 0
        update_interrupts();
        break;
    case SfrIrcon:
    case SfrTcon: case SfrS0con: case SfrS1con: case SfrIrcon2:
        if (address == SfrIrcon && (value & IrconStRequest))
            ++diagnostics_.ircon_bit6_writes; // "must be written 0": requests ST
        sfr_[address] = value;
        // Level-sensitive sources (port interrupts) re-assert a flag that
        // firmware cleared while their condition persists.
        for (const auto &peripheral : peripherals_)
            peripheral->cpu_flags_written();
        update_interrupts();
        peripheral_accessed();
        break;
    case SfrIp0: case SfrIp1:
        sfr_[address] = value;
        update_interrupts();
        break;
    default:
        if (cc2530::Peripheral *owner = sfr_owner_[address]) {
            owner->write(cc2530::sfr_reg(address), value);
            peripheral_accessed();
        } else {
            sfr_[address] = value;
        }
        break;
    }
}

void Cc2530F256::write_sfr(std::uint8_t address, std::uint8_t value) {
    if (address < 0x80)
        throw std::out_of_range("SFR addresses start at 0x80");
    sfr_write(address, value);
}

void Cc2530F256::write_sram(std::uint16_t address, std::uint8_t value) {
    if (address >= SramSize)
        throw std::out_of_range("SRAM address beyond 8 KiB");
    sram_[address] = value;
}

std::uint8_t Cc2530F256::checked_flag_mask(unsigned source, std::uint8_t flag_mask) const {
    if (source >= IrqSources.size())
        throw std::out_of_range("CC2530 interrupt sources are 0..17");
    const IrqSource &irq = IrqSources[source];
    if (flag_mask == 0 || (flag_mask & ~irq.flag_mask) != 0)
        throw std::invalid_argument("flag mask is not a subset of the source's flag bits");
    return flag_mask;
}

void Cc2530F256::set_interrupt_flags(unsigned source, std::uint8_t flag_mask) {
    const IrqSource &irq = IrqSources[source];
    sfr_[irq.flag_sfr] = static_cast<std::uint8_t>(sfr_[irq.flag_sfr] | flag_mask);
    update_interrupts();
}

void Cc2530F256::raise_interrupt_flag_for_test(unsigned source) {
    if (source >= IrqSources.size())
        throw std::out_of_range("CC2530 interrupt sources are 0..17");
    raise_interrupt_flag_for_test(source, IrqSources[source].flag_mask);
}

void Cc2530F256::raise_interrupt_flag_for_test(unsigned source, std::uint8_t flag_mask) {
    set_interrupt_flags(source, checked_flag_mask(source, flag_mask));
    ++diagnostics_.test_injected_flags;
}

void Cc2530F256::schedule_interrupt_flag_for_test(std::uint64_t at_cycle, unsigned source) {
    if (source >= IrqSources.size())
        throw std::out_of_range("CC2530 interrupt sources are 0..17");
    schedule_interrupt_flag_for_test(at_cycle, source, IrqSources[source].flag_mask);
}

void Cc2530F256::schedule_interrupt_flag_for_test(std::uint64_t at_cycle, unsigned source,
                                                  std::uint8_t flag_mask) {
    const Event event{at_cycle, event_sequence_++, source, checked_flag_mask(source, flag_mask)};
    const auto position =
        std::upper_bound(events_.begin(), events_.end(), event,
                         [](const Event &a, const Event &b) { return a.at < b.at; });
    events_.insert(position, event);
}

// Processes every test event and peripheral deadline due at the current
// instruction boundary, earliest first; on equal times test events come
// first (in scheduling order), then peripherals in construction order.
void Cc2530F256::apply_due_events() {
    constexpr unsigned MaxServicesPerBoundary = 100000;
    const std::uint64_t now = engine_cycles();
    for (unsigned services = 0;;) {
        if (pending_reset_ || !stop_error_.empty())
            return;
        cc2530::Peripheral *due = nullptr;
        std::uint64_t due_at = cc2530::Never;
        for (const auto &peripheral : peripherals_) {
            const std::uint64_t deadline = peripheral->deadline();
            if (deadline <= now && deadline < due_at) {
                due = peripheral.get();
                due_at = deadline;
            }
        }
        // Oscillator stops and the clock-loss reset come first at equal times.
        const std::uint64_t stop_at = next_oscillator_stop();
        const std::uint64_t loss_at = clock_loss_deadline();
        const std::uint64_t clock_at = std::min(stop_at, loss_at);
        if (clock_at <= now && clock_at <= due_at &&
            (events_.empty() || clock_at <= events_.front().at)) {
            if (clock_at == stop_at) {
                for (unsigned osc = 0; osc < osc_stop_at_.size(); ++osc)
                    if (osc_stop_at_[osc] == stop_at)
                        osc_stops_applied_ |= static_cast<std::uint8_t>(1u << osc);
                refresh_lf_domain(stop_at);
                continue;
            }
            // SWRU191F 5.2: a clock-loss detector reset, SLEEPSTA.RST = 11.
            bus_->request_reset(Cc2530ResetCause::ClockLoss);
            return;
        }
        if (!events_.empty() && events_.front().at <= now && events_.front().at <= due_at) {
            const Event event = events_.front();
            events_.erase(events_.begin());
            ++diagnostics_.scheduled_flag_events;
            set_interrupt_flags(event.source, event.flag_mask);
            continue;
        }
        if (!due)
            return;
        if (due == soc_->debug && engine_running_) {
            // A debug command may halt the CPU or execute DEBUG_INSTR on the
            // core: it is processed between engine runs, at this boundary.
            uc_emu_stop(reinterpret_cast<uc_engine *>(uc_));
            return;
        }
        if (++services > MaxServicesPerBoundary) {
            request_stop(std::string(due->name()) +
                         ": more than 100000 services at one instruction boundary");
            return;
        }
        due->service(now);
        ++diagnostics_.peripheral_services;
        if (due->deadline() <= now) {
            request_stop(std::string(due->name()) +
                         ": service() left its deadline at or before now");
            return;
        }
    }
}

std::uint64_t Cc2530F256::hf_now() const {
    const std::uint64_t now = engine_cycles();
    std::uint64_t stopped = hf_stopped_;
    if (power_mode_ == Cc2530PowerMode::Pm1 || power_mode_ == Cc2530PowerMode::Pm2 ||
        power_mode_ == Cc2530PowerMode::Pm3)
        stopped += now - power_mode_since_;
    return now - stopped;
}

std::uint64_t Cc2530F256::lf_now() const {
    const std::uint64_t now = engine_cycles();
    std::uint64_t stopped = lf_stopped_;
    if (lf_frozen_since_)
        stopped += now - *lf_frozen_since_;
    return now - stopped;
}

// ---- clock loss (stop_oscillator) ----
//
// SWRU191F 5.2: with CLD.EN = 1 the 32 MHz XOSC and the 32 kHz clock monitor
// each other; when one stops, a clock-loss reset (SLEEPSTA.RST = 11) follows
// "within a certain maximum time-out period": 0.25 ms for the 32 MHz XOSC,
// 0.5 ms for the 32 kHz clock. The detector stops in PM1/PM2 and restarts
// when the clocks start again. Model-defined rules where the text is silent:
// - the reset comes exactly at the maximum time-out after the later of the
//   stop and the start of monitoring (CLD.EN 0 -> 1, a new oscillator
//   selection in CLKCONCMD, the wake from PM1-PM3);
// - the XOSC is watched while CLKCONCMD.OSC selects it ("It is assumed that
//   the 32-MHz XOSC is selected as system clock source"), the 32 kHz clock is
//   the source CLKCONSTA.OSC32K selects; the 16 MHz RCOSC is not watched
//   (the text names only the two XOSC sources and the 32 kHz RCOSC);
// - PM3 with CLD.EN = 1 is not specified (firmware must disable the
//   detector first): the detector is treated as stopped, as in PM1/PM2, and
//   the entry is counted in diagnostics().clock_loss_pm3_entries.
// Without a reset, a stopped clock has its datasheet consequences: the CPU
// stalls without its system clock (4.4.2), the sleep timer and the watchdog
// stop with the 32 kHz clock (4.4.3), and a switch to a stopped oscillator
// never completes, since the source changes only once it is stable (4.4.2).
// Model-defined approximation: peripherals clocked by the system clock keep
// their virtual time while the CPU stalls.

void Cc2530F256::stop_oscillator(Cc2530Oscillator osc, std::uint64_t at_cycle) {
    const auto index = static_cast<unsigned>(osc);
    if (index >= osc_stop_at_.size())
        throw std::out_of_range("unknown CC2530 oscillator");
    const std::uint64_t at = std::max(at_cycle, engine_cycles());
    if (at >= osc_stop_at_[index])
        return; // already stopped (or stopping) earlier
    osc_stop_at_[index] = at;
    osc_stops_applied_ &= static_cast<std::uint8_t>(~(1u << index));
}

bool Cc2530F256::clock_stalled() const { return !sleep_requested_ && cpu_clock_stopped(); }

bool Cc2530F256::oscillator_stopped(Cc2530Oscillator osc, std::uint64_t at) const {
    return osc_stop_at_[static_cast<unsigned>(osc)] <= at;
}

bool Cc2530F256::cpu_clock_stopped() const {
    const Cc2530Oscillator clock = (sfr_[SfrClkconsta] & ClkconOsc) ? Cc2530Oscillator::Rcosc16M
                                                                     : Cc2530Oscillator::Xosc32M;
    return oscillator_stopped(clock, engine_cycles());
}

std::uint64_t Cc2530F256::next_oscillator_stop() const {
    std::uint64_t next = UINT64_MAX;
    for (unsigned osc = 0; osc < osc_stop_at_.size(); ++osc)
        if (!(osc_stops_applied_ & (1u << osc)))
            next = std::min(next, osc_stop_at_[osc]);
    return next;
}

std::uint64_t Cc2530F256::clock_loss_deadline() const {
    if (!(xreg_[XregCld - XregStart] & CldEn) || power_mode_ == Cc2530PowerMode::Pm1 ||
        power_mode_ == Cc2530PowerMode::Pm2 || power_mode_ == Cc2530PowerMode::Pm3)
        return UINT64_MAX;
    std::uint64_t deadline = UINT64_MAX;
    const auto watch = [&](Cc2530Oscillator osc, std::uint64_t timeout) {
        const std::uint64_t stop = osc_stop_at_[static_cast<unsigned>(osc)];
        if (stop != UINT64_MAX)
            deadline = std::min(deadline, std::max(stop, cld_since_) + timeout);
    };
    if (!(sfr_[SfrClkconcmd] & ClkconOsc))
        watch(Cc2530Oscillator::Xosc32M, CldXoscTimeout);
    watch((sfr_[SfrClkconsta] & ClkconOsc32k) ? Cc2530Oscillator::Rcosc32k
                                              : Cc2530Oscillator::Xosc32k,
          Cld32kTimeout);
    return deadline;
}

std::uint8_t Cc2530F256::clock_status(std::uint8_t command, bool on_rcosc) const {
    // The clock switch completes at once; oscillator start-up and
    // stabilization time are not modeled. SWRU191F 4.4.2: the source changes
    // only once the new one is stable, which a stopped oscillator never is.
    std::uint8_t status = command;
    const bool to_rcosc = (command & ClkconOsc) != 0;
    if (to_rcosc != on_rcosc &&
        oscillator_stopped(to_rcosc ? Cc2530Oscillator::Rcosc16M : Cc2530Oscillator::Xosc32M,
                           engine_cycles()))
        status = static_cast<std::uint8_t>(on_rcosc ? (status | ClkconOsc) : (status & ~ClkconOsc));
    if (status & ClkconOsc) {
        // The 16 MHz RC oscillator limits TICKSPD and CLKSPD to 16 MHz.
        if ((status & 0x38u) == 0)
            status |= 0x08u;
        if ((status & 0x07u) == 0)
            status |= 0x01u;
    }
    // SWRU191F 4.4: a TICKSPD faster than the system clock (smaller
    // divider code than CLKSPD) shows, and runs, at the system clock.
    if (((status >> 3) & 0x07u) < (status & 0x07u))
        status = static_cast<std::uint8_t>((status & ~0x38u) | ((status & 0x07u) << 3));
    return status;
}

void Cc2530F256::set_clock_status(std::uint8_t status) {
    const std::uint8_t previous = sfr_[SfrClkconsta];
    if (status == previous)
        return;
    sfr_[SfrClkconsta] = status;
    if ((previous ^ status) & ClkconOsc32k)
        refresh_lf_domain(engine_cycles());
    for (const auto &peripheral : peripherals_)
        peripheral->clock_changed();
    peripheral_accessed();
}

void Cc2530F256::refresh_lf_domain(std::uint64_t at) {
    // The 32 kHz domain (sleep timer, watchdog, CLK32K) is frozen in PM3 and
    // while the selected 32 kHz source is stopped (SWRU191F 4.4.3).
    const Cc2530Oscillator clock = (sfr_[SfrClkconsta] & ClkconOsc32k) ? Cc2530Oscillator::Rcosc32k
                                                                       : Cc2530Oscillator::Xosc32k;
    const bool frozen = power_mode_ == Cc2530PowerMode::Pm3 || oscillator_stopped(clock, at);
    if (frozen && !lf_frozen_since_) {
        lf_frozen_since_ = at;
    } else if (!frozen && lf_frozen_since_) {
        lf_stopped_ += at - *lf_frozen_since_;
        lf_frozen_since_.reset();
    }
}

std::uint64_t Cc2530F256::engine_cycles() const {
    std::uint64_t cycles = 0;
    check_error(uc_reg_read(reinterpret_cast<uc_engine *>(uc_), UC_MCS51_REG_CYCLES, &cycles),
                "read cycles");
    return cycles;
}

void Cc2530F256::set_ieee_address(std::uint64_t address) {
    for (unsigned i = 0; i < 8; ++i)
        info_page_[IeeeAddressOffset - InfoPageStart + i] =
            static_cast<std::uint8_t>(address >> (8 * i));
}

std::uint16_t Cc2530F256::engine_irq_vector() const {
    std::uint16_t vector = 0;
    check_error(uc_reg_read(reinterpret_cast<uc_engine *>(uc_), UC_MCS51_REG_IRQ_VECTOR,
                            &vector),
                "read pending interrupt");
    return vector;
}

std::uint8_t Cc2530F256::engine_levels() const {
    std::uint8_t levels = 0;
    check_error(uc_reg_read(reinterpret_cast<uc_engine *>(uc_),
                            UC_MCS51_REG_INTERRUPT_LEVELS, &levels),
                "read in-progress interrupt levels");
    return levels;
}

bool Cc2530F256::engine_inhibit() const {
    std::uint8_t inhibit = 0;
    check_error(uc_reg_read(reinterpret_cast<uc_engine *>(uc_),
                            UC_MCS51_REG_INTERRUPT_INHIBIT, &inhibit),
                "read post-RETI inhibit");
    return inhibit != 0;
}

// The request the core sees: the highest priority level among the enabled
// pending sources (SWRU191F Tables 2-6/2-7), ties resolved by the polling
// sequence of Table 2-8. Whether it may preempt the ISRs in progress is the
// core's decision (UC_MCS51_REG_IRQ_LEVEL).
Cc2530F256::Request Cc2530F256::pending_interrupt() const {
    Request best;
    if (!(sfr_[SfrIen0] & Ien0Ea))
        return best;
    int best_level = -1;
    for (const std::uint8_t source : PollingOrder) {
        const IrqSource &irq = IrqSources[source];
        if (!flag_requested(sfr_, source) || !(sfr_[irq.enable_sfr] & irq.enable_mask))
            continue;
        const int level = (((sfr_[SfrIp1] >> irq.group) & 1) << 1) |
                          ((sfr_[SfrIp0] >> irq.group) & 1);
        if (level > best_level) {
            best.source = source;
            best.level = static_cast<std::uint8_t>(level);
            best_level = level;
        }
    }
    return best;
}

bool Cc2530F256::sync_interrupt_consumption() {
    // The core clears its pending vector when it vectors to it. Every run
    // stops right at the vector (BREAK_PC, or the code hook), so the flag is
    // cleared before the first ISR instruction executes. A DEBUG_INSTR runs
    // with the vector withdrawn; debug_execute() re-posts it.
    if (debug_executing_ || posted_vector_ == 0 || engine_irq_vector() != 0)
        return false;
    const IrqSource &irq = IrqSources[static_cast<std::size_t>(posted_source_)];
    if (irq.hardware_clear)
        sfr_[irq.flag_sfr] &= static_cast<std::uint8_t>(~irq.flag_mask);
    ++diagnostics_.interrupts_taken;
    posted_vector_ = 0;
    posted_level_ = 0;
    posted_source_ = -1;
    return true;
}

void Cc2530F256::check_interrupts_at_boundary() {
    if (!sync_interrupt_consumption()) {
        const Request request = pending_interrupt();
        const std::uint16_t vector = request.source < 0 ? 0 : irq_vector(request.source);
        if (vector != posted_vector_ || (vector != 0 && request.level != posted_level_))
            ++diagnostics_.stale_interrupt_posts;
    }
    update_interrupts();
}

void Cc2530F256::update_interrupts() {
    if (debug_executing_)
        return; // debug_execute() re-evaluates afterwards
    sync_interrupt_consumption();
    const Request request = pending_interrupt();
    const std::uint16_t vector = request.source < 0 ? 0 : irq_vector(request.source);
    if (vector == posted_vector_ && request.level == posted_level_)
        return;
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    // The level first: the core reads it when it takes the vector.
    check_error(uc_reg_write(engine, UC_MCS51_REG_IRQ_LEVEL, &request.level),
                "post interrupt level");
    posted_level_ = request.level;
    posted_source_ = request.source;
    if (vector == posted_vector_)
        return; // an IP write changed the level of the posted request
    check_error(uc_reg_write(engine, UC_MCS51_REG_IRQ_VECTOR, &vector),
                "post interrupt vector");
    posted_vector_ = vector;
    // The fast interpreter only stops at the BREAK_PC armed when it started;
    // return to run() so the new vector becomes the stop address.
    if (running_fast_)
        uc_emu_stop(engine);
}

bool Cc2530F256::interrupt_deliverable(bool honor_inhibit) const {
    if (posted_vector_ == 0 || (honor_inhibit && engine_inhibit()))
        return false;
    const std::uint8_t levels = engine_levels();
    if (levels == 0)
        return true;
    unsigned highest = 7;
    while (!(levels & (1u << highest)))
        --highest;
    return posted_level_ > highest;
}

void Cc2530F256::enter_power_mode() {
    ++diagnostics_.power_mode_entries;
    // SLEEPCMD.MODE selects the mode PCON.IDLE enters (SWRU191F 4.1, 4.3).
    const unsigned mode_bits = sfr_[SfrSleepcmd] & 0x03u;
    const auto mode = static_cast<Cc2530PowerMode>(
        static_cast<unsigned>(Cc2530PowerMode::Idle) + mode_bits);
    ++diagnostics_.power_mode_entries_by_mode[static_cast<unsigned>(mode)];
    // While the PCON write executes, the core's PC is that of the writing
    // instruction (SWRU191F 4.2 wants the next one off a 4-byte boundary).
    const std::uint16_t at = pc();
    if (((at + instruction_size(read_code(at))) & 3u) == 0)
        ++diagnostics_.idle_alignment_violations;
    // A request that already satisfies the mode's wake rule (wake_request)
    // ends the mode at once. The same rule applies while asleep, so the
    // outcome does not depend on whether the request arrives before or after
    // this write.
    update_interrupts();
    if (wake_request(mode)) {
        ++diagnostics_.power_mode_immediate_wakes;
        wake_from_power_mode();
        return;
    }
    // The mode takes effect at the next instruction boundary
    // (settle_power_mode), when the PCON write has completed.
    requested_mode_ = mode;
    sleep_requested_ = true;
    uc_emu_stop(reinterpret_cast<uc_engine *>(uc_));
}

// Wake rules (MODEL-DEFINED from SWRU191F 4.1):
//  - IDLE: "any enabled interrupt wakes up the CPU core": any posted request
//    (EA, the source's enable bit and its flag set), whatever its priority
//    relative to the ISRs in progress.
//  - PM1/PM2: "reset, an external interrupt or when the sleep timer
//    expires": EA plus enable and flag of P0INT, P1INT, P2INT or ST (the
//    ST request includes IRCON bit 6, Figure 2-4).
//  - PM3: "reset or an external interrupt": EA plus enable and flag of
//    P0INT, P1INT or P2INT.
// Resets are applied separately (apply_pending_reset).
bool Cc2530F256::wake_request(Cc2530PowerMode mode) const {
    if (mode == Cc2530PowerMode::Active || mode == Cc2530PowerMode::Idle)
        return posted_vector_ != 0;
    if (!(sfr_[SfrIen0] & Ien0Ea))
        return false;
    using cc2530::Irq;
    for (const Irq source : {Irq::P0Int, Irq::P1Int, Irq::P2Int, Irq::St}) {
        if (source == Irq::St && mode == Cc2530PowerMode::Pm3)
            continue;
        const IrqSource &irq = IrqSources[static_cast<unsigned>(source)];
        if (flag_requested(sfr_, static_cast<unsigned>(source)) &&
            (sfr_[irq.enable_sfr] & irq.enable_mask))
            return true;
    }
    return false;
}

void Cc2530F256::notify_power_mode(Cc2530PowerMode from, Cc2530PowerMode to) {
    for (const auto &peripheral : peripherals_)
        peripheral->power_mode_changed(from, to);
    update_interrupts();
}

// Makes the requested mode effective at the boundary after the PCON write.
// Due events have been processed at this boundary already.
void Cc2530F256::settle_power_mode() {
    if (!sleep_requested_ || power_mode_ != Cc2530PowerMode::Active)
        return;
    power_mode_ = requested_mode_;
    power_mode_since_ = engine_cycles();
    refresh_lf_domain(power_mode_since_);
    if (power_mode_ == Cc2530PowerMode::Pm3 && (xreg_[XregCld - XregStart] & CldEn))
        ++diagnostics_.clock_loss_pm3_entries;
    notify_power_mode(Cc2530PowerMode::Active, power_mode_);
}

void Cc2530F256::wake_from_power_mode() {
    // SWRU191F 4.2: the instruction after the one that set PCON.IDLE runs
    // before the ISR of the waking interrupt; the core's one-instruction
    // inhibit provides exactly that. SWRU191F 4.1: after PM1-PM3 the chip
    // runs on the 16 MHz RCOSC until the XOSC is stable again; CLKCONSTA
    // keeps reporting the commanded clock (approximation, no start-up time).
    const Cc2530PowerMode from = power_mode_;
    if (from != Cc2530PowerMode::Active) {
        const std::uint64_t elapsed = engine_cycles() - power_mode_since_;
        diagnostics_.power_mode_cycles_by_mode[static_cast<unsigned>(from)] += elapsed;
        if (from != Cc2530PowerMode::Idle)
            hf_stopped_ += elapsed;
    }
    power_mode_ = requested_mode_ = Cc2530PowerMode::Active;
    refresh_lf_domain(engine_cycles());
    sleep_requested_ = false;
    ++diagnostics_.power_mode_wakes;
    const std::uint8_t inhibit = 1;
    check_error(uc_reg_write(reinterpret_cast<uc_engine *>(uc_),
                             UC_MCS51_REG_INTERRUPT_INHIBIT, &inhibit),
                "arm the post-wake instruction");
    if (from != Cc2530PowerMode::Active)
        notify_power_mode(from, Cc2530PowerMode::Active);
    if (from == Cc2530PowerMode::Pm1 || from == Cc2530PowerMode::Pm2 ||
        from == Cc2530PowerMode::Pm3) {
        // SWRU191F 5.2: the detector stopped in PM1/PM2 restarts "when the
        // clocks start up again". 4.1: the chip wakes on the 16 MHz RCOSC
        // and changes to the XOSC once it is stable; a stopped XOSC never
        // is, so CLKCONSTA.OSC stays 1. Without stopped oscillators the
        // status equals the commanded one, as before the power mode.
        cld_since_ = engine_cycles();
        set_clock_status(clock_status(sfr_[SfrClkconcmd], true));
    }
}

void Cc2530F256::request_stop(std::string error) {
    if (stop_error_.empty())
        stop_error_ = std::move(error);
    uc_emu_stop(reinterpret_cast<uc_engine *>(uc_));
}

bool Cc2530F256::idle_forever() const {
    const std::uint16_t at = pc();
    const std::uint8_t opcode = read_code(at);
    bool self_loop = false;
    if (opcode == 0x80) // SJMP rel
        self_loop = read_code(static_cast<std::uint16_t>(at + 1)) == 0xfe;
    else if (opcode == 0x02) // LJMP addr16
        self_loop = ((read_code(static_cast<std::uint16_t>(at + 1)) << 8) |
                     read_code(static_cast<std::uint16_t>(at + 2))) == at;
    else if ((opcode & 0x1fu) == 0x01u) // AJMP addr11
        self_loop = ((((at + 2) & 0xf800u) | ((opcode & 0xe0u) << 3) |
                      read_code(static_cast<std::uint16_t>(at + 1))) & 0xffffu) == at;
    if (!self_loop || !events_.empty() || pending_reset_ || stall_until_ > engine_cycles() ||
        clock_loss_deadline() != UINT64_MAX)
        return false;
    for (const auto &peripheral : peripherals_)
        if (peripheral->may_interrupt())
            return false;
    return !interrupt_deliverable(false);
}

bool Cc2530F256::may_wake() const {
    if (!events_.empty() || pending_reset_ || clock_loss_deadline() != UINT64_MAX)
        return true;
    for (const auto &peripheral : peripherals_)
        if (peripheral->may_interrupt())
            return true;
    return false;
}

void Cc2530F256::code_hook(uc_struct *opaque_uc, std::uint64_t address, std::uint32_t,
                           void *user_data) {
    // Installed only while tracing.
    // Stops before `address` executes, with the precedence run() applies
    // between fast-mode stops: cycle limit, then breakpoint. The first call
    // is the start address, which run() has already checked.
    auto *self = static_cast<Cc2530F256 *>(user_data);
    if (self->debug_executing_)
        return; // a DEBUG_INSTR is not a firmware instruction boundary
    auto *engine = reinterpret_cast<uc_engine *>(opaque_uc);
    const auto pc = static_cast<std::uint16_t>(address);
    self->apply_due_events();
    self->check_interrupts_at_boundary();
    // A reset, CPU stall, power mode, debug command, RESET_N change or fault
    // requested by the previous instruction or by a peripheral serviced at
    // this boundary takes effect before this instruction; run() handles it.
    cc2530::Debug &debug = *self->soc_->debug;
    if (self->pending_reset_ || self->sleep_requested_ || !self->stop_error_.empty() ||
        self->stall_until_ > self->engine_cycles() || self->cpu_clock_stopped() ||
        debug.deadline() <= self->engine_cycles() || !self->pending_reset_levels_.empty()) {
        uc_emu_stop(engine);
        return;
    }
    if (self->hook_calls_++ != 0) {
        debug.progressed();
        std::uint64_t cycles = 0;
        uc_reg_read(engine, UC_MCS51_REG_CYCLES, &cycles);
        if (cycles >= self->trace_cycle_limit_) {
            uc_emu_stop(engine);
            return;
        }
        // Debug-interface hardware breakpoints come before the emulator's
        // own breakpoint, as in run().
        if (debug.breakpoint_at(pc, self->sfr_[SfrFmap])) {
            debug.breakpoint_hit();
            uc_emu_stop(engine);
            return;
        }
        if (self->breakpoint_ && *self->breakpoint_ == pc) {
            self->break_hit_ = true;
            uc_emu_stop(engine);
            return;
        }
    }
    if (self->trace_output_)
        self->trace_instruction(pc);
}

void Cc2530F256::trace_instruction(std::uint16_t pc) {
    auto &out = *trace_output_;
    out << "pc=" << hex(pc, 4) << " fmap=" << unsigned(sfr_[SfrFmap]);
    if (const auto physical = code_physical(pc))
        out << " phys=" << hex(*physical, 5);
    else
        out << " phys=sram:" << hex(pc - 0x8000u, 4);
    if (interrupt_deliverable(true)) {
        // The core enters an interrupt instead of executing this instruction.
        out << "  irq -> " << hex(posted_vector_, 4) << ' '
            << IrqSources[static_cast<std::size_t>(posted_source_)].name
            << " level=" << unsigned(posted_level_) << '\n';
        return;
    }
    const std::uint8_t opcode = read_code(pc);
    const unsigned size = instruction_size(opcode);
    out << "  " << std::hex << std::setfill('0');
    for (unsigned i = 0; i < size; ++i)
        out << std::setw(2) << unsigned(read_code(static_cast<std::uint16_t>(pc + i))) << ' ';
    out << std::dec << std::setfill(' ') << '\n';
}

void Cc2530F256::set_code_hook_enabled(bool enabled) {
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    if (enabled == code_hook_installed_)
        return;
    if (enabled) {
        check_error(uc_hook_add(engine, reinterpret_cast<uc_hook *>(&code_hook_),
                                UC_HOOK_CODE,
                                reinterpret_cast<void *>(
                                    static_cast<uc_cb_hookcode_t>(Cc2530F256::code_hook)),
                                this, 1, 0),
                    "install code hook");
        code_hook_installed_ = true;
    } else {
        check_error(uc_hook_del(engine, static_cast<uc_hook>(code_hook_)),
                    "remove code hook");
        code_hook_ = 0;
        code_hook_installed_ = false;
    }
    // Translated blocks bake in whether a hook wants every instruction.
    check_error(uc_ctl_flush_tb(engine), "flush translated blocks");
}

Cc2530RunResult Cc2530F256::run(std::size_t instruction_limit, std::uint64_t cycle_limit) {
    Cc2530RunResult result;
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    std::uint64_t before = 0;
    check_error(uc_reg_read(engine, UC_MCS51_REG_INSTRUCTIONS, &before),
                "read instruction counter");
    const bool tracing = trace_output_ != nullptr;
    set_code_hook_enabled(tracing);
    stop_error_.clear();
    break_hit_ = false;

    uc_err error = UC_ERR_OK;
    bool idled = false;
    cc2530::Debug &debug = *soc_->debug;
    while (result.instructions < instruction_limit) {
        apply_pending_reset_pin();
        apply_pending_reset();
        apply_due_events();
        if (!stop_error_.empty())
            break;
        if (pending_reset_ || !pending_reset_levels_.empty())
            continue;
        check_interrupts_at_boundary();
        std::uint64_t cycles = engine_cycles();
        if (cpu_held()) {
            // RESET_N low, or halted by the debug interface (SWRU191F 3.3):
            // no instruction or interrupt entry; time and the peripherals
            // (as the debug configuration allows) advance to the next event.
            if (result.instructions != 0)
                break; // the CPU was halted during this call
            const std::uint64_t until = std::min(cycle_limit, next_deadline());
            if (until == UINT64_MAX || until <= cycles)
                break;
            (reset_held_ ? diagnostics_.reset_pin_cycles : diagnostics_.debug_halted_cycles) +=
                until - cycles;
            check_error(uc_reg_write(engine, UC_MCS51_REG_CYCLES, &until), "advance halted time");
            idled = true;
            continue;
        }
        if (stall_until_ > cycles) {
            // SWRU191F 6.3.1: the CPU waits for a flash write/erase started
            // from flash; time and peripherals advance, no instruction or
            // interrupt entry happens.
            const std::uint64_t until = std::min({stall_until_, cycle_limit, next_deadline()});
            if (until <= cycles)
                break; // stalled at the cycle limit
            diagnostics_.cpu_stall_cycles += until - cycles;
            check_error(uc_reg_write(engine, UC_MCS51_REG_CYCLES, &until), "advance stall time");
            idled = true;
            continue;
        }
        if (sleep_requested_) {
            settle_power_mode();
            if (wake_request(power_mode_)) {
                wake_from_power_mode();
                continue;
            }
            const std::uint64_t until = std::min(cycle_limit, next_deadline());
            if (until == UINT64_MAX || until <= cycles)
                break; // asleep at the limit, or for good
            if (!may_wake() && cycle_limit == UINT64_MAX)
                break; // nothing can end this power mode
            diagnostics_.idle_cycles += until - cycles;
            check_error(uc_reg_write(engine, UC_MCS51_REG_CYCLES, &until), "advance idle time");
            idled = true;
            continue;
        }
        if (cpu_clock_stopped()) {
            // Model-defined (stop_oscillator): without its system clock the
            // CPU executes nothing and takes no interrupt; time advances to
            // the next event, which may be a clock-loss or watchdog reset.
            const std::uint64_t until = std::min(cycle_limit, next_deadline());
            if (until == UINT64_MAX || until <= cycles)
                break; // stalled at the limit, or for good
            diagnostics_.clock_stall_cycles += until - cycles;
            check_error(uc_reg_write(engine, UC_MCS51_REG_CYCLES, &until), "advance stall time");
            idled = true;
            continue;
        }
        if (cycles >= cycle_limit && (result.instructions != 0 || idled))
            break;
        std::uint16_t pc = 0;
        check_error(uc_reg_read(engine, UC_MCS51_REG_PC, &pc), "read PC");
        if (debug.breakpoint_at(pc, sfr_[SfrFmap])) {
            debug.breakpoint_hit(); // HALT_STATUS (Table 3-3)
            continue;
        }
        if (breakpoint_ && *breakpoint_ == pc && result.instructions != 0) {
            break_hit_ = true;
            break;
        }

        // STEP_INSTR executes exactly one instruction (or interrupt entry).
        const bool stepping = debug.stepping();
        if (tracing) {
            hook_calls_ = 0;
            trace_cycle_limit_ = cycle_limit;
            engine_running_ = true;
            error = uc_emu_start(engine, pc, CodeSpace, 0,
                                 stepping ? 1 : instruction_limit - result.instructions);
            engine_running_ = false;
        } else {
            std::uint64_t current = before + result.instructions;
            std::uint64_t instruction_stop =
                instruction_limit > UINT64_MAX - before ? UINT64_MAX
                                                        : before + instruction_limit;
            std::uint32_t break_pc = NoBreak;
            if (stepping) {
                instruction_stop = std::min(instruction_stop, current + 1);
            } else if (debug.breakpoints_armed()) {
                // Every address the engine must stop at: the posted vector,
                // the emulator breakpoint and the enabled hardware
                // breakpoints (any bank: run() compares FMAP). One is the
                // break address; more are single-stepped.
                std::array<std::uint32_t, 6> stops{};
                unsigned count = 0;
                const auto add_stop = [&](std::uint32_t address) {
                    if (std::find(stops.begin(), stops.begin() + count, address) ==
                        stops.begin() + count)
                        stops[count++] = address;
                };
                if (posted_vector_ != 0)
                    add_stop(posted_vector_);
                if (breakpoint_)
                    add_stop(*breakpoint_);
                for (const cc2530::Debug::Breakpoint &hardware : debug.breakpoints())
                    if (hardware.enabled)
                        add_stop(hardware.address);
                if (count > 1)
                    instruction_stop = std::min(instruction_stop, current + 1);
                else if (count == 1)
                    break_pc = stops[0];
            } else if (posted_vector_ != 0 && breakpoint_)
                instruction_stop = std::min(instruction_stop, current + 1); // single-step
            else if (posted_vector_ != 0)
                break_pc = posted_vector_;
            else if (breakpoint_)
                break_pc = *breakpoint_;
            // Stop at the first boundary at which the next scheduled event or
            // peripheral deadline is due, as trace mode (code_hook) and single
            // steps do. Register accesses that move a deadline earlier lower
            // the stop (peripheral_accessed).
            fast_event_stop_ = std::min(cycle_limit, next_deadline());
            check_error(uc_reg_write(engine, UC_MCS51_REG_EVENT_CYCLES, &fast_event_stop_),
                        "set cycle limit");
            check_error(uc_reg_write(engine, UC_MCS51_REG_INSTRUCTION_LIMIT,
                                     &instruction_stop),
                        "set instruction limit");
            check_error(uc_reg_write(engine, UC_MCS51_REG_BREAK_PC, &break_pc),
                        "set break address");
            running_fast_ = true;
            engine_running_ = true;
            error = uc_emu_start(engine, pc, CodeSpace, 0, 0);
            engine_running_ = false;
            running_fast_ = false;
        }
        std::uint64_t current = 0;
        check_error(uc_reg_read(engine, UC_MCS51_REG_INSTRUCTIONS, &current),
                    "read instruction counter");
        const auto executed = static_cast<std::size_t>(current - before);
        if (error != UC_ERR_OK || !stop_error_.empty())
            break;
        if (executed != result.instructions) {
            debug.progressed();
            if (stepping)
                debug.step_completed(sfr_[SfrAcc]);
        }
        if (executed == result.instructions && !break_hit_ && !sleep_requested_ &&
            !pending_reset_ && stall_until_ <= engine_cycles() && !cpu_clock_stopped() &&
            !cpu_held() && debug.deadline() > engine_cycles() && pending_reset_levels_.empty()) {
            error = UC_ERR_EXCEPTION; // no progress
            break;
        }
        result.instructions = executed;
        if (break_hit_)
            break;
    }
    // The closing boundary gets the same treatment as every other boundary,
    // so the state at a stop does not depend on whether the run ended on its
    // instruction or cycle limit: pending reset, due events, a power mode
    // taking effect and a wake request.
    if (stop_error_.empty() && error == UC_ERR_OK) {
        apply_pending_reset_pin();
        apply_pending_reset();
        apply_due_events();
        apply_pending_reset();
    }
    check_interrupts_at_boundary();
    if (sleep_requested_ && stop_error_.empty() && !cpu_held()) {
        settle_power_mode();
        if (wake_request(power_mode_))
            wake_from_power_mode();
    }
    // A hardware breakpoint at the closing boundary halts the CPU in this
    // call, as at any boundary inside a longer run, so where an instruction
    // or cycle limit ended the call does not move the halt.
    if (stop_error_.empty() && error == UC_ERR_OK && !cpu_held() && !sleep_requested_ &&
        !pending_reset_ && stall_until_ <= engine_cycles()) {
        std::uint16_t pc = 0;
        check_error(uc_reg_read(engine, UC_MCS51_REG_PC, &pc), "read PC");
        if (debug.breakpoint_at(pc, sfr_[SfrFmap]))
            debug.breakpoint_hit();
    }
    std::uint64_t after = 0;
    check_error(uc_reg_read(engine, UC_MCS51_REG_INSTRUCTIONS, &after),
                "read instruction counter");
    result.instructions = static_cast<std::size_t>(after - before);
    check_error(uc_reg_read(engine, UC_MCS51_REG_PC, &result.pc), "read PC");
    check_error(uc_reg_read(engine, UC_MCS51_REG_CYCLES, &result.cycles), "read cycles");
    result.fmap = sfr_[SfrFmap];
    result.break_hit = break_hit_;
    result.sleeping = sleep_requested_;
    result.debug_halted = debug.halted();
    result.reset_held = reset_held_;
    if (error != UC_ERR_OK)
        result.error = std::string(uc_strerror(error)) + " at pc=" + hex(result.pc, 4) +
                       " fmap=" + std::to_string(result.fmap);
    else if (!stop_error_.empty())
        result.error = stop_error_ + " (pc=" + hex(result.pc, 4) + ")";
    result.step_limit_reached = result.error.empty() && !result.break_hit &&
                                result.instructions >= instruction_limit;
    return result;
}

// ---- RESET_N and the debug interface ----

bool Cc2530F256::cpu_held() const { return reset_held_ || soc_->debug->cpu_held(); }

void Cc2530F256::set_reset_pin(bool high) {
    if (engine_running_) {
        // A pad change made by the running firmware (a GPIO wired to the
        // chip's own RESET_N): applied at the next instruction boundary.
        pending_reset_levels_.push_back(high);
        uc_emu_stop(reinterpret_cast<uc_engine *>(uc_));
        return;
    }
    if (reset_held_ == !high)
        return;
    if (!high) {
        // SWRU191F 4.3: RESET_N low resets the chip; it stays in reset (no
        // instruction executes) while the pin is low. The debug interface
        // counts DC falling edges from here on (3.1).
        reset_held_ = true;
        reset_core(Cc2530ResetCause::External);
        return;
    }
    // Model-defined: the chip restarts from a fresh reset state at the
    // release (time advanced while held), counted as one external reset.
    const unsigned edges = soc_->debug->entry_edges();
    reset_held_ = false;
    reset_core(Cc2530ResetCause::External, false);
    if (edges >= 2)
        soc_->debug->enter(); // 3.1: two DC falling edges while RESET_N low
}

void Cc2530F256::apply_pending_reset_pin() {
    if (pending_reset_levels_.empty())
        return;
    const std::vector<bool> levels = std::move(pending_reset_levels_);
    pending_reset_levels_.clear();
    for (const bool high : levels)
        set_reset_pin(high);
}

void Cc2530F256::debug_if_enter() {
    if (engine_running_)
        throw std::logic_error("debug_if_enter() inside run()");
    pending_reset_levels_.clear();
    reset_held_ = false;
    reset_core(Cc2530ResetCause::External);
    soc_->debug->enter();
}

bool Cc2530F256::debug_if_active() const { return soc_->debug->active(); }
bool Cc2530F256::debug_if_halted() const { return soc_->debug->halted(); }
bool Cc2530F256::debug_if_locked() const { return soc_->debug->locked(); }
std::uint8_t Cc2530F256::debug_if_status() const { return soc_->debug->status(); }
std::uint8_t Cc2530F256::debug_if_config() const { return soc_->debug->config(); }

unsigned Cc2530F256::debug_if_input_length(std::uint8_t command, std::uint8_t second) {
    return cc2530::Debug::input_length(command, second);
}

unsigned Cc2530F256::debug_if_output_length(std::uint8_t command) {
    return cc2530::Debug::output_length(command);
}

void Cc2530F256::debug_if_submit(const std::vector<std::uint8_t> &command) {
    if (engine_running_)
        throw std::logic_error("debug_if_submit() inside run()");
    soc_->debug->host_submit(command);
}

std::uint64_t Cc2530F256::debug_if_due_cycle() const { return soc_->debug->host_due(); }

std::optional<Cc2530DebugIfResponse> Cc2530F256::debug_if_take_response() {
    std::optional<cc2530::Debug::Response> response = soc_->debug->host_take();
    if (!response)
        return std::nullopt;
    return Cc2530DebugIfResponse{std::move(response->bytes), response->accepted, response->at};
}

Cc2530DebugIfResponse Cc2530F256::debug_if_command(const std::vector<std::uint8_t> &command) {
    constexpr std::uint64_t Timeout = 32000000; // one second of virtual time
    debug_if_submit(command);
    const std::uint64_t start = engine_cycles();
    for (;;) {
        if (std::optional<Cc2530DebugIfResponse> response = debug_if_take_response())
            return std::move(*response);
        const std::uint64_t now = engine_cycles();
        if (now - start > Timeout)
            throw std::runtime_error("CC2530 debug interface: no response to command " +
                                     hex(command.front(), 2) + " within 1 s");
        const Cc2530RunResult result =
            run(SIZE_MAX, std::max(now + 1, soc_->debug->host_due()));
        if (!result.error.empty())
            throw std::runtime_error("CC2530 debug interface: " + result.error);
    }
}

std::uint8_t Cc2530F256::debug_execute(const std::uint8_t *code, unsigned length) {
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    if (engine_running_ || length == 0 || length > 3) {
        request_stop("DEBUG_INSTR outside an instruction boundary");
        return sfr_[SfrAcc];
    }
    // Model-defined: 0xA5 is the one undefined MCS-51 opcode; SWRU191F does
    // not say what the CPU does with it, so it is not executed.
    if (code[0] == 0xa5) {
        ++diagnostics_.debug_instructions;
        return sfr_[SfrAcc];
    }
    // SWRU191F Table 3-1: "the supplied instruction will be executed by the
    // CPU without incrementing the program counter". The model feeds the
    // bytes to the real core by placing them at PC in the CODE view in use,
    // runs one instruction without interrupts and in zero virtual time
    // (model-defined), and restores the view.
    const std::uint16_t start_pc = pc();
    std::uint64_t instructions = 0;
    std::uint64_t cycles = 0;
    check_error(uc_reg_read(engine, UC_MCS51_REG_INSTRUCTIONS, &instructions),
                "read instruction counter");
    check_error(uc_reg_read(engine, UC_MCS51_REG_CYCLES, &cycles), "read cycles");
    const std::uint8_t inhibit = engine_inhibit() ? 1 : 0;
    const bool xmap = (sfr_[SfrMemctr] & MemctrXmap) != 0;
    std::uint8_t *buffer =
        xmap ? xmap_view_.data() : views_.data() + (sfr_[SfrFmap] & 0x07u) * CodeSpace;
    const auto at = [&](unsigned i) { return static_cast<std::uint16_t>(start_pc + i); };
    // The instruction comes from the debug interface, not from memory: a
    // MOVC that reads one of the patched addresses gets the memory byte.
    std::optional<std::uint16_t> movc_target;
    if (code[0] == 0x93) { // MOVC A,@A+DPTR
        const bool dptr1 = (sfr_[SfrDps] & 1u) != 0;
        const std::uint16_t dptr = static_cast<std::uint16_t>(
            sfr_[dptr1 ? SfrDph1 : SfrDph0] << 8 | sfr_[dptr1 ? SfrDpl1 : SfrDpl0]);
        movc_target = static_cast<std::uint16_t>(dptr + sfr_[SfrAcc]);
    } else if (code[0] == 0x83) { // MOVC A,@A+PC
        movc_target = static_cast<std::uint16_t>(start_pc + 1 + sfr_[SfrAcc]);
    }
    std::array<std::uint8_t, 3> original{};
    for (unsigned i = 0; i < length; ++i) {
        original[i] = buffer[at(i)];
        buffer[at(i)] = code[i];
    }
    debug_executing_ = true;
    const std::uint16_t no_vector = 0;
    check_error(uc_reg_write(engine, UC_MCS51_REG_IRQ_VECTOR, &no_vector),
                "withdraw interrupt");
    uc_err error = UC_ERR_OK;
    engine_running_ = true;
    if (code_hook_installed_) {
        error = uc_emu_start(engine, start_pc, CodeSpace, 0, 1);
    } else {
        const std::uint64_t no_event = UINT64_MAX;
        const std::uint64_t one = instructions + 1;
        const std::uint32_t no_break = NoBreak;
        check_error(uc_reg_write(engine, UC_MCS51_REG_EVENT_CYCLES, &no_event), "set cycle limit");
        check_error(uc_reg_write(engine, UC_MCS51_REG_INSTRUCTION_LIMIT, &one),
                    "set instruction limit");
        check_error(uc_reg_write(engine, UC_MCS51_REG_BREAK_PC, &no_break), "set break address");
        error = uc_emu_start(engine, start_pc, CodeSpace, 0, 0);
    }
    engine_running_ = false;
    for (unsigned i = 0; i < length; ++i) {
        const std::uint16_t address = at(i);
        // Through XMAP the patched bytes are SRAM: keep a byte the
        // instruction itself wrote.
        if (xmap && address >= BankSize && address < BankSize + SramSize &&
            buffer[address] != code[i])
            continue;
        buffer[address] = original[i];
    }
    install_code_view(); // the XMAP view may have copied patched bytes
    if (movc_target) {
        for (unsigned i = 0; i < length; ++i) {
            if (*movc_target != at(i))
                continue;
            const std::uint8_t value = original[i];
            sfr_[SfrAcc] = value;
            unsigned ones = 0;
            for (std::uint8_t bits = value; bits != 0; bits &= static_cast<std::uint8_t>(bits - 1))
                ++ones;
            sfr_[SfrPsw] = static_cast<std::uint8_t>((sfr_[SfrPsw] & 0xfeu) | (ones & 1u));
        }
    }
    // Zero virtual time, not a firmware instruction.
    check_error(uc_reg_write(engine, UC_MCS51_REG_INSTRUCTIONS, &instructions),
                "restore instruction counter");
    check_error(uc_reg_write(engine, UC_MCS51_REG_CYCLES, &cycles), "restore cycles");
    if (code[0] != 0x32) // RETI sets the post-RETI rule itself
        check_error(uc_reg_write(engine, UC_MCS51_REG_INTERRUPT_INHIBIT, &inhibit),
                    "restore post-RETI inhibit");
    // Re-post the request the core had, then re-evaluate it.
    check_error(uc_reg_write(engine, UC_MCS51_REG_IRQ_LEVEL, &posted_level_),
                "restore interrupt level");
    check_error(uc_reg_write(engine, UC_MCS51_REG_IRQ_VECTOR, &posted_vector_),
                "restore interrupt");
    debug_executing_ = false;
    update_interrupts();
    // The PC stays unless the instruction transfers control: an
    // unconditional jump, call or return, or a taken conditional branch.
    std::uint16_t new_pc = 0;
    check_error(uc_reg_read(engine, UC_MCS51_REG_PC, &new_pc), "read PC");
    const std::uint8_t opcode = code[0];
    const bool transfer = opcode == 0x02 || opcode == 0x12 || (opcode & 0x0fu) == 0x01u ||
                          opcode == 0x22 || opcode == 0x32 || opcode == 0x80 || opcode == 0x73;
    if (!transfer && new_pc == at(length))
        check_error(uc_reg_write(engine, UC_MCS51_REG_PC, &start_pc), "restore PC");
    if (error != UC_ERR_OK)
        request_stop(std::string("DEBUG_INSTR ") + hex(opcode, 2) + ": " + uc_strerror(error));
    ++diagnostics_.debug_instructions;
    return sfr_[SfrAcc];
}

namespace {
void check_pad(unsigned port, unsigned pin) {
    if (port > 2 || pin > 7 || (port == 2 && pin > 4))
        throw std::out_of_range("CC2530 pads are P0_0-P0_7, P1_0-P1_7, P2_0-P2_4");
}
void check_usart(unsigned index) {
    if (index > 1)
        throw std::out_of_range("CC2530 USART index is 0 or 1");
}
} // namespace

void Cc2530F256::set_pin_input(unsigned port, unsigned pin, std::optional<bool> level) {
    check_pad(port, pin);
    soc_->io->set_input(port, pin, level);
}

void Cc2530F256::set_pin_analog_mv(unsigned port, unsigned pin,
                                   std::optional<std::uint32_t> millivolts) {
    check_pad(port, pin);
    soc_->io->set_analog_mv(port, pin, millivolts);
}

Cc2530PinDrive Cc2530F256::pin_drive(unsigned port, unsigned pin) const {
    check_pad(port, pin);
    return soc_->io->drive(port, pin);
}

bool Cc2530F256::pin_level(unsigned port, unsigned pin) const {
    check_pad(port, pin);
    return soc_->io->level(port, pin);
}

void Cc2530F256::set_pin_listener(
    std::function<void(unsigned, unsigned, Cc2530PinDrive, std::uint64_t)> listener) {
    soc_->io->set_listener(std::move(listener));
}

void Cc2530F256::set_temperature_celsius(double celsius) {
    soc_->adc->set_temperature_celsius(celsius);
}

void Cc2530F256::set_vdd_mv(std::uint32_t millivolts) { soc_->adc->set_vdd_mv(millivolts); }

bool Cc2530F256::uart_can_receive(unsigned index) const {
    check_usart(index);
    return soc_->usart[index]->can_receive();
}

void Cc2530F256::uart_receive(unsigned index, std::uint8_t byte) {
    check_usart(index);
    soc_->usart[index]->receive(byte);
}

void Cc2530F256::uart_receive_frame(unsigned index, const Cc2530UartFrame &frame) {
    check_usart(index);
    soc_->usart[index]->receive(frame);
}

std::vector<std::uint8_t> Cc2530F256::uart_take_transmitted(unsigned index) {
    check_usart(index);
    return soc_->usart[index]->take_transmitted();
}

void Cc2530F256::uart_capture_frames(unsigned index, bool on) {
    check_usart(index);
    soc_->usart[index]->set_frame_capture(on);
}

std::vector<Cc2530UartFrame> Cc2530F256::uart_take_transmitted_frames(unsigned index) {
    check_usart(index);
    return soc_->usart[index]->take_transmitted_frames();
}

void Cc2530F256::set_spi_device(unsigned index,
                                std::function<std::uint8_t(std::uint8_t)> device) {
    check_usart(index);
    soc_->usart[index]->set_spi_device(std::move(device));
}

void Cc2530F256::set_noise_seed(std::uint64_t seed) {
    soc_->adc->set_noise_seed(seed);
    soc_->radio->set_noise_seed(seed);
}

void Cc2530F256::attach_air(mcuemu::VirtualAir &air, std::string node,
                            std::uint64_t ns_offset) {
    soc_->radio->attach_air(air, std::move(node), ns_offset);
}

// ---- flash fault model (docs/cc2530.md "Flash fault injection") ----

void Cc2530F256::set_flash_fault_config(const Cc2530FlashFaultConfig &config) {
    soc_->flash->configure(config);
}

const Cc2530FlashFaultConfig &Cc2530F256::flash_fault_config() const {
    return soc_->flash->config();
}

void Cc2530F256::power_cut() {
    // Model-defined power cut: the supply drops and returns at once. The
    // flash operation in flight gets its interrupted outcome, SRAM loses its
    // content (power_on_sram_fill), flash, the info page and virtual time
    // are kept, and the chip comes out of a power-on reset (SLEEPSTA.RST =
    // 00, "Power-on reset and brownout detection", SWRU191F 4.3).
    soc_->flash->interrupt_operation(Cc2530FlashCut::PowerCut, Cc2530ResetCause::PowerOn);
    power_on_sram_fill();
    ++diagnostics_.power_cuts;
    diagnostics_.last_power_cut_cycle = engine_cycles();
    reset_core(Cc2530ResetCause::PowerOn);
}

void Cc2530F256::schedule_power_cut(std::uint64_t at_cycle) {
    soc_->flash->schedule_power_cut(at_cycle);
}

void Cc2530F256::add_flash_trigger(const Cc2530FlashTrigger &trigger) {
    soc_->flash->add_trigger(trigger);
}

std::size_t Cc2530F256::pending_power_faults() const { return soc_->flash->pending_faults(); }

const std::vector<Cc2530FlashLogEntry> &Cc2530F256::flash_log() const {
    return soc_->flash->log();
}

std::vector<Cc2530FlashLogEntry> Cc2530F256::take_flash_log() { return soc_->flash->take_log(); }

void Cc2530F256::restore_flash(const std::vector<std::uint8_t> &image) {
    if (image.size() != FlashSize)
        throw std::invalid_argument("flash image must be exactly 256 KiB");
    std::vector<std::string> warnings = std::move(diagnostics_.load_warnings);
    diagnostics_ = {};
    diagnostics_.load_warnings = std::move(warnings);
    soc_->flash->flash_replaced();
    flash_ = image;
    rebuild_views();
    power_on_reset();
}

void Cc2530F256::request_power_fault(Cc2530FaultAction action) {
    if (action == Cc2530FaultAction::PowerCut) {
        // Overrides any other pending reset: the power cut includes one.
        pending_reset_ = Cc2530ResetCause::PowerOn;
        pending_power_cut_ = true;
        uc_emu_stop(reinterpret_cast<uc_engine *>(uc_));
    } else {
        bus_->request_reset(Cc2530ResetCause::External);
    }
}

void Cc2530F256::power_on_sram_fill() {
    const Cc2530FlashFaultConfig &config = soc_->flash->config();
    if (config.sram_fill) {
        std::fill_n(sram_, SramSize, *config.sram_fill);
        return;
    }
    // Model-defined seeded content; the stream changes with every power-on
    // reset since the load so successive power cuts differ.
    constexpr std::uint64_t SramStream = 0x5352414d'00000000ull;
    const std::uint64_t stream =
        SramStream + diagnostics_.resets_by_cause[static_cast<unsigned>(Cc2530ResetCause::PowerOn)];
    for (std::uint32_t i = 0; i < SramSize; i += 8) {
        const std::uint64_t value = cc2530::fault_draw(config.seed, stream, i / 8);
        for (unsigned j = 0; j < 8 && i + j < SramSize; ++j)
            sram_[i + j] = static_cast<std::uint8_t>(value >> (8 * j));
    }
}

std::uint16_t Cc2530F256::pc() const {
    std::uint16_t value = 0;
    check_error(uc_reg_read(reinterpret_cast<uc_engine *>(uc_), UC_MCS51_REG_PC, &value),
                "read PC");
    return value;
}

std::uint8_t Cc2530F256::read_idata(std::uint8_t address) const { return idata_[address]; }

std::uint8_t Cc2530F256::peek_xdata(std::uint16_t address) const {
    return xdata_value(address);
}

std::optional<std::uint32_t> Cc2530F256::code_physical(std::uint16_t logical) const {
    if (logical < 0x8000)
        return logical;
    if ((sfr_[SfrMemctr] & MemctrXmap) && logical < 0x8000 + SramSize)
        return std::nullopt;
    return (sfr_[SfrFmap] & 0x07u) * BankSize + (logical - 0x8000u);
}

std::uint8_t Cc2530F256::read_code(std::uint16_t logical) const {
    if (const auto physical = code_physical(logical))
        return flash_[*physical];
    return sram_[logical - 0x8000u];
}

std::uint8_t Cc2530F256::read_flash(std::uint32_t physical) const {
    if (physical >= FlashSize)
        throw std::out_of_range("flash address beyond 256 KiB");
    return flash_[physical];
}

Cc2530Registers Cc2530F256::registers() const {
    auto *engine = reinterpret_cast<uc_engine *>(uc_);
    Cc2530Registers regs;
    check_error(uc_reg_read(engine, UC_MCS51_REG_PC, &regs.pc), "read PC");
    regs.a = sfr_[0xe0];
    regs.b = sfr_[0xf0];
    regs.psw = sfr_[0xd0];
    regs.sp = sfr_[SfrSp];
    regs.dpl = sfr_[0x82];
    regs.dph = sfr_[0x83];
    const unsigned bank = (regs.psw >> 3) & 3u;
    for (unsigned i = 0; i < 8; ++i)
        regs.r[i] = idata_[bank * 8 + i];
    regs.fmap = sfr_[SfrFmap];
    regs.memctr = sfr_[SfrMemctr];
    regs.mpage = sfr_[SfrMpage];
    check_error(uc_reg_read(engine, UC_MCS51_REG_CYCLES, &regs.cycles), "read cycles");
    check_error(uc_reg_read(engine, UC_MCS51_REG_INSTRUCTIONS, &regs.instructions),
                "read instruction counter");
    regs.dpl1 = sfr_[SfrDpl1];
    regs.dph1 = sfr_[SfrDph1];
    regs.dps = sfr_[SfrDps];
    regs.interrupt_levels = engine_levels();
    regs.interrupt_inhibit = engine_inhibit();
    regs.posted_vector = posted_vector_;
    regs.posted_level = posted_level_;
    regs.posted_source = posted_source_;
    return regs;
}

} // namespace mcs51emu
