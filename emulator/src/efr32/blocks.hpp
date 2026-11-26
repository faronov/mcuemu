#pragma once

#include "efr32emu/efr32mg21.hpp"

#include <array>
#include <deque>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace efr32emu {

constexpr std::uint64_t Never = std::numeric_limits<std::uint64_t>::max();

// One peripheral block. `reg` is the offset inside the block (0..0xFFF); the
// Series 2 set/clear/toggle aliases are resolved by the chip before the call.
class Block {
public:
    explicit Block(Efr32mg21 &chip) : chip_(chip) {}
    virtual ~Block() = default;

    virtual std::uint32_t read(std::uint32_t reg) = 0;
    virtual void write(std::uint32_t reg, std::uint32_t value) = 0;
    // The value an alias operation starts from; reading must not have side effects.
    virtual std::uint32_t peek(std::uint32_t reg) { return read(reg); }
    // Advances time-driven state to `now` (core cycles); returns the next cycle
    // at which the block needs attention.
    virtual std::uint64_t service(std::uint64_t now) {
        (void)now;
        return Never;
    }
    virtual void reset() {}

protected:
    std::uint64_t now() const;
    std::uint64_t clock_hz() const;
    void set_irq_level(unsigned irq, bool level);
    Efr32mg21 &chip() noexcept { return chip_; }

    Efr32mg21 &chip_;
};

// Registers that read back what was written, for blocks the model does not
// implement. Firmware that waits for hardware behind such a block stops at the
// wait, which `--trace-mmio` makes visible.
class RegBlock : public Block {
public:
    using Block::Block;
    std::uint32_t read(std::uint32_t reg) override { return regs_[(reg >> 2) & 0x3ff]; }
    void write(std::uint32_t reg, std::uint32_t value) override { regs_[(reg >> 2) & 0x3ff] = value; }
    void reset() override { regs_.fill(0); }

protected:
    std::array<std::uint32_t, 1024> regs_{};
};

// EMU (0x40004000): only RSTCAUSE (+0x94) is modelled; the rest reads back what was written. The
// register is INFERRED from the Silicon Labs Zigbee NCP's start-up (halInternalClassifyReset reads
// EMU+0x94 and treats bit 0 as a power-on reset, bit 6 as a system-request reset: the image's table for the lowest set bit maps bit 6 to the
// "software reset, look at the reset marker in RAM" case). Without it
// the NCP reports "unknown reset" and sends an ASH ERROR frame instead of RSTACK.
class Emu final : public RegBlock {
public:
    static constexpr std::uint32_t Rstcause = 0x094;
    static constexpr std::uint32_t PowerOnReset = 1u << 0, SystemRequestReset = 1u << 6;
    // TEMP (+0x088): the die temperature in quarter kelvin (bits 10:0, GSDK EMU_TemperatureGet). INFERRED
    // constant: 1193 is 25.1 C; the model has no thermal state.
    static constexpr std::uint32_t Temp = 0x088, RoomTemperature = 1193;
    explicit Emu(Efr32mg21 &chip) : RegBlock(chip) {
        regs_[Rstcause >> 2] = PowerOnReset;
        regs_[Temp >> 2] = RoomTemperature;
    }
    // A reset the core asked for (SYSRESETREQ) is a system-request reset; any other reset is a
    // power-up.
    void reset() override;
};

// LDMA (0x40040000), as much as the Silicon Labs NCP's UART receive needs. INFERRED from that
// image's register accesses and the Series 2 layout: CHEN/CHDIS/CHSTATUS/CHDONE at +0x24..+0x34,
// LINKLOAD +0x48, IF/IEN +0x50/+0x54, then per channel (stride 0x20) CFG +0x5C, LOOP +0x60,
// CTRL +0x64, SRC +0x68, DST +0x6C, LINK +0x70; a descriptor is {CTRL, SRC, DST, LINK} in RAM.
// Only transfers from the USART0 receive data register run: each received byte goes to DST
// (byte size), XFERCNT (CTRL bits 14:4, count - 1) counts down, and the end sets CHDONE and, with
// DONEIEN (CTRL bit 20), IF and IRQ 21. Other requests (peripheral request selection, memory
// to memory, the TX side) are not modelled.
class Ldma final : public RegBlock {
public:
    static constexpr unsigned Irq = 21;
    static constexpr unsigned Channels = 8;
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    void reset() override;
    // A byte received on USART0; true when a channel took it.
    bool usart0_rx(std::uint8_t byte);

private:
    std::uint32_t channel_reg(unsigned ch, std::uint32_t off) const { return regs_[(0x5c + ch * 0x20 + off) >> 2]; }
    void set_channel_reg(unsigned ch, std::uint32_t off, std::uint32_t v) { regs_[(0x5c + ch * 0x20 + off) >> 2] = v; }
    void load_descriptor(unsigned ch, std::uint32_t address);
    void transfer_done(unsigned ch);
    std::uint32_t desc_addr_[Channels]{};
    void update_irq();
    std::uint32_t enabled_{0};
};

// CMU (0x40008000). Only the system clock selection is observable.
class Cmu final : public RegBlock {
public:
    static constexpr std::uint32_t Sysclkctrl = 0x070;
    using RegBlock::RegBlock;
    void reset() override;
    // SYSCLKCTRL.CLKSEL: 1 FSRCO, 2 HFRCODPLL, 3 HFXO, 4 CLKIN0
    std::uint32_t clock_select() const { return regs_[Sysclkctrl >> 2] & 7u; }
};

// HFXO0 (0x4000C000). STATUS follows CTRL and the system clock request.
class Hfxo final : public RegBlock {
public:
    Hfxo(Efr32mg21 &chip, Cmu &cmu) : RegBlock(chip), cmu_(cmu) {}
    std::uint32_t read(std::uint32_t reg) override;

private:
    Cmu &cmu_;
};

// DPLL0 (0x4001C000): lock follows the enable bit at once. No frequency model;
// CMU_DPLLLock only waits for IF.LOCK after EN_SET.
class Dpll final : public RegBlock {
public:
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
};

// RTCC (0x48000000 secure, 0x58000000 non-secure alias). The Zephyr kernel tick
// runs on the sleeptimer, which runs on this counter, so it has to count.
// Counter: 32-bit, clocked at 32768 Hz (inferred: the LFXO/LFRCO clock is not
// modelled) divided by 2^CFG.CNTPRESC, runs while EN and CMD.START are set;
// STATUS.RUNNING (bit 0) reads set meanwhile. Compare channel n (0..2, CTRL at
// +0x30 + 12n, OCVALUE 4 bytes later) in output-compare mode sets IF.CCn (bit
// 2 + n) when the counter reaches OCVALUE + 1 (inferred from
// sl_sleeptimer_hal_rtcc.c, which programs OCVALUE = target - 1 and says the
// match fires one tick later). The counter wrapping to 0 sets IF.OF (bit 0).
class Rtcc final : public RegBlock {
public:
    static constexpr unsigned Irq = 10; // RTCC_IRQn
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;

private:
    bool counting() const;
    std::uint64_t ticks(std::uint64_t at) const;
    std::uint64_t cycle_for(std::uint64_t tick) const;
    void freeze();
    void arm();
    void update_irq();

    static constexpr unsigned Channels = 3;
    // The counter is ticks_at_ at core cycle cycle_at_ while it runs; armed_tick_[n]
    // is channel n's next compare match (Never when the channel is not armed) and
    // wrap_tick_ the next overflow.
    std::uint64_t ticks_at_{0}, cycle_at_{0}, wrap_tick_{Never};
    std::array<std::uint64_t, Channels> armed_tick_{Never, Never, Never};
    bool started_{false};
};

// TIMER0..3 (0x40048000 + 0x4000 n): the counter only, as the zigbee-rs HAL uses TIMER0 for its
// busy-wait clock (efr32s2-hal delay.rs). CMD.START (bit 0) / STOP (bit 1) run and stop it while EN
// (+0x30) bit 0 is set, STATUS.RUNNING (bit 0) follows, SYNCBUSY reads clear, and CNT (+0x24) counts
// core cycles divided by CFG.PRESC + 1 (bits 27:18) up to TOP (+0x1C), then wraps to 0. TIMER0 is
// 32-bit, TIMER1..3 16-bit (efr32mg21a010f1024im32.h TIMERn_CNTWIDTH), and TOP resets to 0xFFFF on
// every timer (_TIMER_TOP_RESETVALUE), so a 32-bit count needs TOP written. INFERRED: the counter
// keeps counting while the core sleeps (the model has no EM2 clock gating). Compare/capture channels,
// overflow flags and interrupts are not modelled.
class Timer final : public RegBlock {
public:
    Timer(Efr32mg21 &chip, std::uint32_t mask) : RegBlock(chip), mask_(mask) {}
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    void reset() override;

private:
    std::uint32_t count(std::uint64_t at) const;
    void freeze();

    std::uint32_t mask_;
    std::uint32_t count_at_{0};
    std::uint64_t cycle_at_{0};
    bool running_{false};
};

// USART0 (0x40058000), the console. Transmission is instantaneous: TXC, TXBL and
// TXIDLE read as set, and each byte written to TXDATA is appended to the chip's
// UART0 output. Everything else reads back what was written.
class Usart final : public RegBlock {
public:
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
};

// IADC0 (0x4A004000): a conversion completes the moment it is started. CMD.SINGLESTART (bit 0) or SCANSTART
// (bit 3) puts one result in the matching FIFO (STATUS.SINGLEFIFODV bit 8 / SCANFIFODV bit 9, IF bits 9 / 8 and
// 0 / 1); reading SINGLEFIFODATA (+0x74) or SCANFIFODATA (+0x84) takes it. The result is a fixed mid-scale
// value: no analog input is modelled. Offsets are from the SDK's efr32mg21_iadc.h; the instant completion is
// INFERRED from a Tuya thermometer image that polls STATUS & 0x140 after SINGLESTART.
class Iadc final : public RegBlock {
public:
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    void reset() override {
        RegBlock::reset();
        single_dv_ = scan_dv_ = false;
    }
    std::uint32_t peek(std::uint32_t reg) override;

private:
    bool single_dv_{false}, scan_dv_{false};
};

// SEMAILBOX host (0x4C000000 secure, 0x5C000000 non-secure): the mailbox to the
// Secure Element that se_manager uses for its commands. The SE itself is not
// modelled; the mailbox answers every command with SUCCESS and no output data.
// Inferred from sli_se_manager_mailbox.c: the host writes TX_HEADER (bytes) and
// then the command words to FIFO; the response is ready (RX_STATUS.RXINT) once
// the words the header announces have arrived, and RX_HEADER holds the response.
class SeMailbox final : public RegBlock {
public:
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    void reset() override;

private:
    // Runs the command in fifo_: AES-ECB encryption (the stack builds its CCM* and key hashing on it).
    void execute();
    unsigned words_{0};
    std::vector<std::uint32_t> fifo_;
};

// Radio blocks RAIL drives directly, outside the zigbee-rs register map: the
// BUFC (0x44004000 secure, 0x54004000 non-secure), the RAC (0xA8020000 / 0xB8020000)
// and the sequencer and FRC RAM (0xA0000000 / 0xB0000000). RAIL polls several
// words here. Each word is plain storage unless a hook below says otherwise.
// Every hook is an inference from the code that polls the word (see
// docs/efr32mg21.md, "RAIL registers"), not a documented behaviour.
class RailRegs final {
public:
    std::uint32_t read(std::uint32_t address);
    void write(std::uint32_t address, std::uint32_t value, unsigned size = 4);
    void reset() { words_.clear(); }
    // Direct access for the protimer model: plain stored value, no aliasing.
    std::uint32_t peek(std::uint32_t address) const;
    // The stored value of the register `address` (set/clear/toggle alias addresses fold onto it), without the
    // RAIL stand-in behaviour of read().
    std::uint32_t stored(std::uint32_t address) const;
    void set_bits(std::uint32_t address, std::uint32_t mask);
    void clear_bits(std::uint32_t address, std::uint32_t mask);
    // True once after a write starts a transmit (RAC CMD.TXEN); the chip then
    // schedules the end of the frame.
    bool take_tx_start();
    // RX is requested while RAC RXENSRCEN bit 1 is set (RAIL sets it through the
    // SET alias before it starts a receive, and clears all of bits 0..7 to stop).
    bool rx_enabled() const;
    // RXFRAME (RAC STATUS.STATE 3) from a frame's delivery until the firmware
    // acknowledges it (BUFC IF bit 18 cleared). RAIL's helper reads this state to
    // decide the frame is being received (sets its gate flag B).
    void set_rx_frame(bool on) { rx_frame_ = on; }
    // A frame is being received (a data request event is pending): RAC reports RXFRAME regardless of
    // the BUF2 acknowledgements of earlier frames.
    void hold_rx_frame(bool on) { rx_frame_hold_ = on; }
    // RXSEARCH (2) once the receiver has warmed up after the RX request; before that RXWARM (1).
    void set_rx_ready(bool on) { rx_ready_ = on; }
    bool rx_frame() const { return rx_frame_; }

private:
    std::unordered_map<std::uint32_t, std::uint32_t> words_;
    bool tx_started_{false};
    bool rx_frame_{false};
    bool rx_frame_hold_{false};
    bool rx_ready_{false};
};

// MSC (0x40030000): page erase and word programming by the register protocol
// the zigbee-rs flash driver uses.
class Msc final : public RegBlock {
public:
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    void reset() override;

private:
    bool unlocked_{false};
    bool invalid_address_{false};
    std::uint32_t address_{0};
};

// GPIO (0x4003C000): four ports, EXTI channels 0..7 (GPIO_EVEN/GPIO_ODD).
class Gpio final : public Block {
public:
    explicit Gpio(Efr32mg21 &chip) : Block(chip) {}
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    void reset() override;

    PadDrive drive(unsigned pad) const;
    bool level(unsigned pad) const;
    void set_external(unsigned pad, bool high);
    // I2C0 routing (GPIO I2CROUTE[0], from the SDK header: ROUTEEN +0x498, SCLROUTE +0x49C, SDAROUTE +0x4A0; port in bits 1:0,
    // pin in bits 19:16): the pad numbers, or -1 when the line is not enabled.
    int i2c_pad(bool scl) const;
    // The I2C master pulls its routed pad low (open drain); otherwise the pad's own mode and DOUT apply.
    void set_i2c_low(bool scl, bool low);

private:
    static constexpr unsigned PortCount = 4;
    unsigned mode(unsigned pad) const { return (modes_[pad / 16][(pad % 16) / 8] >> (((pad % 16) % 8) * 4)) & 0xfu; }
    bool dout(unsigned pad) const { return (dout_[pad / 16] >> (pad % 16)) & 1u; }
    void changed(unsigned pad);
    void refresh_exti(unsigned pad, bool before, bool after);
    void update_irq();

    std::array<std::array<std::uint32_t, 2>, PortCount> modes_{};
    std::array<std::uint32_t, PortCount> dout_{};
    std::array<std::uint32_t, PortCount> ctrl_{};
    std::uint32_t extipsel_{0}, extipinsel_{0}, extirise_{0}, extifall_{0}, if_{0}, ien_{0};
    std::array<bool, 64> external_valid_{};
    std::array<bool, 64> external_{};
    std::array<bool, 64> last_{};
    std::array<std::uint32_t, 3> i2c_route_{};
    bool i2c_low_[2]{false, false}; // SCL, SDA
};

// I2C0 (0x4A010000), master only. CMD START/STOP/ACK/NACK, TXDATA, RXDATA and the IF flags follow emlib's I2C_Transfer: the
// address byte is the first TXDATA, a received byte waits for CMD ACK or NACK (or AUTOACK on the RXDATA read), IF.ACK / IF.NACK
// report each byte, IF.MSTOP the stop. The bit timing comes from CLKDIV at the 38.4 MHz clock and the lines are driven
// through the GPIO routing as open drain, so a Lua I2C slave on the nets sees a real bus. Offsets and flags are from the SDK's
// efr32mg21_i2c.h; the behaviour is INFERRED from emlib (no firmware of this repository uses I2C0 yet).
class I2c final : public RegBlock {
public:
    using RegBlock::RegBlock;
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    std::uint32_t peek(std::uint32_t reg) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;

private:
    enum class Kind { Drive, SampleAck, SampleBit, WriteDone, ReadDone, Stopped, Started, RepeatedStart };
    struct Step {
        std::uint64_t cycle;
        Kind kind;
        bool sda_low, scl_low;
    };
    void progress();
    void queue_start();
    void queue_write_byte(std::uint8_t value);
    void queue_read_byte();
    void queue_ack_bit(bool nack);
    void queue_stop();
    void apply(bool sda_low, bool scl_low);
    void update_irq();
    std::uint64_t quarter() const;

    std::deque<Step> steps_;
    std::uint64_t at_{0}; // the cycle after the last queued step
    std::uint32_t if_{0}, ien_{0};
    bool enabled_{false}, master_{false}, transmitter_{false}, nacked_{false}, bus_busy_{false}, hold_{false};
    bool start_pending_{false}, stop_pending_{false}, addr_phase_{false}, wait_ack_cmd_{false};
    bool scl_low_{false}, sda_low_{false};
    bool tx_full_{false}, rx_full_{false}, last_ack_{false};
    std::uint8_t txdata_{0}, rxdata_{0}, shift_{0}, sent_{0};
};

// The radio blocks (FRC, BUFC, SYNTH, RAC, MODEM, AGC) at 0x40090000. See
// radio.cpp: the register interface is the one zigbee-rs's driver uses, not
// the silicon's.
class Radio final : public Block {
public:
    explicit Radio(Efr32mg21 &chip) : Block(chip) {}
    std::uint32_t read(std::uint32_t reg) override;
    void write(std::uint32_t reg, std::uint32_t value) override;
    std::uint32_t peek(std::uint32_t reg) override;
    std::uint64_t service(std::uint64_t now) override;
    void reset() override;
    void connect(mcuemu::VirtualAir &air, const std::string &node);
    // Sends a TX FIFO image laid out as the RAIL path writes it (PHR, then the
    // PSDU) on `channel`. Returns the frame's time on air in ns.
    std::uint64_t transmit_fifo(const std::vector<std::uint8_t> &fifo, std::uint8_t channel);
    // Frames that ended on `channel` since the previous call, each as the RAIL RX
    // buffer holds it: PHR (payload plus the two FCS bytes), then the payload.
    std::vector<std::vector<std::uint8_t>> receive_fifo(std::uint8_t channel, std::uint64_t now_ns);
    // Frames that ended on `channel` since the previous call and whose start-of-frame delimiter had not
    // passed at `listening_since_ns` (the receiver was searching when they began), for RsmRadio.
    std::vector<mcuemu::RadioFrame> receive_since(std::uint8_t channel, std::uint64_t now_ns,
                                                  std::uint64_t listening_since_ns);
    // The strongest frame of another node on `channel` at `at_ns` (dBm, -110 for a quiet channel).
    std::int8_t energy_dbm(std::uint64_t at_ns, std::uint8_t channel) const;

private:
    enum class State : std::uint32_t { Off = 0, Idle = 1, Rx = 2, Tx = 3 };

    void command(std::uint32_t value);
    void begin_transmit();
    void poll_air(std::uint64_t ns);
    std::uint8_t channel() const { return static_cast<std::uint8_t>(11 + (std::min<std::uint32_t>(chno_, 15))); }
    std::int8_t energy_dbm(std::uint64_t at_ns) const;
    void update_irq();
    std::uint64_t reschedule() const;
    std::uint64_t to_ns(std::uint64_t cycles) const;

    mcuemu::VirtualAir *air_{nullptr};
    std::string node_;
    std::array<std::uint32_t, 0x6000 / 4> regs_{};
    State state_{State::Idle};
    std::uint32_t rac_if_{0}, rac_ien_{0}, frc_if_{0}, chno_{0};
    std::vector<std::uint8_t> tx_fifo_;
    std::vector<std::uint8_t> rx_fifo_;
    std::size_t rx_read_{0};
    bool rx_pending_{false};
    std::int8_t rx_rssi_{-127};
    std::uint64_t tx_end_ns_{0};
    bool tx_active_{false};
    std::uint64_t rx_start_ns_{0}, last_poll_ns_{0}, rail_poll_ns_{0}, rsm_poll_ns_{0};
};

// The radio as the zigbee-rs `efr32s2-hal` driver runs it (efr32s2-hal/src/radio, xg21.rs): no RAIL
// and no sequencer microcode. The driver clock-gates nothing on xG21; it loads its own 16-slot compact
// vector table into sequencer RAM where every slot is END except the warm states, which it parks on a
// self-JUMP while it calibrates the synthesizer, then releases by writing END over the jump. The model
// takes over the radio registers once the firmware has loaded such a table (RAC.SEQCTRL.COMPACT with
// RAC.VECTADDR at sequencer RAM and only END/self-JUMP slots); RAIL images never do, so they keep the
// RAIL path in efr32mg21.cpp. What it implements, on the registers the driver uses (secure view):
//
//   RAC.STATUS.STATE (+0x0C, bits 27:24): OFF 0, RXWARM 1, RXSEARCH 2, RXOVERFLOW 6, TXWARM 8, TX 9,
//     TXPD 10. RXENSRCEN (+0x08) != 0 starts RX, 0 stops it; CMD (+0x10) TXEN (bit 0) starts TX,
//     CLEARRXOVERFLOW (bit 6) leaves RXOVERFLOW. A state whose vector slot is a self-JUMP is held there.
//   TX: the frame is BUF0's bytes from READOFFSET (PHR, then the PSDU without FCS); it goes on air when
//     TXWARM is released and FRC.IF.TXDONE (bit 0) is set when it ends, then the RSM enters TXPD.
//   RX: each frame heard while searching goes into BUF1 as [PHR][PSDU - FCS][RSSI] at WRITEOFFSET
//     (wrapping at 64 << CTRL.SIZE bytes; the offsets count modulo 8 KiB), STATUS is WRITEOFFSET -
//     READOFFSET, and FRC.IF.RXDONE (bit 4) is set; a frame that does not fit sets FRC.IF.RXOF (bit 8)
//     and RXOVERFLOW. BUFn.CMD.CLEAR empties a buffer. FRC (IRQ 34) follows FRC.IF & FRC.IEN.
//   AGC.RSSI (+0x10): the channel energy in quarter dB in bits 15:6 while searching, 0x8000 (invalid)
//     otherwise.
//   SYNTH: CMD (+0x0C) START/STOP and ENABLEIF/DISABLEIF; LOCNTCTRL (+0x88) RUN sets IF.LOCNTDONE
//     (bit 9) and LOCNTSTATUS (+0x8C) holds the LO count over 2^NUMCYCLE reference periods. With the
//     loop started the LO is FREQ (+0x34) plus or minus IFFREQ (+0x38), as the driver computes its
//     target; open loop it falls linearly with VCOTUNING (+0x4C, bits 10:0) from 2.52 to 2.36 GHz.
//   RAC.RFSTATUS.MODRAMPUPDONE (+0x98 bit 0) follows RAC.PAENCTRL.PARAMP (+0xA4 bit 8).
//
// INFERRED, not the silicon: the open-loop VCO curve, a loop that always locks, instant LO counts,
// warm-up and RX-to-search times of zero (the driver's own waits pace it), the RSSI scale (61 dB above
// the antenna level, the -50 dB chip term plus the -11 dB board offset the driver subtracts), and no
// RAC.IF.STATECHANGE interrupt (the driver's RAC_RSM handler only acknowledges it). Frames are not
// address-filtered (the driver filters and acknowledges in software) and RXFRAME is not reported.
class RsmRadio final {
public:
    RsmRadio(Efr32mg21 &chip, RailRegs &regs, Radio &air) : chip_(chip), regs_(regs), air_(air) {}
    bool active() const noexcept { return active_; }
    void reset();
    // After RailRegs stored a firmware write to `address` (secure view, alias bits kept).
    void written(std::uint32_t address, std::uint32_t value);
    // The value of a register with behaviour; false for plain registers.
    bool read(std::uint32_t address, std::uint32_t &value);
    // Advances to `now` (core cycles); returns the next cycle that needs a service.
    std::uint64_t service(std::uint64_t now);

private:
    enum State : std::uint32_t { Off = 0, RxWarm = 1, RxSearch = 2, RxOverflow = 6, TxWarm = 8, Tx = 9, TxPd = 10 };

    void check_vector_table();
    bool held(State state) const;
    void enter(State state, std::uint64_t now);
    void advance(std::uint64_t now);
    void poll(std::uint64_t now);
    void deliver(const mcuemu::RadioFrame &frame);
    void start_transmit(std::uint64_t now);
    std::uint8_t channel() const;
    std::uint32_t lo_count() const;
    void clear_buffer(std::uint32_t buffer);
    void update_irq();
    std::uint64_t now() const;

    Efr32mg21 &chip_;
    RailRegs &regs_;
    Radio &air_;
    bool active_{false};
    State state_{Off};
    std::uint64_t tx_end_{Never}, next_poll_{Never}, listening_since_ns_{0};
    bool synth_running_{false}, synth_if_{false};
    std::uint32_t locnt_{0};
};

} // namespace efr32emu
