#include "../test_support.hpp"

#include <iostream>

namespace mcuemu_tests {
void test_efr32_m33_exceptions();
void test_efr32_application_slot_boot();
void test_efr32_clock_and_hfxo();
void test_efr32_gpio_aliases_and_exti();
void test_efr32_msc_erase_and_program();
void test_efr32_radio_frame_exchange();
void test_efr32_radio_state_and_rssi();
void test_efr32_rtcc_counts_and_compares();
void test_efr32_semailbox_answers_commands();
void test_efr32_rtcc_keeps_tick_fraction();
void test_efr32_rail_registers();
void test_efr32_radioaes();
void test_efr32_rail_transmit();
void test_efr32_rail_receive_and_ack();
void test_efr32_rtcc_cc0_running_and_overflow();
void test_efr32_timer_counts();
void test_efr32_rsm_radio();
} // namespace mcuemu_tests

#define RUN(test)                                                                \
    do {                                                                         \
        std::cerr << "[ run ] " #test "\n" << std::flush;                        \
        test();                                                                  \
    } while (0)

int main() {
    using namespace mcuemu_tests;
    RUN(test_efr32_m33_exceptions);
    RUN(test_efr32_application_slot_boot);
    RUN(test_efr32_clock_and_hfxo);
    RUN(test_efr32_gpio_aliases_and_exti);
    RUN(test_efr32_msc_erase_and_program);
    RUN(test_efr32_radio_frame_exchange);
    RUN(test_efr32_radio_state_and_rssi);
    RUN(test_efr32_rtcc_counts_and_compares);
    RUN(test_efr32_semailbox_answers_commands);
    RUN(test_efr32_rtcc_keeps_tick_fraction);
    RUN(test_efr32_rail_registers);
    RUN(test_efr32_radioaes);
    RUN(test_efr32_rail_transmit);
    RUN(test_efr32_rail_receive_and_ack);
    RUN(test_efr32_rtcc_cc0_running_and_overflow);
    RUN(test_efr32_timer_counts);
    RUN(test_efr32_rsm_radio);
    if (failures != 0) {
        std::cerr << failures << " check(s) failed\n";
        return 1;
    }
    return 0;
}
