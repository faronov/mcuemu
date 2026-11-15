cmake_minimum_required(VERSION 3.20)
project(mcuemu VERSION 0.1.0 LANGUAGES C CXX)

# An unset build type compiles without optimisation, which makes the emulator
# about 5x slower; default to Release for single-config generators.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(MCUEMU_BUILD_TESTS "Build the emulator self-tests" ON)
option(MCUEMU_BUILD_WASM "Build the browser-facing WebAssembly control module" OFF)

if(MCUEMU_BUILD_WASM)
    if(NOT EMSCRIPTEN)
        message(FATAL_ERROR "MCUEMU_BUILD_WASM requires the Emscripten toolchain")
    endif()
    # The native test executables are not browser targets.
    set(MCUEMU_BUILD_TESTS OFF CACHE BOOL "Build the emulator self-tests" FORCE)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fexceptions>)
endif()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/engine/unicorn/CMakeLists.txt")
    message(FATAL_ERROR
        "Unicorn source checkout is missing. Run scripts/bootstrap-unicorn.sh first.")
endif()

# The board wrapper depends on the TC32 architecture implemented in this local
# Unicorn checkout. Keep the engine as a separate Git repository under ./engine.
set(UNICORN_ARCH "tc32;mcs51;arm;riscv" CACHE STRING "Architectures built for device emulators" FORCE)
set(UNICORN_BUILD_TESTS OFF CACHE BOOL "Build Unicorn tests through the wrapper" FORCE)
set(UNICORN_INSTALL OFF CACHE BOOL "Install Unicorn through the wrapper" FORCE)
add_subdirectory(engine/unicorn EXCLUDE_FROM_ALL)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    # <windows.h> (pulled in by WinSock) must not define min/max macros.
    add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
    set(MCUEMU_WARNING_FLAGS /W4)
    # A Tlsr8258 holds ~130 KiB of inline RAM and tests keep several on the
    # stack; the 1 MiB Windows default overflows, other platforms give 8 MiB.
    add_link_options(/STACK:8388608)
else()
    set(MCUEMU_WARNING_FLAGS -Wall -Wextra -Wpedantic)
endif()

add_library(mcuemu-core
    emulator/src/cpu.cpp
    emulator/src/tlsr8258.cpp
    emulator/src/tlsr826x.cpp
    emulator/src/tlsr8258_i2c.cpp
    emulator/src/tlsr8258_adc.cpp
    emulator/src/radio.cpp
    emulator/src/api.cpp
    emulator/src/aes.cpp
)
target_include_directories(mcuemu-core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(mcuemu-core PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(mcuemu-core PUBLIC unicorn)

if(MCUEMU_BUILD_WASM)
    add_executable(mcuemu-wasm emulator/src/wasm_api.cpp)
    target_link_libraries(mcuemu-wasm PRIVATE simcore)
    target_link_options(mcuemu-wasm PRIVATE
        --no-entry
        -fexceptions
        -sMODULARIZE=1
        -sEXPORT_ES6=1
        -sENVIRONMENT=web,worker
        -sALLOW_MEMORY_GROWTH=1
        -sEMULATE_FUNCTION_POINTER_CASTS=1
        -sEXPORTED_RUNTIME_METHODS=ccall,FS
        # TCI helper calls intentionally use cast function pointers. The
        # adapter pass works at O1; Binaryen currently asserts on this module
        # at O3.
        -O1
    )
    set_target_properties(mcuemu-wasm PROPERTIES SUFFIX ".mjs")
elseif(NOT EMSCRIPTEN)
    add_executable(mcuemu emulator/src/main.cpp)
    target_link_libraries(mcuemu PRIVATE mcuemu-core mcs51emu nrf52emu efr32emu esp32emu phy6252emu stm32wb55emu simcore)
endif()

add_library(mcs51emu
    emulator/src/mcs51/ca51f253l3.cpp
    emulator/src/mcs51/cc2530f256.cpp
    emulator/src/mcs51/cc2530_aes.cpp
    emulator/src/mcs51/cc2530_analog.cpp
    emulator/src/mcs51/cc2530_debug.cpp
    emulator/src/mcs51/cc2530_debug_programmer.cpp
    emulator/src/mcs51/cc2530_dma.cpp
    emulator/src/mcs51/cc2530_flash.cpp
    emulator/src/mcs51/cc2530_hit_report.cpp
    emulator/src/mcs51/cc2530_io.cpp
    emulator/src/mcs51/cc2530_mactimer.cpp
    emulator/src/mcs51/cc2530_radio.cpp
    emulator/src/mcs51/cc2530_timers.cpp
    emulator/src/mcs51/cc2530_usart.cpp
    emulator/src/mcs51/run_command.cpp
)
target_include_directories(mcs51emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(mcs51emu PRIVATE ${MCUEMU_WARNING_FLAGS})
# The CC2530 radio joins mcuemu::VirtualAir.
target_link_libraries(mcs51emu PUBLIC unicorn mcuemu-core)

# Generic Cortex-M core on the Unicorn fork's ARM backend (NVIC, SysTick, WFI/WFE).
add_library(cortexm emulator/src/cortexm/core.cpp)
target_include_directories(cortexm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(cortexm PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(cortexm PUBLIC unicorn)

add_library(stm32wb55emu emulator/src/stm32wb55/stm32wb55.cpp emulator/src/stm32wb55/st_mac.cpp)
target_include_directories(stm32wb55emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(stm32wb55emu PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(stm32wb55emu PUBLIC cortexm mcuemu-core)

add_library(phy6252emu emulator/src/phy6252/image.cpp emulator/src/phy6252/phy6252.cpp emulator/src/phy6252/run_command.cpp)
target_include_directories(phy6252emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(phy6252emu PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(phy6252emu PUBLIC cortexm mcuemu-core)

# Generic RV32 hart on the fork's RISC-V backend (traps, host interrupt line, WFI, cycle clock).
add_library(riscvcore emulator/src/riscv/core.cpp)
target_include_directories(riscvcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(riscvcore PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(riscvcore PUBLIC unicorn)

add_library(esp32emu
    emulator/src/esp32/esp32.cpp
    emulator/src/esp32/spi_mem.cpp
    emulator/src/esp32/rom.cpp
    emulator/src/esp32/timg.cpp
    emulator/src/esp32/timers.cpp
    emulator/src/esp32/intc.cpp
    emulator/src/esp32/ieee802154.cpp
    emulator/src/esp32/aes.cpp
    emulator/src/esp32/gpio.cpp
    emulator/src/esp32/elf_symbols.cpp
    emulator/src/esp32/run_command.cpp
)
target_include_directories(esp32emu PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/emulator/src/esp32)
target_include_directories(esp32emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(esp32emu PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(esp32emu PUBLIC riscvcore mcuemu-core)

add_library(bl702emu
    emulator/src/bl702/bl702.cpp
    emulator/src/bl702/rom.cpp
    emulator/src/bl702/peripherals.cpp
    emulator/src/bl702/radio.cpp
    emulator/src/bl702/clic.cpp
)
target_include_directories(bl702emu PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/emulator/src/bl702)
target_include_directories(bl702emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(bl702emu PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(bl702emu PUBLIC riscvcore mcuemu-core)

add_library(nrf52emu
    emulator/src/nrf52/nrf52840.cpp
    emulator/src/nrf52/power_clock.cpp
    emulator/src/nrf52/nvmc.cpp
    emulator/src/nrf52/gpio.cpp
    emulator/src/nrf52/rtc.cpp
    emulator/src/nrf52/timer.cpp
    emulator/src/nrf52/misc.cpp
    emulator/src/nrf52/saadc.cpp
    emulator/src/nrf52/ecb.cpp
    emulator/src/nrf52/radio.cpp
    emulator/src/nrf52/uarte.cpp
    emulator/src/nrf52/twim.cpp
    emulator/src/nrf52/run_command.cpp
)
target_include_directories(nrf52emu PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/emulator/src)
target_include_directories(nrf52emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(nrf52emu PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(nrf52emu PUBLIC cortexm mcuemu-core)

add_library(efr32emu
    emulator/src/efr32/efr32mg21.cpp
    emulator/src/efr32/blocks.cpp
    emulator/src/efr32/radio.cpp
    emulator/src/efr32/radioaes.cpp
    emulator/src/efr32/run_command.cpp
)
target_include_directories(efr32emu PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/emulator/src)
target_include_directories(efr32emu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(efr32emu PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(efr32emu PUBLIC cortexm mcuemu-core)

find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
    include(FetchContent)
    # yaml-cpp 0.8.0 declares cmake_minimum_required(VERSION 3.4), which CMake 4
    # rejects; accept it instead of patching the dependency.
    if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM)
        set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
    endif()
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG 0.8.0)
    FetchContent_MakeAvailable(yaml-cpp)
endif()
if(TARGET yaml-cpp::yaml-cpp)
    set(MCUEMU_YAML_TARGET yaml-cpp::yaml-cpp)
else()
    set(MCUEMU_YAML_TARGET yaml-cpp)
endif()

# Lua 5.4 for scripted parts, built from source so behavior does not depend on
# the system's Lua version. The upstream repository has no CMakeLists.txt.
include(FetchContent)
FetchContent_Declare(lua
    GIT_REPOSITORY https://github.com/lua/lua.git
    GIT_TAG v5.4.8)
FetchContent_MakeAvailable(lua)
file(GLOB MCUEMU_LUA_SOURCES "${lua_SOURCE_DIR}/*.c")
list(FILTER MCUEMU_LUA_SOURCES EXCLUDE REGEX "/(lua|luac|onelua|ltests)\\.c$")
add_library(lua54 STATIC ${MCUEMU_LUA_SOURCES})
target_include_directories(lua54 PUBLIC "${lua_SOURCE_DIR}")
set_target_properties(lua54 PROPERTIES C_STANDARD 99)
if(UNIX)
    target_compile_definitions(lua54 PRIVATE LUA_USE_POSIX)
    target_link_libraries(lua54 PUBLIC m)
endif()
target_compile_options(lua54 PRIVATE -w)

# JSON for the control API.
FetchContent_Declare(json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

# Chip-independent simulation core: scheduler, pins/nets, endpoints, world.
add_library(simcore
    emulator/src/sim/scheduler.cpp
    emulator/src/sim/net.cpp
    emulator/src/sim/endpoint.cpp
    emulator/src/sim/world.cpp
    emulator/src/sim/uart_endpoint.cpp
    emulator/src/sim/rtt_endpoint.cpp
    emulator/src/sim/sink.cpp
    emulator/src/sim/simulation.cpp
    emulator/src/sim/rpc.cpp
    emulator/src/sim/websocket.cpp
    emulator/src/sim/server.cpp
    emulator/src/sim/part.cpp
    emulator/src/sim/parts/ds18b20.cpp
    emulator/src/sim/parts/i2c_slave.cpp
    emulator/src/sim/parts/builtin.cpp
    emulator/src/sim/parts/lua_part.cpp
    emulator/src/sim/config.cpp
    emulator/src/sim/system.cpp
    emulator/src/sim/cli.cpp
    emulator/src/sim/tlsr8258_mcu.cpp
    emulator/src/sim/tlsr826x_mcu.cpp
    emulator/src/sim/ca51f253l3_mcu.cpp
    emulator/src/sim/cc2530f256_mcu.cpp
    emulator/src/sim/nrf52840_mcu.cpp
    emulator/src/sim/efr32mg21_mcu.cpp
    emulator/src/sim/stm32wb55_mcu.cpp
    emulator/src/sim/esp32_mcu.cpp
    emulator/src/sim/bl702_mcu.cpp
    emulator/src/sim/phy6252_mcu.cpp
)
target_include_directories(simcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/emulator/include)
target_compile_options(simcore PRIVATE ${MCUEMU_WARNING_FLAGS})
target_link_libraries(simcore PUBLIC mcuemu-core mcs51emu nrf52emu efr32emu esp32emu bl702emu phy6252emu stm32wb55emu ${MCUEMU_YAML_TARGET} lua54
    nlohmann_json::nlohmann_json)
if(WIN32)
    target_link_libraries(simcore PUBLIC ws2_32)
endif()

if(MCUEMU_BUILD_TESTS)
    enable_testing()
    add_executable(phy6252emu-tests tests/phy6252/test_image.cpp)
    target_link_libraries(phy6252emu-tests PRIVATE phy6252emu cortexm)
    add_test(NAME phy6252emu-tests COMMAND phy6252emu-tests)
    add_executable(mcuemu-tests
        tests/test_main.cpp
        tests/tc32/test_cpu.cpp
        tests/machine/test_tlsr8258.cpp
        tests/machine/test_tlsr826x.cpp
        tests/machine/test_api.cpp
        tests/radio/test_radio.cpp
    )
    target_link_libraries(mcuemu-tests PRIVATE mcuemu-core)
    add_test(NAME mcuemu-tests COMMAND mcuemu-tests)

    add_executable(mcs51emu-tests
        tests/mcs51/test_ca51f253l3.cpp
    )
    target_link_libraries(mcs51emu-tests PRIVATE mcs51emu)
    add_test(NAME mcs51emu-tests COMMAND mcs51emu-tests)

    add_executable(cc2530emu-tests
        tests/mcs51/test_cc2530f256.cpp
    )
    target_link_libraries(cc2530emu-tests PRIVATE mcs51emu)
    add_test(NAME cc2530emu-tests COMMAND cc2530emu-tests)

    add_executable(cc2530emu-r2-tests
        tests/mcs51/test_cc2530f256_r2.cpp
    )
    target_link_libraries(cc2530emu-r2-tests PRIVATE mcs51emu)
    add_test(NAME cc2530emu-r2-tests COMMAND cc2530emu-r2-tests)

    add_executable(cc2530emu-lockstep-tests
        tests/mcs51/test_cc2530f256_lockstep.cpp
    )
    target_link_libraries(cc2530emu-lockstep-tests PRIVATE mcs51emu)
    add_test(NAME cc2530emu-lockstep-tests COMMAND cc2530emu-lockstep-tests)

    # CC2530 peripheral models (white-box access to src/mcs51 internals).
    foreach(suite timers io_analog_usart dma_flash_aes mactimer radio spi uart_errors flash_powercut minor_gaps debug_if)
        add_executable(cc2530emu-${suite}-tests tests/mcs51/test_cc2530_${suite}.cpp)
        target_include_directories(cc2530emu-${suite}-tests PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/emulator/src/mcs51)
        target_compile_options(cc2530emu-${suite}-tests PRIVATE ${MCUEMU_WARNING_FLAGS})
        target_link_libraries(cc2530emu-${suite}-tests PRIVATE mcs51emu)
        add_test(NAME cc2530emu-${suite}-tests COMMAND cc2530emu-${suite}-tests)
    endforeach()

    add_executable(nrf52emu-tests
        tests/nrf52/test_main.cpp
        tests/nrf52/test_cortexm.cpp
        tests/nrf52/test_chip.cpp
        tests/nrf52/test_radio.cpp
    )
    target_include_directories(nrf52emu-tests PRIVATE tests/nrf52)
    target_link_libraries(nrf52emu-tests PRIVATE nrf52emu)
    add_test(NAME nrf52emu-tests COMMAND nrf52emu-tests)

    add_executable(riscvemu-tests
        tests/riscv/test_main.cpp
        tests/riscv/test_riscv.cpp
        tests/riscv/test_core.cpp
    )
    target_include_directories(riscvemu-tests PRIVATE tests/riscv)
    target_link_libraries(riscvemu-tests PRIVATE riscvcore)
    add_test(NAME riscvemu-tests COMMAND riscvemu-tests)

    add_executable(esp32emu-tests
        tests/esp32/test_main.cpp
        tests/esp32/test_esp32.cpp
        tests/esp32/test_mac.cpp
    )
    target_include_directories(esp32emu-tests PRIVATE tests/esp32 emulator/src/esp32)
    target_link_libraries(esp32emu-tests PRIVATE esp32emu)
    add_test(NAME esp32emu-tests COMMAND esp32emu-tests)

    add_executable(bl702emu-tests
        tests/bl702/test_main.cpp
        tests/bl702/test_bl702.cpp
    )
    target_include_directories(bl702emu-tests PRIVATE tests/bl702)
    target_link_libraries(bl702emu-tests PRIVATE bl702emu)
    add_test(NAME bl702emu-tests COMMAND bl702emu-tests)

    add_executable(stm32wb55emu-tests tests/stm32wb55/test_st_mac.cpp)
    target_link_libraries(stm32wb55emu-tests PRIVATE stm32wb55emu)
    add_test(NAME stm32wb55emu-tests COMMAND stm32wb55emu-tests)

    add_executable(efr32emu-tests
        tests/efr32/test_main.cpp
        tests/efr32/test_efr32.cpp
    )
    target_include_directories(efr32emu-tests PRIVATE tests/efr32)
    target_link_libraries(efr32emu-tests PRIVATE efr32emu)
    add_test(NAME efr32emu-tests COMMAND efr32emu-tests)

    add_executable(simcore-tests
        tests/sim/test_sim.cpp
        tests/sim/test_config.cpp
        tests/sim/test_parts.cpp
        tests/sim/test_periph.cpp
        tests/sim/test_lua.cpp
        tests/sim/test_radio_medium.cpp
        tests/sim/test_attach.cpp
        tests/sim/test_nrf52.cpp
        tests/sim/test_efr32.cpp
        tests/sim/test_bl702.cpp
        tests/sim/test_control.cpp
        tests/sim/test_cc2530.cpp
        tests/sim/test_cc2530_uart_frames.cpp
        tests/sim/test_cc2530_flash.cpp
    )
    target_link_libraries(simcore-tests PRIVATE simcore)
    target_compile_definitions(simcore-tests PRIVATE
        MCUEMU_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_test(NAME simcore-tests COMMAND simcore-tests)

    add_test(NAME mcuemu-cli-exit-codes
             COMMAND ${CMAKE_COMMAND} -DMCUEMU=$<TARGET_FILE:mcuemu>
                     -DWORK=${CMAKE_CURRENT_BINARY_DIR}/cli-exit-codes
                     -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/cli/cli_exit_codes.cmake)

    # TLSR8269 smoke firmware built from the Telink SDK (skipped without the SDK/toolchain).
    set(MCUEMU_TELINK_SDK "${CMAKE_CURRENT_SOURCE_DIR}/../telink/telink_zigbee_sdk/tl_zigbee_sdk")
    set(MCUEMU_TC32_CLANG "${CMAKE_CURRENT_SOURCE_DIR}/../telink/llvm-tc32-arm-based/toolchains/tc32-stage2/llvm/bin/clang")
    if((EXISTS "${MCUEMU_TELINK_SDK}" OR DEFINED ENV{TELINK_SDK}) AND
       (EXISTS "${MCUEMU_TC32_CLANG}" OR DEFINED ENV{TC32_BIN}))
        add_test(NAME firmware_tlsr826x_smoke
                 COMMAND ${CMAKE_COMMAND} -DMCUEMU=$<TARGET_FILE:mcuemu>
                         -DSRC=${CMAKE_CURRENT_SOURCE_DIR}
                         -DOUT=${CMAKE_CURRENT_BINARY_DIR}/tlsr826x
                         -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/tlsr826x/smoke.cmake)
        set_tests_properties(firmware_tlsr826x_smoke PROPERTIES SKIP_REGULAR_EXPRESSION "SKIP")
    endif()

    set(MCUEMU_SAMPLE_GW_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/samplegw_tb03f.bin")
    set(MCUEMU_SAMPLE_LIGHT_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/samplelight_tb03f.bin")
    if(EXISTS "${MCUEMU_SAMPLE_GW_FIRMWARE}" AND
       EXISTS "${MCUEMU_SAMPLE_LIGHT_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_zigbee_join
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_zigbee_join.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_SAMPLE_GW_FIRMWARE}"
                        "${MCUEMU_SAMPLE_LIGHT_FIRMWARE}"
                        join)
            add_test(
                NAME firmware_zigbee_button_light
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_zigbee_join.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_SAMPLE_GW_FIRMWARE}"
                        "${MCUEMU_SAMPLE_LIGHT_FIRMWARE}"
                        button)
            set_tests_properties(firmware_zigbee_join firmware_zigbee_button_light
                PROPERTIES
                TIMEOUT 600
                LABELS "firmware;zigbee"
            )
        else()
            message(STATUS
                "Skipping firmware_zigbee_join: Python 3 interpreter not found")
        endif()
    else()
        message(STATUS
            "Skipping firmware_zigbee_join: local sampleGW/sampleLight images not found in firmware/")
    endif()

    set(MCUEMU_PHY6252_ZEPHYR_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/phy6252-zephyr-ed.bin")
    set(MCUEMU_PHY6252_ROM "${CMAKE_CURRENT_SOURCE_DIR}/firmware/phy6222-rom.bin")
    set(MCUEMU_PHY6252_EFUSE "${CMAKE_CURRENT_SOURCE_DIR}/firmware/phy6222-efuse.bin")
    set(MCUEMU_PHY6252_RF_CALIBRATION
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/phy6222-rfcal.bin")
    if(EXISTS "${MCUEMU_PHY6252_ZEPHYR_FIRMWARE}" AND
       EXISTS "${MCUEMU_PHY6252_ROM}" AND
       EXISTS "${MCUEMU_PHY6252_EFUSE}" AND
       EXISTS "${MCUEMU_PHY6252_RF_CALIBRATION}" AND
       EXISTS "${MCUEMU_SAMPLE_GW_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_phy6252_zephyr_join
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_phy6252_zephyr_join.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_SAMPLE_GW_FIRMWARE}"
                        "${MCUEMU_PHY6252_ZEPHYR_FIRMWARE}"
                        "${MCUEMU_PHY6252_ROM}"
                        "${MCUEMU_PHY6252_EFUSE}"
                        "${MCUEMU_PHY6252_RF_CALIBRATION}")
            set_tests_properties(firmware_phy6252_zephyr_join PROPERTIES
                TIMEOUT 900
                LABELS "firmware;zigbee;phy6252"
            )
        else()
            message(STATUS
                "Skipping firmware_phy6252_zephyr_join: Python 3 interpreter not found")
        endif()
    else()
        message(STATUS
            "Skipping firmware_phy6252_zephyr_join: local PB-03F Zephyr, PHY6222 ROM/calibration or sampleGW inputs not found in firmware/")
    endif()

    set(MCUEMU_PHY6252_TS0001_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/phy6252-ts0001.bin")
    if(EXISTS "${MCUEMU_PHY6252_TS0001_FIRMWARE}" AND
       EXISTS "${MCUEMU_PHY6252_EFUSE}" AND
       EXISTS "${MCUEMU_PHY6252_RF_CALIBRATION}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_phy6252_ts0001_boot
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_phy6252_ts0001_boot.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_PHY6252_TS0001_FIRMWARE}"
                        "${MCUEMU_PHY6252_EFUSE}"
                        "${MCUEMU_PHY6252_RF_CALIBRATION}")
            set_tests_properties(firmware_phy6252_ts0001_boot PROPERTIES
                TIMEOUT 900
                LABELS "firmware;phy6252;tuya"
            )
        else()
            message(STATUS
                "Skipping firmware_phy6252_ts0001_boot: Python 3 interpreter not found")
        endif()
    else()
        message(STATUS
            "Skipping firmware_phy6252_ts0001_boot: local TS0001 image and PHY6222 calibration inputs not found in firmware/")
    endif()

    # The TS0001 joins the Telink sampleGW coordinator (channel 15) by itself.
    set(MCUEMU_PHY6252_JOIN_COORDINATOR "${MCUEMU_SAMPLE_GW_FIRMWARE}")
    if(EXISTS "${MCUEMU_PHY6252_TS0001_FIRMWARE}" AND
       EXISTS "${MCUEMU_PHY6252_EFUSE}" AND
       EXISTS "${MCUEMU_PHY6252_RF_CALIBRATION}" AND
       EXISTS "${MCUEMU_PHY6252_JOIN_COORDINATOR}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            set(MCUEMU_PHY6252_JOIN_ROM_ARGS)
            if(EXISTS "${MCUEMU_PHY6252_ROM}")
                list(APPEND MCUEMU_PHY6252_JOIN_ROM_ARGS "${MCUEMU_PHY6252_ROM}")
            endif()
            add_test(
                NAME firmware_phy6252_ts0001_join
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_phy6252_ts0001_join.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_PHY6252_JOIN_COORDINATOR}"
                        "${MCUEMU_PHY6252_TS0001_FIRMWARE}"
                        "${MCUEMU_PHY6252_EFUSE}"
                        "${MCUEMU_PHY6252_RF_CALIBRATION}"
                        ${MCUEMU_PHY6252_JOIN_ROM_ARGS})
            set_tests_properties(firmware_phy6252_ts0001_join PROPERTIES
                TIMEOUT 900
                LABELS "firmware;zigbee;phy6252;tuya"
            )
        else()
            message(STATUS
                "Skipping firmware_phy6252_ts0001_join: Python 3 interpreter not found")
        endif()
    else()
        message(STATUS
            "Skipping firmware_phy6252_ts0001_join: local TS0001, PHY6222 inputs, or the sampleGW (firmware/samplegw_tb03f.bin) not found")
    endif()

    set(MCUEMU_ZEPHYR_PM_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-pmtest.bin")
    if(EXISTS "${MCUEMU_ZEPHYR_PM_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_zephyr_pm_wake
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_zephyr_pm_wake.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_ZEPHYR_PM_FIRMWARE}")
            set_tests_properties(firmware_zephyr_pm_wake PROPERTIES
                TIMEOUT 180
                LABELS "firmware;power"
            )
        else()
            message(STATUS
                "Skipping firmware_zephyr_pm_wake: Python 3 interpreter not found")
        endif()
    endif()

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-ed-stimer-before.bin" AND
       EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-ed-stimer-after.bin")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_zephyr_stimer_wake
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_zephyr_stimer_wake.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-ed-stimer-before.bin"
                        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-ed-stimer-after.bin")
            set_tests_properties(firmware_zephyr_stimer_wake PROPERTIES
                TIMEOUT 120
                LABELS "firmware;power"
            )
        endif()
    endif()

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-ed-stimer-before.bin" AND
       EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-ed-stimer-after.bin" AND
       EXISTS "${MCUEMU_SAMPLE_GW_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_zephyr_ed_poll
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_zephyr_ed_poll.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_SAMPLE_GW_FIRMWARE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-ed-stimer-before.bin"
                        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-ed-stimer-after.bin")
            set_tests_properties(firmware_zephyr_ed_poll PROPERTIES
                TIMEOUT 600
                LABELS "firmware;zigbee;power"
            )
        endif()
    endif()

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-router-tick-start-no-retry.bin" AND
       EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-router-tick-start-retry.bin" AND
       EXISTS "${MCUEMU_SAMPLE_GW_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_zephyr_tick_start
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_zephyr_tick_start.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_SAMPLE_GW_FIRMWARE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-router-tick-start-no-retry.bin"
                        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zephyr-router-tick-start-retry.bin")
            set_tests_properties(firmware_zephyr_tick_start PROPERTIES
                TIMEOUT 600
                LABELS "firmware;zigbee"
            )
        endif()
    endif()

    set(MCUEMU_NRF52840_SENSOR_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/nrf52840-sensor.bin")
    if(EXISTS "${MCUEMU_NRF52840_SENSOR_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_nrf52840_boot
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_nrf52840_boot.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_NRF52840_SENSOR_FIRMWARE}")
            set_tests_properties(firmware_nrf52840_boot PROPERTIES
                TIMEOUT 120
                LABELS "firmware"
            )
        else()
            message(STATUS
                "Skipping firmware_nrf52840_boot: Python 3 interpreter not found")
        endif()
    else()
        message(STATUS
            "Skipping firmware_nrf52840_boot: local nrf52840-sensor.bin not found in firmware/")
    endif()

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/nrf52840-sensor-sht31.bin" AND
       EXISTS "${MCUEMU_SAMPLE_GW_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_nrf52840_zigbee_join
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_nrf52840_zigbee_join.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${CMAKE_CURRENT_SOURCE_DIR}/examples/nrf52840_zigbee.yaml")
            set_tests_properties(firmware_nrf52840_zigbee_join PROPERTIES
                TIMEOUT 300
                LABELS "firmware;zigbee"
            )
        endif()
    endif()

    # zigbee-rs Nucleo-WB55 router (CPU1 image) on the built-in ST 802.15.4 MAC
    # model of CPU2 joins the TLSR8258 sampleGW (docs/stm32wb55.md).
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/stm32wb55-zigbee-rs-router.bin" AND
       EXISTS "${MCUEMU_SAMPLE_GW_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_stm32wb55_zigbee_rs_router_join
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/zigbee_air_check.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${CMAKE_CURRENT_SOURCE_DIR}/examples/stm32wb55_zigbee_rs_router.yaml"
                        --workdir "${CMAKE_CURRENT_BINARY_DIR}/stm32wb55-zigbee-rs-router"
                        --pcap stm32wb55-zigbee-rs-air.pcap
                        --for 30s --no-uart-trace --min-up 2 --min-unicast 1 --min-down 2
                        --expect "Joined network as 0x[0-9A-F]{4}"
                        --expect "online: addr=0x[0-9A-F]{4} ch=[0-9]+ pan=0x[0-9A-F]{4}")
            set_tests_properties(firmware_stm32wb55_zigbee_rs_router_join PROPERTIES
                TIMEOUT 600
                LABELS "firmware;zigbee"
            )
        endif()
    else()
        message(STATUS
            "Skipping firmware_stm32wb55_zigbee_rs_router_join: firmware/stm32wb55-zigbee-rs-router.bin or the sampleGW not found")
    endif()

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/samplesw_8269.bin" AND
       EXISTS "${MCUEMU_SAMPLE_GW_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            # Vendor SDK sampleSwitch for the TLSR8269 joins the TLSR8258 sampleGW
            # (build: tests/ci/telink with -DAPP=sampleSwitch -DTELINK_PLATFORM=826x).
            add_test(
                NAME firmware_tlsr8269_zigbee_join
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/zigbee_air_check.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/scenarios/zigbee_tlsr8269_ed.yaml"
                        --workdir "${CMAKE_CURRENT_BINARY_DIR}/tlsr8269-join"
                        --for 40s --min-up 3 --min-unicast 2 --min-down 2)
            set_tests_properties(firmware_tlsr8269_zigbee_join PROPERTIES
                TIMEOUT 300
                LABELS "firmware;zigbee"
            )
        endif()
    endif()

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zbdonglee-ncp.bin")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            # Silicon Labs EZSP NCP (scripts/gbl2bin.py of the Nerivec release GBL): sends RSTACK.
            add_test(
                NAME firmware_efr32mg21_ncp_rstack
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_efr32mg21_ncp_rstack.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zbdonglee-ncp.bin")
            set_tests_properties(firmware_efr32mg21_ncp_rstack PROPERTIES
                TIMEOUT 180
                LABELS "firmware"
            )
            if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/telink-gw-ch11.bin")
                # Active scan against a channel-11 Telink coordinator on a shared air.
                add_test(
                    NAME firmware_efr32mg21_ncp_scan
                    COMMAND "${Python3_EXECUTABLE}"
                            "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_efr32mg21_ncp_scan.py"
                            "$<TARGET_FILE:mcuemu>"
                            "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zbdonglee-ncp.bin"
                            "${CMAKE_CURRENT_SOURCE_DIR}/firmware/telink-gw-ch11.bin")
                set_tests_properties(firmware_efr32mg21_ncp_scan PROPERTIES
                    TIMEOUT 300
                    LABELS "firmware"
                )
            endif()
            foreach(joiner efr32:efr32-zephyr-ed esp32h2:esp32h2-sensor)
                string(REPLACE ":" ";" joiner_parts "${joiner}")
                list(GET joiner_parts 0 joiner_kind)
                list(GET joiner_parts 1 joiner_image)
                if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/${joiner_image}.bin")
                    # The NCP forms a network (EZSP host in scripts/ncp_host.py) and a device of another family joins.
                    add_test(
                        NAME firmware_efr32mg21_ncp_join_${joiner_kind}
                        COMMAND "${Python3_EXECUTABLE}"
                                "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_efr32mg21_ncp_join.py"
                                "$<TARGET_FILE:mcuemu>"
                                "${CMAKE_CURRENT_SOURCE_DIR}/firmware/zbdonglee-ncp.bin"
                                "${CMAKE_CURRENT_SOURCE_DIR}/firmware/${joiner_image}.bin"
                                ${joiner_kind})
                    set_tests_properties(firmware_efr32mg21_ncp_join_${joiner_kind} PROPERTIES
                        TIMEOUT 600
                        LABELS "firmware;zigbee"
                    )
                endif()
            endforeach()
        endif()
    endif()

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/efr32mg21-sensor.bin" AND
       EXISTS "${MCUEMU_SAMPLE_GW_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_efr32mg21_zigbee_join
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_efr32mg21_zigbee_join.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${CMAKE_CURRENT_SOURCE_DIR}/examples/efr32mg21_zigbee.yaml")
            set_tests_properties(firmware_efr32mg21_zigbee_join PROPERTIES
                TIMEOUT 300
                LABELS "firmware;zigbee"
            )
        endif()
    endif()

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/esp32h2-sensor.bin" AND
       EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/firmware/telink-gw.bin")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_esp32h2_zigbee_join
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_esp32h2_zigbee_join.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${CMAKE_CURRENT_SOURCE_DIR}/examples/esp32h2_zigbee.yaml")
            set_tests_properties(firmware_esp32h2_zigbee_join PROPERTIES
                TIMEOUT 300
                LABELS "firmware;zigbee"
            )
        endif()
    endif()

    set(MCUEMU_VENDOR_SLEEP_1S_FIRMWARE
        "${CMAKE_CURRENT_SOURCE_DIR}/firmware/vendor_sleep_1s.bin")
    if(EXISTS "${MCUEMU_VENDOR_SLEEP_1S_FIRMWARE}")
        find_package(Python3 COMPONENTS Interpreter QUIET)
        if(Python3_Interpreter_FOUND)
            add_test(
                NAME firmware_vendor_sleep_1s
                COMMAND "${Python3_EXECUTABLE}"
                        "${CMAKE_CURRENT_SOURCE_DIR}/tests/firmware/test_vendor_sleep_1s.py"
                        "$<TARGET_FILE:mcuemu>"
                        "${MCUEMU_VENDOR_SLEEP_1S_FIRMWARE}")
            set_tests_properties(firmware_vendor_sleep_1s PROPERTIES
                TIMEOUT 60
                LABELS "firmware;power"
            )
        else()
            message(STATUS
                "Skipping firmware_vendor_sleep_1s: Python 3 interpreter not found")
        endif()
    else()
        message(STATUS
            "Skipping firmware_vendor_sleep_1s: local vendor_sleep_1s.bin not found in firmware/")
    endif()
endif()
