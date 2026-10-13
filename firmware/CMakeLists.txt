# Elf firmware, Linux host build: the hardware-independent modules, their unit tests and (from
# plan 4.2) the virtual board. The ESP32-S3 target build is a separate ESP-IDF project.
cmake_minimum_required(VERSION 3.20)
project(elf_firmware C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror -Wshadow -Wconversion -Wno-sign-conversion)

set(CHAR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../hardware/characterized CACHE PATH "characterisation files")
set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
file(MAKE_DIRECTORY ${GEN_DIR})
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_custom_command(
    OUTPUT ${GEN_DIR}/tables_gen.h ${GEN_DIR}/tables_gen.c
    COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_tables.py ${CHAR_DIR} ${GEN_DIR}
    DEPENDS tools/gen_tables.py ${CHAR_DIR}/bpf_bank.json ${CHAR_DIR}/controller.json ${CHAR_DIR}/power.json
    COMMENT "Generating firmware tables from the characterisation files")

add_library(elf_core STATIC
    core/lo_plan.c core/si5351.c core/relays.c core/radio.c core/supervisor.c core/cat.c core/audio.c
    ${GEN_DIR}/tables_gen.c)
target_include_directories(elf_core PUBLIC core ${GEN_DIR})

add_library(elf_vhw STATIC host/vhw.c)
target_include_directories(elf_vhw PUBLIC host)
target_link_libraries(elf_vhw PUBLIC elf_core)

enable_testing()
# the unit tests run under UBSan: undefined behaviour (overflow, bad shifts) fails them
add_library(elf_core_ubsan STATIC
    core/lo_plan.c core/si5351.c core/relays.c core/radio.c core/supervisor.c core/cat.c core/audio.c
    ${GEN_DIR}/tables_gen.c host/vhw.c)
target_include_directories(elf_core_ubsan PUBLIC core host ${GEN_DIR})
target_compile_options(elf_core_ubsan PUBLIC -fsanitize=undefined -fno-sanitize-recover=all)
target_link_options(elf_core_ubsan PUBLIC -fsanitize=undefined)
foreach(t lo_plan si5351 relays radio supervisor cat audio)
    add_executable(test_${t} tests/test_${t}.c tests/test_hal.c)
    target_include_directories(test_${t} PRIVATE tests)
    target_link_libraries(test_${t} PRIVATE elf_core_ubsan)
    add_test(NAME ${t} COMMAND test_${t})
endforeach()

add_executable(lo_plan_dump tests/lo_plan_dump.c)
target_link_libraries(lo_plan_dump PRIVATE elf_core)
add_executable(si5351_dump tests/si5351_dump.c)
target_link_libraries(si5351_dump PRIVATE elf_core)

# ---- USB: TinyUSB (third_party/tinyusb, release 0.21.0) and the virtual board ----
set(TUSB ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/tinyusb/src)
add_library(tinyusb STATIC
    ${TUSB}/tusb.c ${TUSB}/common/tusb_fifo.c ${TUSB}/device/usbd.c
    ${TUSB}/class/audio/audio_device.c ${TUSB}/class/cdc/cdc_device.c)
target_include_directories(tinyusb SYSTEM PUBLIC ${TUSB})
target_include_directories(tinyusb PUBLIC usb host)
target_compile_definitions(tinyusb PUBLIC CFG_TUSB_MCU=OPT_MCU_NONE)
# third-party code: build it as it is, without the project's strict warnings
target_compile_options(tinyusb PRIVATE -w)

# the device controller driver, descriptors and class callbacks are objects, not an archive:
# TinyUSB calls back into them
add_library(elf_usb OBJECT usb/usb_desc.c usb/usb_app.c host/dcd_usbip.c)
target_link_libraries(elf_usb PUBLIC tinyusb elf_core)

add_executable(elf-virtual host/main.c host/hal_virtual.c host/twin_link.c $<TARGET_OBJECTS:elf_usb>)
target_link_libraries(elf-virtual PRIVATE elf_usb elf_vhw tinyusb elf_core m)
