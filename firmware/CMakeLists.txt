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
    DEPENDS tools/gen_tables.py ${CHAR_DIR}/bpf_bank.json ${CHAR_DIR}/controller.json
    COMMENT "Generating firmware tables from the characterisation files")

add_library(elf_core STATIC
    core/lo_plan.c core/si5351.c core/relays.c core/radio.c core/supervisor.c core/cat.c core/audio.c
    ${GEN_DIR}/tables_gen.c)
target_include_directories(elf_core PUBLIC core ${GEN_DIR})

add_library(elf_vhw STATIC host/vhw.c)
target_include_directories(elf_vhw PUBLIC host)
target_link_libraries(elf_vhw PUBLIC elf_core)

enable_testing()
foreach(t si5351 relays radio supervisor cat audio)
    add_executable(test_${t} tests/test_${t}.c tests/test_hal.c)
    target_include_directories(test_${t} PRIVATE tests)
    target_link_libraries(test_${t} PRIVATE elf_core elf_vhw)
    add_test(NAME ${t} COMMAND test_${t})
endforeach()

add_executable(lo_plan_dump tests/lo_plan_dump.c)
target_link_libraries(lo_plan_dump PRIVATE elf_core)
add_executable(si5351_dump tests/si5351_dump.c)
target_link_libraries(si5351_dump PRIVATE elf_core)
