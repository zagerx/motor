# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(motor)

add_subdirectory(drivers)
add_subdirectory(lib)
add_subdirectory(canard)

zephyr_include_directories(
    include
    ProtocolV4/uavcan/.cFolder 
    ProtocolV4/uavcan/libcanard/libcanard
    statemachine
)
zephyr_library_sources(ProtocolV4/uavcan/libcanard/libcanard/canard.c)

target_sources(app PRIVATE modulestest/main.c)
