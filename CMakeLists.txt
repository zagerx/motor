# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor)


add_subdirectory(drivers)
add_subdirectory(lib)
zephyr_include_directories(include)


target_sources(app PRIVATE modulestest/main.c)

#target_link_libraries(app PRIVATE cmsis-dsp)


# target_link_libraries(app PRIVATE ..__motor__drivers__svpwm)  # 关键：使用实际库名
# target_link_libraries(app PRIVATE ..__motor__lib__utils)  # 关键：使用实际库名