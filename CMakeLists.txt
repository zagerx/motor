cmake_minimum_required(VERSION 3.20)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(motor)

# 添加主程序源码
target_sources(app PRIVATE src/main.c)