# 必须包含SOC系列级配置
set(SUPPORTED_SOCS stm32g473xx)
include(${ZEPHYR_BASE}/soc/st/stm32/stm32g4x/Kconfig.defconfig)  # 路径确认
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)