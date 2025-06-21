# Copyright (c) 2021 Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_MOTOR_MODEL)

    add_subdirectory(bldcmotor)
    add_subdirectory(foc)
    add_subdirectory(focutils)

elseif(CONFIG_BOARD_ZGM_003)

else()
    message(FATAL_ERROR "No valid board selected!")
endif()