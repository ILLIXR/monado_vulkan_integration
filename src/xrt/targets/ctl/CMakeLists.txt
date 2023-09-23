# Copyright 2020-2021, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

add_executable(monado_vk-ctl main.c)
add_sanitizers(monado_vk-ctl)

target_include_directories(monado_vk-ctl PRIVATE ipc)

target_link_libraries(monado_vk-ctl PRIVATE aux_util ipc_client)

install(TARGETS monado_vk-ctl RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
