# Copyright 2019-2021, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

######
# Create a cli interface for Monado.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads)

add_executable(
	cli_vk
	cli_cmd_calibration_dump.c
	cli_cmd_lighthouse.c
	cli_cmd_probe.c
	cli_cmd_slambatch.c
	cli_cmd_test.c
	cli_common.h
	cli_main.c
	)
add_sanitizers(cli_vk)

if(NOT WIN32)
	# No getline on Windows, so until we have a portable impl
	target_sources(cli_vk PRIVATE cli_cmd_calibrate.c)
endif()

if(XRT_HAVE_OPENCV)
	target_link_libraries(cli_vk PRIVATE aux_tracking)
endif()

set_target_properties(cli_vk PROPERTIES OUTPUT_NAME monado_vk-cli PREFIX "")

target_link_libraries(
	cli_vk
		PUBLIC Threads::Threads
	PRIVATE
		aux_os
		aux_os_ble
		aux_util
		aux_math
		target_instance_no_comp
		drv_includes
	)

install(TARGETS cli_vk RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
