# Copyright 2020, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads)

add_library(monado-service MODULE service_target.cpp)

target_link_libraries(
	monado-service
		PUBLIC Threads::Threads
	PRIVATE
		aux_util
		st_prober
		ipc_server
		comp_main
		target_lists
		target_instance
		xrt-external-jni-wrap
	)
