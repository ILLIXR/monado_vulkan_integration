# Copyright 2022, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads)

add_subdirectory(mercury)

###
# Async wrapper around sync helper.
#
add_library(hand_async STATIC t_hand_tracking_async.c)
target_link_libraries(hand_async PRIVATE xrt-interfaces aux_os aux_util aux_math Threads::Threads)
