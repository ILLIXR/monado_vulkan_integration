// Copyright 2020-2021, The Board of Trustees of the University of Illinois.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  ILLIXR plugin
 * @author RSIM Group <illixr@cs.illinois.edu>
 * @ingroup drv_illixr
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_core.h>

#include "util/u_string_list.h"
#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 *
 * Hand tracking constants and structures
 *
 */

/**
 * @brief Number of joints per hand (matches OpenXR XR_HAND_JOINT_COUNT_EXT)
 */
#define ILLIXR_HAND_JOINT_COUNT 26

/**
 * @brief Hand joint data for a single joint
 *
 * Contains position, orientation, radius, and tracking validity.
 */
struct illixr_hand_joint {
    struct xrt_vec3 position;       //!< 3D position in meters
    struct xrt_quat orientation;    //!< Rotation quaternion
    float radius;                   //!< Joint radius in meters
    struct xrt_vec3 linear_velocity;  //!< Linear velocity m/s
    struct xrt_vec3 angular_velocity; //!< Angular velocity rad/s
    uint32_t location_flags;         //!< Tracking validity flags
};

/**
 * @brief Hand tracking data for a single hand
 */
struct illixr_single_hand {
    struct illixr_hand_joint joints[ILLIXR_HAND_JOINT_COUNT]; //!< All 26 joints
    bool is_active;     //!< Whether hand is currently tracked
    float confidence;   //!< Tracking confidence 0.0-1.0
};

/**
 * @brief Hand tracking data for both hands
 */
struct illixr_hand_tracking_data {
    struct illixr_single_hand left_hand;   //!< Left hand data
    struct illixr_single_hand right_hand;  //!< Right hand data
    bool valid;                            //!< Whether data is valid
};

/*
 *
 * Plugin lifecycle functions
 *
 */

/**
 * @brief Create the ILLIXR plugin and register with phonebook
 * @param pb Phonebook pointer
 * @return Plugin instance
 */
void *
illixr_monado_create_plugin(void *pb);
/**
 * @brief Wait for ILLIXR initialization to complete
 */
void
illixr_monado_wait_for_init(void);

/*
 *
 * Pose tracking functions
 *
 */

/**
 * @brief Read the current head pose from ILLIXR
 * @return Current head pose
 */
struct xrt_pose
illixr_read_pose(void);

/*
 *
 * Hand-tracking functions
 *
 */

bool illixr_hand_tracking_supported(void);
bool illixr_read_hand_tracking(struct illixr_hand_tracking_data *out_data);
bool illixr_read_single_hand(int hand, struct illixr_single_hand *out_hand);

/*
 *
 * Vulkan/display functions
 *
 */

void illixr_initialize_vulkan_display_service(VkInstance instance, VkPhysicalDevice physical_device, VkDevice device, VkQueue queue, uint32_t queue_family_index, struct u_string_list *enabled_instance_extensions, struct u_string_list *enabled_device_extensions);
void illixr_initialize_timewarp(VkRenderPass render_pass, uint32_t subpass, VkExtent2D extent, VkImage* image, VkImageView* image_view, VkDeviceMemory* device_memory, VkDeviceSize* size, VkDeviceSize* offset, uint32_t num_buffers_per_eye);
int8_t illixr_src_acquire();
void illixr_src_release(int8_t buffer_ind, struct xrt_pose l_pose, struct xrt_pose r_pose);
void illixr_destroy_timewarp(void);
bool illixr_offload_frames();
int illixr_sleep_time();
void illixr_tw_update_uniforms(struct xrt_pose l_pose, struct xrt_pose r_pose);
void illixr_tw_record_command_buffer(VkCommandBuffer commandBuffer, VkFramebuffer framebuffer, int buffer_ind, int left);
void illixr_publish_vsync_estimate(uint64_t display_time_ns);

#ifdef __cplusplus
}
#endif
