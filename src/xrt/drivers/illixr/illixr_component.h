// Copyright 2020-2021, The Board of Trustees of the University of Illinois.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  ILLIXR plugin
 * @author RSIM Group <illixr@cs.illinois.edu>
 * @ingroup drv_illixr
 */

#pragma once

#include <vulkan/vulkan.h>
#include <vulkan/vulkan_core.h>

#include "util/u_string_list.h"

#ifdef __cplusplus
extern "C" {
#endif

void *
illixr_monado_create_plugin(void *pb);
struct xrt_pose
illixr_read_pose(void);

void illixr_monado_wait_for_init(void);

void illixr_initialize_vulkan_display_service(VkInstance instance, VkPhysicalDevice physical_device, VkDevice device, VkQueue queue, uint32_t queue_family_index, struct u_string_list *enabled_instance_extensions, struct u_string_list *enabled_device_extensions);
void illixr_initialize_timewarp(VkRenderPass render_pass, uint32_t subpass, VkExtent2D extent, VkImage* image, VkImageView* image_view, VkDeviceMemory* device_memory, VkDeviceSize* size, VkDeviceSize* offset, uint32_t num_buffers_per_eye);
int8_t illixr_src_acquire();
void illixr_src_release(int8_t buffer_ind, struct xrt_pose l_pose, struct xrt_pose r_pose);
void illixr_destroy_timewarp(void);
void illixr_tw_update_uniforms(struct xrt_pose l_pose, struct xrt_pose r_pose);
void illixr_tw_record_command_buffer(VkCommandBuffer commandBuffer, VkFramebuffer framebuffer, int buffer_ind, int left);
void illixr_publish_vsync_estimate(uint64_t display_time_ns);

#ifdef __cplusplus
}
#endif
