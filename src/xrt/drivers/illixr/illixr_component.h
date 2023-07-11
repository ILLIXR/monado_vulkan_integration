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

#ifdef __cplusplus
extern "C" {
#endif

void *
illixr_monado_create_plugin(void *pb);
struct xrt_pose
illixr_read_pose();

void illixr_initialize_vulkan_display_service(VkInstance instance, VkPhysicalDevice physical_device, VkDevice device, VkQueue queue, uint32_t queue_family_index);
void illixr_initialize_timewarp(VkRenderPass render_pass, uint32_t subpass, VkImageView* buffer_pool, uint32_t num_buffers);
void illixr_tw_update_uniforms();
void illixr_tw_record_command_buffer(VkCommandBuffer commandBuffer, int buffer_ind, int left);

#ifdef __cplusplus
}
#endif
