// Copyright 2020-2021, The Board of Trustees of the University of Illinois.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  ILLIXR plugin
 * @author RSIM Group <illixr@cs.illinois.edu>
 * @ingroup drv_illixr
 */


#include "xrt/xrt_device.h"

#include <vulkan/vulkan.h>

#include <iostream>
#include "common/plugin.hpp"
#include "common/phonebook.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/pose_prediction.hpp"
#include "common/vk_util/render_pass.hpp"

using namespace ILLIXR;

/// Simulated plugin class for an instance during phonebook registration
class illixr_plugin : public plugin
{
public:
	illixr_plugin(std::string name_, phonebook *pb_)
	    : plugin{name_, pb_}, sb{pb->lookup_impl<switchboard>()}, sb_pose{pb->lookup_impl<pose_prediction>()}
		, sb_timewarp{pb->lookup_impl<timewarp>()}
	{}

	const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<pose_prediction> sb_pose;
	const std::shared_ptr<timewarp> sb_timewarp;
};

class monado_vulkan_display_sink : public display_sink {

}

static illixr_plugin *illixr_plugin_obj = nullptr;

extern "C" plugin *
illixr_monado_create_plugin(phonebook *pb)
{
	illixr_plugin_obj = new illixr_plugin{"illixr_plugin", pb};
	illixr_plugin_obj->start();
	return illixr_plugin_obj;
}

extern "C" struct xrt_pose
illixr_read_pose()
{
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");

	if (!illixr_plugin_obj->sb_pose->fast_pose_reliable()) {
		std::cerr << "Pose not reliable yet; returning best guess" << std::endl;
	}
	struct xrt_pose ret;
	const fast_pose_type fast_pose = illixr_plugin_obj->sb_pose->get_fast_pose();
	const pose_type pose = fast_pose.pose;

	ret.orientation.x = pose.orientation.x();
	ret.orientation.y = pose.orientation.y();
	ret.orientation.z = pose.orientation.z();
	ret.orientation.w = pose.orientation.w();
	ret.position.x = pose.position.x();
	ret.position.y = pose.position.y();
	ret.position.z = pose.position.z();

	return ret;
}

extern "C" void *illixr_initialize_vulkan_display_service(VkInstance instance, VkPhysicalDevice physical_device, VkDevice device, VkQueue queue, uint32_t queue_family_index, VkExtent2D swapchain_extent) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	auto ds = std::make_shared<monado_vulkan_display_sink>();
	ds->vk_instance = instance;
	ds->vk_physical_device = physical_device;
	ds->vk_device = device;
	ds->graphics_queue = queue;
	ds->graphics_queue_family = queue_family_index;
	ds->swapchain_extent = swapchain_extent;
}

extern "C" void *illixr_initialize_timewarp(VkRenderPass render_pass, uint32_t subpass, VkImageView* buffer_pool, uint32_t num_buffers_per_eye) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	std::vector<VkImageView> left_eye_views(buffer_pool, buffer_pool + num_buffers_per_eye);
	std::vector<VkImageView> right_eye_views(buffer_pool + num_buffers_per_eye, buffer_pool + 2 * num_buffers_per_eye);
	std::array<std::vector<VkImageView>, 2> eye_views = {left_eye_views, right_eye_views};
	illixr_plugin_obj->sb_timewarp->setup(render_pass, subpass, std::move(eye_views));
}