// Copyright 2020-2021, The Board of Trustees of the University of Illinois.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  ILLIXR plugin
 * @author RSIM Group <illixr@cs.illinois.edu>
 * @ingroup drv_illixr
 */


#include "xrt/xrt_device.h"

#include <memory>
#include <vulkan/vulkan.h>

#include <iostream>
#include "common/plugin.hpp"
#include "common/phonebook.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/pose_prediction.hpp"
#include "common/vk_util/render_pass.hpp"
#include "common/vk_util/display_sink.hpp"

using namespace ILLIXR;

const std::string PREFIX = "\e[0;32m[Monado ILLIXR]\e[0m ";

class monado_vulkan_display_sink : public display_sink {

};

/// Simulated plugin class for an instance during phonebook registration
class illixr_plugin : public plugin
{
public:
	illixr_plugin(std::string name_, phonebook *pb_)
	    : plugin{name_, pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, sb_pose{pb->lookup_impl<pose_prediction>()}
		, sb_clock{pb->lookup_impl<RelativeClock>()}
		, ds{std::make_shared<monado_vulkan_display_sink>()}
		, _m_vsync{sb->get_writer<switchboard::event_wrapper<time_point>>("vsync_estimate")}
	{
		pb_->register_impl<display_sink>(std::static_pointer_cast<display_sink>(ds));
		sb_timewarp = pb_->lookup_impl<timewarp>();
	}

	const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<pose_prediction> sb_pose;
	const std::shared_ptr<RelativeClock> sb_clock;
	std::shared_ptr<timewarp> sb_timewarp;

	std::shared_ptr<display_sink> ds;
	switchboard::writer<switchboard::event_wrapper<time_point>> _m_vsync;

};

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

extern "C" void illixr_initialize_vulkan_display_service(VkInstance instance, VkPhysicalDevice physical_device, VkDevice device, VkQueue queue, uint32_t queue_family_index) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	auto ds = illixr_plugin_obj->ds;
	ds->vk_instance = instance;
	ds->vk_physical_device = physical_device;
	ds->vk_device = device;
	ds->graphics_queue = queue;
	ds->graphics_queue_family = queue_family_index;

	illixr_plugin_obj->ds = ds;
}

extern "C" void illixr_destroy_timewarp() {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	illixr_plugin_obj->sb_timewarp->destroy();
}

extern "C" void illixr_initialize_timewarp(VkRenderPass render_pass, uint32_t subpass, VkImageView* buffer_pool, uint32_t num_buffers_per_eye) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	std::cout << PREFIX << "Initializing timewarp" << std::endl;
	std::vector<VkImageView> left_eye_views(buffer_pool, buffer_pool + num_buffers_per_eye);
	std::vector<VkImageView> right_eye_views(buffer_pool + num_buffers_per_eye, buffer_pool + 2 * num_buffers_per_eye);
	std::array<std::vector<VkImageView>, 2> eye_views = {left_eye_views, right_eye_views};
	illixr_plugin_obj->sb_timewarp->setup(render_pass, subpass, std::move(eye_views), false);
}

extern "C" void illixr_tw_update_uniforms(xrt_pose *l_pose, xrt_pose *r_pose) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");

	pose_type pose {time_point{}, 
					{(l_pose->position.x + r_pose->position.x) / 2, (l_pose->position.y + r_pose->position.y) / 2, (l_pose->position.z + r_pose->position.z) / 2},
					{(l_pose->orientation.w), (l_pose->orientation.x), (l_pose->orientation.y), (l_pose->orientation.z)}
					};
	illixr_plugin_obj->sb_timewarp->update_uniforms(pose);
}

extern "C" void illixr_tw_record_command_buffer(VkCommandBuffer commandBuffer, int buffer_ind, int left) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	illixr_plugin_obj->sb_timewarp->record_command_buffer(commandBuffer, buffer_ind, left);
}

extern "C" void illixr_publish_vsync_estimate(uint64_t display_time_ns) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	auto relative_time = time_point{time_point{std::chrono::nanoseconds(display_time_ns)} - illixr_plugin_obj->sb_clock->start_time()};
	illixr_plugin_obj->_m_vsync.put(illixr_plugin_obj->_m_vsync.allocate<switchboard::event_wrapper<time_point>>(relative_time));
}