// Copyright 2020-2021, The Board of Trustees of the University of Illinois.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  ILLIXR plugin
 * @author RSIM Group <illixr@cs.illinois.edu>
 * @ingroup drv_illixr
 */

#define VMA_IMPLEMENTATION

#include <atomic>
#include <cassert>
#include <chrono>

#include "xrt/xrt_device.h"
#include "util/u_string_list.h"

#include <memory>
#include <vulkan/vulkan.h>

#include <iostream>
#include <thread>
#include <vector>

#include "illixr/plugin.hpp"
#include "illixr/phonebook.hpp"
#include "illixr/switchboard.hpp"
#include "illixr/data_format/pose_prediction.hpp"
#include "illixr/data_format/hand_tracking.hpp"
#include "illixr/vk/render_pass.hpp"
#include "illixr/vk/display_provider.hpp"
#include "illixr/vk/vulkan_objects.hpp"

#include <chrono>
#include <cstdlib>
#include <mutex>
#include <queue>
#include <string>

#include "illixr_component.h"

using namespace ILLIXR;
using namespace ILLIXR::vulkan;
using namespace ILLIXR::data_format;

const std::string PREFIX = "\e[0;32m[Monado ILLIXR]\e[0m ";

class monado_vulkan_display_provider : public display_provider {

};

class monado_compositor_app : public app {

};

static std::atomic<bool> _ds_ready = false;

/// Simulated plugin class for an instance during phonebook registration
class illixr_plugin : public plugin
{
public:
	illixr_plugin(const std::string& name_, phonebook *pb_)
	    : plugin{name_, pb_}
        , pb{pb_}
		, sb{phonebook_->lookup_impl<switchboard>()}
		, sb_pose{phonebook_->lookup_impl<pose_prediction>()}
		, sb_clock{phonebook_->lookup_impl<relative_clock>()}
		, ds{std::make_shared<monado_vulkan_display_provider>()}
		, _m_vsync{sb->get_writer<switchboard::event_wrapper<time_point>>("vsync_estimate")}
		, hand_tracking_reader_{sb->get_reader<hand_tracking_data>("hand_tracking")}
	{
		sb_timewarp = pb_->lookup_impl<timewarp>();

		if (std::getenv("ILLIXR_OFFLOAD_FRAMES") != nullptr) {
			offload_frames = std::stoi(std::getenv("ILLIXR_OFFLOAD_FRAMES"));
		}

		if (std::getenv("ILLIXR_COMPOSITOR_SLEEP_NS") != nullptr) {
			sleep_time = std::stoi(std::getenv("ILLIXR_COMPOSITOR_SLEEP_NS"));
		}

		// Check if hand tracking is enabled
		if (std::getenv("ILLIXR_USE_HAND_TRACKING") != nullptr) {
			std::string val = std::getenv("ILLIXR_USE_HAND_TRACKING");
			hand_tracking_enabled_ = (val == "1" || val == "true" || val == "TRUE");
		} else {
			// Default to enabled if offloading frames
			hand_tracking_enabled_ = offload_frames;
		}

		std::cout << PREFIX << "Hand tracking "
		          << (hand_tracking_enabled_ ? "enabled" : "disabled") << std::endl;
	}

	std::atomic<bool> ready = false;

	bool offload_frames = false;
	int sleep_time = -1;
	bool hand_tracking_enabled_ = false;

	phonebook *pb;
	const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<pose_prediction> sb_pose;
	const std::shared_ptr<relative_clock> sb_clock;
	std::shared_ptr<timewarp> sb_timewarp;
	std::shared_ptr<vulkan::buffer_pool<fast_pose_type>> buffer_pool;

	std::shared_ptr<display_provider> ds;
	switchboard::writer<switchboard::event_wrapper<time_point>> _m_vsync;

	// Hand tracking reader
	switchboard::reader<hand_tracking_data> hand_tracking_reader_;

	pose_type last_pose;
};

static illixr_plugin *illixr_plugin_obj = nullptr;

extern "C" plugin *
illixr_monado_create_plugin(phonebook *phonebook_)
{
	illixr_plugin_obj = new illixr_plugin{"illixr_plugin", phonebook_};
	illixr_plugin_obj->start();
	return illixr_plugin_obj;
}

extern "C" void illixr_monado_wait_for_init(void) {
	while (!_ds_ready) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
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
	const pose_type curr_pose = fast_pose.pose;
	ret.orientation.x = curr_pose.orientation.x();
	ret.orientation.y = curr_pose.orientation.y();
	ret.orientation.z = curr_pose.orientation.z();
	ret.orientation.w = curr_pose.orientation.w();
	ret.position.x = curr_pose.position.x();
	ret.position.y = curr_pose.position.y();
	ret.position.z = curr_pose.position.z();

	return ret;
}

/*
 *
 * Hand-tracking functions
 *
 */

extern "C" bool
illixr_hand_tracking_supported(void)
{
	if (!illixr_plugin_obj) {
		return false;
	}
	return illixr_plugin_obj->hand_tracking_enabled_;
}

/**
 * @brief Convert ILLIXR hand_joint_pose to illixr_hand_joint
 */
static void
convert_joint(const hand_joint_pose& src, struct illixr_hand_joint* dst)
{
	dst->position.x = src.position.x();
	dst->position.y = src.position.y();
	dst->position.z = src.position.z();

	dst->orientation.x = src.orientation.x();
	dst->orientation.y = src.orientation.y();
	dst->orientation.z = src.orientation.z();
	dst->orientation.w = src.orientation.w();

	dst->radius = src.radius;

	dst->linear_velocity.x = src.linear_velocity.x();
	dst->linear_velocity.y = src.linear_velocity.y();
	dst->linear_velocity.z = src.linear_velocity.z();

	dst->angular_velocity.x = src.angular_velocity.x();
	dst->angular_velocity.y = src.angular_velocity.y();
	dst->angular_velocity.z = src.angular_velocity.z();

	dst->location_flags = src.location_flags;
}

/**
 * @brief Convert ILLIXR single_hand_state to illixr_single_hand
 */
static void
convert_single_hand(const single_hand_state& src, struct illixr_single_hand* dst)
{
	dst->is_active = src.is_active;
	dst->confidence = src.confidence;

	for (size_t i = 0; i < HAND_JOINT_COUNT && i < ILLIXR_HAND_JOINT_COUNT; ++i) {
		convert_joint(src.joints[i], &dst->joints[i]);
	}
}

extern "C" bool
illixr_read_hand_tracking(struct illixr_hand_tracking_data *out_data)
{
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	assert(out_data && "out_data must not be null.");

	if (!illixr_plugin_obj->hand_tracking_enabled_) {
		out_data->valid = false;
		return false;
	}

	// Try to get hand tracking data from switchboard
	std::shared_ptr<const hand_tracking_data> hand_data =
		illixr_plugin_obj->hand_tracking_reader_.get_ro_nullable();

	if (!hand_data || !hand_data->has_any_tracking()) {
		out_data->valid = false;
		out_data->left_hand.is_active = false;
		out_data->right_hand.is_active = false;
		return false;
	}

	// Convert left hand
	convert_single_hand(hand_data->left_hand, &out_data->left_hand);

	// Convert right hand
	convert_single_hand(hand_data->right_hand, &out_data->right_hand);

	out_data->valid = true;
	return true;
}

extern "C" bool
illixr_read_single_hand(int hand, struct illixr_single_hand *out_hand)
{
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	assert(out_hand && "out_hand must not be null.");
	assert(hand == 0 || hand == 1 && "hand must be 0 (left) or 1 (right).");

	if (!illixr_plugin_obj->hand_tracking_enabled_) {
		out_hand->is_active = false;
		return false;
	}

	// Try to get hand tracking data from the switchboard
	std::shared_ptr<const hand_tracking_data> hand_data =
		illixr_plugin_obj->hand_tracking_reader_.get_ro_nullable();

	if (!hand_data) {
		out_hand->is_active = false;
		return false;
	}

	const single_hand_state& src = (hand == 0) ? hand_data->left_hand : hand_data->right_hand;

	if (!src.is_active) {
		out_hand->is_active = false;
		return false;
	}

	convert_single_hand(src, out_hand);
	return true;
}

/*
 *
 * Vulkan display service functions
 *
 */

extern "C" void illixr_initialize_vulkan_display_service(VkInstance instance, VkPhysicalDevice physical_device,
                                                         VkDevice device, VkQueue queue, uint32_t queue_family_index,
                                                         struct u_string_list* enabled_instance_extensions,
                                                         struct u_string_list* enabled_device_extensions) {
	printf("Initializing vulkan display service\n");
	auto ds = std::make_shared<monado_vulkan_display_provider>();
	ds->vk_instance_ = instance;
	ds->vk_physical_device_ = physical_device;
	ds->vk_device_ = device;
	ds->queues_[queue::GRAPHICS] = {queue, queue_family_index, queue::GRAPHICS, std::make_shared<std::mutex>()};

	const char* const * exts = u_string_list_get_data(enabled_instance_extensions);
	uint32_t ext_count = u_string_list_get_size(enabled_instance_extensions);

	for (uint32_t i = 0; i < ext_count; i++) {
		ds->enabled_instance_extensions_.push_back(exts[i]);
	}

	const char* const * dev_exts = u_string_list_get_data(enabled_device_extensions);
	uint32_t dev_ext_count = u_string_list_get_size(enabled_device_extensions);

	for (uint32_t i = 0; i < dev_ext_count; i++) {
		ds->enabled_device_extensions_.push_back(dev_exts[i]);
	}

	_ds_ready = true;

	illixr_plugin_obj->pb->register_impl<display_provider>(std::static_pointer_cast<display_provider>(ds));
	illixr_plugin_obj->ds = ds;
}

extern "C" void illixr_destroy_timewarp() {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	illixr_plugin_obj->sb_timewarp->destroy();
}

extern "C" void illixr_initialize_timewarp(VkRenderPass render_pass, uint32_t subpass, VkExtent2D extent, VkImage* image, VkImageView* image_view, VkDeviceMemory* device_memory, VkDeviceSize* size, VkDeviceSize* offset, uint32_t num_buffers_per_eye) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	std::cout << PREFIX << "Initializing timewarp" << std::endl;
	// Num buffers per eye should be twice as large if we're passing in depth.
	// std::vector<VkImageView> left_eye_views(buffer_pool, buffer_pool + num_buffers_per_eye);
	// std::vector<VkImageView> right_eye_views(buffer_pool + num_buffers_per_eye, buffer_pool + 2 * num_buffers_per_eye);
	// std::array<std::vector<VkImageView>, 2> eye_views = {left_eye_views, right_eye_views};
	// // for (int eye = 0; eye < 2; eye++) {
	// // 	for (int buffer = 0; buffer < num_buffers_per_eye; buffer++) {
	// // 		assert(eye_views[eye][buffer] != VK_NULL_HANDLE && "Eye buffer image view is a null handle!");
	// // 	}
	// // }
	// illixr_plugin_obj->sb_timewarp->setup(render_pass, subpass, std::move(eye_views), false);

	// Depth images are interleaved with the color images, e.g.
	// left image 0, left depth 0, right image 0, right depth 0
	std::vector<std::array<vulkan::vk_image, 2>> image_pool;
	for (auto i = 0; i < num_buffers_per_eye; i++) {
		std::array<vulkan::vk_image, 2> image_arr;
		for (auto eye = 0; eye < 2; eye++) {
			image_arr[eye].image = image[i * 4 + eye * 2];
			image_arr[eye].image_view = image_view[i * 4 + eye  * 2];
			image_arr[eye].allocation_info.size = size[i * 4 + eye * 2];
			image_arr[eye].allocation_info.offset = offset[i * 4 + eye * 2];
			image_arr[eye].allocation_info.deviceMemory = device_memory[i * 4 + eye * 2];
			image_arr[eye].image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
			image_arr[eye].image_info.format = VK_FORMAT_B8G8R8A8_UNORM;
			image_arr[eye].image_info.extent = {extent.width, extent.height, 1};
		}
		image_pool.push_back(image_arr);
	}

	std::vector<std::array<vulkan::vk_image, 2>> depth_image_pool;
	for (auto i = 0; i < num_buffers_per_eye; i++) {
		std::array<vulkan::vk_image, 2> image_arr;
		for (auto eye = 0; eye < 2; eye++) {
			image_arr[eye].image = image[i * 4 + eye * 2 + 1];
			image_arr[eye].image_view = image_view[i * 4 + eye  * 2 + 1];
			image_arr[eye].allocation_info.size = size[i * 4 + eye * 2 + 1];
			image_arr[eye].allocation_info.offset = offset[i * 4 + eye * 2 + 1];
			image_arr[eye].allocation_info.deviceMemory = device_memory[i * 4 + eye * 2 + 1];
			image_arr[eye].image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
			image_arr[eye].image_info.format = VK_FORMAT_B8G8R8A8_UNORM;
			image_arr[eye].image_info.extent = {extent.width, extent.height, 1};
		}
		depth_image_pool.push_back(image_arr);
	}

	auto buffer_pool = std::make_shared<vulkan::buffer_pool<fast_pose_type>>(image_pool, depth_image_pool);
	illixr_plugin_obj->buffer_pool = buffer_pool;
	illixr_plugin_obj->sb_timewarp->setup(render_pass, subpass, std::move(buffer_pool), true);
	std::cout << PREFIX << "Initialized timewarp" << std::endl;
}

extern "C" int8_t illixr_src_acquire() {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	return illixr_plugin_obj->buffer_pool->src_acquire_image();
}

extern "C" void illixr_src_release(int8_t buffer_ind, struct xrt_pose l_pose, struct xrt_pose r_pose) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	pose_type pose {time_point{},
					Eigen::Vector3f {(l_pose.position.x + r_pose.position.x) / 2, (l_pose.position.y + r_pose.position.y) / 2, (l_pose.position.z + r_pose.position.z) / 2},
					Eigen::Quaternionf {(l_pose.orientation.w), (l_pose.orientation.x), (l_pose.orientation.y), (l_pose.orientation.z)}
					};
	illixr_plugin_obj->buffer_pool->src_release_image(buffer_ind, fast_pose_type {pose, {}, {}});
}

extern "C" bool illixr_offload_frames() {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	return illixr_plugin_obj->offload_frames;
}

extern "C" int illixr_sleep_time() {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	return illixr_plugin_obj->sleep_time;
}

extern "C" void illixr_tw_update_uniforms(xrt_pose l_pose, xrt_pose r_pose) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");

	if (!illixr_plugin_obj->offload_frames) {
		pose_type pose {time_point{},
					Eigen::Vector3f {(l_pose.position.x + r_pose.position.x) / 2, (l_pose.position.y + r_pose.position.y) / 2, (l_pose.position.z + r_pose.position.z) / 2},
					Eigen::Quaternionf {(l_pose.orientation.w), (l_pose.orientation.x), (l_pose.orientation.y), (l_pose.orientation.z)}
					};
		illixr_plugin_obj->last_pose = pose;
	}
}

extern "C" void illixr_tw_record_command_buffer(VkCommandBuffer commandBuffer, VkFramebuffer framebuffer, int buffer_ind, int left) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");

	if (!illixr_plugin_obj->offload_frames) {
		illixr_plugin_obj->sb_timewarp->update_uniforms(illixr_plugin_obj->last_pose);
		illixr_plugin_obj->sb_timewarp->record_command_buffer(commandBuffer, framebuffer, buffer_ind, left);
	}
}

extern "C" void illixr_publish_vsync_estimate(uint64_t display_time_ns) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");

	if (!illixr_plugin_obj->offload_frames) {
		auto relative_time = time_point{time_point{std::chrono::nanoseconds(display_time_ns)} - illixr_plugin_obj->sb_clock->start_time()};
	  illixr_plugin_obj->_m_vsync.put(illixr_plugin_obj->_m_vsync.allocate(relative_time));
	}
}
