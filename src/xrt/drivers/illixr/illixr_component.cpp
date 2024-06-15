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
#include "illixr/data_format.hpp"
#include "illixr/pose_prediction.hpp"
#include "illixr/vk/render_pass.hpp"
#include "illixr/vk/display_provider.hpp"
#include "illixr/vk/vulkan_objects.hpp"

#include <chrono>
#include <cstdlib>
#include <mutex>
#include <queue>
#include <string>

using namespace ILLIXR;
using namespace ILLIXR::vulkan;

const std::string PREFIX = "\e[0;32m[Monado ILLIXR]\e[0m ";

class monado_vulkan_display_provider : public display_provider {

};

static std::atomic<bool> _ds_ready = false;

/// Simulated plugin class for an instance during phonebook registration
class illixr_plugin : public plugin
{
public:
	illixr_plugin(std::string name_, phonebook *pb_)
	    : plugin{name_, pb_}
		, pb{pb_}
		, sb{pb->lookup_impl<switchboard>()}
		, sb_pose{pb->lookup_impl<pose_prediction>()}
		, sb_clock{pb->lookup_impl<RelativeClock>()}
		, _m_vsync{sb->get_writer<switchboard::event_wrapper<time_point>>("vsync_estimate")}
	{
		sb_timewarp = pb_->lookup_impl<timewarp>();

		if (std::getenv("ILLIXR_USE_LOSSY_DEPTH") != nullptr) {
			use_lossy_depth = std::stoi(std::getenv("ILLIXR_USE_LOSSY_DEPTH"));
		}

		if (std::getenv("ILLIXR_OFFLOAD_FRAMES") != nullptr) {
			offload_frames = std::stoi(std::getenv("ILLIXR_OFFLOAD_FRAMES"));
		}
		
		if (std::getenv("ILLIXR_ARTIFICIAL_LATENCY_MS") != nullptr) {
			artificial_latency = std::stoi(std::getenv("ILLIXR_ARTIFICIAL_LATENCY_MS"));
		}

		if (std::getenv("ILLIXR_POSE_DUMP") != nullptr) {
			dump_poses = true;
			std::string file_dump = std::getenv("ILLIXR_POSE_DUMP");
			pose_file.open(file_dump);
			pose_start_time = std::chrono::system_clock::now();
		}
	}

	std::atomic<bool> ready = false;
	
	bool offload_frames = false;
	bool use_lossy_depth = false;

	phonebook *pb;
	const std::shared_ptr<switchboard> sb;
	const std::shared_ptr<pose_prediction> sb_pose;
	const std::shared_ptr<RelativeClock> sb_clock;
	std::shared_ptr<timewarp> sb_timewarp;
	std::shared_ptr<vulkan::buffer_pool<fast_pose_type>> buffer_pool;

	std::shared_ptr<display_provider> ds;
	switchboard::writer<switchboard::event_wrapper<time_point>> _m_vsync;

	pose_type last_pose;

	std::chrono::time_point<std::chrono::system_clock> buffer_start_time;
	std::queue<pose_type> buffered_poses;
	std::mutex buffered_pose_mutex;
	uint64_t artificial_latency = 0; 

	bool dump_poses = false;
	std::ofstream pose_file;
	std::chrono::time_point<std::chrono::system_clock> pose_start_time;
};

static illixr_plugin *illixr_plugin_obj = nullptr;

extern "C" plugin *
illixr_monado_create_plugin(phonebook *pb)
{
	illixr_plugin_obj = new illixr_plugin{"illixr_plugin", pb};
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

	if (illixr_plugin_obj->artificial_latency > 0) {
		std::lock_guard<std::mutex> lock(illixr_plugin_obj->buffered_pose_mutex);
		
		if (illixr_plugin_obj->buffered_poses.empty()) {
			illixr_plugin_obj->buffer_start_time = std::chrono::system_clock::now();
		}
		illixr_plugin_obj->buffered_poses.push(fast_pose.pose);

		std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - illixr_plugin_obj->buffer_start_time).count() > illixr_plugin_obj->artificial_latency) {
			const pose_type past_pose = illixr_plugin_obj->buffered_poses.front();
			illixr_plugin_obj->buffered_poses.pop(); 

			ret.orientation.x = past_pose.orientation.x();
			ret.orientation.y = past_pose.orientation.y();
			ret.orientation.z = past_pose.orientation.z();
			ret.orientation.w = past_pose.orientation.w();
			ret.position.x = past_pose.position.x();
			ret.position.y = past_pose.position.y();
			ret.position.z = past_pose.position.z();
		} else {
			ret.orientation.x = 0;
			ret.orientation.y = 0;
			ret.orientation.z = 0;
			ret.orientation.w = 1;
			ret.position.x = 0;
			ret.position.y = 0;
			ret.position.z = 0;
		}
	} else {
		const pose_type curr_pose = fast_pose.pose;
		ret.orientation.x = curr_pose.orientation.x();
		ret.orientation.y = curr_pose.orientation.y();
		ret.orientation.z = curr_pose.orientation.z();
		ret.orientation.w = curr_pose.orientation.w();
		ret.position.x = curr_pose.position.x();
		ret.position.y = curr_pose.position.y();
		ret.position.z = curr_pose.position.z();
	}

	return ret;
}

extern "C" void illixr_initialize_vulkan_display_service(VkInstance instance, VkPhysicalDevice physical_device,
                                                         VkDevice device, VkQueue queue, uint32_t queue_family_index,
                                                         struct u_string_list* enabled_instance_extensions,
                                                         struct u_string_list* enabled_device_extensions) {
	printf("Initializing vulkan display service\n");
	auto ds = std::make_shared<monado_vulkan_display_provider>();
	ds->vk_instance = instance;
	ds->vk_physical_device = physical_device;
	ds->vk_device = device;
	ds->queues[queue::GRAPHICS] = {queue, queue_family_index, queue::GRAPHICS, std::make_shared<std::mutex>()};

	const char* const * exts = u_string_list_get_data(enabled_instance_extensions);
	uint32_t ext_count = u_string_list_get_size(enabled_instance_extensions);

	for (uint32_t i = 0; i < ext_count; i++) {
		ds->enabled_instance_extensions.push_back(exts[i]);
	}

	const char* const * dev_exts = u_string_list_get_data(enabled_device_extensions);
	uint32_t dev_ext_count = u_string_list_get_size(enabled_device_extensions);

	for (uint32_t i = 0; i < dev_ext_count; i++) {
		ds->enabled_device_extensions.push_back(dev_exts[i]);
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
			image_arr[eye].image_info.format = illixr_plugin_obj->use_lossy_depth ? VK_FORMAT_B8G8R8A8_UNORM : VK_FORMAT_D32_SFLOAT;
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

extern "C" bool illixr_use_lossy_depth() {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	return illixr_plugin_obj->use_lossy_depth;
}

extern "C" bool illixr_offload_frames() {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
	return illixr_plugin_obj->offload_frames;
}

extern "C" void illixr_tw_update_uniforms(xrt_pose l_pose, xrt_pose r_pose) {
	assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");

	if (!illixr_plugin_obj->offload_frames) {
		pose_type pose {time_point{},
					Eigen::Vector3f {(l_pose.position.x + r_pose.position.x) / 2, (l_pose.position.y + r_pose.position.y) / 2, (l_pose.position.z + r_pose.position.z) / 2},
					Eigen::Quaternionf {(l_pose.orientation.w), (l_pose.orientation.x), (l_pose.orientation.y), (l_pose.orientation.z)}
					};
		illixr_plugin_obj->last_pose = pose;

		if (illixr_plugin_obj->dump_poses) {
			std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
			uint64_t milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - illixr_plugin_obj->pose_start_time).count();

			// Prints as: [time] [position] [orientation] without any metadata
			illixr_plugin_obj->pose_file << milliseconds << '\t'
				<< pose.position.x() << " " << pose.position.y() << " " << pose.position.z() << " " 
				<< pose.orientation.x() << " " << pose.orientation.y() << " " << pose.orientation.z() << " " << pose.orientation.w() << '\n';
			illixr_plugin_obj->pose_file.flush();
		}
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
		illixr_plugin_obj->_m_vsync.put(illixr_plugin_obj->_m_vsync.allocate<switchboard::event_wrapper<time_point>>(relative_time));
	}
}