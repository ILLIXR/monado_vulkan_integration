// Copyright 2020-2021, The Board of Trustees of the University of Illinois.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  ILLIXR HMD with hand tracking support
 * @author RSIM Group <illixr@cs.illinois.edu>
 * @ingroup drv_illixr
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <string>
#include <sstream>
#include <chrono>

// Platform-specific includes
#ifdef _WIN32
    #include <malloc.h>  // for alloca on Windows
    #include <io.h>      // Windows I/O functions
    // Windows doesn't have unistd.h or dlfcn.h
    // Dynamic library loading is handled by ILLIXR::dynamic_lib
#else
    #include <unistd.h>
    #include <dlfcn.h>
    #include <alloca.h>
#endif

#include "math/m_api.h"
#include "xrt/xrt_device.h"
#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_time.h"
#include "util/u_distortion_mesh.h"

// Include os_time.h for timestamp utilities if available
// This provides os_monotonic_get_ns() on supported platforms
#ifdef XRT_HAVE_TIMESPEC
#include "os/os_time.h"
#endif

#include "illixr_component.h"
#include "illixr/dynamic_lib.hpp"
#include "illixr/runtime.hpp"
#include "illixr/global_module_defs.hpp"

/*
 *
 * Structs and defines.
 *
 */

struct illixr_hmd
{
	struct xrt_device base;

	struct xrt_pose pose;

	bool print_spew;
	bool print_debug;

	const char *path;
	const char *comp;
	ILLIXR::dynamic_lib *runtime_lib;
	ILLIXR::runtime *runtime;

	// Hand tracking support
	bool hand_tracking_supported;
};


/*
 *
 * Helper Functions
 *
 */

/**
 * @brief Get current monotonic time in nanoseconds (portable implementation)
 */
static uint64_t
get_timestamp_ns(void)
{
#if defined(XRT_HAVE_TIMESPEC) && !defined(_WIN32)
	// Use Monado's utility if available on non-Windows
	return os_monotonic_get_ns();
#else
	// Portable fallback using C++ chrono
	auto now = std::chrono::steady_clock::now();
	auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
	return static_cast<uint64_t>(ns.count());
#endif
}


/*
 *
 * Functions
 *
 */

static inline struct illixr_hmd *
illixr_hmd(struct xrt_device *xdev)
{
	return (struct illixr_hmd *)xdev;
}

DEBUG_GET_ONCE_BOOL_OPTION(illixr_spew, "ILLIXR_PRINT_SPEW", false)
DEBUG_GET_ONCE_BOOL_OPTION(illixr_debug, "ILLIXR_PRINT_DEBUG", false)

#define DH_SPEW(dh, ...)                                                                                               \
	do {                                                                                                           \
		if (dh->print_spew) {                                                                                  \
			fprintf(stderr, "%s - ", __func__);                                                            \
			fprintf(stderr, __VA_ARGS__);                                                                  \
			fprintf(stderr, "\n");                                                                         \
		}                                                                                                      \
	} while (false)

#define DH_DEBUG(dh, ...)                                                                                              \
	do {                                                                                                           \
		if (dh->print_debug) {                                                                                 \
			fprintf(stderr, "%s - ", __func__);                                                            \
			fprintf(stderr, __VA_ARGS__);                                                                  \
			fprintf(stderr, "\n");                                                                         \
		}                                                                                                      \
	} while (false)

#define DH_ERROR(dh, ...)                                                                                              \
	do {                                                                                                           \
		fprintf(stderr, "%s - ", __func__);                                                                    \
		fprintf(stderr, __VA_ARGS__);                                                                          \
		fprintf(stderr, "\n");                                                                                 \
	} while (false)

static void
illixr_hmd_destroy(struct xrt_device *xdev)
{
	struct illixr_hmd *dh = illixr_hmd(xdev);
	dh->runtime->stop();
	delete dh->runtime;
	delete dh->runtime_lib;

	// Remove the variable tracking.
	u_var_remove_root(dh);

	u_device_free(&dh->base);
}

static void
illixr_hmd_update_inputs(struct xrt_device *xdev)
{
	// poses are fetched on demand
	(void)xdev;
}

static void
illixr_hmd_get_tracked_pose(struct xrt_device *xdev,
                            enum xrt_input_name name,
                            uint64_t at_timestamp_ns,
                            struct xrt_space_relation *out_relation)
{
	(void)at_timestamp_ns;
	struct illixr_hmd *dh = illixr_hmd(xdev);
	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		DH_ERROR(dh, "unknown input name for head pose");
		return;
	}

	out_relation->pose = illixr_read_pose();
	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | 
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	    XRT_SPACE_RELATION_POSITION_VALID_BIT | 
	    XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
}

/**
 * @brief Convert illixr_hand_joint to xrt_hand_joint_value
 * 
 * OpenXR XrSpaceLocationFlags:
 *   XR_SPACE_LOCATION_ORIENTATION_VALID_BIT  = 0x00000001
 *   XR_SPACE_LOCATION_POSITION_VALID_BIT     = 0x00000002
 *   XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT = 0x00000004
 *   XR_SPACE_LOCATION_POSITION_TRACKED_BIT    = 0x00000008
 */
static void
convert_illixr_joint_to_xrt(const struct illixr_hand_joint *src,
                            struct xrt_hand_joint_value *dst)
{
	// Position and orientation
	dst->relation.pose.position.x = src->position.x;
	dst->relation.pose.position.y = src->position.y;
	dst->relation.pose.position.z = src->position.z;

	dst->relation.pose.orientation.x = src->orientation.x;
	dst->relation.pose.orientation.y = src->orientation.y;
	dst->relation.pose.orientation.z = src->orientation.z;
	dst->relation.pose.orientation.w = src->orientation.w;

	// Radius
	dst->radius = src->radius;

	// Build relation flags from OpenXR-style location_flags
	enum xrt_space_relation_flags flags = (enum xrt_space_relation_flags)0;

	// OpenXR flag mapping (from XrSpaceLocationFlags)
	if (src->location_flags & 0x01) {  // XR_SPACE_LOCATION_ORIENTATION_VALID_BIT
		flags = (enum xrt_space_relation_flags)(flags | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);
	}
	if (src->location_flags & 0x02) {  // XR_SPACE_LOCATION_POSITION_VALID_BIT
		flags = (enum xrt_space_relation_flags)(flags | XRT_SPACE_RELATION_POSITION_VALID_BIT);
	}
	if (src->location_flags & 0x04) {  // XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT
		flags = (enum xrt_space_relation_flags)(flags | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
	}
	if (src->location_flags & 0x08) {  // XR_SPACE_LOCATION_POSITION_TRACKED_BIT
		flags = (enum xrt_space_relation_flags)(flags | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
	}

	// If no flags set but we have data, assume valid and tracked
	if (flags == 0) {
		flags = (enum xrt_space_relation_flags)(
		    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		    XRT_SPACE_RELATION_POSITION_VALID_BIT |
		    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
		    XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
	}

	// Velocities (if provided)
	dst->relation.linear_velocity.x = src->linear_velocity.x;
	dst->relation.linear_velocity.y = src->linear_velocity.y;
	dst->relation.linear_velocity.z = src->linear_velocity.z;

	dst->relation.angular_velocity.x = src->angular_velocity.x;
	dst->relation.angular_velocity.y = src->angular_velocity.y;
	dst->relation.angular_velocity.z = src->angular_velocity.z;

	// Check if velocities are non-zero, add flags
	if (src->linear_velocity.x != 0 || src->linear_velocity.y != 0 || src->linear_velocity.z != 0) {
		flags = (enum xrt_space_relation_flags)(flags | XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);
	}
	if (src->angular_velocity.x != 0 || src->angular_velocity.y != 0 || src->angular_velocity.z != 0) {
		flags = (enum xrt_space_relation_flags)(flags | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	}

	dst->relation.relation_flags = flags;
}

/**
 * @brief Get hand tracking data from ILLIXR
 */
static void
illixr_hmd_get_hand_tracking(struct xrt_device *xdev,
                             enum xrt_input_name name,
                             uint64_t desired_timestamp_ns,
                             struct xrt_hand_joint_set *out_value,
                             uint64_t *out_timestamp_ns)
{
	(void)desired_timestamp_ns;
	struct illixr_hmd *dh = illixr_hmd(xdev);

	// Debug: log that we're being called
	static uint64_t call_count = 0;
	call_count++;

	// Determine which hand
	int hand_index = -1;
	const char *hand_name = "unknown";
	
	if (name == XRT_INPUT_GENERIC_HAND_TRACKING_LEFT) {
		hand_index = 0;
		hand_name = "left";
	} else if (name == XRT_INPUT_GENERIC_HAND_TRACKING_RIGHT) {
		hand_index = 1;
		hand_name = "right";
	} else {
		DH_ERROR(dh, "unknown input name for hand tracking: %d", name);
		out_value->is_active = false;
		return;
	}

	// Log first few calls and then periodically
	if (call_count <= 5 || call_count % 300 == 0) {
		printf("[ILLIXR] get_hand_tracking called for %s hand (call #%llu)\n",
		       hand_name, (unsigned long long)call_count);
	}

	// Check if hand tracking is supported
	if (!dh->hand_tracking_supported) {
		if (call_count <= 5) {
			printf("[ILLIXR] Hand tracking not supported on device\n");
		}
		out_value->is_active = false;
		return;
	}

	// Get hand data from ILLIXR
	struct illixr_single_hand hand_data;
	if (!illixr_read_single_hand(hand_index, &hand_data)) {
		out_value->is_active = false;
		if (call_count <= 5 || call_count % 300 == 0) {
			printf("[ILLIXR] No hand data available for %s hand\n", hand_name);
		}
		return;
	}

	// Set the active state
	out_value->is_active = hand_data.is_active;
	if (!hand_data.is_active) {
		if (call_count <= 5 || call_count % 300 == 0) {
			printf("[ILLIXR] %s hand not active\n", hand_name);
		}
		return;
	}

	// Convert all joints
	for (int i = 0; i < XRT_HAND_JOINT_COUNT && i < ILLIXR_HAND_JOINT_COUNT; i++) {
		convert_illixr_joint_to_xrt(&hand_data.joints[i], &out_value->values.hand_joint_set_default[i]);
	}

	// Return the current timestamp using portable helper
	*out_timestamp_ns = get_timestamp_ns();

	// Log success periodically
	if (call_count <= 5 || call_count % 300 == 0) {
		printf("[ILLIXR] %s hand: ACTIVE, confidence=%.2f, wrist=(%.3f, %.3f, %.3f)\n",
		       hand_name, hand_data.confidence,
		       hand_data.joints[1].position.x,
		       hand_data.joints[1].position.y,
		       hand_data.joints[1].position.z);
	}
}

static void
illixr_hmd_get_view_poses(struct xrt_device *xdev,
                          const struct xrt_vec3 *default_eye_relation,
                          uint64_t at_timestamp_ns,
                          uint32_t view_count,
                          struct xrt_space_relation *out_head_relation,
                          struct xrt_fov *out_fovs,
                          struct xrt_pose *out_poses)
{
	u_device_get_view_poses(xdev, default_eye_relation, at_timestamp_ns, view_count, out_head_relation, out_fovs,
	                        out_poses);
}

std::vector<std::string>
split(const std::string &s, char delimiter)
{
	std::vector<std::string> tokens;
	std::string token;
	std::istringstream tokenStream{s};
	while (std::getline(tokenStream, token, delimiter)) {
		tokens.push_back(token);
	}
	return tokens;
}

uint32_t get_server_width() {
	if (std::getenv("ILLIXR_SERVER_WIDTH") == nullptr) {
		printf("[Monado] Display width not specified, defaulting to %d pixels.\n", ILLIXR::display_params::width_pixels);
		return ILLIXR::display_params::width_pixels;
	}

	return std::stoi(std::getenv("ILLIXR_SERVER_WIDTH"));
}

uint32_t get_server_height() {
	if (std::getenv("ILLIXR_SERVER_HEIGHT") == nullptr) {
		printf("[Monado] Display height not specified, defaulting to %d pixels.\n", ILLIXR::display_params::height_pixels);
		return ILLIXR::display_params::height_pixels;
	}

	return std::stoi(std::getenv("ILLIXR_SERVER_HEIGHT"));
}

static int
illixr_rt_launch(struct illixr_hmd *dh, const char *path, const char *comp)
{
	dh->runtime_lib = new ILLIXR::dynamic_lib{ILLIXR::dynamic_lib::create(std::string{path})};
	dh->runtime = dh->runtime_lib->get<ILLIXR::runtime *(*)()>("runtime_factory")();
#if defined(_WIN32) || defined(_WIN64)
	dh->runtime->load_so(split(std::string{comp}, ';'));
#else
	dh->runtime->load_so(split(std::string{comp}, ':'));
#endif
	dh->runtime->load_plugin_factory((ILLIXR::plugin_factory)illixr_monado_create_plugin);

	return 0;
}

extern "C" struct xrt_device *
illixr_hmd_create(const char *path_in, const char *comp_in)
{
	struct illixr_hmd *dh;
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	// Allocate with 3 inputs: head pose + left hand + right hand
	dh = U_DEVICE_ALLOCATE(struct illixr_hmd, flags, 3, 0);
	dh->base.update_inputs = illixr_hmd_update_inputs;
	dh->base.get_tracked_pose = illixr_hmd_get_tracked_pose;
	dh->base.get_view_poses = illixr_hmd_get_view_poses;
	dh->base.get_hand_tracking = illixr_hmd_get_hand_tracking;
	dh->base.destroy = illixr_hmd_destroy;
	dh->base.name = XRT_DEVICE_GENERIC_HMD;
	dh->base.device_type = XRT_DEVICE_TYPE_HMD;
	dh->base.orientation_tracking_supported = true;
	dh->base.position_tracking_supported = true;
	dh->base.hand_tracking_supported = true;

	// Read framerate from environment variable
	if (std::getenv("ILLIXR_OFFLOAD_RENDERING_FRAMERATE") != nullptr) {
		dh->base.hmd->screens[0].nominal_frame_interval_ns = 1000000000 / std::stoi(std::getenv("ILLIXR_OFFLOAD_RENDERING_FRAMERATE"));
	} else {
		dh->base.hmd->screens[0].nominal_frame_interval_ns = 1000000000 / 90;
	}

	size_t idx = 0;
	dh->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	dh->base.hmd->blend_mode_count = idx;

	dh->pose.orientation.w = 1.0f; // All other values set to zero.
	dh->print_spew = debug_get_bool_option_illixr_spew();
	dh->print_debug = debug_get_bool_option_illixr_debug();
	dh->path = path_in;
	dh->comp = comp_in;

	// Print name.
	snprintf(dh->base.str, XRT_DEVICE_NAME_LEN, "ILLIXR");
	snprintf(dh->base.serial, XRT_DEVICE_NAME_LEN, "ILLIXR");

	// Setup inputs: head pose + hand tracking
	dh->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	dh->base.inputs[1].name = XRT_INPUT_GENERIC_HAND_TRACKING_LEFT;
	dh->base.inputs[2].name = XRT_INPUT_GENERIC_HAND_TRACKING_RIGHT;

	// Setup info.
	struct u_device_simple_info info;
	info.display.w_pixels = get_server_width();
	info.display.h_pixels = get_server_height();
	info.display.w_meters = 0.122f;
	info.display.h_meters = 0.07f;
	info.lens_horizontal_separation_meters = 0.13f / 2.0f;
	info.lens_vertical_position_meters = 0.07f / 2.0f;
	info.fov[0] = 85.0f * (M_PI / 180.0f);
	info.fov[1] = 85.0f * (M_PI / 180.0f);

	if (!u_device_setup_split_side_by_side(&dh->base, &info)) {
		DH_ERROR(dh, "Failed to setup basic device info");
		illixr_hmd_destroy(&dh->base);
		return NULL;
	}

	// Read ILLIXR_OVERSCAN from environment variable
	float scale = 1.0f;
	if (std::getenv("ILLIXR_OVERSCAN") != nullptr) {
		scale = std::stof(std::getenv("ILLIXR_OVERSCAN"));
	}

	// The server may render at a different FOV than the client.
	for (int eye = 0; eye < 2; eye++) {
		float fov_left = scale * ILLIXR::server_params::fov_left[eye];
		float fov_right = scale * ILLIXR::server_params::fov_right[eye];
		float fov_up = scale * ILLIXR::server_params::fov_up[eye];
		float fov_down = scale * ILLIXR::server_params::fov_down[eye];

		dh->base.hmd->distortion.fov[eye].angle_left = fov_left;
		dh->base.hmd->distortion.fov[eye].angle_right = fov_right;
		dh->base.hmd->distortion.fov[eye].angle_up = fov_up;
		dh->base.hmd->distortion.fov[eye].angle_down = fov_down;
	}

	// Setup variable tracker.
	u_var_add_root(dh, "ILLIXR", true);
	u_var_add_pose(dh, &dh->pose, "pose");
	u_var_add_bool(dh, &dh->hand_tracking_supported, "hand_tracking_supported");

	if (dh->base.hmd->distortion.preferred == XRT_DISTORTION_MODEL_NONE) {
		// Setup the distortion mesh.
		u_distortion_mesh_set_none(&dh->base);
	}

	// start ILLIXR runtime
	if (illixr_rt_launch(dh, dh->path, dh->comp) != 0) {
		DH_ERROR(dh, "Failed to load ILLIXR Runtime");
		illixr_hmd_destroy(&dh->base);
		return NULL;
	}

	// Check environment variables directly for hand tracking support
	// We can't rely on illixr_hand_tracking_supported() here because the plugin
	// may not be fully initialized yet (load_plugin_factory registers but may not call)
	bool ht_enabled = false;
	
	// Check ILLIXR_USE_HAND_TRACKING first
	const char* ht_env = std::getenv("ILLIXR_USE_HAND_TRACKING");
	if (ht_env != nullptr) {
		std::string val(ht_env);
		ht_enabled = (val == "1" || val == "true" || val == "TRUE" || val == "yes" || val == "YES");
	} else {
		// Fall back to ILLIXR_OFFLOAD_FRAMES
		const char* offload_env = std::getenv("ILLIXR_OFFLOAD_FRAMES");
		if (offload_env != nullptr) {
			ht_enabled = (std::stoi(offload_env) != 0);
		}
	}
	
	dh->hand_tracking_supported = ht_enabled;
	dh->base.hand_tracking_supported = ht_enabled;

	printf("[ILLIXR] ==========================================\n");
	printf("[ILLIXR] HMD device created successfully\n");
	printf("[ILLIXR]   ILLIXR_USE_HAND_TRACKING=%s\n", 
	       std::getenv("ILLIXR_USE_HAND_TRACKING") ? std::getenv("ILLIXR_USE_HAND_TRACKING") : "(not set)");
	printf("[ILLIXR]   ILLIXR_OFFLOAD_FRAMES=%s\n",
	       std::getenv("ILLIXR_OFFLOAD_FRAMES") ? std::getenv("ILLIXR_OFFLOAD_FRAMES") : "(not set)");
	printf("[ILLIXR]   Hand tracking: %s\n", 
	       dh->hand_tracking_supported ? "ENABLED" : "DISABLED");
	printf("[ILLIXR]   get_hand_tracking callback: %p\n", 
	       (void*)dh->base.get_hand_tracking);
	printf("[ILLIXR]   hand_tracking_supported flag: %d\n",
	       dh->base.hand_tracking_supported);
	printf("[ILLIXR] ==========================================\n");

	return &dh->base;
}
