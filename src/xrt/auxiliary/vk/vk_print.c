// Copyright 2019-2022, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Printing helper code.
 *
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Christoph Haag <christoph.haag@collabora.com>
 * @ingroup aux_vk
 */

#include "vk/vk_helpers.h"


void
vk_print_device_info(struct vk_bundle *vk,
                     enum u_logging_level log_level,
                     VkPhysicalDeviceProperties *pdp,
                     uint32_t gpu_index,
                     const char *title)
{
	U_LOG_IFL(log_level, vk->log_level,
	          "%s"
	          "\tname: %s\n"
	          "\tvendor: 0x%04x\n"
	          "\tproduct: 0x%04x\n"
	          "\tapiVersion: %u.%u.%u\n"
	          "\tdriverVersion: 0x%08x",
	          title,                             //
	          pdp->deviceName,                   //
	          pdp->vendorID,                     //
	          pdp->deviceID,                     //
	          VK_VERSION_MAJOR(pdp->apiVersion), //
	          VK_VERSION_MINOR(pdp->apiVersion), //
	          VK_VERSION_PATCH(pdp->apiVersion), //
	          pdp->driverVersion);               // Driver specific
}

void
vk_print_opened_device_info(struct vk_bundle *vk, enum u_logging_level log_level)
{
	VkPhysicalDeviceProperties pdp;
	vk->vkGetPhysicalDeviceProperties(vk->physical_device, &pdp);

	vk_print_device_info(vk, log_level, &pdp, 0, "Device info:\n");
	vk_print_device_extensions_info(vk, log_level);
}

void
vk_print_features_info(struct vk_bundle *vk, enum u_logging_level log_level)
{
	U_LOG_IFL(log_level, vk->log_level,                                       //
	          "Features:"                                                     //
	          "\n\ttimestamp_compute_and_graphics: %s"                        //
	          "\n\ttimestamp_period: %f"                                      //
	          "\n\ttimestamp_valid_bits: %u"                                  //
	          "\n\ttimeline_semaphore: %s",                                   //
	          vk->features.timestamp_compute_and_graphics ? "true" : "false", //
	          vk->features.timestamp_period,                                  //
	          vk->features.timestamp_valid_bits,                              //
	          vk->features.timeline_semaphore ? "true" : "false");            //
}

void
vk_print_external_handles_info(struct vk_bundle *vk, enum u_logging_level log_level)
{

#if defined(XRT_GRAPHICS_BUFFER_HANDLE_IS_WIN32_HANDLE)

	U_LOG_IFL(log_level, vk->log_level,                                               //
	          "Supported images:"                                                     //
	          "\n\t%s:\n\t\tcolor import=%s export=%s\n\t\tdepth import=%s export=%s" //
	          "\n\t%s:\n\t\tcolor import=%s export=%s\n\t\tdepth import=%s export=%s" //
	          ,                                                                       //
	          "VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT",                      //
	          vk->external.color_image_import_opaque_win32 ? "true" : "false",        //
	          vk->external.color_image_export_opaque_win32 ? "true" : "false",        //
	          vk->external.depth_image_import_opaque_win32 ? "true" : "false",        //
	          vk->external.depth_image_export_opaque_win32 ? "true" : "false",        //
	          "VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D11_TEXTURE_BIT",                     //
	          vk->external.color_image_import_d3d11 ? "true" : "false",               //
	          vk->external.color_image_export_d3d11 ? "true" : "false",               //
	          vk->external.depth_image_import_d3d11 ? "true" : "false",               //
	          vk->external.depth_image_export_d3d11 ? "true" : "false"                //
	);                                                                                //

#elif defined(XRT_GRAPHICS_BUFFER_HANDLE_IS_FD)

	U_LOG_IFL(log_level, vk->log_level,                                               //
	          "Supported images:"                                                     //
	          "\n\t%s:\n\t\tcolor import=%s export=%s\n\t\tdepth import=%s export=%s" //
	          ,                                                                       //
	          "VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT",                         //
	          vk->external.color_image_import_opaque_fd ? "true" : "false",           //
	          vk->external.color_image_export_opaque_fd ? "true" : "false",           //
	          vk->external.depth_image_import_opaque_fd ? "true" : "false",           //
	          vk->external.depth_image_export_opaque_fd ? "true" : "false"            //
	);                                                                                //


#elif defined(XRT_GRAPHICS_BUFFER_HANDLE_IS_AHARDWAREBUFFER)

	U_LOG_IFL(log_level, vk->log_level,                                               //
	          "Supported images:"                                                     //
	          "\n\t%s:\n\t\tcolor import=%s export=%s\n\t\tdepth import=%s export=%s" //
	          "\n\t%s:\n\t\tcolor import=%s export=%s\n\t\tdepth import=%s export=%s" //
	          ,                                                                       //
	          "VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT",                         //
	          vk->external.color_image_import_opaque_fd ? "true" : "false",           //
	          vk->external.color_image_export_opaque_fd ? "true" : "false",           //
	          vk->external.depth_image_import_opaque_fd ? "true" : "false",           //
	          vk->external.depth_image_export_opaque_fd ? "true" : "false",           //
	          "VK_EXTERNAL_MEMORY_HANDLE_TYPE_ANDROID_HARDWARE_BUFFER_BIT_ANDROID",   //
	          vk->external.color_image_import_ahardwarebuffer ? "true" : "false",     //
	          vk->external.color_image_export_ahardwarebuffer ? "true" : "false",     //
	          vk->external.depth_image_import_ahardwarebuffer ? "true" : "false",     //
	          vk->external.depth_image_export_ahardwarebuffer ? "true" : "false"      //
	);                                                                                //

#endif

#if defined(XRT_GRAPHICS_SYNC_HANDLE_IS_FD)

	U_LOG_IFL(log_level, vk->log_level,                         //
	          "Supported fences:\n\t%s: %s\n\t%s: %s",          //
	          "VK_EXTERNAL_FENCE_HANDLE_TYPE_SYNC_FD_BIT",      //
	          vk->external.fence_sync_fd ? "true" : "false",    //
	          "VK_EXTERNAL_FENCE_HANDLE_TYPE_OPAQUE_FD_BIT",    //
	          vk->external.fence_opaque_fd ? "true" : "false"); //

	U_LOG_IFL(log_level, vk->log_level,                                        //
	          "Supported semaphores:\n\t%s: %s\n\t%s: %s\n\t%s: %s\n\t%s: %s", //
	          "VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_SYNC_FD_BIT(binary)",         //
	          vk->external.binary_semaphore_sync_fd ? "true" : "false",        //
	          "VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT(binary)",       //
	          vk->external.binary_semaphore_opaque_fd ? "true" : "false",      //
	          "VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_SYNC_FD_BIT(timeline)",       //
	          vk->external.timeline_semaphore_sync_fd ? "true" : "false",      //
	          "VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT(timeline)",     //
	          vk->external.timeline_semaphore_opaque_fd ? "true" : "false");   //

#elif defined(XRT_GRAPHICS_SYNC_HANDLE_IS_WIN32_HANDLE)

	U_LOG_IFL(log_level, vk->log_level,                            //
	          "Supported fences:\n\t%s: %s",                       //
	          "VK_EXTERNAL_FENCE_HANDLE_TYPE_OPAQUE_WIN32_BIT",    //
	          vk->external.fence_win32_handle ? "true" : "false"); //

	U_LOG_IFL(log_level, vk->log_level,                                         //
	          "Supported semaphores:\n\t%s: %s\n\t%s: %s\n\t%s: %s\n\t%s: %s",  //
	          "VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_D3D12_FENCE_BIT(binary)",      //
	          vk->external.binary_semaphore_d3d12_fence ? "true" : "false",     //
	          "VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT(binary)",     //
	          vk->external.binary_semaphore_win32_handle ? "true" : "false",    //
	          "VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_D3D12_FENCE_BIT(timeline)",    //
	          vk->external.timeline_semaphore_d3d12_fence ? "true" : "false",   //
	          "VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_WIN32_BIT(timeline)",   //
	          vk->external.timeline_semaphore_win32_handle ? "true" : "false"); //

#else
#error "Need port for fence sync handles printers"
#endif
}

void
vk_print_device_extensions_info(struct vk_bundle *vk,
                     enum u_logging_level log_level)
{
	U_LOG_IFL(log_level, vk->log_level,
			"Supported Device Extensions:\n"
			"\tVK_KHR_external_fence_fd: %s\n"
			"\tVK_KHR_external_semaphore_fd: %s\n"
			"\tVK_KHR_format_feature_flags2: %s\n"
			"\tVK_KHR_global_priority: %s\n"
			"\tVK_KHR_image_format_list: %s\n"
			"\tVK_KHR_maintenance1: %s\n"
			"\tVK_KHR_maintenance2: %s\n"
			"\tVK_KHR_maintenance3: %s\n"
			"\tVK_KHR_maintenance4: %s\n"
			"\tVK_KHR_timeline_semaphore: %s\n"
			"\tVK_EXT_calibrated_timestamps: %s\n"
			"\tVK_EXT_display_control: %s\n"
			"\tVK_EXT_external_memory_dma_buf: %s\n"
			"\tVK_EXT_global_priority: %s\n"
			"\tVK_EXT_image_drm_format_modifier: %s\n"
			"\tVK_EXT_robustness2: %s\n"
			"\tVK_GOOGLE_display_timing: %s\n",
			vk->has_KHR_external_fence_fd ? "Supported" : "Not Supported",
			vk->has_KHR_external_semaphore_fd ? "Supported" : "Not Supported",
			vk->has_KHR_format_feature_flags2 ? "Supported" : "Not Supported",
			vk->has_KHR_global_priority ? "Supported" : "Not Supported",
			vk->has_KHR_image_format_list ? "Supported" : "Not Supported",
			vk->has_KHR_maintenance1 ? "Supported" : "Not Supported",
			vk->has_KHR_maintenance2 ? "Supported" : "Not Supported",
			vk->has_KHR_maintenance3 ? "Supported" : "Not Supported",
			vk->has_KHR_maintenance4 ? "Supported" : "Not Supported",
			vk->has_KHR_timeline_semaphore ? "Supported" : "Not Supported",
			vk->has_KHR_video_queue ? "Supported" : "Not Supported",
			vk->has_KHR_video_encode_queue ? "Supported" : "Not Supported",
			vk->has_EXT_video_encode_h264 ? "Supported" : "Not Supported",
			vk->has_EXT_calibrated_timestamps ? "Supported" : "Not Supported",
			vk->has_EXT_display_control ? "Supported" : "Not Supported",
			vk->has_EXT_external_memory_dma_buf ? "Supported" : "Not Supported",
			vk->has_EXT_global_priority ? "Supported" : "Not Supported",
			vk->has_EXT_image_drm_format_modifier ? "Supported" : "Not Supported",
			vk->has_EXT_robustness2 ? "Supported" : "Not Supported",
			vk->has_GOOGLE_display_timing ? "Supported" : "Not Supported"
			);
}