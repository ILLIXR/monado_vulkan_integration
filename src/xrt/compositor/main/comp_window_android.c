// Copyright 2019-2020, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Android window code.
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup comp_main
 */

#include <errno.h>
#include <linux/input.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include "xrt/xrt_compiler.h"
#include "main/comp_window.h"
#include "util/u_misc.h"
#include "android/android_globals.h"
#include "android/android_custom_surface.h"

#include <android/native_window.h>


/*
 *
 * Private structs.
 *
 */

/*!
 * An Android window.
 *
 * @implements comp_target_swapchain
 */
struct comp_window_android
{
	struct comp_target_swapchain base;

	struct android_custom_surface *custom_surface;
};

/*
 *
 * Functions.
 *
 */

static inline struct vk_bundle *
get_vk(struct comp_window_android *cwa)
{
	return &cwa->base.base.c->vk;
}

static bool
comp_window_android_init(struct comp_target *ct)
{
	(void)ct;

	return true;
}

static void
comp_window_android_destroy(struct comp_target *ct)
{
	struct comp_window_android *cwa = (struct comp_window_android *)ct;

	comp_target_swapchain_cleanup(&cwa->base);

	android_custom_surface_destroy(&cwa->custom_surface);

	free(ct);
}

static void
comp_window_android_update_window_title(struct comp_target *ct,
                                        const char *title)
{
	(void)ct;
}

static VkResult
comp_window_android_create_surface(struct comp_window_android *w,
                                   VkSurfaceKHR *vk_surface)
{
	struct vk_bundle *vk = get_vk(w);
	VkResult ret;

	w->custom_surface = android_custom_surface_async_start(
	    android_globals_get_vm(), android_globals_get_activity());
	if (w->custom_surface == NULL) {
		COMP_ERROR(
		    w->base.base.c,
		    "comp_window_android_create_surface: could not "
		    "start asynchronous attachment of our custom surface");
		return VK_ERROR_INITIALIZATION_FAILED;
	}
	struct ANativeWindow *window =
	    android_custom_surface_wait_get_surface(w->custom_surface, 2000);
	if (window == NULL) {
		COMP_ERROR(w->base.base.c,
		           "comp_window_android_create_surface: could not "
		           "convert surface to ANativeWindow");
		return VK_ERROR_INITIALIZATION_FAILED;
	}
	VkAndroidSurfaceCreateInfoKHR surface_info = {
	    .sType = VK_STRUCTURE_TYPE_ANDROID_SURFACE_CREATE_INFO_KHR,
	    .flags = 0,
	    .window = window,
	};

	ret = vk->vkCreateAndroidSurfaceKHR(vk->instance, &surface_info, NULL,
	                                    vk_surface);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(w->base.base.c, "vkCreateAndroidSurfaceKHR: %s",
		           vk_result_string(ret));
		return ret;
	}

	return VK_SUCCESS;
}

static bool
comp_window_android_init_swapchain(struct comp_target *ct,
                                   uint32_t width,
                                   uint32_t height)
{
	struct comp_window_android *w_android =
	    (struct comp_window_android *)ct;
	VkResult ret;

	ret = comp_window_android_create_surface( //
	    w_android,                            //
	    &w_android->base.surface.handle);     //
	if (ret != VK_SUCCESS) {
		COMP_ERROR(ct->c, "Failed to create surface!");
		return false;
	}

	return true;
}


static void
comp_window_android_flush(struct comp_target *ct)
{
	(void)ct;
}

struct comp_target *
comp_window_android_create(struct comp_compositor *c)
{
	struct comp_window_android *w =
	    U_TYPED_CALLOC(struct comp_window_android);

	comp_target_swapchain_init_set_fnptrs(&w->base);

	w->base.base.name = "Android";
	w->base.base.destroy = comp_window_android_destroy;
	w->base.base.flush = comp_window_android_flush;
	w->base.base.init_pre_vulkan = comp_window_android_init;
	w->base.base.init_post_vulkan = comp_window_android_init_swapchain;
	w->base.base.set_title = comp_window_android_update_window_title;
	w->base.base.c = c;

	return &w->base.base;
}
