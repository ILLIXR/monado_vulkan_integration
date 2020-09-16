// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Main prober code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup st_prober
 */

#pragma once

#include "xrt/xrt_config_have.h"
#include "xrt/xrt_config_os.h"
#include "xrt/xrt_compiler.h"
#include "xrt/xrt_prober.h"
#include "xrt/xrt_settings.h"

#ifdef XRT_HAVE_LIBUSB
#include <libusb-1.0/libusb.h>
#endif

#ifdef XRT_HAVE_LIBUVC
#include <libuvc/libuvc.h>
#endif

#ifndef __KERNEL__
#include <sys/types.h>
#endif

/*
 *
 * Struct and defines
 *
 */

#define P_SPEW(p, ...)                                                         \
	do {                                                                   \
		if (p->print_spew) {                                           \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)

#define P_DEBUG(p, ...)                                                        \
	do {                                                                   \
		if (p->print_debug) {                                          \
			fprintf(stderr, "%s - ", __func__);                    \
			fprintf(stderr, __VA_ARGS__);                          \
			fprintf(stderr, "\n");                                 \
		}                                                              \
	} while (false)

#define P_ERROR(p, ...)                                                        \
	do {                                                                   \
		fprintf(stderr, "%s - ", __func__);                            \
		fprintf(stderr, __VA_ARGS__);                                  \
		fprintf(stderr, "\n");                                         \
	} while (false)

#define MAX_AUTO_PROBERS 8


#ifdef XRT_OS_LINUX
/*!
 * A hidraw interface that a @ref prober_device exposes.
 */
struct prober_hidraw
{
	ssize_t interface;
	const char *path;
};

/*!
 * A v4l interface that a @ref prober_device exposes.
 */
struct prober_v4l
{
	const char *path;
	int32_t usb_iface;
	uint32_t v4l_index;
};
#endif

/*!
 * A single device found by a @ref prober.
 *
 * @implements xrt_prober_device
 */
struct prober_device
{
	struct xrt_prober_device base;

	struct
	{
		uint16_t bus;
		uint16_t addr;

#ifdef XRT_OS_LINUX
		const char *product;
		const char *manufacturer;
		const char *serial;
		const char *path;
#endif

		uint8_t ports[8];
		uint32_t num_ports;

#ifdef XRT_HAVE_LIBUSB
		libusb_device *dev;
#endif
	} usb;

	struct
	{
		uint64_t id;
	} bluetooth;

#ifdef XRT_HAVE_LIBUVC
	struct
	{
		uvc_device_t *dev;
	} uvc;
#endif

#ifdef XRT_OS_LINUX
	size_t num_v4ls;
	struct prober_v4l *v4ls;

	size_t num_hidraws;
	struct prober_hidraw *hidraws;
#endif
};

/*!
 * @implements xrt_prober
 */
struct prober
{
	struct xrt_prober base;

	struct xrt_prober_entry_lists *lists;

	struct
	{
		//! For error reporting, was it loaded but not parsed?
		bool file_loaded;

		cJSON *root;
	} json;

#ifdef XRT_HAVE_LIBUSB
	struct
	{
		libusb_context *ctx;
		libusb_device **list;
		ssize_t count;
	} usb;
#endif

#ifdef XRT_HAVE_LIBUVC
	struct
	{
		uvc_context_t *ctx;
		uvc_device_t **list;
		ssize_t count;
	} uvc;
#endif

	struct xrt_auto_prober *auto_probers[MAX_AUTO_PROBERS];

	size_t num_devices;
	struct prober_device *devices;

	size_t num_entries;
	struct xrt_prober_entry **entries;

	bool print_debug;
	bool print_spew;
};


/*
 *
 * Functions.
 *
 */

/*!
 * Load the JSON config file.
 *
 * @public @memberof prober
 */
void
p_json_open_or_create_main_file(struct prober *p);

/*!
 * Extract tracking settings from the JSON.
 *
 * @public @memberof prober
 * @relatesalso xrt_settings_tracking
 */
bool
p_json_get_tracking_settings(struct prober *p, struct xrt_settings_tracking *s);

/*!
 * Dump the given device to stdout.
 *
 * @public @memberof prober
 */
void
p_dump_device(struct prober *p, struct prober_device *pdev, int id);

/*!
 * Get or create a @ref prober_device from the device.
 *
 * @public @memberof prober
 */
int
p_dev_get_usb_dev(struct prober *p,
                  uint16_t bus,
                  uint16_t addr,
                  uint16_t vendor_id,
                  uint16_t product_id,
                  struct prober_device **out_pdev);

/*!
 * Get or create a @ref prober_device from the device.
 *
 * @public @memberof prober
 */
int
p_dev_get_bluetooth_dev(struct prober *p,
                        uint64_t id,
                        uint16_t vendor_id,
                        uint16_t product_id,
                        struct prober_device **out_pdev);

/*!
 * @name Tracking systems
 * @{
 */
/*!
 * Init the tracking factory.
 *
 * @private @memberof prober
 * @relatesalso xrt_tracking_factory
 */
int
p_tracking_init(struct prober *p);

/*!
 * Teardown the tracking factory.
 *
 * @private @memberof prober
 * @relatesalso xrt_tracking_factory
 */
void
p_tracking_teardown(struct prober *p);

/*!
 * @}
 */

#ifdef XRT_HAVE_LIBUSB
/*!
 * @name libusb
 * @{
 */
/*!
 * @private @memberof prober
 */
int
p_libusb_init(struct prober *p);

/*!
 * @private @memberof prober
 */
void
p_libusb_teardown(struct prober *p);

/*!
 * @private @memberof prober
 */
int
p_libusb_probe(struct prober *p);

/*!
 * @private @memberof prober
 */
int
p_libusb_get_string_descriptor(struct prober *p,
                               struct prober_device *pdev,
                               enum xrt_prober_string which_string,
                               unsigned char *buffer,
                               int length);

/*!
 * @private @memberof prober
 */
bool
p_libusb_can_open(struct prober *p, struct prober_device *pdev);

/*!
 * @}
 */
#endif

#ifdef XRT_HAVE_LIBUVC
/*!
 * @name libuvc
 * @{
 */
/*!
 * @private @memberof prober
 */
int
p_libuvc_init(struct prober *p);

/*!
 * @private @memberof prober
 */
void
p_libuvc_teardown(struct prober *p);

/*!
 * @private @memberof prober
 */
int
p_libuvc_probe(struct prober *p);

/*!
 * @}
 */
#endif

#ifdef XRT_HAVE_LIBUDEV
/*!
 * @name udev
 * @{
 */
/*!
 * @private @memberof prober
 */
int
p_udev_probe(struct prober *p);
/*!
 * @}
 */
#endif
