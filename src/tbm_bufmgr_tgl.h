/**************************************************************************
 *
 * libtbm
 *
 * Copyright 2012 Samsung Electronics co., Ltd. All Rights Reserved.
 *
 * Contact: SooChan Lim <sc1.lim@samsung.com>, Sangjin Lee <lsj119@samsung.com>
 * Boram Park <boram1288.park@samsung.com>, Changyeon Lee <cyeon.lee@samsung.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * **************************************************************************/

#ifndef __TBM_BUFMGR_TGL_H__
#define __TBM_BUFMGR_TGL_H__

#include <linux/ioctl.h>

#ifdef ENABLE_CACHECRTL
static char tgl_devfile[] = "/dev/slp_global_lock";
static char tgl_devfile1[] = "/dev/tgl";
#endif

#define TGL_IOCTL_BASE		0x32
#define TGL_IO(nr)			_IO(TGL_IOCTL_BASE, nr)
#define TGL_IOR(nr, type)	_IOR(TGL_IOCTL_BASE, nr, type)
#define TGL_IOW(nr, type)	_IOW(TGL_IOCTL_BASE, nr, type)
#define TGL_IOWR(nr, type)	_IOWR(TGL_IOCTL_BASE, nr, type)

/**
 * struct tgl_ver_data - tgl version data structure
 * @major: major version
 * @minor: minor version
 */
struct tgl_ver_data {
	unsigned int major;
	unsigned int minor;
};

/**
 * struct tgl_reg_data - tgl  data structure
 * @key: lookup key
 * @timeout_ms: timeout value for waiting event
 */
struct tgl_reg_data {
	unsigned int key;
	unsigned int timeout_ms;
};

enum tgl_type_data {
	TGL_TYPE_NONE = 0,
	TGL_TYPE_READ = (1 << 0),
	TGL_TYPE_WRITE = (1 << 1),
};

/**
 * struct tgl_lock_data - tgl lock data structure
 * @key: lookup key
 * @type: lock type that is in tgl_type_data
 */
struct tgl_lock_data {
	unsigned int key;
	enum tgl_type_data type;
};

enum tgl_status_data {
	TGL_STATUS_UNLOCKED,
	TGL_STATUS_LOCKED,
};

/**
 * struct tgl_usr_data - tgl user data structure
 * @key: lookup key
 * @data1: user data 1
 * @data2: user data 2
 * @status: lock status that is in tgl_status_data
 */
struct tgl_usr_data {
	unsigned int key;
	unsigned int data1;
	unsigned int data2;
	enum tgl_status_data status;
};

enum {
	_TGL_GET_VERSION,
	_TGL_REGISTER,
	_TGL_UNREGISTER,
	_TGL_LOCK,
	_TGL_UNLOCK,
	_TGL_SET_DATA,
	_TGL_GET_DATA,
};

/* get version information */
#define TGL_IOCTL_GET_VERSION	TGL_IOR(_TGL_GET_VERSION, struct tgl_ver_data)
/* register key */
#define TGL_IOCTL_REGISTER		TGL_IOW(_TGL_REGISTER, struct tgl_reg_data)
/* unregister key */
#define TGL_IOCTL_UNREGISTER	TGL_IOW(_TGL_UNREGISTER, struct tgl_reg_data)
/* lock with key */
#define TGL_IOCTL_LOCK			TGL_IOW(_TGL_LOCK, struct tgl_lock_data)
/* unlock with key */
#define TGL_IOCTL_UNLOCK		TGL_IOW(_TGL_UNLOCK, struct tgl_lock_data)
/* set user data with key */
#define TGL_IOCTL_SET_DATA		TGL_IOW(_TGL_SET_DATA, struct tgl_usr_data)
/* get user data with key */
#define TGL_IOCTL_GET_DATA		TGL_IOR(_TGL_GET_DATA, struct tgl_usr_data)

#ifdef ENABLE_CACHECRTL
/* indicate cache units. */
enum e_drm_exynos_gem_cache_sel {
	EXYNOS_DRM_L1_CACHE		= 1 << 0,
	EXYNOS_DRM_L2_CACHE		= 1 << 1,
	EXYNOS_DRM_ALL_CORES		= 1 << 2,
	EXYNOS_DRM_ALL_CACHES		= EXYNOS_DRM_L1_CACHE |
						EXYNOS_DRM_L2_CACHE,
	EXYNOS_DRM_ALL_CACHES_CORES	= EXYNOS_DRM_L1_CACHE |
						EXYNOS_DRM_L2_CACHE |
						EXYNOS_DRM_ALL_CORES,
	EXYNOS_DRM_CACHE_SEL_MASK	= EXYNOS_DRM_ALL_CACHES_CORES
};

/* indicate cache operation types. */
enum e_drm_exynos_gem_cache_op {
	EXYNOS_DRM_CACHE_INV_ALL	= 1 << 3,
	EXYNOS_DRM_CACHE_INV_RANGE	= 1 << 4,
	EXYNOS_DRM_CACHE_CLN_ALL	= 1 << 5,
	EXYNOS_DRM_CACHE_CLN_RANGE	= 1 << 6,
	EXYNOS_DRM_CACHE_FSH_ALL	= EXYNOS_DRM_CACHE_INV_ALL |
						EXYNOS_DRM_CACHE_CLN_ALL,
	EXYNOS_DRM_CACHE_FSH_RANGE	= EXYNOS_DRM_CACHE_INV_RANGE |
						EXYNOS_DRM_CACHE_CLN_RANGE,
	EXYNOS_DRM_CACHE_OP_MASK	= EXYNOS_DRM_CACHE_FSH_ALL |
						EXYNOS_DRM_CACHE_FSH_RANGE
};

/**
 * A structure for cache operation.
 *
 * @usr_addr: user space address.
 *	P.S. it SHOULD BE user space.
 * @size: buffer size for cache operation.
 * @flags: select cache unit and cache operation.
 * @gem_handle: a handle to a gem object.
 *	this gem handle is needed for cache range operation to L2 cache.
 */
struct drm_exynos_gem_cache_op {
	uint64_t usr_addr;
	unsigned int size;
	unsigned int flags;
	unsigned int gem_handle;
};

#define DRM_EXYNOS_GEM_CACHE_OP		0x12

#define DRM_IOCTL_EXYNOS_GEM_CACHE_OP  DRM_IOWR(DRM_COMMAND_BASE + \
		DRM_EXYNOS_GEM_CACHE_OP, struct drm_exynos_gem_cache_op)

#endif

#endif							/* __TBM_BUFMGR_TGL_H__ */
