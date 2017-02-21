/**************************************************************************

libtbm_exynos

Copyright 2012 Samsung Electronics co., Ltd. All Rights Reserved.

Contact: SooChan Lim <sc1.lim@samsung.com>, Sangjin Lee <lsj119@samsung.com>

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sub license, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice (including the
next paragraph) shall be included in all copies or substantial portions
of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
IN NO EVENT SHALL PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR
ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <xf86drm.h>
#include <tbm_bufmgr.h>
#include <tbm_bufmgr_backend.h>
#include <exynos_drm.h>
#include <pthread.h>
#include <tbm_surface.h>
#include <tbm_surface_internal.h>
#include <tbm_drm_helper.h>

#include <libudev.h>

#include "tbm_bufmgr_tgl.h"
#include <vc4_drm.h>

#define DEBUG
#define USE_DMAIMPORT
#define TBM_COLOR_FORMAT_COUNT 4

#define BCM_DRM_NAME "bcm2837"

#ifdef DEBUG
#define LOG_TAG	"TBM_BACKEND"
#include <dlog.h>
static int bDebug;

char *target_name()
{
	FILE *f;
	char *slash;
	static int initialized = 0;
	static char app_name[128];

	if (initialized)
		return app_name;

	/* get the application name */
	f = fopen("/proc/self/cmdline", "r");

	if (!f)
		return 0;

	memset(app_name, 0x00, sizeof(app_name));

	if (fgets(app_name, 100, f) == NULL) {
		fclose(f);
		return 0;
	}

	fclose(f);

	slash = strrchr(app_name, '/');
	if (slash != NULL)
		memmove(app_name, slash + 1, strlen(slash));

	initialized = 1;

	return app_name;
}

#define TBM_BCM_ERROR(fmt, args...)	LOGE("\033[31m"  "[%s] " fmt "\033[0m", target_name(), ##args)
#define TBM_BCM_DEBUG(fmt, args...)	{if (bDebug&01) LOGD("[%s] " fmt, target_name(), ##args); }
#else
#define TBM_BCM_ERROR(...)
#define TBM_BCM_DEBUG(...)
#endif

#define SIZE_ALIGN(value, base) (((value) + ((base) - 1)) & ~((base) - 1))
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#ifdef ALIGN_EIGHT
#define TBM_SURFACE_ALIGNMENT_PLANE (8)
#define TBM_SURFACE_ALIGNMENT_PITCH_RGB (8)
#else
#define TBM_SURFACE_ALIGNMENT_PLANE (64)
#define TBM_SURFACE_ALIGNMENT_PITCH_RGB (64)
#endif

#define TBM_SURFACE_ALIGNMENT_PLANE_NV12 (4096)
#define TBM_SURFACE_ALIGNMENT_PITCH_YUV (16)

#define SZ_1M                                   0x00100000
#define S5P_FIMV_MAX_FRAME_SIZE                 (2 * SZ_1M)
#define S5P_FIMV_D_ALIGN_PLANE_SIZE             64
#define S5P_FIMV_NUM_PIXELS_IN_MB_ROW           16
#define S5P_FIMV_NUM_PIXELS_IN_MB_COL           16
#define S5P_FIMV_DEC_BUF_ALIGN                  (8 * 1024)
#define S5P_FIMV_NV12MT_HALIGN                  128
#define S5P_FIMV_NV12MT_VALIGN                  64

/* check condition */
#define BCM_RETURN_IF_FAIL(cond) {\
	if (!(cond)) {\
		TBM_BCM_ERROR("[%s] : '%s' failed.\n", __func__, #cond);\
		return;\
	} \
}

#define BCM_RETURN_VAL_IF_FAIL(cond, val) {\
	if (!(cond)) {\
		TBM_BCM_ERROR("[%s] : '%s' failed.\n", __func__, #cond);\
		return val;\
	} \
}

struct dma_buf_info {
	unsigned long	size;
	unsigned int	fence_supported;
	unsigned int	padding;
};

#define DMA_BUF_ACCESS_READ		0x1
#define DMA_BUF_ACCESS_WRITE		0x2
#define DMA_BUF_ACCESS_DMA		0x4
#define DMA_BUF_ACCESS_MAX		0x8

#define DMA_FENCE_LIST_MAX		5

struct dma_buf_fence {
	unsigned long		ctx;
	unsigned int		type;
};

#define DMABUF_IOCTL_BASE	'F'
#define DMABUF_IOWR(nr, type)	_IOWR(DMABUF_IOCTL_BASE, nr, type)

#define DMABUF_IOCTL_GET_INFO	DMABUF_IOWR(0x00, struct dma_buf_info)
#define DMABUF_IOCTL_GET_FENCE	DMABUF_IOWR(0x01, struct dma_buf_fence)
#define DMABUF_IOCTL_PUT_FENCE	DMABUF_IOWR(0x02, struct dma_buf_fence)

/* tgl key values */
#define GLOBAL_KEY   ((unsigned int)(-1))
/* TBM_CACHE */
#define TBM_EXYNOS_CACHE_INV       0x01 /**< cache invalidate  */
#define TBM_EXYNOS_CACHE_CLN       0x02 /**< cache clean */
#define TBM_EXYNOS_CACHE_ALL       0x10 /**< cache all */
#define TBM_EXYNOS_CACHE_FLUSH     (TBM_EXYNOS_CACHE_INV|TBM_EXYNOS_CACHE_CLN) /**< cache flush  */
#define TBM_EXYNOS_CACHE_FLUSH_ALL (TBM_EXYNOS_CACHE_FLUSH|TBM_EXYNOS_CACHE_ALL)	/**< cache flush all */

enum {
	DEVICE_NONE = 0,
	DEVICE_CA,					/* cache aware device */
	DEVICE_CO					/* cache oblivious device */
};

typedef union _tbm_bo_cache_state tbm_bo_cache_state;

union _tbm_bo_cache_state {
	unsigned int val;
	struct {
		unsigned int cntFlush:16;	/*Flush all index for sync */
		unsigned int isCached:1;
		unsigned int isDirtied:2;
	} data;
};

typedef struct _tbm_bufmgr_bcm *tbm_bufmgr_bcm;
typedef struct _tbm_bo_bcm *tbm_bo_bcm;

typedef struct _bcm_private {
	int ref_count;
	struct _tbm_bo_bcm *bo_priv;
} PrivGem;

/* tbm buffor object for exynos */
struct _tbm_bo_bcm {
	int fd;

	unsigned int name;    /* FLINK ID */

	unsigned int gem;     /* GEM Handle */

	unsigned int dmabuf;  /* fd for dmabuf */

	void *pBase;          /* virtual address */

	unsigned int size;

	unsigned int flags_tbm; /*not used now*//*currently no values for the flags,but it may be used in future extension*/

	PrivGem *private;

	pthread_mutex_t mutex;
	struct dma_buf_fence dma_fence[DMA_FENCE_LIST_MAX];
	int device;
	int opt;

	tbm_bo_cache_state cache_state;
	unsigned int map_cnt;
	int last_map_device;
};

/* tbm bufmgr private for exynos */
struct _tbm_bufmgr_bcm {
	int fd;
	int isLocal;
	void *hashBos;

	int use_dma_fence;

	int tgl_fd;

	char *device_name;
	void *bind_display;
};

char *STR_DEVICE[] = {
	"DEF",
	"CPU",
	"2D",
	"3D",
	"MM"
};

char *STR_OPT[] = {
	"NONE",
	"RD",
	"WR",
	"RDWR"
};


uint32_t tbm_bcm_color_format_list[TBM_COLOR_FORMAT_COUNT] = {
										TBM_FORMAT_ARGB8888,
										TBM_FORMAT_XRGB8888,
										TBM_FORMAT_NV12,
										TBM_FORMAT_YUV420
									};
#undef  ENABLE_CACHECRTL
#ifdef ENABLE_CACHECRTL
#ifdef TGL_GET_VERSION
static inline int
_tgl_get_version(int fd)
{
	struct tgl_ver_data data;
	int err;

	err = ioctl(fd, TGL_IOCTL_GET_VERSION, &data);
	if (err) {
		TBM_BCM_ERROR("error(%s) %s:%d\n", strerror(errno));
		return 0;
	}

	TBM_BCM_DEBUG("tgl version is (%u, %u).\n", data.major, data.minor);

	return 1;
}
#endif

static inline int
_tgl_init(int fd, unsigned int key)
{
	struct tgl_reg_data data;
	int err;

	data.key = key;
	data.timeout_ms = 1000;

	err = ioctl(fd, TGL_IOCTL_REGISTER, &data);
	if (err) {
		TBM_BCM_ERROR("error(%s) key:%d\n", strerror(errno), key);
		return 0;
	}

	return 1;
}

static inline int
_tgl_destroy(int fd, unsigned int key)
{
	struct tgl_reg_data data;
	int err;

	data.key = key;
	err = ioctl(fd, TGL_IOCTL_UNREGISTER, &data);
	if (err) {
		TBM_BCM_ERROR("error(%s) key:%d\n", strerror(errno), key);
		return 0;
	}

	return 1;
}

static inline int
_tgl_lock(int fd, unsigned int key, int opt)
{
	struct tgl_lock_data data;
	enum tgl_type_data tgl_type;
	int err;

	switch (opt) {
	case TBM_OPTION_READ:
		tgl_type = TGL_TYPE_READ;
		break;
	case TBM_OPTION_WRITE:
		tgl_type = TGL_TYPE_WRITE;
		break;
	default:
		tgl_type = TGL_TYPE_NONE;
		break;
	}

	data.key = key;
	data.type = tgl_type;

	err = ioctl(fd, TGL_IOCTL_LOCK, &data);
	if (err) {
		TBM_BCM_ERROR("error(%s) key:%d opt:%d\n",
			strerror(errno), key, opt);
		return 0;
	}

	return 1;
}

static inline int
_tgl_unlock(int fd, unsigned int key)
{
	struct tgl_lock_data data;
	int err;

	data.key = key;
	data.type = TGL_TYPE_NONE;

	err = ioctl(fd, TGL_IOCTL_UNLOCK, &data);
	if (err) {
		TBM_BCM_ERROR("error(%s) key:%d\n",
			strerror(errno), key);
		return 0;
	}

	return 1;
}

static inline int
_tgl_set_data(int fd, unsigned int key, unsigned int val)
{
	struct tgl_usr_data data;
	int err;

	data.key = key;
	data.data1 = val;

	err = ioctl(fd, TGL_IOCTL_SET_DATA, &data);
	if (err) {
		TBM_BCM_ERROR("error(%s) key:%d\n",
			strerror(errno), key);
		return 0;
	}

	return 1;
}

static inline unsigned int
_tgl_get_data(int fd, unsigned int key)
{
	struct tgl_usr_data data = { 0, };
	int err;

	data.key = key;

	err = ioctl(fd, TGL_IOCTL_GET_DATA, &data);
	if (err) {
		TBM_BCM_ERROR("error(%s) key:%d\n",
			strerror(errno), key);
		return 0;
	}

	return data.data1;
}

static int
_exynos_cache_flush(tbm_bufmgr_bcm bufmgr_bcm, tbm_bo_bcm bo_bcm, int flags)
{
	BCM_RETURN_VAL_IF_FAIL(bufmgr_bcm != NULL, 0);

	/* cache flush is managed by kernel side when using dma-fence. */
	if (bufmgr_bcm->use_dma_fence)
		return 1;

	struct drm_exynos_gem_cache_op cache_op = {0, };
	int ret;

	/* if bo_bcm is null, do cache_flush_all */
	if (bo_bcm) {
		cache_op.flags = 0;
		cache_op.usr_addr = (uint64_t)((uint32_t)bo_bcm->pBase);
		cache_op.size = bo_bcm->size;
	} else {
		flags = TBM_EXYNOS_CACHE_FLUSH_ALL;
		cache_op.flags = 0;
		cache_op.usr_addr = 0;
		cache_op.size = 0;
	}

	if (flags & TBM_EXYNOS_CACHE_INV) {
		if (flags & TBM_EXYNOS_CACHE_ALL)
			cache_op.flags |= EXYNOS_DRM_CACHE_INV_ALL;
		else
			cache_op.flags |= EXYNOS_DRM_CACHE_INV_RANGE;
	}

	if (flags & TBM_EXYNOS_CACHE_CLN) {
		if (flags & TBM_EXYNOS_CACHE_ALL)
			cache_op.flags |= EXYNOS_DRM_CACHE_CLN_ALL;
		else
			cache_op.flags |= EXYNOS_DRM_CACHE_CLN_RANGE;
	}

	if (flags & TBM_EXYNOS_CACHE_ALL)
		cache_op.flags |= EXYNOS_DRM_ALL_CACHES_CORES;

	ret = drmCommandWriteRead(bufmgr_bcm->fd, DRM_EXYNOS_GEM_CACHE_OP, &cache_op,
				  sizeof(cache_op));
	if (ret) {
		TBM_BCM_ERROR("fail to flush the cache.\n");
		return 0;
	}

	return 1;
}
#endif

static int
_bo_init_cache_state(tbm_bufmgr_bcm bufmgr_bcm, tbm_bo_bcm bo_bcm, int import)
{
#ifdef ENABLE_CACHECRTL
	BCM_RETURN_VAL_IF_FAIL(bufmgr_bcm != NULL, 0);
	BCM_RETURN_VAL_IF_FAIL(bo_bcm != NULL, 0);

	if (bufmgr_bcm->use_dma_fence)
		return 1;

	_tgl_init(bufmgr_bcm->tgl_fd, bo_bcm->name);

	tbm_bo_cache_state cache_state;

	if (import == 0) {
		cache_state.data.isDirtied = DEVICE_NONE;
		cache_state.data.isCached = 0;
		cache_state.data.cntFlush = 0;

		_tgl_set_data(bufmgr_bcm->tgl_fd, bo_bcm->name, cache_state.val);
	}
#endif

	return 1;
}

static int
_bo_set_cache_state(tbm_bufmgr_bcm bufmgr_bcm, tbm_bo_bcm bo_bcm, int device, int opt)
{
#ifdef ENABLE_CACHECRTL
	BCM_RETURN_VAL_IF_FAIL(bufmgr_bcm != NULL, 0);
	BCM_RETURN_VAL_IF_FAIL(bo_bcm != NULL, 0);

	if (bufmgr_bcm->use_dma_fence)
		return 1;

	char need_flush = 0;
	unsigned short cntFlush = 0;

	/* get cache state of a bo */
	bo_bcm->cache_state.val = _tgl_get_data(bufmgr_bcm->tgl_fd,
				     bo_bcm->name);

	/* get global cache flush count */
	cntFlush = (unsigned short)_tgl_get_data(bufmgr_bcm->tgl_fd, GLOBAL_KEY);

	if (device == TBM_DEVICE_CPU) {
		if (bo_bcm->cache_state.data.isDirtied == DEVICE_CO &&
		    bo_bcm->cache_state.data.isCached)
			need_flush = TBM_EXYNOS_CACHE_INV;

		bo_bcm->cache_state.data.isCached = 1;
		if (opt & TBM_OPTION_WRITE)
			bo_bcm->cache_state.data.isDirtied = DEVICE_CA;
		else {
			if (bo_bcm->cache_state.data.isDirtied != DEVICE_CA)
				bo_bcm->cache_state.data.isDirtied = DEVICE_NONE;
		}
	} else {
		if (bo_bcm->cache_state.data.isDirtied == DEVICE_CA &&
		    bo_bcm->cache_state.data.isCached &&
		    bo_bcm->cache_state.data.cntFlush == cntFlush)
			need_flush = TBM_EXYNOS_CACHE_CLN | TBM_EXYNOS_CACHE_ALL;

		if (opt & TBM_OPTION_WRITE)
			bo_bcm->cache_state.data.isDirtied = DEVICE_CO;
		else {
			if (bo_bcm->cache_state.data.isDirtied != DEVICE_CO)
				bo_bcm->cache_state.data.isDirtied = DEVICE_NONE;
		}
	}

	if (need_flush) {
		if (need_flush & TBM_EXYNOS_CACHE_ALL)
			_tgl_set_data(bufmgr_bcm->tgl_fd, GLOBAL_KEY, (unsigned int)(++cntFlush));

		/* call cache flush */
		_exynos_cache_flush(bufmgr_bcm, bo_bcm, need_flush);

		TBM_BCM_DEBUG(" \tcache(%d,%d)....flush:0x%x, cntFlush(%d)\n",
		    bo_bcm->cache_state.data.isCached,
		    bo_bcm->cache_state.data.isDirtied,
		    need_flush,
		    cntFlush);
	}
#endif

	return 1;
}

static int
_bo_save_cache_state(tbm_bufmgr_bcm bufmgr_bcm, tbm_bo_bcm bo_bcm)
{
#ifdef ENABLE_CACHECRTL
	BCM_RETURN_VAL_IF_FAIL(bufmgr_bcm != NULL, 0);
	BCM_RETURN_VAL_IF_FAIL(bo_bcm != NULL, 0);

	if (bufmgr_bcm->use_dma_fence)
		return 1;

	unsigned short cntFlush = 0;

	/* get global cache flush count */
	cntFlush = (unsigned short)_tgl_get_data(bufmgr_bcm->tgl_fd, GLOBAL_KEY);

	/* save global cache flush count */
	bo_bcm->cache_state.data.cntFlush = cntFlush;
	_tgl_set_data(bufmgr_bcm->tgl_fd, bo_bcm->name,
		      bo_bcm->cache_state.val);
#endif

	return 1;
}

static void
_bo_destroy_cache_state(tbm_bufmgr_bcm bufmgr_bcm, tbm_bo_bcm bo_bcm)
{
#ifdef ENABLE_CACHECRTL
	BCM_RETURN_IF_FAIL(bufmgr_bcm != NULL);
	BCM_RETURN_IF_FAIL(bo_bcm != NULL);

	if (bufmgr_bcm->use_dma_fence)
		return ;

	_tgl_destroy(bufmgr_bcm->tgl_fd, bo_bcm->name);
#endif
}

static int
_bufmgr_init_cache_state(tbm_bufmgr_bcm bufmgr_bcm)
{
#ifdef ENABLE_CACHECRTL
	BCM_RETURN_VAL_IF_FAIL(bufmgr_bcm != NULL, 0);

	if (bufmgr_bcm->use_dma_fence)
		return 1;

	/* open tgl fd for saving cache flush data */
	bufmgr_bcm->tgl_fd = open(tgl_devfile, O_RDWR);

	if (bufmgr_bcm->tgl_fd < 0) {
	    bufmgr_bcm->tgl_fd = open(tgl_devfile1, O_RDWR);
	    if (bufmgr_bcm->tgl_fd < 0) {
		    TBM_BCM_ERROR("fail to open global_lock:%s\n",
					tgl_devfile1);
		    return 0;
	    }
	}

#ifdef TGL_GET_VERSION
	if (!_tgl_get_version(bufmgr_bcm->tgl_fd)) {
		TBM_BCM_ERROR("fail to get tgl_version. tgl init failed.\n");
		close(bufmgr_sprd->tgl_fd);
		return 0;
	}
#endif

	if (!_tgl_init(bufmgr_bcm->tgl_fd, GLOBAL_KEY)) {
		TBM_BCM_ERROR("fail to initialize the tgl\n");
		close(bufmgr_bcm->tgl_fd);
		return 0;
	}
#endif

	return 1;
}

static void
_bufmgr_deinit_cache_state(tbm_bufmgr_bcm bufmgr_bcm)
{
#ifdef ENABLE_CACHECRTL
	BCM_RETURN_IF_FAIL(bufmgr_bcm != NULL);

	if (bufmgr_bcm->use_dma_fence)
		return;

	if (bufmgr_bcm->tgl_fd >= 0)
		close(bufmgr_bcm->tgl_fd);
#endif
}

static int
_tbm_bcm_open_drm()
{
	int fd = -1;

	fd = drmOpen(BCM_DRM_NAME, NULL);
	if (fd < 0) {
		TBM_BCM_ERROR("fail to open drm.(%s)\n", BCM_DRM_NAME);
	}

	if (fd < 0) {
		struct udev *udev = NULL;
		struct udev_enumerate *e = NULL;
		struct udev_list_entry *entry = NULL;
		struct udev_device *device = NULL, *drm_device = NULL, *device_parent = NULL;
		const char *filepath;
		struct stat s;
		int ret;

		TBM_BCM_DEBUG("search drm-device by udev\n");

		udev = udev_new();
		if (!udev) {
			TBM_BCM_ERROR("udev_new() failed.\n");
			return -1;
		}

		e = udev_enumerate_new(udev);
		udev_enumerate_add_match_subsystem(e, "drm");
		udev_enumerate_add_match_sysname(e, "card[0-9]*");
		udev_enumerate_scan_devices(e);

		udev_list_entry_foreach(entry, udev_enumerate_get_list_entry(e)) {
			device = udev_device_new_from_syspath(udev_enumerate_get_udev(e),
							      udev_list_entry_get_name(entry));
			device_parent = udev_device_get_parent(device);
			/* Not need unref device_parent. device_parent and device have same refcnt */
			if (device_parent) {
				if (strcmp(udev_device_get_sysname(device_parent), "exynos-drm") == 0) {
					drm_device = device;
					TBM_BCM_DEBUG("[%s] Found render device: '%s' (%s)\n",
					    target_name(),
					    udev_device_get_syspath(drm_device),
					    udev_device_get_sysname(device_parent));
					break;
				}
			}
			udev_device_unref(device);
		}

		udev_enumerate_unref(e);

		/* Get device file path. */
		filepath = udev_device_get_devnode(drm_device);
		if (!filepath) {
			TBM_BCM_ERROR("udev_device_get_devnode() failed.\n");
			udev_device_unref(drm_device);
			udev_unref(udev);
			return -1;
		}

		/* Open DRM device file and check validity. */
		fd = open(filepath, O_RDWR | O_CLOEXEC);
		if (fd < 0) {
			TBM_BCM_ERROR("open(%s, O_RDWR | O_CLOEXEC) failed.\n");
			udev_device_unref(drm_device);
			udev_unref(udev);
			return -1;
		}

		ret = fstat(fd, &s);
		if (ret) {
			TBM_BCM_ERROR("fstat() failed %s.\n");
			close(fd);
			udev_device_unref(drm_device);
			udev_unref(udev);
			return -1;
		}

		udev_device_unref(drm_device);
		udev_unref(udev);
	}

	return fd;
}

static int
_check_render_node(void)  //TODO
{
	struct udev *udev = NULL;
	struct udev_enumerate *e = NULL;
	struct udev_list_entry *entry = NULL;
	struct udev_device *device = NULL, *drm_device = NULL, *device_parent = NULL;

#ifndef USE_RENDER_NODE
	return 0;
#endif

	udev = udev_new();
	if (!udev) {
		TBM_BCM_ERROR("udev_new() failed.\n");
		return -1;
	}

	e = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(e, "drm");
	udev_enumerate_add_match_sysname(e, "renderD[0-9]*");
	udev_enumerate_scan_devices(e);

	udev_list_entry_foreach(entry, udev_enumerate_get_list_entry(e)) {
		device = udev_device_new_from_syspath(udev_enumerate_get_udev(e),
						      udev_list_entry_get_name(entry));
		device_parent = udev_device_get_parent(device);
		/* Not need unref device_parent. device_parent and device have same refcnt */
		if (device_parent) {
			if (strcmp(udev_device_get_sysname(device_parent), "exynos-drm") == 0) {
				drm_device = device;
				TBM_BCM_DEBUG("Found render device: '%s' (%s)\n",
				    udev_device_get_syspath(drm_device),
				    udev_device_get_sysname(device_parent));
				break;
			}
		}
		udev_device_unref(device);
	}

	udev_enumerate_unref(e);
	udev_unref(udev);

	if (!drm_device) {
		udev_device_unref(drm_device);
		return 0;
	}

	udev_device_unref(drm_device);
	return 1;
}

static int
_get_render_node(void)//TODO
{
	struct udev *udev = NULL;
	struct udev_enumerate *e = NULL;
	struct udev_list_entry *entry = NULL;
	struct udev_device *device = NULL, *drm_device = NULL, *device_parent = NULL;
	const char *filepath;
	struct stat s;
	int fd = -1;
	int ret;

	udev = udev_new();
	if (!udev) {
		TBM_BCM_ERROR("udev_new() failed.\n");
		return -1;
	}

	e = udev_enumerate_new(udev);
	udev_enumerate_add_match_subsystem(e, "drm");
	udev_enumerate_add_match_sysname(e, "renderD[0-9]*");
	udev_enumerate_scan_devices(e);

	udev_list_entry_foreach(entry, udev_enumerate_get_list_entry(e)) {
		device = udev_device_new_from_syspath(udev_enumerate_get_udev(e),
						      udev_list_entry_get_name(entry));
		device_parent = udev_device_get_parent(device);
		/* Not need unref device_parent. device_parent and device have same refcnt */
		if (device_parent) {
			if (strcmp(udev_device_get_sysname(device_parent), "exynos-drm") == 0) {
				drm_device = device;
				TBM_BCM_DEBUG("Found render device: '%s' (%s)\n",
				    udev_device_get_syspath(drm_device),
				    udev_device_get_sysname(device_parent));
				break;
			}
		}
		udev_device_unref(device);
	}

	udev_enumerate_unref(e);

	/* Get device file path. */
	filepath = udev_device_get_devnode(drm_device);
	if (!filepath) {
		TBM_BCM_ERROR("udev_device_get_devnode() failed.\n");
		udev_device_unref(drm_device);
		udev_unref(udev);
		return -1;
	}

	/* Open DRM device file and check validity. */
	fd = open(filepath, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		TBM_BCM_ERROR("open(%s, O_RDWR | O_CLOEXEC) failed.\n");
		udev_device_unref(drm_device);
		udev_unref(udev);
		return -1;
	}

	ret = fstat(fd, &s);
	if (ret) {
		TBM_BCM_ERROR("fstat() failed %s.\n");
		udev_device_unref(drm_device);
		udev_unref(udev);
		close(fd);
		return -1;
	}

	udev_device_unref(drm_device);
	udev_unref(udev);

	return fd;
}

static unsigned int
_get_name(int fd, unsigned int gem)
{
	struct drm_gem_flink arg = {0,};

	arg.handle = gem;
	if (drmIoctl(fd, DRM_IOCTL_GEM_FLINK, &arg)) {
		TBM_BCM_ERROR("fail to DRM_IOCTL_GEM_FLINK gem:%d", gem);
		return 0;
	}

	return (unsigned int)arg.name;
}

static tbm_bo_handle
_bcm_bo_handle(tbm_bo_bcm bo_bcm, int device)
{
	tbm_bo_handle bo_handle;

	memset(&bo_handle, 0x0, sizeof(uint64_t));

	switch (device) {
	case TBM_DEVICE_DEFAULT:
	case TBM_DEVICE_2D:
		bo_handle.u32 = (uint32_t)bo_bcm->gem;
		break;
	case TBM_DEVICE_CPU:
		if (!bo_bcm->pBase) {
			void *map = NULL;
			struct drm_vc4_mmap_bo arg = {0, };
			arg.handle = bo_bcm->gem;
			if (drmIoctl(bo_bcm->fd, DRM_IOCTL_VC4_MMAP_BO, &arg)){
				TBM_BCM_ERROR("Cannot map_dumb gem=%d\n", bo_bcm->gem);
				return (tbm_bo_handle) NULL;
			}

			map = mmap(NULL, bo_bcm->size, PROT_READ | PROT_WRITE, MAP_SHARED,
				   bo_bcm->fd, arg.offset);
			if (map == MAP_FAILED) {
				TBM_BCM_ERROR("Cannot usrptr gem=%d\n", bo_bcm->gem);
				return (tbm_bo_handle) NULL;
			}
			bo_bcm->pBase = map;
		}
		bo_handle.ptr = (void *)bo_bcm->pBase;
		break;
	case TBM_DEVICE_3D:
#ifdef USE_DMAIMPORT
		if (bo_bcm->dmabuf) {
			bo_handle.u32 = (uint32_t)bo_bcm->dmabuf;
			break;
		}

		if (!bo_bcm->dmabuf) {
			struct drm_prime_handle arg = {0, };

			arg.handle = bo_bcm->gem;
			if (drmIoctl(bo_bcm->fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &arg)) {
				TBM_BCM_ERROR("Cannot dmabuf=%d\n", bo_bcm->gem);
				return (tbm_bo_handle) NULL;
			}
			bo_bcm->dmabuf = arg.fd;
		}

		bo_handle.u32 = (uint32_t)bo_bcm->dmabuf;
#endif
		break;
	case TBM_DEVICE_MM:
		if (!bo_bcm->dmabuf) {
			struct drm_prime_handle arg = {0, };

			arg.handle = bo_bcm->gem;
			if (drmIoctl(bo_bcm->fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &arg)) {
				TBM_BCM_ERROR("Cannot dmabuf=%d\n", bo_bcm->gem);
				return (tbm_bo_handle) NULL;
			}
			bo_bcm->dmabuf = arg.fd;
		}

		bo_handle.u32 = (uint32_t)bo_bcm->dmabuf;
		break;
	default:
		TBM_BCM_ERROR("Not supported device:%d\n", device);
		bo_handle.ptr = (void *) NULL;
		break;
	}

	return bo_handle;
}

static int
tbm_bcm_bo_size(tbm_bo bo)
{
	BCM_RETURN_VAL_IF_FAIL(bo != NULL, 0);

	tbm_bo_bcm bo_bcm;

	bo_bcm = (tbm_bo_bcm)tbm_backend_get_bo_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bo_bcm != NULL, 0);

	return bo_bcm->size;
}

static void *
tbm_bcm_bo_alloc(tbm_bo bo, int size, int flags)
{
	BCM_RETURN_VAL_IF_FAIL(bo != NULL, 0);

	tbm_bo_bcm bo_bcm;
	tbm_bufmgr_bcm bufmgr_bcm;

	bufmgr_bcm = (tbm_bufmgr_bcm)tbm_backend_get_bufmgr_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bufmgr_bcm != NULL, 0);

	bo_bcm = calloc(1, sizeof(struct _tbm_bo_bcm));
	if (!bo_bcm) {
		TBM_BCM_ERROR("fail to allocate the bo private\n");
		return 0;
	}

	struct drm_vc4_create_bo arg = {0, };
	arg.flags = flags;/*currently no values for the flags,but it may be used in future extension*/
	arg.size = (__u32)size;
	if (drmIoctl(bufmgr_bcm->fd, DRM_IOCTL_VC4_CREATE_BO, &arg)){
		TBM_BCM_ERROR("Cannot create bo(flag:%x, size:%d)\n", arg.flags,
			       (unsigned int)arg.size);
		free(bo_bcm);
		return 0;
	}

	bo_bcm->fd = bufmgr_bcm->fd;
	bo_bcm->gem = (unsigned int)arg.handle;
	bo_bcm->size = size;
	bo_bcm->flags_tbm = flags;
	bo_bcm->name = _get_name(bo_bcm->fd, bo_bcm->gem);

	if (!_bo_init_cache_state(bufmgr_bcm, bo_bcm, 0)) {
		TBM_BCM_ERROR("fail init cache state(%d)\n", bo_bcm->name);
		free(bo_bcm);
		return 0;
	}

	pthread_mutex_init(&bo_bcm->mutex, NULL);

	if (bufmgr_bcm->use_dma_fence
	    && !bo_bcm->dmabuf) {
		struct drm_prime_handle arg = {0, };

		arg.handle = bo_bcm->gem;
		if (drmIoctl(bo_bcm->fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &arg)) {
			TBM_BCM_ERROR("Cannot dmabuf=%d\n", bo_bcm->gem);
			free(bo_bcm);
			return 0;
		}
		bo_bcm->dmabuf = arg.fd;
	}

	/* add bo to hash */
	PrivGem *privGem = calloc(1, sizeof(PrivGem));

	if (!privGem) {
		TBM_BCM_ERROR("fail to calloc privGem\n");
		free(bo_bcm);
		return 0;
	}

	privGem->ref_count = 1;
	privGem->bo_priv = bo_bcm;

	if (drmHashInsert(bufmgr_bcm->hashBos, bo_bcm->name,
			  (void *)privGem) < 0) {
		TBM_BCM_ERROR("Cannot insert bo to Hash(%d)\n", bo_bcm->name);
	}

	TBM_BCM_DEBUG("     bo:%p, gem:%d(%d), flags:%d, size:%d\n",
	    bo,
	    bo_bcm->gem, bo_bcm->name,
	    flags,
	    bo_bcm->size);

	return (void *)bo_bcm;
}

static void
tbm_bcm_bo_free(tbm_bo bo)
{
	tbm_bo_bcm bo_bcm;
	tbm_bufmgr_bcm bufmgr_bcm;

	if (!bo)
		return;

	bufmgr_bcm = (tbm_bufmgr_bcm)tbm_backend_get_bufmgr_priv(bo);
	BCM_RETURN_IF_FAIL(bufmgr_bcm != NULL);

	bo_bcm = (tbm_bo_bcm)tbm_backend_get_bo_priv(bo);
	BCM_RETURN_IF_FAIL(bo_bcm != NULL);

	TBM_BCM_DEBUG("      bo:%p, gem:%d(%d), fd:%d, size:%d\n",
	    bo,
	    bo_bcm->gem, bo_bcm->name,
	    bo_bcm->dmabuf,
	    bo_bcm->size);

	if (bo_bcm->pBase) {
		if (munmap(bo_bcm->pBase, bo_bcm->size) == -1) {
			TBM_BCM_ERROR("bo:%p fail to munmap(%s)\n",
				       bo, strerror(errno));
		}
	}

	/* close dmabuf */
	if (bo_bcm->dmabuf) {
		close(bo_bcm->dmabuf);
		bo_bcm->dmabuf = 0;
	}

	/* delete bo from hash */
	PrivGem *privGem = NULL;
	int ret;

	ret = drmHashLookup(bufmgr_bcm->hashBos, bo_bcm->name,
			     (void **)&privGem);
	if (ret == 0) {
		privGem->ref_count--;
		if (privGem->ref_count == 0) {
			drmHashDelete(bufmgr_bcm->hashBos, bo_bcm->name);
			free(privGem);
			privGem = NULL;
		}
	} else {
		TBM_BCM_ERROR("Cannot find bo to Hash(%d), ret=%d\n",
			bo_bcm->name, ret);
	}

	_bo_destroy_cache_state(bufmgr_bcm, bo_bcm);

	/* Free gem handle */
	struct drm_gem_close arg = {0, };

	memset(&arg, 0, sizeof(arg));
	arg.handle = bo_bcm->gem;
	if (drmIoctl(bo_bcm->fd, DRM_IOCTL_GEM_CLOSE, &arg)) {
		TBM_BCM_ERROR("bo:%p fail to gem close.(%s)\n",
			       bo, strerror(errno));
	}

	free(bo_bcm);
}


static void *
tbm_bcm_bo_import(tbm_bo bo, unsigned int key)
{
	BCM_RETURN_VAL_IF_FAIL(bo != NULL, 0);

	tbm_bufmgr_bcm bufmgr_bcm;
	tbm_bo_bcm bo_bcm;
	PrivGem *privGem = NULL;
	int ret;

	bufmgr_bcm = (tbm_bufmgr_bcm)tbm_backend_get_bufmgr_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bufmgr_bcm != NULL, 0);

	ret = drmHashLookup(bufmgr_bcm->hashBos, key, (void **)&privGem);
	if (ret == 0)
		return privGem->bo_priv;

	struct drm_gem_open arg = {0, };

	arg.name = key;
	if (drmIoctl(bufmgr_bcm->fd, DRM_IOCTL_GEM_OPEN, &arg)) {
		TBM_BCM_ERROR("Cannot open gem name=%d\n", key);
		return 0;
	}

	bo_bcm = calloc(1, sizeof(struct _tbm_bo_bcm));
	if (!bo_bcm) {
		TBM_BCM_ERROR("fail to allocate the bo private\n");
		return 0;
	}

	bo_bcm->fd = bufmgr_bcm->fd;
	bo_bcm->gem = arg.handle;
	bo_bcm->size = arg.size;
	bo_bcm->name = key;
	bo_bcm->flags_tbm = 0;

	if (!_bo_init_cache_state(bufmgr_bcm, bo_bcm, 1)) {
		TBM_BCM_ERROR("fail init cache state(%d)\n", bo_bcm->name);
		free(bo_bcm);
		return 0;
	}

	if (!bo_bcm->dmabuf) {
		struct drm_prime_handle arg = {0, };

		arg.handle = bo_bcm->gem;
		if (drmIoctl(bo_bcm->fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &arg)) {
			TBM_BCM_ERROR("fail to DRM_IOCTL_PRIME_HANDLE_TO_FD gem=%d\n", bo_bcm->gem);
			free(bo_bcm);
			return 0;
		}
		bo_bcm->dmabuf = arg.fd;
	}

	/* add bo to hash */
	privGem = NULL;

	privGem = calloc(1, sizeof(PrivGem));
	if (!privGem) {
		TBM_BCM_ERROR("fail to calloc privGem\n");
		free(bo_bcm);
		return 0;
	}

	privGem->ref_count = 1;
	privGem->bo_priv = bo_bcm;

	if (drmHashInsert(bufmgr_bcm->hashBos, bo_bcm->name,
			   (void *)privGem) < 0) {
		TBM_BCM_ERROR("Cannot insert bo to Hash(%d)\n", bo_bcm->name);
	}

	TBM_BCM_DEBUG("    bo:%p, gem:%d(%d), fd:%d, flags:%d, size:%d\n",
	    bo,
	    bo_bcm->gem, bo_bcm->name,
	    bo_bcm->dmabuf,
	    bo_bcm->flags_tbm,
	    bo_bcm->size);

	return (void *)bo_bcm;
}

static void *
tbm_bcm_bo_import_fd(tbm_bo bo, tbm_fd key)
{
	BCM_RETURN_VAL_IF_FAIL(bo != NULL, 0);

	tbm_bufmgr_bcm bufmgr_bcm;
	tbm_bo_bcm bo_bcm;
	PrivGem *privGem = NULL;
	unsigned int name;
	int ret;

	bufmgr_bcm = (tbm_bufmgr_bcm)tbm_backend_get_bufmgr_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bufmgr_bcm != NULL, 0);

	/*getting handle from fd*/
	unsigned int gem = 0;
	struct drm_prime_handle arg = {0, };

	arg.fd = key;
	arg.flags = 0;
	if (drmIoctl(bufmgr_bcm->fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &arg)) {
		TBM_BCM_ERROR("bo:%p Cannot get gem handle from fd:%d (%s)\n",
			       bo, arg.fd, strerror(errno));
		return NULL;
	}
	gem = arg.handle;

	name = _get_name(bufmgr_bcm->fd, gem);
	if (!name) {
		TBM_BCM_ERROR("bo:%p Cannot get name from gem:%d, fd:%d (%s)\n",
			       bo, gem, key, strerror(errno));
		return 0;
	}

	ret = drmHashLookup(bufmgr_bcm->hashBos, name, (void **)&privGem);
	if (ret == 0) {
		if (gem == privGem->bo_priv->gem)
			return privGem->bo_priv;
	}

	unsigned int real_size = -1;
	//struct drm_exynos_gem_info info = {0, };

	/* Determine size of bo.  The fd-to-handle ioctl really should
	 * return the size, but it doesn't.  If we have kernel 3.12 or
	 * later, we can lseek on the prime fd to get the size.  Older
	 * kernels will just fail, in which case we fall back to the
	 * provided (estimated or guess size).
	 */
	real_size = lseek(key, 0, SEEK_END);

	/*info.handle = gem;
	if (drmCommandWriteRead(bufmgr_bcm->fd,
				DRM_EXYNOS_GEM_GET,
				&info,
				sizeof(struct drm_exynos_gem_info))) {
		TBM_BCM_ERROR("bo:%p Cannot get gem info from gem:%d, fd:%d (%s)\n",
			       bo, gem, key, strerror(errno));
		return 0;
	}*/

	struct drm_gem_open open_arg = {0, };

	open_arg.name = name;
	if (drmIoctl(bufmgr_bcm->fd, DRM_IOCTL_GEM_OPEN, &open_arg)) {
		TBM_BCM_ERROR("Cannot open gem name=%d\n", name);
		return 0;
	}

	if (real_size == -1)
		real_size = open_arg.size;

	bo_bcm = calloc(1, sizeof(struct _tbm_bo_bcm));
	if (!bo_bcm) {
		TBM_BCM_ERROR("bo:%p fail to allocate the bo private\n", bo);
		return 0;
	}

	bo_bcm->fd = bufmgr_bcm->fd;
	bo_bcm->gem = gem;
	bo_bcm->size = real_size;
	bo_bcm->flags_tbm = 0;
	bo_bcm->name = name;

	if (!_bo_init_cache_state(bufmgr_bcm, bo_bcm, 1)) {
		TBM_BCM_ERROR("fail init cache state(%d)\n", bo_bcm->name);
		free(bo_bcm);
		return 0;
	}

	/* add bo to hash */
	privGem = NULL;

	privGem = calloc(1, sizeof(PrivGem));
	if (!privGem) {
		TBM_BCM_ERROR("fail to calloc privGem\n");
		free(bo_bcm);
		return 0;
	}

	privGem->ref_count = 1;
	privGem->bo_priv = bo_bcm;

	if (drmHashInsert(bufmgr_bcm->hashBos, bo_bcm->name,
			   (void *)privGem) < 0) {
		TBM_BCM_ERROR("bo:%p Cannot insert bo to Hash(%d) from gem:%d, fd:%d\n",
			       bo, bo_bcm->name, gem, key);
	}

	TBM_BCM_DEBUG(" bo:%p, gem:%d(%d), fd:%d, key_fd:%d, flags:%d, size:%d\n",
	    bo,
	    bo_bcm->gem, bo_bcm->name,
	    bo_bcm->dmabuf,
	    key,
	    bo_bcm->flags_tbm,
	    bo_bcm->size);

	return (void *)bo_bcm;
}

static unsigned int
tbm_bcm_bo_export(tbm_bo bo)
{
	BCM_RETURN_VAL_IF_FAIL(bo != NULL, 0);

	tbm_bo_bcm bo_bcm;

	bo_bcm = (tbm_bo_bcm)tbm_backend_get_bo_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bo_bcm != NULL, 0);

	if (!bo_bcm->name) {
		bo_bcm->name = _get_name(bo_bcm->fd, bo_bcm->gem);
		if (!bo_bcm->name) {
			TBM_BCM_ERROR("Cannot get name\n");
			return 0;
		}
	}

	TBM_BCM_DEBUG("    bo:%p, gem:%d(%d), fd:%d, flags:%d, size:%d\n",
	    bo,
	    bo_bcm->gem, bo_bcm->name,
	    bo_bcm->dmabuf,
	    bo_bcm->flags_tbm,
	    bo_bcm->size);

	return (unsigned int)bo_bcm->name;
}

tbm_fd
tbm_bcm_bo_export_fd(tbm_bo bo)
{
	BCM_RETURN_VAL_IF_FAIL(bo != NULL, -1);

	tbm_bo_bcm bo_bcm;
	int ret;

	bo_bcm = (tbm_bo_bcm)tbm_backend_get_bo_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bo_bcm != NULL, -1);

	struct drm_prime_handle arg = {0, };

	arg.handle = bo_bcm->gem;
	ret = drmIoctl(bo_bcm->fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &arg);
	if (ret) {
		TBM_BCM_ERROR("bo:%p Cannot dmabuf=%d (%s)\n",
			       bo, bo_bcm->gem, strerror(errno));
		return (tbm_fd) ret;
	}

	TBM_BCM_DEBUG(" bo:%p, gem:%d(%d), fd:%d, key_fd:%d, flags:%d, size:%d\n",
	    bo,
	    bo_bcm->gem, bo_bcm->name,
	    bo_bcm->dmabuf,
	    arg.fd,
	    bo_bcm->flags_tbm,
	    bo_bcm->size);

	return (tbm_fd)arg.fd;
}

static tbm_bo_handle
tbm_bcm_bo_get_handle(tbm_bo bo, int device)
{
	BCM_RETURN_VAL_IF_FAIL(bo != NULL, (tbm_bo_handle) NULL);

	tbm_bo_handle bo_handle;
	tbm_bo_bcm bo_bcm;

	bo_bcm = (tbm_bo_bcm)tbm_backend_get_bo_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bo_bcm != NULL, (tbm_bo_handle) NULL);

	if (!bo_bcm->gem) {
		TBM_BCM_ERROR("Cannot map gem=%d\n", bo_bcm->gem);
		return (tbm_bo_handle) NULL;
	}

	TBM_BCM_DEBUG("bo:%p, gem:%d(%d), fd:%d, flags:%d, size:%d, %s\n",
	    bo,
	    bo_bcm->gem, bo_bcm->name,
	    bo_bcm->dmabuf,
	    bo_bcm->flags_tbm,
	    bo_bcm->size,
	    STR_DEVICE[device]);

	/*Get mapped bo_handle*/
	bo_handle = _bcm_bo_handle(bo_bcm, device);
	if (bo_handle.ptr == NULL) {
		TBM_BCM_ERROR("Cannot get handle: gem:%d, device:%d\n",
			bo_bcm->gem, device);
		return (tbm_bo_handle) NULL;
	}

	return bo_handle;
}

static tbm_bo_handle
tbm_bcm_bo_map(tbm_bo bo, int device, int opt)
{
	BCM_RETURN_VAL_IF_FAIL(bo != NULL, (tbm_bo_handle) NULL);

	tbm_bo_handle bo_handle;
	tbm_bo_bcm bo_bcm;
	tbm_bufmgr_bcm bufmgr_bcm;

	bufmgr_bcm = (tbm_bufmgr_bcm)tbm_backend_get_bufmgr_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bufmgr_bcm != NULL, (tbm_bo_handle)NULL);

	bo_bcm = (tbm_bo_bcm)tbm_backend_get_bo_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bo_bcm != NULL, (tbm_bo_handle) NULL);

	if (!bo_bcm->gem) {
		TBM_BCM_ERROR("Cannot map gem=%d\n", bo_bcm->gem);
		return (tbm_bo_handle) NULL;
	}

	TBM_BCM_DEBUG("       bo:%p, gem:%d(%d), fd:%d, %s, %s\n",
	    bo,
	    bo_bcm->gem, bo_bcm->name,
	    bo_bcm->dmabuf,
	    STR_DEVICE[device],
	    STR_OPT[opt]);

	/*Get mapped bo_handle*/
	bo_handle = _bcm_bo_handle(bo_bcm, device);
	if (bo_handle.ptr == NULL) {
		TBM_BCM_ERROR("Cannot get handle: gem:%d, device:%d, opt:%d\n",
			       bo_bcm->gem, device, opt);
		return (tbm_bo_handle) NULL;
	}

	if (bo_bcm->map_cnt == 0)
		_bo_set_cache_state(bufmgr_bcm, bo_bcm, device, opt);

	bo_bcm->last_map_device = device;

	bo_bcm->map_cnt++;

	return bo_handle;
}

static int
tbm_bcm_bo_unmap(tbm_bo bo)
{
	BCM_RETURN_VAL_IF_FAIL(bo != NULL, 0);

	tbm_bo_bcm bo_bcm;
	tbm_bufmgr_bcm bufmgr_bcm;

	bufmgr_bcm = (tbm_bufmgr_bcm)tbm_backend_get_bufmgr_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bufmgr_bcm != NULL, 0);

	bo_bcm = (tbm_bo_bcm)tbm_backend_get_bo_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bo_bcm != NULL, 0);


	if (!bo_bcm->gem)
		return 0;

	bo_bcm->map_cnt--;

	if (bo_bcm->map_cnt == 0)
		_bo_save_cache_state(bufmgr_bcm, bo_bcm);

#ifdef ENABLE_CACHECRTL
	if (bo_bcm->last_map_device == TBM_DEVICE_CPU)
		_exynos_cache_flush(bufmgr_bcm, bo_bcm, TBM_EXYNOS_CACHE_FLUSH_ALL);
#endif

	bo_bcm->last_map_device = -1;

	TBM_BCM_DEBUG("     bo:%p, gem:%d(%d), fd:%d\n",
	    bo,
	    bo_bcm->gem, bo_bcm->name,
	    bo_bcm->dmabuf);

	return 1;
}

static int
tbm_bcm_bo_lock(tbm_bo bo, int device, int opt)
{
	BCM_RETURN_VAL_IF_FAIL(bo != NULL, 0);

#ifndef ALWAYS_BACKEND_CTRL
	tbm_bufmgr_bcm bufmgr_bcm;
	tbm_bo_bcm bo_bcm;
	struct dma_buf_fence fence;
	struct flock filelock;
	int ret = 0;

	if (device != TBM_DEVICE_3D && device != TBM_DEVICE_CPU) {
		TBM_BCM_DEBUG("Not support device type,\n");
		return 0;
	}

	bo_bcm = (tbm_bo_bcm)tbm_backend_get_bo_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bo_bcm != NULL, 0);

	bufmgr_bcm = (tbm_bufmgr_bcm)tbm_backend_get_bufmgr_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bufmgr_bcm != NULL, 0);

	memset(&fence, 0, sizeof(struct dma_buf_fence));

	/* Check if the given type is valid or not. */
	if (opt & TBM_OPTION_WRITE) {
		if (device == TBM_DEVICE_3D)
			fence.type = DMA_BUF_ACCESS_WRITE | DMA_BUF_ACCESS_DMA;
	} else if (opt & TBM_OPTION_READ) {
		if (device == TBM_DEVICE_3D)
			fence.type = DMA_BUF_ACCESS_READ | DMA_BUF_ACCESS_DMA;
	} else {
		TBM_BCM_ERROR("Invalid argument\n");
		return 0;
	}

	/* Check if the tbm manager supports dma fence or not. */
	if (!bufmgr_bcm->use_dma_fence) {
		TBM_BCM_ERROR("Not support DMA FENCE(%s)\n", strerror(errno));
		return 0;

	}

	if (device == TBM_DEVICE_3D) {
		ret = ioctl(bo_bcm->dmabuf, DMABUF_IOCTL_GET_FENCE, &fence);
		if (ret < 0) {
			TBM_BCM_ERROR("Cannot set GET FENCE(%s)\n", strerror(errno));
			return 0;
		}
	} else {
		if (opt & TBM_OPTION_WRITE)
			filelock.l_type = F_WRLCK;
		else
			filelock.l_type = F_RDLCK;

		filelock.l_whence = SEEK_CUR;
		filelock.l_start = 0;
		filelock.l_len = 0;

		if (-1 == fcntl(bo_bcm->dmabuf, F_SETLKW, &filelock))
			return 0;
	}

	pthread_mutex_lock(&bo_bcm->mutex);

	if (device == TBM_DEVICE_3D) {
		int i;

		for (i = 0; i < DMA_FENCE_LIST_MAX; i++) {
			if (bo_bcm->dma_fence[i].ctx == 0) {
				bo_bcm->dma_fence[i].type = fence.type;
				bo_bcm->dma_fence[i].ctx = fence.ctx;
				break;
			}
		}

		if (i == DMA_FENCE_LIST_MAX) {
			/*TODO: if dma_fence list is full, it needs realloc. I will fix this. by minseok3.kim*/
			TBM_BCM_ERROR("fence list is full\n");
		}
	}

	pthread_mutex_unlock(&bo_bcm->mutex);

	TBM_BCM_DEBUG("DMABUF_IOCTL_GET_FENCE! bo:%p, gem:%d(%d), fd:%ds\n",
	    bo,
	    bo_bcm->gem, bo_bcm->name,
	    bo_bcm->dmabuf);
#endif /* ALWAYS_BACKEND_CTRL */

	return 1;
}

static int
tbm_bcm_bo_unlock(tbm_bo bo)
{
	BCM_RETURN_VAL_IF_FAIL(bo != NULL, 0);

#ifndef ALWAYS_BACKEND_CTRL
	tbm_bo_bcm bo_bcm;
	struct dma_buf_fence fence;
	struct flock filelock;
	unsigned int dma_type = 0;
	int ret = 0;

	bo_bcm = (tbm_bo_bcm)tbm_backend_get_bo_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bo_bcm != NULL, 0);

	if (bo_bcm->dma_fence[0].type & DMA_BUF_ACCESS_DMA)
		dma_type = 1;

	if (!bo_bcm->dma_fence[0].ctx && dma_type) {
		TBM_BCM_DEBUG("FENCE not support or ignored,\n");
		return 0;
	}

	if (!bo_bcm->dma_fence[0].ctx && dma_type) {
		TBM_BCM_DEBUG("device type is not 3D/CPU,\n");
		return 0;
	}

	pthread_mutex_lock(&bo_bcm->mutex);

	if (dma_type) {
		fence.type = bo_bcm->dma_fence[0].type;
		fence.ctx = bo_bcm->dma_fence[0].ctx;
		int i;

		for (i = 1; i < DMA_FENCE_LIST_MAX; i++) {
			bo_bcm->dma_fence[i - 1].type = bo_bcm->dma_fence[i].type;
			bo_bcm->dma_fence[i - 1].ctx = bo_bcm->dma_fence[i].ctx;
		}
		bo_bcm->dma_fence[DMA_FENCE_LIST_MAX - 1].type = 0;
		bo_bcm->dma_fence[DMA_FENCE_LIST_MAX - 1].ctx = 0;
	}
	pthread_mutex_unlock(&bo_bcm->mutex);

	if (dma_type) {
		ret = ioctl(bo_bcm->dmabuf, DMABUF_IOCTL_PUT_FENCE, &fence);
		if (ret < 0) {
			TBM_BCM_ERROR("Can not set PUT FENCE(%s)\n", strerror(errno));
			return 0;
		}
	} else {
		filelock.l_type = F_UNLCK;
		filelock.l_whence = SEEK_CUR;
		filelock.l_start = 0;
		filelock.l_len = 0;

		if (-1 == fcntl(bo_bcm->dmabuf, F_SETLKW, &filelock))
			return 0;
	}

	TBM_BCM_DEBUG("DMABUF_IOCTL_PUT_FENCE! bo:%p, gem:%d(%d), fd:%ds\n",
	    bo,
	    bo_bcm->gem, bo_bcm->name,
	    bo_bcm->dmabuf);
#endif /* ALWAYS_BACKEND_CTRL */

	return 1;
}

static void
tbm_bcm_bufmgr_deinit(void *priv)
{
	BCM_RETURN_IF_FAIL(priv != NULL);

	tbm_bufmgr_bcm bufmgr_bcm;

	bufmgr_bcm = (tbm_bufmgr_bcm)priv;

	if (bufmgr_bcm->hashBos) {
		unsigned long key;
		void *value;

		while (drmHashFirst(bufmgr_bcm->hashBos, &key, &value) > 0) {
			free(value);
			drmHashDelete(bufmgr_bcm->hashBos, key);
		}

		drmHashDestroy(bufmgr_bcm->hashBos);
		bufmgr_bcm->hashBos = NULL;
	}

	_bufmgr_deinit_cache_state(bufmgr_bcm);

	if (bufmgr_bcm->bind_display)
		tbm_drm_helper_wl_auth_server_deinit();

	if (bufmgr_bcm->device_name)
		free(bufmgr_bcm->device_name);

	if (tbm_backend_is_display_server())
		tbm_drm_helper_unset_tbm_master_fd();
	else
		tbm_drm_helper_unset_fd();

	close(bufmgr_bcm->fd);

	free(bufmgr_bcm);
}

int
tbm_bcm_surface_supported_format(uint32_t **formats, uint32_t *num)
{
	uint32_t *color_formats = NULL;

	color_formats = (uint32_t *)calloc(1,
					   sizeof(uint32_t) * TBM_COLOR_FORMAT_COUNT);

	if (color_formats == NULL)
		return 0;

	memcpy(color_formats, tbm_bcm_color_format_list,
	       sizeof(uint32_t)*TBM_COLOR_FORMAT_COUNT);

	*formats = color_formats;
	*num = TBM_COLOR_FORMAT_COUNT;

	TBM_BCM_DEBUG("tbm_bcm_surface_supported_format  count = %d\n", *num);

	return 1;
}

static int
_new_calc_plane_nv12(int width, int height)
{
	int mbX, mbY;

	mbX = DIV_ROUND_UP(width, S5P_FIMV_NUM_PIXELS_IN_MB_ROW);
	mbY = DIV_ROUND_UP(height, S5P_FIMV_NUM_PIXELS_IN_MB_COL);

	if (width * height < S5P_FIMV_MAX_FRAME_SIZE)
		mbY = (mbY + 1) / 2 * 2;

	return ((mbX * S5P_FIMV_NUM_PIXELS_IN_MB_COL) * (mbY *
			S5P_FIMV_NUM_PIXELS_IN_MB_ROW));
}

static int
_calc_yplane_nv12(int width, int height)
{
	int mbX, mbY;

	mbX = SIZE_ALIGN(width + 24, S5P_FIMV_NV12MT_HALIGN);
	mbY = SIZE_ALIGN(height + 16, S5P_FIMV_NV12MT_VALIGN);

	return SIZE_ALIGN(mbX * mbY, S5P_FIMV_DEC_BUF_ALIGN);
}

static int
_calc_uvplane_nv12(int width, int height)
{
	int mbX, mbY;

	mbX = SIZE_ALIGN(width + 16, S5P_FIMV_NV12MT_HALIGN);
	mbY = SIZE_ALIGN(height + 4, S5P_FIMV_NV12MT_VALIGN);

	return SIZE_ALIGN((mbX * mbY) >> 1, S5P_FIMV_DEC_BUF_ALIGN);
}

static int
_new_calc_yplane_nv12(int width, int height)
{
	return SIZE_ALIGN(_new_calc_plane_nv12(width,
						height) + S5P_FIMV_D_ALIGN_PLANE_SIZE,
			   TBM_SURFACE_ALIGNMENT_PLANE_NV12);
}

static int
_new_calc_uvplane_nv12(int width, int height)
{
	return SIZE_ALIGN((_new_calc_plane_nv12(width,
						height) >> 1) + S5P_FIMV_D_ALIGN_PLANE_SIZE,
			  TBM_SURFACE_ALIGNMENT_PLANE_NV12);
}

/**
 * @brief get the plane data of the surface.
 * @param[in] width : the width of the surface
 * @param[in] height : the height of the surface
 * @param[in] format : the format of the surface
 * @param[in] plane_idx : the format of the surface
 * @param[out] size : the size of the plane
 * @param[out] offset : the offset of the plane
 * @param[out] pitch : the pitch of the plane
 * @param[out] padding : the padding of the plane
 * @return 1 if this function succeeds, otherwise 0.
 */
int
tbm_bcm_surface_get_plane_data(int width, int height,
				  tbm_format format, int plane_idx, uint32_t *size, uint32_t *offset,
				  uint32_t *pitch, int *bo_idx)
{
	int ret = 1;
	int bpp;
	int _offset = 0;
	int _pitch = 0;
	int _size = 0;
	int _bo_idx = 0;

	switch (format) {
		/* 16 bpp RGB */
	case TBM_FORMAT_XRGB4444:
	case TBM_FORMAT_XBGR4444:
	case TBM_FORMAT_RGBX4444:
	case TBM_FORMAT_BGRX4444:
	case TBM_FORMAT_ARGB4444:
	case TBM_FORMAT_ABGR4444:
	case TBM_FORMAT_RGBA4444:
	case TBM_FORMAT_BGRA4444:
	case TBM_FORMAT_XRGB1555:
	case TBM_FORMAT_XBGR1555:
	case TBM_FORMAT_RGBX5551:
	case TBM_FORMAT_BGRX5551:
	case TBM_FORMAT_ARGB1555:
	case TBM_FORMAT_ABGR1555:
	case TBM_FORMAT_RGBA5551:
	case TBM_FORMAT_BGRA5551:
	case TBM_FORMAT_RGB565:
		bpp = 16;
		_offset = 0;
		_pitch = SIZE_ALIGN((width * bpp) >> 3, TBM_SURFACE_ALIGNMENT_PITCH_RGB);
		_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
		_bo_idx = 0;
		break;
		/* 24 bpp RGB */
	case TBM_FORMAT_RGB888:
	case TBM_FORMAT_BGR888:
		bpp = 24;
		_offset = 0;
		_pitch = SIZE_ALIGN((width * bpp) >> 3, TBM_SURFACE_ALIGNMENT_PITCH_RGB);
		_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
		_bo_idx = 0;
		break;
		/* 32 bpp RGB */
	case TBM_FORMAT_XRGB8888:
	case TBM_FORMAT_XBGR8888:
	case TBM_FORMAT_RGBX8888:
	case TBM_FORMAT_BGRX8888:
	case TBM_FORMAT_ARGB8888:
	case TBM_FORMAT_ABGR8888:
	case TBM_FORMAT_RGBA8888:
	case TBM_FORMAT_BGRA8888:
		bpp = 32;
		_offset = 0;
		_pitch = SIZE_ALIGN((width * bpp) >> 3, TBM_SURFACE_ALIGNMENT_PITCH_RGB);
		_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
		_bo_idx = 0;
		break;

		/* packed YCbCr */
	case TBM_FORMAT_YUYV:
	case TBM_FORMAT_YVYU:
	case TBM_FORMAT_UYVY:
	case TBM_FORMAT_VYUY:
	case TBM_FORMAT_AYUV:
		bpp = 32;
		_offset = 0;
		_pitch = SIZE_ALIGN((width * bpp) >> 3, TBM_SURFACE_ALIGNMENT_PITCH_YUV);
		_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
		_bo_idx = 0;
		break;

		/*
		* 2 plane YCbCr
		* index 0 = Y plane, [7:0] Y
		* index 1 = Cr:Cb plane, [15:0] Cr:Cb little endian
		* or
		* index 1 = Cb:Cr plane, [15:0] Cb:Cr little endian
		*/
	case TBM_FORMAT_NV12:
		bpp = 12;
		if (plane_idx == 0) {
			_offset = 0;
			_pitch = SIZE_ALIGN(width, TBM_SURFACE_ALIGNMENT_PITCH_YUV);
			_size = MAX(_calc_yplane_nv12(width, height), _new_calc_yplane_nv12(width,
					height));
			_bo_idx = 0;
		} else if (plane_idx == 1) {
			_offset = 0;
			_pitch = SIZE_ALIGN(width, TBM_SURFACE_ALIGNMENT_PITCH_YUV / 2);
			_size = MAX(_calc_uvplane_nv12(width, height), _new_calc_uvplane_nv12(width,
					height));
			_bo_idx = 1;
		}
		break;
	case TBM_FORMAT_NV21:
		bpp = 12;
		if (plane_idx == 0) {
			_offset = 0;
			_pitch = SIZE_ALIGN(width, TBM_SURFACE_ALIGNMENT_PITCH_YUV);
			_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
		} else if (plane_idx == 1) {
			_offset = width * height;
			_pitch = SIZE_ALIGN(width, TBM_SURFACE_ALIGNMENT_PITCH_YUV / 2);
			_size = SIZE_ALIGN(_pitch * (height / 2), TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
		}
		break;

	case TBM_FORMAT_NV16:
	case TBM_FORMAT_NV61:
		bpp = 16;
		/*if(plane_idx == 0)*/
		{
			_offset = 0;
			_pitch = SIZE_ALIGN(width, TBM_SURFACE_ALIGNMENT_PITCH_YUV);
			_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
			if (plane_idx == 0)
				break;
		}
		/*else if( plane_idx ==1 )*/
		{
			_offset += _size;
			_pitch = SIZE_ALIGN(width, TBM_SURFACE_ALIGNMENT_PITCH_YUV / 2);
			_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
		}
		break;

		/*
		* 3 plane YCbCr
		* index 0: Y plane, [7:0] Y
		* index 1: Cb plane, [7:0] Cb
		* index 2: Cr plane, [7:0] Cr
		* or
		* index 1: Cr plane, [7:0] Cr
		* index 2: Cb plane, [7:0] Cb
		*/

		/*
		* NATIVE_BUFFER_FORMAT_YV12
		* NATIVE_BUFFER_FORMAT_I420
		*/
	case TBM_FORMAT_YUV410:
	case TBM_FORMAT_YVU410:
		bpp = 9;
		/*if(plane_idx == 0)*/
		{
			_offset = 0;
			_pitch = SIZE_ALIGN(width, TBM_SURFACE_ALIGNMENT_PITCH_YUV);
			_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
			if (plane_idx == 0)
				break;
		}
		/*else if(plane_idx == 1)*/
		{
			_offset += _size;
			_pitch = SIZE_ALIGN(width / 4, TBM_SURFACE_ALIGNMENT_PITCH_YUV / 4);
			_size = SIZE_ALIGN(_pitch * (height / 4), TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
			if (plane_idx == 1)
				break;
		}
		/*else if (plane_idx == 2)*/
		{
			_offset += _size;
			_pitch = SIZE_ALIGN(width / 4, TBM_SURFACE_ALIGNMENT_PITCH_YUV / 4);
			_size = SIZE_ALIGN(_pitch * (height / 4), TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
		}
		break;
	case TBM_FORMAT_YUV411:
	case TBM_FORMAT_YVU411:
	case TBM_FORMAT_YUV420:
	case TBM_FORMAT_YVU420:
		bpp = 12;
		/*if(plane_idx == 0)*/
		{
			_offset = 0;
			_pitch = SIZE_ALIGN(width, TBM_SURFACE_ALIGNMENT_PITCH_YUV);
			_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
			if (plane_idx == 0)
				break;
		}
		/*else if(plane_idx == 1)*/
		{
			_offset += _size;
			_pitch = SIZE_ALIGN(width / 2, TBM_SURFACE_ALIGNMENT_PITCH_YUV / 2);
			_size = SIZE_ALIGN(_pitch * (height / 2), TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
			if (plane_idx == 1)
				break;
		}
		/*else if (plane_idx == 2)*/
		{
			_offset += _size;
			_pitch = SIZE_ALIGN(width / 2, TBM_SURFACE_ALIGNMENT_PITCH_YUV / 2);
			_size = SIZE_ALIGN(_pitch * (height / 2), TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
		}
		break;
	case TBM_FORMAT_YUV422:
	case TBM_FORMAT_YVU422:
		bpp = 16;
		/*if(plane_idx == 0)*/
		{
			_offset = 0;
			_pitch = SIZE_ALIGN(width, TBM_SURFACE_ALIGNMENT_PITCH_YUV);
			_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
			if (plane_idx == 0)
				break;
		}
		/*else if(plane_idx == 1)*/
		{
			_offset += _size;
			_pitch = SIZE_ALIGN(width / 2, TBM_SURFACE_ALIGNMENT_PITCH_YUV / 2);
			_size = SIZE_ALIGN(_pitch * (height), TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
			if (plane_idx == 1)
				break;
		}
		/*else if (plane_idx == 2)*/
		{
			_offset += _size;
			_pitch = SIZE_ALIGN(width / 2, TBM_SURFACE_ALIGNMENT_PITCH_YUV / 2);
			_size = SIZE_ALIGN(_pitch * (height), TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
		}
		break;
	case TBM_FORMAT_YUV444:
	case TBM_FORMAT_YVU444:
		bpp = 24;
		/*if(plane_idx == 0)*/
		{
			_offset = 0;
			_pitch = SIZE_ALIGN(width, TBM_SURFACE_ALIGNMENT_PITCH_YUV);
			_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
			if (plane_idx == 0)
				break;
		}
		/*else if(plane_idx == 1)*/
		{
			_offset += _size;
			_pitch = SIZE_ALIGN(width, TBM_SURFACE_ALIGNMENT_PITCH_YUV);
			_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
			if (plane_idx == 1)
				break;
		}
		/*else if (plane_idx == 2)*/
		{
			_offset += _size;
			_pitch = SIZE_ALIGN(width, TBM_SURFACE_ALIGNMENT_PITCH_YUV);
			_size = SIZE_ALIGN(_pitch * height, TBM_SURFACE_ALIGNMENT_PLANE);
			_bo_idx = 0;
		}
		break;
	default:
		bpp = 0;
		break;
	}

	*size = _size;
	*offset = _offset;
	*pitch = _pitch;
	*bo_idx = _bo_idx;

	return ret;
}

int
tbm_bcm_bo_get_flags(tbm_bo bo)
{
	BCM_RETURN_VAL_IF_FAIL(bo != NULL, 0);

	tbm_bo_bcm bo_bcm;

	bo_bcm = (tbm_bo_bcm)tbm_backend_get_bo_priv(bo);
	BCM_RETURN_VAL_IF_FAIL(bo_bcm != NULL, 0);

	return bo_bcm->flags_tbm;
}

int
tbm_bcm_bufmgr_bind_native_display(tbm_bufmgr bufmgr, void *native_display)
{
	tbm_bufmgr_bcm bufmgr_bcm;

	bufmgr_bcm = tbm_backend_get_priv_from_bufmgr(bufmgr);
	BCM_RETURN_VAL_IF_FAIL(bufmgr_bcm != NULL, 0);

	if (!tbm_drm_helper_wl_auth_server_init(native_display, bufmgr_bcm->fd,
					   bufmgr_bcm->device_name, 0)) {
		TBM_BCM_ERROR("fail to tbm_drm_helper_wl_server_init\n");
		return 0;
	}

	bufmgr_bcm->bind_display = native_display;

	return 1;
}

MODULEINITPPROTO(init_tbm_bufmgr_priv);

static TBMModuleVersionInfo BcmVersRec = {
	"bcm2837",
	"Broadcom",
	TBM_ABI_VERSION,
};

TBMModuleData tbmModuleData = { &BcmVersRec, init_tbm_bufmgr_priv};

int
init_tbm_bufmgr_priv(tbm_bufmgr bufmgr, int fd)
{
	tbm_bufmgr_backend bufmgr_backend;
	tbm_bufmgr_bcm bufmgr_bcm;
	int fp;

	if (!bufmgr)
		return 0;

	bufmgr_bcm = calloc(1, sizeof(struct _tbm_bufmgr_bcm));
	if (!bufmgr_bcm) {
		TBM_BCM_ERROR("fail to alloc bufmgr_bcm!\n");
		return 0;
	}

	if (tbm_backend_is_display_server()) {
		bufmgr_bcm->fd = tbm_drm_helper_get_master_fd();
		if (bufmgr_bcm->fd < 0) {
			bufmgr_bcm->fd = _tbm_bcm_open_drm();
			if (bufmgr_bcm->fd < 0) {
				TBM_BCM_ERROR("fail to open drm!\n", getpid());
				goto fail_open_drm;
			}
		}

		tbm_drm_helper_set_tbm_master_fd(bufmgr_bcm->fd);

		bufmgr_bcm->device_name = drmGetDeviceNameFromFd(bufmgr_bcm->fd);
		if (!bufmgr_bcm->device_name) {
			TBM_BCM_ERROR("fail to get device name!\n", getpid());

			tbm_drm_helper_unset_tbm_master_fd();
			goto fail_get_device_name;
		}
	} else {
		if (_check_render_node()) {
			bufmgr_bcm->fd = _get_render_node();//TODO
			if (bufmgr_bcm->fd < 0) {
				TBM_BCM_ERROR("fail to get render node\n");
				goto fail_get_render_node;
			}
			TBM_BCM_DEBUG("Use render node:%d\n", bufmgr_bcm->fd);
		} else {
			if (!tbm_drm_helper_get_auth_info(&(bufmgr_bcm->fd), &(bufmgr_bcm->device_name), NULL)) {
				TBM_BCM_ERROR("fail to get auth drm info!\n");
				goto fail_get_auth_info;
			}

			tbm_drm_helper_set_fd(bufmgr_bcm->fd);
		}
	}

	//Check if the tbm manager supports dma fence or not.
	fp = open("/sys/module/dmabuf_sync/parameters/enabled", O_RDONLY);
	if (fp != -1) {
		char buf[1];
		int length = read(fp, buf, 1);

		if (length == 1 && buf[0] == '1')
			bufmgr_bcm->use_dma_fence = 1;

		close(fp);
	}

	if (!_bufmgr_init_cache_state(bufmgr_bcm)) {
		TBM_BCM_ERROR("fail to init bufmgr cache state\n");
		goto fail_init_cache_state;
	}

	/*Create Hash Table*/
	bufmgr_bcm->hashBos = drmHashCreate();

	bufmgr_backend = tbm_backend_alloc();
	if (!bufmgr_backend) {
		TBM_BCM_ERROR("fail to alloc backend!\n");
		goto fail_alloc_backend;
	}

	bufmgr_backend->priv = (void *)bufmgr_bcm;
	bufmgr_backend->bufmgr_deinit = tbm_bcm_bufmgr_deinit;
	bufmgr_backend->bo_size = tbm_bcm_bo_size;
	bufmgr_backend->bo_alloc = tbm_bcm_bo_alloc;
	bufmgr_backend->bo_free = tbm_bcm_bo_free;
	bufmgr_backend->bo_import = tbm_bcm_bo_import;
	bufmgr_backend->bo_import_fd = tbm_bcm_bo_import_fd;
	bufmgr_backend->bo_export = tbm_bcm_bo_export;
	bufmgr_backend->bo_export_fd = tbm_bcm_bo_export_fd;
	bufmgr_backend->bo_get_handle = tbm_bcm_bo_get_handle;
	bufmgr_backend->bo_map = tbm_bcm_bo_map;
	bufmgr_backend->bo_unmap = tbm_bcm_bo_unmap;
	bufmgr_backend->surface_get_plane_data = tbm_bcm_surface_get_plane_data;
	bufmgr_backend->surface_supported_format = tbm_bcm_surface_supported_format;
	bufmgr_backend->bo_get_flags = tbm_bcm_bo_get_flags;
	bufmgr_backend->bo_lock = tbm_bcm_bo_lock;
	bufmgr_backend->bo_unlock = tbm_bcm_bo_unlock;

	if (tbm_backend_is_display_server() && !_check_render_node())
		bufmgr_backend->bufmgr_bind_native_display = tbm_bcm_bufmgr_bind_native_display;

	if (!tbm_backend_init(bufmgr, bufmgr_backend)) {
		TBM_BCM_ERROR("fail to init backend!\n");
		goto fail_init_backend;
	}

#ifdef DEBUG
	{
		char *env;

		env = getenv("TBM_BCM_DEBUG");
		if (env) {
			bDebug = atoi(env);
			TBM_BCM_ERROR("TBM_BCM_DEBUG=%s\n", env);
		} else
			bDebug = 0;
	}
#endif

	TBM_BCM_DEBUG("drm_fd:%d\n", bufmgr_bcm->fd);

	return 1;

fail_init_backend:
	tbm_backend_free(bufmgr_backend);
fail_alloc_backend:
	if (bufmgr_bcm->hashBos)
		drmHashDestroy(bufmgr_bcm->hashBos);
	_bufmgr_deinit_cache_state(bufmgr_bcm);
fail_init_cache_state:
	if (tbm_backend_is_display_server())
		tbm_drm_helper_unset_tbm_master_fd();
	else
		tbm_drm_helper_unset_fd();
fail_get_device_name:
	close(bufmgr_bcm->fd);
fail_get_auth_info:
fail_get_render_node:
fail_open_drm:
	free(bufmgr_bcm);
	return 0;
}

