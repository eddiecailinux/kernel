/*
 * Copyright (C) 2010 - 2012 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef CAMERIC_CORE_H_
#define CAMERIC_CORE_H_

/*#define DEBUG*/

#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/mfd/syscon.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/io.h>
#include <linux/sizes.h>

#include <media/media-entity.h>
#include <media/videobuf2-v4l2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-mediabus.h>
#include <media/drv-intf/cameric.h>

#define dbg(fmt, args...) \
	pr_debug("%s:%d: " fmt "\n", __func__, __LINE__, ##args)

/* Time to wait for next frame VSYNC interrupt while stopping operation. */
#define CAMERIC_SHUTDOWN_TIMEOUT	((100*HZ)/1000)
#define MAX_CAMERIC_CLOCKS		2
#define CAMERIC_DRIVER_NAME	"cameric4-cameric"
#define CAMERIC_MAX_DEVS		4
#define CAMERIC_MAX_OUT_BUFS	4
#define SCALER_MAX_HRATIO	64
#define SCALER_MAX_VRATIO	64
#define DMA_MIN_SIZE		8
#define CAMERIC_CAMIF_MAX_HEIGHT	0x2000
#define CAMERIC_MAX_JPEG_BUF_SIZE	(10 * SZ_1M)
#define CAMERIC_MAX_PLANES		3
#define CAMERIC_PIX_LIMITS_MAX	4
#define CAMERIC_DEF_MIN_SIZE	16
#define CAMERIC_DEF_HEIGHT_ALIGN	2
#define CAMERIC_DEF_HOR_OFFS_ALIGN	1
#define CAMERIC_DEFAULT_WIDTH	640
#define CAMERIC_DEFAULT_HEIGHT	480

/* indices to the clocks array */
enum {
	CLK_BUS,
	CLK_GATE,
};

enum cameric_dev_flags {
	ST_LPM,
	/* m2m node */
	ST_M2M_RUN,
	ST_M2M_PEND,
	ST_M2M_SUSPENDING,
	ST_M2M_SUSPENDED,
	/* capture node */
	ST_CAPT_PEND,
	ST_CAPT_RUN,
	ST_CAPT_STREAM,
	ST_CAPT_ISP_STREAM,
	ST_CAPT_SUSPENDED,
	ST_CAPT_SHUT,
	ST_CAPT_BUSY,
	ST_CAPT_APPLY_CFG,
	ST_CAPT_JPEG,
};

#define cameric_m2m_active(dev) test_bit(ST_M2M_RUN, &(dev)->state)
#define cameric_m2m_pending(dev) test_bit(ST_M2M_PEND, &(dev)->state)

#define cameric_capture_running(dev) test_bit(ST_CAPT_RUN, &(dev)->state)
#define cameric_capture_pending(dev) test_bit(ST_CAPT_PEND, &(dev)->state)
#define cameric_capture_busy(dev) test_bit(ST_CAPT_BUSY, &(dev)->state)

enum cameric_datapath {
	CAMERIC_IO_NONE,
	CAMERIC_IO_CAMERA,
	CAMERIC_IO_DMA,
	CAMERIC_IO_LCDFIFO,
	CAMERIC_IO_WRITEBACK,
	CAMERIC_IO_ISP,
};

enum cameric_color_fmt {
	CAMERIC_FMT_RGB444	= 0x10,
	CAMERIC_FMT_RGB555,
	CAMERIC_FMT_RGB565,
	CAMERIC_FMT_RGB666,
	CAMERIC_FMT_RGB888,
	CAMERIC_FMT_RGB30_LOCAL,
	CAMERIC_FMT_YCBCR420 = 0x20,
	CAMERIC_FMT_YCBYCR422,
	CAMERIC_FMT_YCRYCB422,
	CAMERIC_FMT_CBYCRY422,
	CAMERIC_FMT_CRYCBY422,
	CAMERIC_FMT_YCBCR444_LOCAL,
	CAMERIC_FMT_RAW8 = 0x40,
	CAMERIC_FMT_RAW10,
	CAMERIC_FMT_RAW12,
	CAMERIC_FMT_JPEG = 0x80,
	CAMERIC_FMT_YUYV_JPEG = 0x100,
};

#define cameric_fmt_is_user_defined(x) (!!((x) & 0x180))
#define cameric_fmt_is_rgb(x) (!!((x) & 0x10))

#define IS_M2M(__strt) ((__strt) == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE || \
			__strt == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)

/* The hardware context state. */
#define	CAMERIC_PARAMS		(1 << 0)
#define	CAMERIC_COMPOSE		(1 << 1)
#define	CAMERIC_CTX_M2M		(1 << 16)
#define	CAMERIC_CTX_CAP		(1 << 17)
#define	CAMERIC_CTX_SHUT		(1 << 18)

/* Image conversion flags */
#define	CAMERIC_IN_DMA_ACCESS_TILED	(1 << 0)
#define	CAMERIC_IN_DMA_ACCESS_LINEAR	(0 << 0)
#define	CAMERIC_OUT_DMA_ACCESS_TILED	(1 << 1)
#define	CAMERIC_OUT_DMA_ACCESS_LINEAR	(0 << 1)
#define	CAMERIC_SCAN_MODE_PROGRESSIVE	(0 << 2)
#define	CAMERIC_SCAN_MODE_INTERLACED	(1 << 2)
/*
 * YCbCr data dynamic range for RGB-YUV color conversion.
 * Y/Cb/Cr: (0 ~ 255) */
#define	CAMERIC_COLOR_RANGE_WIDE		(0 << 3)
/* Y (16 ~ 235), Cb/Cr (16 ~ 240) */
#define	CAMERIC_COLOR_RANGE_NARROW		(1 << 3)

/**
 * struct cameric_dma_offset - pixel offset information for DMA
 * @y_h:	y value horizontal offset
 * @y_v:	y value vertical offset
 * @cb_h:	cb value horizontal offset
 * @cb_v:	cb value vertical offset
 * @cr_h:	cr value horizontal offset
 * @cr_v:	cr value vertical offset
 */
struct cameric_dma_offset {
	int	y_h;
	int	y_v;
	int	cb_h;
	int	cb_v;
	int	cr_h;
	int	cr_v;
};

/**
 * struct cameric_effect - color effect information
 * @type:	effect type
 * @pat_cb:	cr value when type is "arbitrary"
 * @pat_cr:	cr value when type is "arbitrary"
 */
struct cameric_effect {
	u32	type;
	u8	pat_cb;
	u8	pat_cr;
};

/**
 * struct cameric_scaler - the configuration data for cameric inetrnal scaler
 * @scaleup_h:		flag indicating scaling up horizontally
 * @scaleup_v:		flag indicating scaling up vertically
 * @copy_mode:		flag indicating transparent DMA transfer (no scaling
 *			and color format conversion)
 * @enabled:		flag indicating if the scaler is used
 * @hfactor:		horizontal shift factor
 * @vfactor:		vertical shift factor
 * @pre_hratio:		horizontal ratio of the prescaler
 * @pre_vratio:		vertical ratio of the prescaler
 * @pre_dst_width:	the prescaler's destination width
 * @pre_dst_height:	the prescaler's destination height
 * @main_hratio:	the main scaler's horizontal ratio
 * @main_vratio:	the main scaler's vertical ratio
 * @real_width:		source pixel (width - offset)
 * @real_height:	source pixel (height - offset)
 */
struct cameric_scaler {
	unsigned int scaleup_h:1;
	unsigned int scaleup_v:1;
	unsigned int copy_mode:1;
	unsigned int enabled:1;
	u32	hfactor;
	u32	vfactor;
	u32	pre_hratio;
	u32	pre_vratio;
	u32	pre_dst_width;
	u32	pre_dst_height;
	u32	main_hratio;
	u32	main_vratio;
	u32	real_width;
	u32	real_height;
};

/**
 * struct cameric_addr - the cameric physical address set for DMA
 * @y:	 luminance plane physical address
 * @cb:	 Cb plane physical address
 * @cr:	 Cr plane physical address
 */
struct cameric_addr {
	u32	y;
	u32	cb;
	u32	cr;
};

/**
 * struct cameric_vid_buffer - the driver's video buffer
 * @vb:    v4l videobuf buffer
 * @list:  linked list structure for buffer queue
 * @paddr: precalculated physical address set
 * @index: buffer index for the output DMA engine
 */
struct cameric_vid_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head	list;
	struct cameric_addr	paddr;
	int			index;
};

/**
 * struct cameric_frame - source/target frame properties
 * @f_width:	image full width (virtual screen size)
 * @f_height:	image full height (virtual screen size)
 * @o_width:	original image width as set by S_FMT
 * @o_height:	original image height as set by S_FMT
 * @offs_h:	image horizontal pixel offset
 * @offs_v:	image vertical pixel offset
 * @width:	image pixel width
 * @height:	image pixel weight
 * @payload:	image size in bytes (w x h x bpp)
 * @bytesperline: bytesperline value for each plane
 * @paddr:	image frame buffer physical addresses
 * @dma_offset:	DMA offset in bytes
 * @fmt:	cameric color format pointer
 */
struct cameric_frame {
	u32	f_width;
	u32	f_height;
	u32	o_width;
	u32	o_height;
	u32	offs_h;
	u32	offs_v;
	u32	width;
	u32	height;
	unsigned int		payload[VIDEO_MAX_PLANES];
	unsigned int		bytesperline[VIDEO_MAX_PLANES];
	struct cameric_addr	paddr;
	struct cameric_dma_offset	dma_offset;
	struct cameric_fmt		*fmt;
	u8			alpha;
};

/**
 * struct cameric_m2m_device - v4l2 memory-to-memory device data
 * @vfd: the video device node for v4l2 m2m mode
 * @m2m_dev: v4l2 memory-to-memory device data
 * @ctx: hardware context data
 * @refcnt: the reference counter
 */
struct cameric_m2m_device {
	struct video_device	vfd;
	struct v4l2_m2m_dev	*m2m_dev;
	struct cameric_ctx		*ctx;
	int			refcnt;
};

#define CAMERIC_SD_PAD_SINK_CAM		0
#define CAMERIC_SD_PAD_SINK_FIFO	1
#define CAMERIC_SD_PAD_SOURCE		2
#define CAMERIC_SD_PADS_NUM			3

/**
 * struct cameric_vid_cap - camera capture device information
 * @ctx: hardware context data
 * @subdev: subdev exposing the cameric processing block
 * @ve: cameric video device entity structure
 * @vd_pad: cameric video capture node pad
 * @sd_pads: cameric video processing block pads
 * @ci_fmt: image format at the cameric camera input (and the scaler output)
 * @wb_fmt: image format at the cameric ISP Writeback input
 * @source_config: external image source related configuration structure
 * @pending_buf_q: the pending buffer queue head
 * @active_buf_q: the queue head of buffers scheduled in hardware
 * @vbq: the capture am video buffer queue
 * @active_buf_cnt: number of video buffers scheduled in hardware
 * @buf_index: index for managing the output DMA buffers
 * @frame_count: the frame counter for statistics
 * @reqbufs_count: the number of buffers requested in REQBUFS ioctl
 * @input_index: input (camera sensor) index
 * @input: capture input type, grp_id of the attached subdev
 * @user_subdev_api: true if subdevs are not configured by the host driver
 * @inh_sensor_ctrls: a flag indicating v4l2 controls are inherited from
 * 		      an image sensor subdev
 */
struct cameric_vid_cap {
	struct cameric_ctx			*ctx;
	struct v4l2_subdev		subdev;
	struct cameric_video_entity	ve;
	struct media_pad		vd_pad;
	struct media_pad		sd_pads[CAMERIC_SD_PADS_NUM];
	struct v4l2_mbus_framefmt	ci_fmt;
	struct v4l2_mbus_framefmt	wb_fmt;
	struct cameric_source_info		source_config;
	struct list_head		pending_buf_q;
	struct list_head		active_buf_q;
	struct vb2_queue		vbq;
	int				active_buf_cnt;
	int				buf_index;
	unsigned int			frame_count;
	unsigned int			reqbufs_count;
	bool				streaming;
	int				input_index;
	u32				input;
	bool				user_subdev_api;
	bool				inh_sensor_ctrls;
};

/**
 *  struct cameric_pix_limit - image pixel size limits in various IP configurations
 *
 *  @scaler_en_w: max input pixel width when the scaler is enabled
 *  @scaler_dis_w: max input pixel width when the scaler is disabled
 *  @in_rot_en_h: max input width with the input rotator is on
 *  @in_rot_dis_w: max input width with the input rotator is off
 *  @out_rot_en_w: max output width with the output rotator on
 *  @out_rot_dis_w: max output width with the output rotator off
 */
struct cameric_pix_limit {
	u16 scaler_en_w;
	u16 scaler_dis_w;
	u16 in_rot_en_h;
	u16 in_rot_dis_w;
	u16 out_rot_en_w;
	u16 out_rot_dis_w;
};

/**
 * struct cameric_variant - cameric device variant information
 * @has_inp_rot: set if has input rotator
 * @has_out_rot: set if has output rotator
 * @has_mainscaler_ext: 1 if extended mainscaler ratios in CIEXTEN register
 *			 are present in this IP revision
 * @has_cam_if: set if this instance has a camera input interface
 * @has_isp_wb: set if this instance has ISP writeback input
 * @pix_limit: pixel size constraints for the scaler
 * @min_inp_pixsize: minimum input pixel size
 * @min_out_pixsize: minimum output pixel size
 * @hor_offs_align: horizontal pixel offset alignment
 * @min_vsize_align: minimum vertical pixel size alignment
 */
struct cameric_variant {
	unsigned int	has_inp_rot:1;
	unsigned int	has_out_rot:1;
	unsigned int	has_mainscaler_ext:1;
	unsigned int	has_cam_if:1;
	unsigned int	has_isp_wb:1;
	const struct cameric_pix_limit *pix_limit;
	u16		min_inp_pixsize;
	u16		min_out_pixsize;
	u16		hor_offs_align;
	u16		min_vsize_align;
};

/**
 * struct cameric_drvdata - per device type driver data
 * @variant: variant information for this device
 * @num_entities: number of cameric instances available in a SoC
 * @lclk_frequency: local bus clock frequency
 * @cistatus2: 1 if the cameric IPs have CISTATUS2 register
 * @dma_pix_hoff: the horizontal DMA offset unit: 1 - pixels, 0 - bytes
 * @alpha_color: 1 if alpha color component is supported
 * @out_buf_count: maximum number of output DMA buffers supported
 */
struct cameric_drvdata {
	const struct cameric_variant *variant[CAMERIC_MAX_DEVS];
	int num_entities;
	unsigned long lclk_frequency;
	/* Fields common to all cameric IP instances */
	u8 cistatus2;
	u8 dma_pix_hoff;
	u8 alpha_color;
	u8 out_buf_count;
};

#define cameric_get_drvdata(_pdev) \
	((struct cameric_drvdata *) platform_get_device_id(_pdev)->driver_data)

struct cameric_ctx;

/**
 * struct cameric_dev - abstraction for cameric entity
 * @slock:	the spinlock protecting this data structure
 * @lock:	the mutex protecting this data structure
 * @pdev:	pointer to the cameric platform device
 * @pdata:	pointer to the device platform data
 * @sysreg:	pointer to the SYSREG regmap
 * @variant:	the IP variant information
 * @id:		cameric device index (0..cameric_MAX_DEVS)
 * @clock:	clocks required for cameric operation
 * @regs:	the mapped hardware registers
 * @irq_queue:	interrupt handler waitqueue
 * @v4l2_dev:	root v4l2_device
 * @m2m:	memory-to-memory V4L2 device information
 * @vid_cap:	camera capture device information
 * @state:	flags used to synchronize m2m and capture mode operation
 * @pipeline:	cameric video capture pipeline data structure
 */
struct cameric_dev {
	spinlock_t			slock;
	struct mutex			lock;
	struct platform_device		*pdev;
	struct s5p_platform_cameric	*pdata;
	struct regmap			*sysreg;
	struct regmap *regmap_grf;
	const struct cameric_variant	*variant;
	const struct cameric_drvdata	*drv_data;
	int				id;
	int irq;
	struct clk			*clock[MAX_CAMERIC_CLOCKS];
	void __iomem			*regs;
	void __iomem *base_addr;

	void __iomem *csihost_base;
	void __iomem *isp_base;

	struct cameric_clk_rk3288 *clk_rst;
	wait_queue_head_t		irq_queue;
	struct v4l2_device		*v4l2_dev;
	struct cameric_m2m_device		m2m;
	struct cameric_vid_cap		vid_cap;
	unsigned long			state;
};

/**
 * struct cameric_ctrls - v4l2 controls structure
 * @handler: the control handler
 * @colorfx: image effect control
 * @colorfx_cbcr: Cb/Cr coefficients control
 * @rotate: image rotation control
 * @hflip: horizontal flip control
 * @vflip: vertical flip control
 * @alpha: RGB alpha control
 * @ready: true if @handler is initialized
 */
struct cameric_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *colorfx;
		struct v4l2_ctrl *colorfx_cbcr;
	};
	struct v4l2_ctrl *rotate;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *alpha;
	bool ready;
};

/**
 * cameric_ctx - the device context data
 * @s_frame:		source frame properties
 * @d_frame:		destination frame properties
 * @out_order_1p:	output 1-plane YCBCR order
 * @out_order_2p:	output 2-plane YCBCR order
 * @in_order_1p		input 1-plane YCBCR order
 * @in_order_2p:	input 2-plane YCBCR order
 * @in_path:		input mode (DMA or camera)
 * @out_path:		output mode (DMA or FIFO)
 * @scaler:		image scaler properties
 * @effect:		image effect
 * @rotation:		image clockwise rotation in degrees
 * @hflip:		indicates image horizontal flip if set
 * @vflip:		indicates image vertical flip if set
 * @flags:		additional flags for image conversion
 * @state:		flags to keep track of user configuration
 * @cameric_dev:		the cameric device this context applies to
 * @fh:			v4l2 file handle
 * @ctrls:		v4l2 controls structure
 */
struct cameric_ctx {
	struct cameric_frame	s_frame;
	struct cameric_frame	d_frame;
	u32			out_order_1p;
	u32			out_order_2p;
	u32			in_order_1p;
	u32			in_order_2p;
	enum cameric_datapath	in_path;
	enum cameric_datapath	out_path;
	struct cameric_scaler	scaler;
	struct cameric_effect	effect;
	int			rotation;
	unsigned int		hflip:1;
	unsigned int		vflip:1;
	u32			flags;
	u32			state;
	struct cameric_dev		*cameric_dev;
	struct v4l2_fh		fh;
	struct cameric_ctrls	ctrls;
};

#define fh_to_ctx(__fh) container_of(__fh, struct cameric_ctx, fh)

static inline void set_frame_bounds(struct cameric_frame *f, u32 width, u32 height)
{
	f->o_width  = width;
	f->o_height = height;
	f->f_width  = width;
	f->f_height = height;
}

static inline void set_frame_crop(struct cameric_frame *f,
				  u32 left, u32 top, u32 width, u32 height)
{
	f->offs_h = left;
	f->offs_v = top;
	f->width  = width;
	f->height = height;
}

static inline u32 cameric_get_format_depth(struct cameric_fmt *ff)
{
	u32 i, depth = 0;

	if (ff != NULL)
		for (i = 0; i < ff->colplanes; i++)
			depth += ff->depth[i];
	return depth;
}

static inline bool cameric_capture_active(struct cameric_dev *cameric)
{
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&cameric->slock, flags);
	ret = !!(cameric->state & (1 << ST_CAPT_RUN) ||
		 cameric->state & (1 << ST_CAPT_PEND));
	spin_unlock_irqrestore(&cameric->slock, flags);
	return ret;
}

static inline void cameric_ctx_state_set(u32 state, struct cameric_ctx *ctx)
{
	unsigned long flags;

	spin_lock_irqsave(&ctx->cameric_dev->slock, flags);
	ctx->state |= state;
	spin_unlock_irqrestore(&ctx->cameric_dev->slock, flags);
}

static inline bool cameric_ctx_state_is_set(u32 mask, struct cameric_ctx *ctx)
{
	unsigned long flags;
	bool ret;

	spin_lock_irqsave(&ctx->cameric_dev->slock, flags);
	ret = (ctx->state & mask) == mask;
	spin_unlock_irqrestore(&ctx->cameric_dev->slock, flags);
	return ret;
}

static inline int tiled_fmt(struct cameric_fmt *fmt)
{
	return fmt->fourcc == V4L2_PIX_FMT_NV12MT;
}

static inline bool cameric_jpeg_fourcc(u32 pixelformat)
{
	return (pixelformat == V4L2_PIX_FMT_JPEG ||
		pixelformat == V4L2_PIX_FMT_S5C_UYVY_JPG);
}

static inline bool cameric_user_defined_mbus_fmt(u32 code)
{
	return (code == MEDIA_BUS_FMT_JPEG_1X8 ||
		code == MEDIA_BUS_FMT_S5C_UYVY_JPEG_1X8);
}

/* Return the alpha component bit mask */
static inline int cameric_get_alpha_mask(struct cameric_fmt *fmt)
{
	switch (fmt->color) {
	case CAMERIC_FMT_RGB444:	return 0x0f;
	case CAMERIC_FMT_RGB555:	return 0x01;
	case CAMERIC_FMT_RGB888:	return 0xff;
	default:		return 0;
	};
}

static inline struct cameric_frame *ctx_get_frame(struct cameric_ctx *ctx,
					       enum v4l2_buf_type type)
{
	struct cameric_frame *frame;

	if (V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE == type) {
		if (cameric_ctx_state_is_set(CAMERIC_CTX_M2M, ctx))
			frame = &ctx->s_frame;
		else
			return ERR_PTR(-EINVAL);
	} else if (V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE == type) {
		frame = &ctx->d_frame;
	} else {
		v4l2_err(ctx->cameric_dev->v4l2_dev,
			"Wrong buffer/video queue type (%d)\n", type);
		return ERR_PTR(-EINVAL);
	}

	return frame;
}

/* -----------------------------------------------------*/
/* cameric-core.c */
int cameric_vidioc_enum_fmt_mplane(struct file *file, void *priv,
				struct v4l2_fmtdesc *f);
int cameric_ctrls_create(struct cameric_ctx *ctx);
void cameric_ctrls_delete(struct cameric_ctx *ctx);
void cameric_ctrls_activate(struct cameric_ctx *ctx, bool active);
void cameric_alpha_ctrl_update(struct cameric_ctx *ctx);
void __cameric_get_format(struct cameric_frame *frame, struct v4l2_format *f);
void cameric_adjust_mplane_format(struct cameric_fmt *fmt, u32 width, u32 height,
			       struct v4l2_pix_format_mplane *pix);
struct cameric_fmt *cameric_find_format(const u32 *pixelformat, const u32 *mbus_code,
				  unsigned int mask, int index);
struct cameric_fmt *cameric_get_format(unsigned int index);

int cameric_check_scaler_ratio(struct cameric_ctx *ctx, int sw, int sh,
			    int dw, int dh, int rotation);
int cameric_set_scaler_info(struct cameric_ctx *ctx);
int cameric_prepare_config(struct cameric_ctx *ctx, u32 flags);
int cameric_prepare_addr(struct cameric_ctx *ctx, struct vb2_buffer *vb,
		      struct cameric_frame *frame, struct cameric_addr *paddr);
void cameric_prepare_dma_offset(struct cameric_ctx *ctx, struct cameric_frame *f);
void cameric_set_yuv_order(struct cameric_ctx *ctx);
void cameric_capture_irq_handler(struct cameric_dev *cameric, int deq_buf);

int cameric_register_m2m_device(struct cameric_dev *cameric,
			     struct v4l2_device *v4l2_dev);
void cameric_unregister_m2m_device(struct cameric_dev *cameric);
int cameric_register_driver(void);
void cameric_unregister_driver(void);

#ifdef CONFIG_MFD_SYSCON
static inline struct regmap * cameric_get_sysreg_regmap(struct device_node *node)
{
	printk(KERN_INFO "%s \n", __func__);
	return syscon_regmap_lookup_by_phandle(node, "samsung,sysreg");
}
#else
#define cameric_get_sysreg_regmap(node) (NULL)
#endif

/* -----------------------------------------------------*/
/* cameric-m2m.c */
void cameric_m2m_job_finish(struct cameric_ctx *ctx, int vb_state);

/* -----------------------------------------------------*/
/* cameric-capture.c					*/
int cameric_initialize_capture_subdev(struct cameric_dev *cameric);
void cameric_unregister_capture_subdev(struct cameric_dev *cameric);
int cameric_capture_ctrls_create(struct cameric_dev *cameric);
void cameric_sensor_notify(struct v4l2_subdev *sd, unsigned int notification,
			void *arg);
int cameric_capture_suspend(struct cameric_dev *cameric);
int cameric_capture_resume(struct cameric_dev *cameric);

/*
 * Buffer list manipulation functions. Must be called with cameric.slock held.
 */

/**
 * cameric_active_queue_add - add buffer to the capture active buffers queue
 * @buf: buffer to add to the active buffers list
 */
static inline void cameric_active_queue_add(struct cameric_vid_cap *vid_cap,
					 struct cameric_vid_buffer *buf)
{
	list_add_tail(&buf->list, &vid_cap->active_buf_q);
	vid_cap->active_buf_cnt++;
}

/**
 * cameric_active_queue_pop - pop buffer from the capture active buffers queue
 *
 * The caller must assure the active_buf_q list is not empty.
 */
static inline struct cameric_vid_buffer *cameric_active_queue_pop(
				    struct cameric_vid_cap *vid_cap)
{
	struct cameric_vid_buffer *buf;
	buf = list_entry(vid_cap->active_buf_q.next,
			 struct cameric_vid_buffer, list);
	list_del(&buf->list);
	vid_cap->active_buf_cnt--;
	return buf;
}

/**
 * cameric_pending_queue_add - add buffer to the capture pending buffers queue
 * @buf: buffer to add to the pending buffers list
 */
static inline void cameric_pending_queue_add(struct cameric_vid_cap *vid_cap,
					  struct cameric_vid_buffer *buf)
{
	list_add_tail(&buf->list, &vid_cap->pending_buf_q);
}

/**
 * cameric_pending_queue_pop - pop buffer from the capture pending buffers queue
 *
 * The caller must assure the pending_buf_q list is not empty.
 */
static inline struct cameric_vid_buffer *cameric_pending_queue_pop(
				     struct cameric_vid_cap *vid_cap)
{
	struct cameric_vid_buffer *buf;
	buf = list_entry(vid_cap->pending_buf_q.next,
			struct cameric_vid_buffer, list);
	list_del(&buf->list);
	return buf;
}

#endif /* CAMERIC_CORE_H_ */
