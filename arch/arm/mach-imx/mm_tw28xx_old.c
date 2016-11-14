#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/videodev2.h>

#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>

#define TW28XX_MAX_WIDTH		(720)
#define TW28XX_MAX_HEIGHT		(288)
#define TW28XX_MIN_WIDTH		48
#define TW28XX_MIN_HEIGHT		32
#define TW28XX_COLUMN_SKIP		0
#define TW28XX_ROW_SKIP			0

#define TW28XX_REG_DEVICE_ID		0x0fe

//#define TW2824_REG_022(x)		(0x022 + (x << 6))
//#define TW2824_REG_100			0x100
//#define TW2824_REG_15f			0x15f
//#define TW2824_REG_171			0x171
//#define TW2824_REG_179			0x179
//#define TW2824_REG_181			0x181
//#define TW2824_REG_27e			0x27e

//#define TW2835_REG_00d(x)		(0x00d + (x << 4))
//#define TW2835_REG_1ac			0x1ac

#define TW2824_FORMAT			V4L2_MBUS_FMT_YUYV8_2X8
#define TW2824_COLORSPACE		V4L2_COLORSPACE_JPEG

#define TW2824_CHIP_ID			0x2824
#define TW2835_CHIP_ID			0x2835
#define TW2836_CHIP_ID			0x2836
#define TW2837_CHIP_ID			0x2837

struct tw28xx_t
{
	int			chip_id;
	struct v4l2_subdev	subdev;
	struct v4l2_rect	rect;
};

static struct tw28xx_t *to_tw28xx(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct tw28xx_t, subdev);
}

static int tw28xx_read(struct i2c_client *client, const uint16_t reg)
{
	int status;
	u8 ret, msgbuf[2] = { reg >> 8, (u8)reg };
	struct i2c_msg msg[2];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = (char*)&msgbuf;
	msg[0].len = sizeof(msgbuf);
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &ret;
	msg[1].len = 1;
	status = i2c_transfer(client->adapter, msg, 2);

	return (status == 2) ? ret : -EIO;
}

static int tw28xx_write(struct i2c_client *client, const uint16_t reg, const u8 data)
{
	u8 msgbuf[3] = { reg >> 8, (u8)reg, data };
	struct i2c_msg msg;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = (char*)&msgbuf;
	msg.len = sizeof(msgbuf);

	return (i2c_transfer(client->adapter, &msg, 1) == 1) ? 0 : -EIO;
}

static int tw28xx_set(struct i2c_client *client, const uint16_t reg, const u8 data)
{
	int ret = tw28xx_read(client, reg);

	return (ret < 0) ? ret : tw28xx_write(client, reg, ret | data);
}

static int tw28xx_clear(struct i2c_client *client, const uint16_t reg, const u8 data)
{
	int ret = tw28xx_read(client, reg);

	return (ret < 0) ? ret : tw28xx_write(client, reg, ret & ~data);
}

/* **************************************************************************************************** */

static u8 tw2824_init_000[] = {
	0x00, 0x84, 0xa5, 0x2e, 0xd0, 0x2d, 0xd0, 0x88,
	0x20, 0x08, 0x20, 0x08, 0x20, 0x0a, 0xd2, 0x80,
	0x80, 0x80, 0x80, 0x2f, 0x50, 0xa0, 0x00, 0xc0,
	0x7f, 0xff, 0x85, 0xff, 0x7f, 0xff, 0x85, 0xff,
	0x07, 0x07, 0x00, 0x91,
};

static u8 tw2824_init_038[] = {
	0x00, 0x00, 0xf0, 0x82, 0x80, 0x80, 0x82, 0x82,
};

static u8 tw2824_init_078[] = {
	0x00, 0x80, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
};

static u8 tw2824_init_0b8[] = {
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static u8 tw2824_init_0f8[] = {
	0x0a, 0x40, 0x3c, 0x10, 0xf0, 0x00, 0x08, 0x00,
};

static u8 tw2824_init_101[] = {
	0x00, 0x00, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x23, 0x2d, 0x24, 0xaf,
};

static u8 tw2824_init_110[] = {
	0x00, 0x02, 0x00, 0x00, 0x5a, 0x00, 0x48, 0x01,
	0x02, 0x00, 0x5a, 0xb4, 0x00, 0x48, 0x02, 0x02,
	0x00, 0x00, 0x5a, 0x48, 0x90, 0x03, 0x02, 0x00,
	0x5a, 0xb4, 0x48, 0x90,
};

static u8 tw2824_init_12c[] = {
	0x20, 0x00, 0x00, 0x00,
};

static u8 tw2824_init_15c[] = {
	0x00, 0x00, 0x00, 0x06,
};

static u8 tw2824_init_160[] = {
	0x78, 0xb4, 0x30, 0x60, 0x00, 0x3c, 0x60, 0x90,
	0x3c, 0x78, 0x60, 0x90, 0x78, 0xb4, 0x60, 0x90,
};

static u8 tw2824_init_170[] = {
	0x77, 0x16, 0x01, 0x80, 0x10, 0x00,
};

//static int tw2824_init(struct soc_camera_device *icd)
//{
//	struct tw28xx_t *tw28xx = container_of(icd, struct tw28xx_t, icd);
//	struct soc_camera_link *icl = tw28xx->client->dev.platform_data;
//	int i, ret;
//
//printk("tw28xx_init\n");
//	if (icl->power)
//	{
//		ret = icl->power(&tw28xx->client->dev, 1);
//		if (ret < 0)
//		{
//			dev_err(icd->vdev->parent, "Platform failed to power-on the camera.\n");
//			return ret;
//		}
//	}
//
//	if (icl->reset)
//		icl->reset(&tw28xx->client->dev);
//
//	/* Soft reset */
//	for (i = 0; i < 4; i++)
//	{
//		ret = tw28xx_write(tw28xx->client, TW2824_REG_022(i), 0x08);
//		udelay(200);
//		if ((ret < 0) || tw28xx_read(tw28xx->client, TW2824_REG_022(i)))
//		{
//			dev_err(&icd->dev, "Resetting TW2824:%u failed!\n", i);
//			return -EIO;
//		}
//	}
//
//	/* Initialize SDRAM */
//	ret = tw28xx_write(tw28xx->client, TW2824_REG_15f, 0x80);
//	udelay(200);
//	if ((ret < 0) || tw28xx_read(tw28xx->client, TW2824_REG_15f))
//	{
//		dev_err(&icd->dev, "Init SDRAM TW2824 failed!\n");
//		return -EIO;
//	}
//
//	/* Init from Datasheet */
//	for (i = 0; i < sizeof(tw2824_init_000); i++)
//		tw28xx_write(tw28xx->client, i + 0x000, tw2824_init_000[i]);
//	for (i = 0; i < sizeof(tw2824_init_000); i++)
//		tw28xx_write(tw28xx->client, i + 0x040, tw2824_init_000[i]);
//	for (i = 0; i < sizeof(tw2824_init_000); i++)
//		tw28xx_write(tw28xx->client, i + 0x080, tw2824_init_000[i]);
//	for (i = 0; i < sizeof(tw2824_init_000); i++)
//		tw28xx_write(tw28xx->client, i + 0x0c0, tw2824_init_000[i]);
//	for (i = 0; i < sizeof(tw2824_init_038); i++)
//		tw28xx_write(tw28xx->client, i + 0x038, tw2824_init_038[i]);
//	for (i = 0; i < sizeof(tw2824_init_078); i++)
//		tw28xx_write(tw28xx->client, i + 0x078, tw2824_init_078[i]);
//	for (i = 0; i < sizeof(tw2824_init_0b8); i++)
//		tw28xx_write(tw28xx->client, i + 0x0b8, tw2824_init_0b8[i]);
//	for (i = 0; i < sizeof(tw2824_init_0f8); i++)
//		tw28xx_write(tw28xx->client, i + 0x0f8, tw2824_init_0f8[i]);
//	tw28xx_write(tw28xx->client, 0x100, 0x80);
//	for (i = 0; i < sizeof(tw2824_init_101); i++)
//		tw28xx_write(tw28xx->client, i + 0x101, tw2824_init_101[i]);
//	for (i = 0; i < sizeof(tw2824_init_101); i++)
//		tw28xx_write(tw28xx->client, i + 0x131, tw2824_init_101[i]);
//	for (i = 0; i < sizeof(tw2824_init_110); i++)
//		tw28xx_write(tw28xx->client, i + 0x110, tw2824_init_110[i]);
//	for (i = 0; i < sizeof(tw2824_init_110); i++)
//		tw28xx_write(tw28xx->client, i + 0x140, tw2824_init_110[i]);
//	for (i = 0; i < sizeof(tw2824_init_12c); i++)
//		tw28xx_write(tw28xx->client, i + 0x12c, tw2824_init_12c[i]);
//	for (i = 0; i < sizeof(tw2824_init_15c); i++)
//		tw28xx_write(tw28xx->client, i + 0x15c, tw2824_init_15c[i]);
//	for (i = 0; i < sizeof(tw2824_init_160); i++)
//		tw28xx_write(tw28xx->client, i + 0x160, tw2824_init_160[i]);
//	for (i = 0; i < sizeof(tw2824_init_170); i++)
//		tw28xx_write(tw28xx->client, i + 0x170, tw2824_init_170[i]);
//
//	/* 50Hz, 625 lines */
//	ret = tw28xx_write(tw28xx->client, TW2824_REG_100, 0x80);
//	/* Enable power down for DAC */
//	if (!ret)
//		ret = tw28xx_write(tw28xx->client, TW2824_REG_171, 0x77);
//	/* Clear display RAM */
//	if (!ret)
//		ret = tw28xx_write(tw28xx->client, TW2824_REG_181, 0x30);
//	/* Disable motion and blind detection */
//	if (!ret)
//		ret = tw28xx_write(tw28xx->client, TW2824_REG_27e, 0xf0);
//
//	/* All defaults */
//	if (!ret)
//		/* AEC, AGC on */
//		ret = tw28xx_set(icd, MT9V022_AEC_AGC_ENABLE, 0x3);
//	if (!ret)
//		ret = tw28xx_write(icd, MT9V022_MAX_TOTAL_SHUTTER_WIDTH, 480);
//	if (!ret)
//		/* default - auto */
//		ret = tw28xx_clear(icd, MT9V022_BLACK_LEVEL_CALIB_CTRL, 1);
//	if (!ret)
//		ret = tw28xx_write(icd, MT9V022_DIGITAL_TEST_PATTERN, 0);
//
//	return ret;
//}

/* ************************************************************************************************* */

static int tw28xx_init(struct i2c_client *client)
{
	struct tw28xx_t *tw28xx = to_tw28xx(client);
	int ret = 0;

printk("tw28xx: init\n");
	/*
	 * Almost the default mode: master, parallel, simultaneous, and an
	 * undocumented bit 0x200, which is present in table 7, but not in 8,
	 * plus snapshot mode to disable scan for now
	 */
//	mt9v022->chip_control |= 0x10;
//	ret = reg_write(client, MT9V022_CHIP_CONTROL, mt9v022->chip_control);
//	if (!ret)
//		ret = reg_write(client, MT9V022_READ_MODE, 0x300);
//
//	/* All defaults */
//	if (!ret)
//		/* AEC, AGC on */
//		ret = reg_set(client, MT9V022_AEC_AGC_ENABLE, 0x3);
//	if (!ret)
//		ret = reg_write(client, MT9V022_ANALOG_GAIN, 16);
//	if (!ret)
//		ret = reg_write(client, MT9V022_TOTAL_SHUTTER_WIDTH, 480);
//	if (!ret)
//		ret = reg_write(client, MT9V022_MAX_TOTAL_SHUTTER_WIDTH, 480);
//	if (!ret)
//		/* default - auto */
//		ret = reg_clear(client, MT9V022_BLACK_LEVEL_CALIB_CTRL, 1);
//	if (!ret)
//		ret = reg_write(client, MT9V022_DIGITAL_TEST_PATTERN, 0);

	return ret;
}

static int tw28xx_set_bus_param(struct soc_camera_device *icd, unsigned long flags)
{
	int ret = 0;
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct tw28xx_t *tw28xx = to_tw28xx(client);
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	if ((flags & SOCAM_DATAWIDTH_MASK) != SOCAM_DATAWIDTH_8)
		return -EINVAL;

	if (!(flags & SOCAM_MASTER))
		return -EINVAL;

	//TODO:
	//
//	ret = tw28xx_write(tw28xx->client, TW2824_REG_15f, (flags & SOCAM_PCLK_SAMPLE_FALLING) ? 0x60 : 0x00);
//	if (ret)
//		return ret;
	//

	if (icl->set_bus_param)
		return icl->set_bus_param(icl, flags);

	return 0;
}

static unsigned long tw28xx_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned int flags = SOCAM_MASTER | SOCAM_DATAWIDTH_8 | SOCAM_DATA_ACTIVE_HIGH | SOCAM_PCLK_SAMPLE_RISING | SOCAM_HSYNC_ACTIVE_LOW | SOCAM_VSYNC_ACTIVE_LOW;

	if (icl->query_bus_param)
		flags = icl->query_bus_param(icl);

	return soc_camera_apply_sensor_flags(icl, flags);
}

static int tw28xx_s_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tw28xx_t *tw28xx = to_tw28xx(client);
	struct v4l2_rect rect = a->c;
//	int ret;

	rect.width	= ALIGN(rect.width, 2);//?
	rect.height	= ALIGN(rect.height, 2);//?

	soc_camera_limit_side(&rect.left, &rect.width, TW28XX_COLUMN_SKIP, TW28XX_MIN_WIDTH, TW28XX_MAX_WIDTH);
	soc_camera_limit_side(&rect.top, &rect.height, TW28XX_ROW_SKIP, TW28XX_MIN_HEIGHT, TW28XX_MAX_HEIGHT);

//	/* Like in example app. Contradicts the datasheet though */
//	ret = reg_read(client, MT9V022_AEC_AGC_ENABLE);
//	if (ret >= 0) {
//		if (ret & 1) /* Autoexposure */
//			ret = reg_write(client, MT9V022_MAX_TOTAL_SHUTTER_WIDTH,
//					rect.height + mt9v022->y_skip_top + 43);
//		else
//			ret = reg_write(client, MT9V022_TOTAL_SHUTTER_WIDTH,
//					rect.height + mt9v022->y_skip_top + 43);
//	}
//	/* Setup frame format: defaults apart from width and height */
//	if (!ret)
//		ret = reg_write(client, MT9V022_COLUMN_START, rect.left);
//	if (!ret)
//		ret = reg_write(client, MT9V022_ROW_START, rect.top);
//	if (!ret)
//		/*
//		 * Default 94, Phytec driver says:
//		 * "width + horizontal blank >= 660"
//		 */
//		ret = reg_write(client, MT9V022_HORIZONTAL_BLANKING,
//				rect.width > 660 - 43 ? 43 :
//				660 - rect.width);
//	if (!ret)
//		ret = reg_write(client, MT9V022_VERTICAL_BLANKING, 45);
//	if (!ret)
//		ret = reg_write(client, MT9V022_WINDOW_WIDTH, rect.width);
//	if (!ret)
//		ret = reg_write(client, MT9V022_WINDOW_HEIGHT,
//				rect.height + mt9v022->y_skip_top);
//
//	if (ret < 0)
//		return ret;

	dev_dbg(&client->dev, "Frame %dx%d pixel\n", rect.width, rect.height);

	tw28xx->rect = rect;

	return 0;
}

static int tw28xx_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tw28xx_t *tw28xx = to_tw28xx(client);

	a->c	= tw28xx->rect;
	a->type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static int tw28xx_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= TW28XX_COLUMN_SKIP;
	a->bounds.top			= TW28XX_ROW_SKIP;
	a->bounds.width			= TW28XX_MAX_WIDTH;
	a->bounds.height		= TW28XX_MAX_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int tw28xx_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tw28xx_t *tw28xx = to_tw28xx(client);

	mf->width	= tw28xx->rect.width;
	mf->height	= tw28xx->rect.height;
	mf->code	= TW2824_FORMAT;
	mf->colorspace	= TW2824_COLORSPACE;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int tw28xx_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tw28xx_t *tw28xx = to_tw28xx(client);
	struct v4l2_crop a = {
		.c = {
			.left	= tw28xx->rect.left,
			.top	= tw28xx->rect.top,
			.width	= mf->width,
			.height	= mf->height,
		},
	};
	int ret;

printk("tw28xx: s_fmt\n");
	if (mf->code != TW2824_FORMAT)
		return -EINVAL;

	/* TODO: No support for scaling on this camera, just crop. */
	ret = tw28xx_s_crop(sd, &a);
	if (!ret)
	{
		mf->width	= tw28xx->rect.width;
		mf->height	= tw28xx->rect.height;
		mf->colorspace	= TW2824_COLORSPACE;
	}

	return ret;
}

static int tw28xx_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	int align = mf->code == TW2824_FORMAT;

	v4l_bound_align_image(&mf->width, TW28XX_MIN_WIDTH, TW28XX_MAX_WIDTH, align, &mf->height, TW28XX_MIN_HEIGHT, TW28XX_MAX_HEIGHT, align, 0);

	mf->code = TW2824_FORMAT;
	mf->colorspace	= TW2824_COLORSPACE;

	return 0;
}

static int tw28xx_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

printk("tw28xx: ident\n");
	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -ENODEV;

	id->ident	= V4L2_IDENT_TVAUDIO;
	id->revision	= 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int tw28xx_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	reg->size = 1;
	reg->val = tw28xx_read(client, reg->reg);

	if (reg->val > 0xffff)
		return -EIO;

	return 0;
}

static int tw28xx_s_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	if (tw28xx_write(client, reg->reg, reg->val) < 0)
		return -EIO;

	return 0;
}
#endif

static const struct v4l2_queryctrl tw28xx_controls[] = {
	{
		.id		= V4L2_CID_VFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Vertically",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	}, {
		.id		= V4L2_CID_HFLIP,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Flip Horizontally",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	}, {
		.id		= V4L2_CID_GAIN,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Analog Gain",
		.minimum	= 64,
		.maximum	= 127,
		.step		= 1,
		.default_value	= 64,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_EXPOSURE,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Exposure",
		.minimum	= 1,
		.maximum	= 255,
		.step		= 1,
		.default_value	= 255,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	}, {
		.id		= V4L2_CID_AUTOGAIN,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Automatic Gain",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 1,
	}, {
		.id		= V4L2_CID_EXPOSURE_AUTO,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Automatic Exposure",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 1,
	}, {
		.id		= V4L2_CID_COLOR_KILLER,
		.type		= V4L2_CTRL_TYPE_BOOLEAN,
		.name		= "Color Killer",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
	},
};

static struct soc_camera_ops tw28xx_ops = {
	.set_bus_param		= tw28xx_set_bus_param,
	.query_bus_param	= tw28xx_query_bus_param,
	.controls		= tw28xx_controls,
	.num_controls		= ARRAY_SIZE(tw28xx_controls),
};

static int tw28xx_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct v4l2_queryctrl *qctrl;
	unsigned long range;
	int data;

	qctrl = soc_camera_find_qctrl(&tw28xx_ops, ctrl->id);
#if 0
	switch (ctrl->id)
	{
		case V4L2_CID_VFLIP:
			data = tw28xx_read(icd, MT9V022_READ_MODE);
			if (data < 0)
				return -EIO;
			ctrl->value = !!(data & 0x10);
			break;
		case V4L2_CID_HFLIP:
			data = tw28xx_read(icd, MT9V022_READ_MODE);
			if (data < 0)
				return -EIO;
			ctrl->value = !!(data & 0x20);
			break;
		case V4L2_CID_EXPOSURE_AUTO:
			data = tw28xx_read(icd, MT9V022_AEC_AGC_ENABLE);
			if (data < 0)
				return -EIO;
			ctrl->value = !!(data & 0x1);
			break;
		case V4L2_CID_AUTOGAIN:
			data = tw28xx_read(icd, MT9V022_AEC_AGC_ENABLE);
			if (data < 0)
				return -EIO;
			ctrl->value = !!(data & 0x2);
			break;
		case V4L2_CID_COLOR_KILLER:
			data = tw28xx_read(tw28xx->client, TW2824_REG_179);
			if (data < 0)
				return -EIO;
			ctrl->value = !!(data & 0x50);
			break;
	}
#endif
	return 0;
}

static int tw28xx_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	int data;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct v4l2_queryctrl *qctrl;

	qctrl = soc_camera_find_qctrl(&tw28xx_ops, ctrl->id);
	if (!qctrl)
		return -EINVAL;

#if 0
	switch (ctrl->id)
	{
		case V4L2_CID_VFLIP:
			if (ctrl->value)
				data = tw28xx_set(icd, MT9V022_READ_MODE, 0x10);
			else
				data = tw28xx_clear(icd, MT9V022_READ_MODE, 0x10);
			if (data < 0)
				return -EIO;
			break;
		case V4L2_CID_HFLIP:
			if (ctrl->value)
				data = tw28xx_set(icd, MT9V022_READ_MODE, 0x20);
			else
				data = tw28xx_clear(icd, MT9V022_READ_MODE, 0x20);
			if (data < 0)
				return -EIO;
			break;
		case V4L2_CID_GAIN:
			/* mt9v022 has minimum == default */
			if (ctrl->value > qctrl->maximum || ctrl->value < qctrl->minimum)
				return -EINVAL;
			else {
				unsigned long range = qctrl->maximum - qctrl->minimum;
				/* Datasheet says 16 to 64. autogain only works properly
				 * after setting gain to maximum 14. Larger values
				 * produce "white fly" noise effect. On the whole,
				 * manually setting analog gain does no good. */
				unsigned long gain = ((ctrl->value - qctrl->minimum) *
						      10 + range / 2) / range + 4;
				if (gain >= 32)
					gain &= ~1;
				/* The user wants to set gain manually, hope, she
				 * knows, what she's doing... Switch AGC off. */

				if (tw28xx_clear(icd, MT9V022_AEC_AGC_ENABLE, 0x2) < 0)
					return -EIO;

				dev_info(&icd->dev, "Setting gain from %d to %lu\n",
					 tw28xx_read(icd, MT9V022_ANALOG_GAIN), gain);
				if (tw28xx_write(icd, MT9V022_ANALOG_GAIN, gain) < 0)
					return -EIO;
				icd->gain = ctrl->value;
			}
			break;
		case V4L2_CID_EXPOSURE:
			/* mt9v022 has maximum == default */
			if (ctrl->value > qctrl->maximum || ctrl->value < qctrl->minimum)
				return -EINVAL;
			else {
				unsigned long range = qctrl->maximum - qctrl->minimum;
				unsigned long shutter = ((ctrl->value - qctrl->minimum) *
						 479 + range / 2) / range + 1;
				/* The user wants to set shutter width manually, hope,
				 * she knows, what she's doing... Switch AEC off. */

				if (tw28xx_clear(icd, MT9V022_AEC_AGC_ENABLE, 0x1) < 0)
					return -EIO;

				dev_dbg(&icd->dev, "Shutter width from %d to %lu\n",
					tw28xx_read(icd, MT9V022_TOTAL_SHUTTER_WIDTH),
					shutter);
				if (tw28xx_write(icd, MT9V022_TOTAL_SHUTTER_WIDTH,
					      shutter) < 0)
					return -EIO;
				icd->exposure = ctrl->value;
			}
			break;
		case V4L2_CID_AUTOGAIN:
			if (ctrl->value)
				data = tw28xx_set(icd, MT9V022_AEC_AGC_ENABLE, 0x2);
			else
				data = tw28xx_clear(icd, MT9V022_AEC_AGC_ENABLE, 0x2);
			if (data < 0)
				return -EIO;
			break;
		case V4L2_CID_EXPOSURE_AUTO:
			if (ctrl->value)
				data = tw28xx_set(icd, MT9V022_AEC_AGC_ENABLE, 0x1);
			else
				data = tw28xx_clear(icd, MT9V022_AEC_AGC_ENABLE, 0x1);
			if (data < 0)
				return -EIO;
			break;
		case V4L2_CID_COLOR_KILLER:
			if (ctrl->value)
				data = tw28xx_set(tw28xx->client, TW2824_REG_179, 0x50);
			else
				data = tw28xx_clear(tw28xx->client, TW2824_REG_179, 0x50);
			if (data < 0)
				return -EIO;
			break;
	}
#endif

	return 0;
}

static int tw28xx_video_probe(struct soc_camera_device *icd, struct i2c_client *client)
{
	struct tw28xx_t *tw28xx = to_tw28xx(client);
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	s32 data;
	int ret;
	unsigned long flags;

	if (!icd->dev.parent || (to_soc_camera_host(icd->dev.parent)->nr != icd->iface))
		return -ENODEV;

	/* Read out the chip version register */
//	data = tw28xx_read(client, TW28XX_REG_DEVICE_ID);

//	switch (data >> 3)
//	{
//		case 1:
			tw28xx->chip_id = TW2824_CHIP_ID;
			dev_info(&client->dev, "Detected TW2824 chip, rev%u.\n", data & 0x07);
//			break;
//		case 5:
//			tw28xx->chip_id = TW2835_CHIP_ID;
//			dev_info(&client->dev, "Detected TW2835 chip, rev%u.\n", data & 0x07);
//			break;
//		default:
//			dev_info(&client->dev, "No TW28xx found, ID register 0x%08x\n", data);
//			goto ei2c;
//	}

	/* Soft reset */
//	ret = reg_write(client, MT9V022_RESET, 1);
//	if (ret < 0)
//		goto ei2c;
	/* 15 clock cycles */
//	udelay(200);
//	if (reg_read(client, MT9V022_RESET)) {
//		dev_err(&client->dev, "Resetting MT9V022 failed!\n");
//		if (ret > 0)
//			ret = -EIO;
//		goto ei2c;
//	}

	ret = tw28xx_init(client);
	if (ret < 0)
		dev_err(&client->dev, "Failed to initialise the chip\n");

	/* Enable color bars */
//	tw28xx_write(tw28xx->client, TW2824_REG_179, 0x80);

ei2c:
	return ret;
}

static int tw28xx_enum_fmt(struct v4l2_subdev *sd, unsigned int index, enum v4l2_mbus_pixelcode *code)
{
	if (index > 0)
		return -EINVAL;

	*code = TW2824_FORMAT;

	return 0;
}

static struct v4l2_subdev_core_ops tw28xx_subdev_core_ops = {
	.g_ctrl		= tw28xx_g_ctrl,
	.s_ctrl		= tw28xx_s_ctrl,
	.g_chip_ident	= tw28xx_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= tw28xx_g_register,
	.s_register	= tw28xx_s_register,
#endif
};

static struct v4l2_subdev_video_ops tw28xx_subdev_video_ops = {
	.s_mbus_fmt	= tw28xx_s_fmt,
	.g_mbus_fmt	= tw28xx_g_fmt,
	.try_mbus_fmt	= tw28xx_try_fmt,
	.s_crop		= tw28xx_s_crop,
	.g_crop		= tw28xx_g_crop,
	.cropcap	= tw28xx_cropcap,
	.enum_mbus_fmt	= tw28xx_enum_fmt,
};

static struct v4l2_subdev_ops tw28xx_subdev_ops = {
	.core		= &tw28xx_subdev_core_ops,
	.video		= &tw28xx_subdev_video_ops,
};

static int tw28xx_probe(struct i2c_client *client, const struct i2c_device_id *did)
{
	struct tw28xx_t *tw28xx;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl;
	int ret;

	if (!icd)
	{
		dev_err(&client->dev, "TW28xx: missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl)
	{
		dev_err(&client->dev, "TW28xx driver needs platform data\n");
		return -EINVAL;
	}

	tw28xx = kzalloc(sizeof(struct tw28xx_t), GFP_KERNEL);
	if (!tw28xx)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&tw28xx->subdev, client, &tw28xx_subdev_ops);

	icd->ops = &tw28xx_ops;

	tw28xx->rect.left	= TW28XX_COLUMN_SKIP;
	tw28xx->rect.top	= TW28XX_ROW_SKIP;
	tw28xx->rect.width	= TW28XX_MAX_WIDTH;
	tw28xx->rect.height	= TW28XX_MAX_HEIGHT;

	ret = tw28xx_video_probe(icd, client);
	if (ret)
	{
		icd->ops = NULL;
		kfree(tw28xx);
	}

	return ret;
}

static int tw28xx_remove(struct i2c_client *client)
{
	struct tw28xx_t *tw28xx = to_tw28xx(client);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	icd->ops = NULL;

	dev_dbg(&icd->dev, "Video removed: %p, %p\n", icd->dev.parent, icd->vdev);
	if (icl->free_bus)
		icl->free_bus(icl);

	client->driver = NULL;
	kfree(tw28xx);

	return 0;
}

static const struct i2c_device_id tw28xx_id[] = {
	{ "tw28xx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tw28xx_id);

static struct i2c_driver tw28xx_i2c_driver = {
	.driver = {
		.name = "tw28xx",
	},
	.probe		= tw28xx_probe,
	.remove		= tw28xx_remove,
	.id_table	= tw28xx_id,
};

static int __init tw28xx_mod_init(void)
{
	return i2c_add_driver(&tw28xx_i2c_driver);
}

static void __exit tw28xx_mod_exit(void)
{
	i2c_del_driver(&tw28xx_i2c_driver);
}

module_init(tw28xx_mod_init);
module_exit(tw28xx_mod_exit);

MODULE_DESCRIPTION("Techwell TW28xx driver");
MODULE_AUTHOR("Alexander Shiyan <shc_work@mail.ru>");
MODULE_LICENSE("GPL");
