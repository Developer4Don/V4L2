// SPDX-License-Identifier: GPL-2.0
/*
 * usb_v4l2_webcam.c - Out-of-tree V4L2 USB webcam driver for Web Camera 5843:e515.
 *
 * This version implements a minimal UVC bulk-streaming capture path for the
 * camera's currently used mode: MJPEG 640x360 @ 30 fps on interface 1 endpoint
 * 0x83. It is intentionally narrow and pragmatic: enough for /dev/video2 to
 * serve the same basic single-frame user-space capture workflow that currently
 * works on /dev/video0.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/unaligned.h>
#include <linux/usb.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#define WCAM_NAME              "usb_v4l2_webcam"
#define WCAM_CARD_NAME         "Web Camera: Web Camera"
#define WCAM_USB_VENDOR_ID     0x5843
#define WCAM_USB_PRODUCT_ID    0xe515
#define WCAM_STREAM_IFNUM      1
#define WCAM_DEF_WIDTH         640
#define WCAM_DEF_HEIGHT        360
#define WCAM_DEF_PIXFMT        V4L2_PIX_FMT_MJPEG
#define WCAM_DEF_SIZEIMAGE     460800
#define WCAM_FPS               30
#define WCAM_EXPECTED_BULK_EP  0x83
#define WCAM_UVC_VERSION       0x0100
#define WCAM_UVC_CTRL_SIZE     26
#define WCAM_FRAME_INTERVAL    333333U
#define WCAM_TRACE_PAYLOAD     102656U
#define WCAM_TRACE_URB_SIZE    16384U
#define WCAM_URB_COUNT         8
#define WCAM_MAX_HEADER_SIZE   64

struct wcam_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct wcam_device {
	struct usb_device *udev;
	struct usb_interface *intf;

	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct vb2_queue vbq;

	struct mutex lock;
	spinlock_t qlock;
	struct list_head queued_bufs;

	bool disconnected;
	bool streaming;

	u16 width;
	u16 height;
	u32 pixfmt;
	u32 sequence;
	u32 frame_interval_ms;
	u32 bulk_in_ep;
	u32 isoc_in_ep;
	u32 max_payload;
	char bus_info[64];

	struct wcam_buffer *cur_buf;
	size_t cur_filled;
	bool cur_error;
	u8 last_fid;

	struct uvc_streaming_control probe;
	u32 bulk_max_payload_size;
	u8 bulk_header[WCAM_MAX_HEADER_SIZE];
	u8 bulk_header_size;
	u32 bulk_payload_size;
	bool bulk_skip_payload;
	u8 mjpeg_format_index;
	u8 mjpeg_frame_index;
	u32 mjpeg_frame_interval;
	u32 mjpeg_max_frame_size;

	struct urb *urbs[WCAM_URB_COUNT];
	u8 *urb_bufs[WCAM_URB_COUNT];
	unsigned int num_urbs;
	unsigned int urb_size;
};

static inline struct wcam_device *video_to_wcam(struct video_device *vdev)
{
	return container_of(vdev, struct wcam_device, vdev);
}

static inline struct wcam_buffer *vb2_to_wcam_buffer(struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb), struct wcam_buffer, vb);
}

static bool wcam_pixfmt_supported(u32 pixfmt)
{
	return pixfmt == WCAM_DEF_PIXFMT;
}

static u32 wcam_bytesperline(u32 pixfmt, u32 width)
{
	if (pixfmt == V4L2_PIX_FMT_MJPEG)
		return 0;

	return width * 2;
}

static u32 wcam_sizeimage(u32 pixfmt, u32 width, u32 height)
{
	if (pixfmt == V4L2_PIX_FMT_MJPEG)
		return WCAM_DEF_SIZEIMAGE;

	return width * height * 2;
}

static void wcam_bulk_reset_state(struct wcam_device *cam)
{
	cam->cur_buf = NULL;
	cam->cur_filled = 0;
	cam->cur_error = false;
	cam->last_fid = 0xff;
	cam->bulk_header_size = 0;
	cam->bulk_payload_size = 0;
	cam->bulk_skip_payload = false;
}

static struct wcam_buffer *wcam_pop_buffer(struct wcam_device *cam)
{
	struct wcam_buffer *buf = NULL;
	unsigned long flags;

	spin_lock_irqsave(&cam->qlock, flags);
	if (!list_empty(&cam->queued_bufs)) {
		buf = list_first_entry(&cam->queued_bufs, struct wcam_buffer, list);
		list_del_init(&buf->list);
	}
	spin_unlock_irqrestore(&cam->qlock, flags);

	return buf;
}

static void wcam_complete_current(struct wcam_device *cam,
				  enum vb2_buffer_state state)
{
	struct wcam_buffer *buf = cam->cur_buf;
	u32 payload = cam->cur_filled;

	if (!buf)
		return;

	cam->cur_buf = NULL;
	cam->cur_filled = 0;
	cam->cur_error = false;

	buf->vb.sequence = cam->sequence++;
	buf->vb.field = V4L2_FIELD_NONE;
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, payload);
	vb2_buffer_done(&buf->vb.vb2_buf, state);
}

static void wcam_return_all_buffers(struct wcam_device *cam,
				    enum vb2_buffer_state state)
{
	struct wcam_buffer *buf;
	struct wcam_buffer *tmp;
	unsigned long flags;

	if (cam->cur_buf)
		wcam_complete_current(cam, state);

	spin_lock_irqsave(&cam->qlock, flags);
	list_for_each_entry_safe(buf, tmp, &cam->queued_bufs, list) {
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&cam->qlock, flags);
}

static int wcam_query_stream_ctrl(struct wcam_device *cam, u8 query, u8 cs,
				  void *data, u16 size)
{
	u8 requesttype = USB_TYPE_CLASS | USB_RECIP_INTERFACE;
	unsigned int pipe;
	int ret;
	int tries = 0;

	if (query & 0x80) {
		pipe = usb_rcvctrlpipe(cam->udev, 0);
		requesttype |= USB_DIR_IN;
	} else {
		pipe = usb_sndctrlpipe(cam->udev, 0);
		requesttype |= USB_DIR_OUT;
	}

	do {
		ret = usb_control_msg(cam->udev, pipe, query, requesttype,
				      cs << 8, WCAM_STREAM_IFNUM,
				      data, size, 5000);
		if (ret == size) {
			dev_info(&cam->intf->dev,
				 "uvc ctrl ok: query=0x%02x cs=0x%02x size=%u\n",
				 query, cs, size);
			return 0;
		}
		if (ret != -EAGAIN)
			break;
		dev_warn(&cam->intf->dev,
			 "uvc ctrl retry %d: query=0x%02x cs=0x%02x ret=%d\n",
			 tries + 1, query, cs, ret);
		msleep(50);
	} while (++tries < 5);

	dev_err(&cam->intf->dev,
		"uvc ctrl failed: query=0x%02x cs=0x%02x size=%u ret=%d\n",
		query, cs, size, ret);
	if (ret >= 0)
		return -EIO;
	return ret;
}

static int wcam_get_video_ctrl(struct wcam_device *cam,
			       struct uvc_streaming_control *ctrl,
			       bool probe, u8 query)
{
	u8 *data;
	int ret;

	data = kzalloc(WCAM_UVC_CTRL_SIZE, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = wcam_query_stream_ctrl(cam, query,
				     probe ? UVC_VS_PROBE_CONTROL : UVC_VS_COMMIT_CONTROL,
				     data, WCAM_UVC_CTRL_SIZE);
	if (ret) {
		kfree(data);
		return ret;
	}

	memset(ctrl, 0, sizeof(*ctrl));
	ctrl->bmHint = get_unaligned_le16(&data[0]);
	ctrl->bFormatIndex = data[2];
	ctrl->bFrameIndex = data[3];
	ctrl->dwFrameInterval = get_unaligned_le32(&data[4]);
	ctrl->wKeyFrameRate = get_unaligned_le16(&data[8]);
	ctrl->wPFrameRate = get_unaligned_le16(&data[10]);
	ctrl->wCompQuality = get_unaligned_le16(&data[12]);
	ctrl->wCompWindowSize = get_unaligned_le16(&data[14]);
	ctrl->wDelay = get_unaligned_le16(&data[16]);
	ctrl->dwMaxVideoFrameSize = get_unaligned_le32(&data[18]);
	ctrl->dwMaxPayloadTransferSize = get_unaligned_le32(&data[22]);

	kfree(data);

	if (!ctrl->dwMaxVideoFrameSize)
		ctrl->dwMaxVideoFrameSize = WCAM_DEF_SIZEIMAGE;
	if (!ctrl->dwMaxPayloadTransferSize)
		ctrl->dwMaxPayloadTransferSize = cam->max_payload ? : 512;

	dev_info(&cam->intf->dev,
		 "get %s ctrl: fmt=%u frame=%u interval=%u frame_size=%u payload=%u\n",
		 probe ? "probe" : "commit",
		 ctrl->bFormatIndex, ctrl->bFrameIndex, ctrl->dwFrameInterval,
		 ctrl->dwMaxVideoFrameSize, ctrl->dwMaxPayloadTransferSize);
	return 0;
}

static int wcam_set_video_ctrl(struct wcam_device *cam,
			       const struct uvc_streaming_control *ctrl,
			       bool probe)
{
	u8 *data;
	int ret;

	data = kzalloc(WCAM_UVC_CTRL_SIZE, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	put_unaligned_le16(ctrl->bmHint, &data[0]);
	data[2] = ctrl->bFormatIndex;
	data[3] = ctrl->bFrameIndex;
	put_unaligned_le32(ctrl->dwFrameInterval, &data[4]);
	put_unaligned_le16(ctrl->wKeyFrameRate, &data[8]);
	put_unaligned_le16(ctrl->wPFrameRate, &data[10]);
	put_unaligned_le16(ctrl->wCompQuality, &data[12]);
	put_unaligned_le16(ctrl->wCompWindowSize, &data[14]);
	put_unaligned_le16(ctrl->wDelay, &data[16]);
	put_unaligned_le32(ctrl->dwMaxVideoFrameSize, &data[18]);
	put_unaligned_le32(ctrl->dwMaxPayloadTransferSize, &data[22]);

	dev_info(&cam->intf->dev,
		 "set %s ctrl: fmt=%u frame=%u interval=%u frame_size=%u payload=%u\n",
		 probe ? "probe" : "commit",
		 ctrl->bFormatIndex, ctrl->bFrameIndex, ctrl->dwFrameInterval,
		 ctrl->dwMaxVideoFrameSize, ctrl->dwMaxPayloadTransferSize);
	ret = wcam_query_stream_ctrl(cam, UVC_SET_CUR,
				     probe ? UVC_VS_PROBE_CONTROL : UVC_VS_COMMIT_CONTROL,
				     data, WCAM_UVC_CTRL_SIZE);
	kfree(data);
	return ret;
}

static int wcam_negotiate_stream(struct wcam_device *cam)
{
	struct uvc_streaming_control ctrl;
	int ret;

	dev_info(&cam->intf->dev,
		 "negotiate stream: max_payload=%u endpoint=0x%02x\n",
		 cam->max_payload, cam->bulk_in_ep);
	ret = wcam_get_video_ctrl(cam, &ctrl, true, UVC_GET_CUR);
	if (ret) {
		dev_warn(&cam->intf->dev,
			 "GET_CUR(PROBE) failed (%d), using traced fallback COMMIT values\n",
			 ret);
		memset(&ctrl, 0, sizeof(ctrl));
		ctrl.bmHint = 0;
		ctrl.bFormatIndex = cam->mjpeg_format_index;
		ctrl.bFrameIndex = cam->mjpeg_frame_index;
		ctrl.dwFrameInterval = cam->mjpeg_frame_interval;
		ctrl.dwMaxVideoFrameSize = cam->mjpeg_max_frame_size;
		ctrl.dwMaxPayloadTransferSize = WCAM_TRACE_PAYLOAD;

		ret = wcam_set_video_ctrl(cam, &ctrl, false);
		if (ret)
			return ret;

		cam->probe = ctrl;
		cam->bulk_max_payload_size = ctrl.dwMaxPayloadTransferSize;
		dev_info(&cam->intf->dev,
			 "stream negotiated by fallback commit: frame_size=%u payload=%u\n",
			 ctrl.dwMaxVideoFrameSize, ctrl.dwMaxPayloadTransferSize);
		return 0;
	}

	ctrl.bmHint = 0;
	ctrl.bFormatIndex = cam->mjpeg_format_index;
	ctrl.bFrameIndex = cam->mjpeg_frame_index;
	ctrl.dwFrameInterval = cam->mjpeg_frame_interval;
	if (!ctrl.dwMaxVideoFrameSize)
		ctrl.dwMaxVideoFrameSize = cam->mjpeg_max_frame_size;
	if (!ctrl.dwMaxPayloadTransferSize)
		ctrl.dwMaxPayloadTransferSize = WCAM_TRACE_PAYLOAD;

	ret = wcam_set_video_ctrl(cam, &ctrl, true);
	if (ret)
		return ret;

	ret = wcam_get_video_ctrl(cam, &ctrl, true, UVC_GET_CUR);
	if (ret)
		return ret;

	if (!ctrl.dwMaxVideoFrameSize)
		ctrl.dwMaxVideoFrameSize = cam->mjpeg_max_frame_size;
	if (!ctrl.dwMaxPayloadTransferSize)
		ctrl.dwMaxPayloadTransferSize = WCAM_TRACE_PAYLOAD;

	ret = wcam_set_video_ctrl(cam, &ctrl, false);
	if (ret)
		return ret;

	cam->probe = ctrl;
	cam->bulk_max_payload_size = ctrl.dwMaxPayloadTransferSize;
	dev_info(&cam->intf->dev,
		 "stream negotiated: frame_size=%u payload=%u\n",
		 ctrl.dwMaxVideoFrameSize, ctrl.dwMaxPayloadTransferSize);
	return 0;
}

static int wcam_parse_streaming_desc_block(struct wcam_device *cam,
					   const u8 *p, int len,
					   const char *where)
{
	u8 current_mjpeg_format = 0;

	while (len >= 3) {
		u8 dlen = p[0];
		u8 dtype = p[1];
		u8 dsub = p[2];

		if (dlen < 3 || dlen > len)
			break;

		if (dtype == USB_DT_CS_INTERFACE) {
			if (dsub == UVC_VS_FORMAT_MJPEG && dlen >= 5) {
				current_mjpeg_format = p[3];
				dev_info(&cam->intf->dev,
					 "found MJPEG format index=%u in %s default_frame=%u\n",
					 p[3], where, dlen >= 8 ? p[6] : 0);
			} else if (dsub == UVC_VS_FRAME_MJPEG && dlen >= 26 &&
				   current_mjpeg_format) {
				u8 frame_index = p[3];
				u16 width = get_unaligned_le16(&p[5]);
				u16 height = get_unaligned_le16(&p[7]);
				u32 max_frame_size = get_unaligned_le32(&p[17]);
				u32 default_interval = get_unaligned_le32(&p[21]);
				u8 interval_type = p[25];
				u32 chosen_interval = default_interval;

				if (width == WCAM_DEF_WIDTH && height == WCAM_DEF_HEIGHT) {
					int i;

					if (interval_type) {
						const u8 *ivals = &p[26];

						for (i = 0; i < interval_type; i++) {
							u32 val;

							if (26 + 4 * (i + 1) > dlen)
								break;
							val = get_unaligned_le32(ivals + i * 4);
							if (val == WCAM_FRAME_INTERVAL) {
								chosen_interval = val;
								break;
							}
						}
					} else if (dlen >= 38) {
						u32 min_i = get_unaligned_le32(&p[26]);
						u32 max_i = get_unaligned_le32(&p[30]);
						u32 step_i = get_unaligned_le32(&p[34]);

						chosen_interval = clamp_t(u32, WCAM_FRAME_INTERVAL,
									  min_i, max_i);
						if (step_i)
							chosen_interval =
								min_i + DIV_ROUND_CLOSEST(chosen_interval - min_i,
											  step_i) * step_i;
					}

					cam->mjpeg_format_index = current_mjpeg_format;
					cam->mjpeg_frame_index = frame_index;
					cam->mjpeg_frame_interval = chosen_interval;
					cam->mjpeg_max_frame_size = max_frame_size ? : WCAM_DEF_SIZEIMAGE;

					dev_info(&cam->intf->dev,
						 "selected MJPEG frame from %s fmt=%u frame=%u %ux%u interval=%u max_frame=%u\n",
						 where,
						 cam->mjpeg_format_index, cam->mjpeg_frame_index,
						 width, height, cam->mjpeg_frame_interval,
						 cam->mjpeg_max_frame_size);
					return 0;
				}
			}
		}

		p += dlen;
		len -= dlen;
	}

	return -ENOENT;
}

static int wcam_parse_streaming_descriptors(struct wcam_device *cam)
{
	struct usb_host_interface *alt = cam->intf->cur_altsetting;
	int ret;
	int i;

	cam->mjpeg_format_index = 0;
	cam->mjpeg_frame_index = 0;
	cam->mjpeg_frame_interval = 0;
	cam->mjpeg_max_frame_size = WCAM_DEF_SIZEIMAGE;

	if (alt->extra && alt->extralen > 0) {
		ret = wcam_parse_streaming_desc_block(cam, alt->extra, alt->extralen,
						      "altsetting extra");
		if (!ret)
			return 0;
	}

	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		struct usb_host_endpoint *ep = &alt->endpoint[i];
		char where[32];

		if (!ep->extra || ep->extralen <= 0)
			continue;

		snprintf(where, sizeof(where), "endpoint %u extra", i);
		ret = wcam_parse_streaming_desc_block(cam, ep->extra, ep->extralen,
						      where);
		if (!ret)
			return 0;
	}

	dev_err(&cam->intf->dev,
		"failed to find MJPEG %ux%u streaming descriptor in altsetting or endpoint extras\n",
		WCAM_DEF_WIDTH, WCAM_DEF_HEIGHT);
	return -EINVAL;
}

static void wcam_copy_video_data(struct wcam_device *cam, const u8 *data, int len)
{
	void *vaddr;
	u32 remaining;
	u32 copy;

	if (!cam->cur_buf || len <= 0)
		return;

	vaddr = vb2_plane_vaddr(&cam->cur_buf->vb.vb2_buf, 0);
	if (!vaddr) {
		cam->cur_error = true;
		cam->bulk_skip_payload = true;
		wcam_complete_current(cam, VB2_BUF_STATE_ERROR);
		return;
	}

	remaining = wcam_sizeimage(cam->pixfmt, cam->width, cam->height) - cam->cur_filled;
	copy = min_t(u32, len, remaining);
	memcpy(vaddr + cam->cur_filled, data, copy);
	cam->cur_filled += copy;

	if (copy < len) {
		cam->cur_error = true;
		cam->bulk_skip_payload = true;
		wcam_complete_current(cam, VB2_BUF_STATE_ERROR);
	}
}

static void wcam_decode_bulk(struct wcam_device *cam, struct urb *urb)
{
	u8 *mem;
	int len;

	if (urb->actual_length == 0 && cam->bulk_header_size == 0)
		return;

	mem = urb->transfer_buffer;
	len = urb->actual_length;
	cam->bulk_payload_size += len;

	if (cam->bulk_header_size == 0 && !cam->bulk_skip_payload) {
		u8 header_len;
		u8 flags;
		u8 fid;

		if (len < 2 || mem[0] < 2 || mem[0] > len || mem[0] > WCAM_MAX_HEADER_SIZE) {
			cam->bulk_skip_payload = true;
		} else {
			header_len = mem[0];
			flags = mem[1];
			fid = flags & UVC_STREAM_FID;

			if (cam->last_fid != 0xff && fid != cam->last_fid && cam->cur_buf && cam->cur_filled)
				wcam_complete_current(cam, cam->cur_error ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);

			cam->last_fid = fid;

			if (!cam->cur_buf) {
				cam->cur_buf = wcam_pop_buffer(cam);
				cam->cur_filled = 0;
				cam->cur_error = false;
			}

			if (!cam->cur_buf) {
				cam->bulk_skip_payload = true;
			} else {
				if (flags & UVC_STREAM_ERR)
					cam->cur_error = true;

				memcpy(cam->bulk_header, mem, header_len);
				cam->bulk_header_size = header_len;
				mem += header_len;
				len -= header_len;
			}
		}
	}

	if (!cam->bulk_skip_payload && cam->cur_buf && len > 0)
		wcam_copy_video_data(cam, mem, len);

	if (urb->actual_length < urb->transfer_buffer_length ||
	    cam->bulk_payload_size >= cam->bulk_max_payload_size) {
		if (!cam->bulk_skip_payload && cam->cur_buf && cam->bulk_header_size >= 2 &&
		    (cam->bulk_header[1] & UVC_STREAM_EOF) && cam->cur_filled)
			wcam_complete_current(cam, cam->cur_error ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);

		cam->bulk_header_size = 0;
		cam->bulk_payload_size = 0;
		cam->bulk_skip_payload = false;
	}
}

static void wcam_free_urbs(struct wcam_device *cam)
{
	unsigned int i;

	for (i = 0; i < cam->num_urbs; i++) {
		usb_free_urb(cam->urbs[i]);
		cam->urbs[i] = NULL;
		kfree(cam->urb_bufs[i]);
		cam->urb_bufs[i] = NULL;
	}

	cam->num_urbs = 0;
	cam->urb_size = 0;
}

static void wcam_kill_urbs(struct wcam_device *cam)
{
	unsigned int i;

	for (i = 0; i < cam->num_urbs; i++)
		if (cam->urbs[i])
			usb_kill_urb(cam->urbs[i]);
}

static void wcam_urb_complete(struct urb *urb)
{
	struct wcam_device *cam = urb->context;
	int ret;

	if (!cam)
		return;

	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		return;
		break;
	default:
		cam->cur_error = true;
		break;
	}

	if (!READ_ONCE(cam->streaming) || READ_ONCE(cam->disconnected))
		return;

	wcam_decode_bulk(cam, urb);

	if (!READ_ONCE(cam->streaming) || READ_ONCE(cam->disconnected))
		return;

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret < 0)
		dev_err(&cam->intf->dev, "Failed to resubmit video URB: %d\n", ret);
}

static int wcam_alloc_urbs(struct wcam_device *cam)
{
	unsigned int i;
	unsigned int psize;
	unsigned int pipe;

	psize = cam->max_payload ? : 512;
	cam->urb_size = WCAM_TRACE_URB_SIZE;
	cam->num_urbs = WCAM_URB_COUNT;
	pipe = usb_rcvbulkpipe(cam->udev, cam->bulk_in_ep);
	dev_info(&cam->intf->dev,
		 "alloc urbs: count=%u psize=%u bulk_payload=%u urb_size=%u\n",
		 cam->num_urbs, psize, cam->bulk_max_payload_size, cam->urb_size);

	for (i = 0; i < cam->num_urbs; i++) {
		cam->urb_bufs[i] = kmalloc(cam->urb_size, GFP_KERNEL);
		if (!cam->urb_bufs[i]) {
			dev_err(&cam->intf->dev, "urb buffer alloc failed at %u\n", i);
			goto err;
		}

		cam->urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!cam->urbs[i]) {
			dev_err(&cam->intf->dev, "urb alloc failed at %u\n", i);
			goto err;
		}

		usb_fill_bulk_urb(cam->urbs[i], cam->udev, pipe,
				  cam->urb_bufs[i], cam->urb_size,
				  wcam_urb_complete, cam);
	}

	return 0;

err:
	wcam_free_urbs(cam);
	return -ENOMEM;
}

static int wcam_submit_urbs(struct wcam_device *cam)
{
	unsigned int i;
	int ret;

	for (i = 0; i < cam->num_urbs; i++) {
		ret = usb_submit_urb(cam->urbs[i], GFP_KERNEL);
		if (ret < 0) {
			dev_err(&cam->intf->dev,
				"usb_submit_urb failed at %u ret=%d\n", i, ret);
			while (i-- > 0)
				usb_kill_urb(cam->urbs[i]);
			return ret;
		}
	}

	return 0;
}

static int wcam_queue_setup(struct vb2_queue *vq,
			    unsigned int *nbuffers,
			    unsigned int *nplanes,
			    unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct wcam_device *cam = vb2_get_drv_priv(vq);
	u32 size = wcam_sizeimage(cam->pixfmt, cam->width, cam->height);

	if (*nplanes) {
		if (sizes[0] < size)
			return -EINVAL;
	} else {
		*nplanes = 1;
		sizes[0] = size;
	}

	if (*nbuffers < 3)
		*nbuffers = 3;

	alloc_devs[0] = cam->v4l2_dev.dev;
	return 0;
}

static int wcam_buf_prepare(struct vb2_buffer *vb)
{
	struct wcam_device *cam = vb2_get_drv_priv(vb->vb2_queue);
	u32 size = wcam_sizeimage(cam->pixfmt, cam->width, cam->height);

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, 0);
	return 0;
}

static void wcam_buf_queue(struct vb2_buffer *vb)
{
	struct wcam_device *cam = vb2_get_drv_priv(vb->vb2_queue);
	struct wcam_buffer *buf = vb2_to_wcam_buffer(vb);
	unsigned long flags;

	INIT_LIST_HEAD(&buf->list);
	spin_lock_irqsave(&cam->qlock, flags);
	list_add_tail(&buf->list, &cam->queued_bufs);
	spin_unlock_irqrestore(&cam->qlock, flags);
}

static int wcam_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct wcam_device *cam = vb2_get_drv_priv(vq);
	int ret;

	dev_info(&cam->intf->dev, "STREAMON requested queued=%u\n", count);
	if (cam->disconnected) {
		ret = -ENODEV;
		goto err_out;
	}

	ret = wcam_negotiate_stream(cam);
	if (ret)
		goto err_out;

	wcam_bulk_reset_state(cam);
	ret = wcam_alloc_urbs(cam);
	if (ret)
		goto err_out;

	ret = usb_clear_halt(cam->udev, usb_rcvbulkpipe(cam->udev, cam->bulk_in_ep));
	if (ret)
		dev_warn(&cam->intf->dev, "usb_clear_halt before STREAMON returned %d\n", ret);

	cam->streaming = true;
	ret = wcam_submit_urbs(cam);
	if (ret)
		goto err_stop;

	dev_info(&cam->intf->dev, "STREAMON success\n");
	return 0;

err_stop:
	cam->streaming = false;
	wcam_kill_urbs(cam);
	wcam_free_urbs(cam);
err_out:
	wcam_return_all_buffers(cam, VB2_BUF_STATE_QUEUED);
	dev_err(&cam->intf->dev, "STREAMON failed ret=%d\n", ret);
	return ret;
}

static void wcam_stop_streaming(struct vb2_queue *vq)
{
	struct wcam_device *cam = vb2_get_drv_priv(vq);
	unsigned int pipe;

	cam->streaming = false;

	wcam_kill_urbs(cam);
	if (cam->bulk_in_ep) {
		pipe = usb_rcvbulkpipe(cam->udev, cam->bulk_in_ep);
		usb_clear_halt(cam->udev, pipe);
	}
	wcam_free_urbs(cam);
	wcam_bulk_reset_state(cam);
	wcam_return_all_buffers(cam, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops wcam_vb2_ops = {
	.queue_setup = wcam_queue_setup,
	.buf_prepare = wcam_buf_prepare,
	.buf_queue = wcam_buf_queue,
	.start_streaming = wcam_start_streaming,
	.stop_streaming = wcam_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int wcam_querycap(struct file *file, void *fh,
			 struct v4l2_capability *cap)
{
	struct wcam_device *cam = video_drvdata(file);

	strscpy(cap->driver, WCAM_NAME, sizeof(cap->driver));
	strscpy(cap->card, WCAM_CARD_NAME, sizeof(cap->card));
	strscpy(cap->bus_info, cam->bus_info, sizeof(cap->bus_info));
	return 0;
}

static int wcam_enum_fmt_vid_cap(struct file *file, void *fh,
				 struct v4l2_fmtdesc *f)
{
	if (f->index > 0)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_MJPEG;
	strscpy(f->description, "Motion-JPEG", sizeof(f->description));
	return 0;
}

static int wcam_g_fmt_vid_cap(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct wcam_device *cam = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width = cam->width;
	pix->height = cam->height;
	pix->pixelformat = cam->pixfmt;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = wcam_bytesperline(cam->pixfmt, cam->width);
	pix->sizeimage = wcam_sizeimage(cam->pixfmt, cam->width, cam->height);
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_SRGB;
	return 0;
}

static void wcam_try_fmt_common(struct v4l2_pix_format *pix)
{
	if (!wcam_pixfmt_supported(pix->pixelformat))
		pix->pixelformat = WCAM_DEF_PIXFMT;

	pix->field = V4L2_FIELD_NONE;
	pix->width = WCAM_DEF_WIDTH;
	pix->height = WCAM_DEF_HEIGHT;
	pix->bytesperline = wcam_bytesperline(pix->pixelformat, pix->width);
	pix->sizeimage = wcam_sizeimage(pix->pixelformat, pix->width, pix->height);
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_SRGB;
}

static int wcam_try_fmt_vid_cap(struct file *file, void *fh,
				struct v4l2_format *f)
{
	wcam_try_fmt_common(&f->fmt.pix);
	return 0;
}

static int wcam_s_fmt_vid_cap(struct file *file, void *fh,
			    struct v4l2_format *f)
{
	struct wcam_device *cam = video_drvdata(file);
	int ret;

	ret = vb2_is_busy(&cam->vbq);
	if (ret)
		return ret;

	wcam_try_fmt_common(&f->fmt.pix);

	mutex_lock(&cam->lock);
	cam->width = f->fmt.pix.width;
	cam->height = f->fmt.pix.height;
	cam->pixfmt = f->fmt.pix.pixelformat;
	mutex_unlock(&cam->lock);
	return 0;
}

static int wcam_enum_framesizes(struct file *file, void *fh,
				struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > 0 || fsize->pixel_format != V4L2_PIX_FMT_MJPEG)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = WCAM_DEF_WIDTH;
	fsize->discrete.height = WCAM_DEF_HEIGHT;
	return 0;
}

static int wcam_enum_frameintervals(struct file *file, void *fh,
				    struct v4l2_frmivalenum *fival)
{
	if (fival->index > 0 ||
	    fival->pixel_format != V4L2_PIX_FMT_MJPEG ||
	    fival->width != WCAM_DEF_WIDTH ||
	    fival->height != WCAM_DEF_HEIGHT)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = WCAM_FPS;
	return 0;
}

static const struct v4l2_ioctl_ops wcam_ioctl_ops = {
	.vidioc_querycap = wcam_querycap,
	.vidioc_enum_fmt_vid_cap = wcam_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = wcam_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = wcam_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = wcam_try_fmt_vid_cap,
	.vidioc_enum_framesizes = wcam_enum_framesizes,
	.vidioc_enum_frameintervals = wcam_enum_frameintervals,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_expbuf = vb2_ioctl_expbuf,
};

static int wcam_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct wcam_device *cam = video_to_wcam(vdev);
	int ret;

	mutex_lock(&cam->lock);
	if (cam->disconnected)
		ret = -ENODEV;
	else
		ret = v4l2_fh_open(file);
	mutex_unlock(&cam->lock);
	return ret;
}

static const struct v4l2_file_operations wcam_fops = {
	.owner = THIS_MODULE,
	.open = wcam_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

static int wcam_detect_endpoints(struct wcam_device *cam)
{
	struct usb_host_interface *alt = cam->intf->cur_altsetting;
	int i;

	cam->bulk_in_ep = 0;
	cam->isoc_in_ep = 0;
	cam->max_payload = 0;

	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		struct usb_endpoint_descriptor *ep = &alt->endpoint[i].desc;

		if (usb_endpoint_dir_in(ep) && usb_endpoint_is_bulk_in(ep) && !cam->bulk_in_ep) {
			cam->bulk_in_ep = ep->bEndpointAddress;
			cam->max_payload = usb_endpoint_maxp(ep);
		}

		if (usb_endpoint_dir_in(ep) && usb_endpoint_xfer_isoc(ep) && !cam->isoc_in_ep)
			cam->isoc_in_ep = ep->bEndpointAddress;
	}

	dev_info(&cam->intf->dev,
		 "interface=%u bulk-in=0x%02x isoc-in=0x%02x max_payload=%u\n",
		 alt->desc.bInterfaceNumber,
		 cam->bulk_in_ep, cam->isoc_in_ep, cam->max_payload);

	if (alt->desc.bInterfaceNumber != WCAM_STREAM_IFNUM)
		return -ENODEV;
	if (cam->bulk_in_ep != WCAM_EXPECTED_BULK_EP)
		return -ENODEV;

	return 0;
}

static int wcam_register_video(struct wcam_device *cam)
{
	int ret;

	cam->width = WCAM_DEF_WIDTH;
	cam->height = WCAM_DEF_HEIGHT;
	cam->pixfmt = WCAM_DEF_PIXFMT;
	cam->frame_interval_ms = DIV_ROUND_CLOSEST(1000, WCAM_FPS);
	cam->sequence = 0;
	wcam_bulk_reset_state(cam);

	INIT_LIST_HEAD(&cam->queued_bufs);
	mutex_init(&cam->lock);
	spin_lock_init(&cam->qlock);

	usb_make_path(cam->udev, cam->bus_info, sizeof(cam->bus_info));
	strscpy(cam->v4l2_dev.name, WCAM_NAME, sizeof(cam->v4l2_dev.name));

	ret = v4l2_device_register(&cam->intf->dev, &cam->v4l2_dev);
	if (ret)
		return ret;

	cam->vbq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cam->vbq.io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	cam->vbq.drv_priv = cam;
	cam->vbq.buf_struct_size = sizeof(struct wcam_buffer);
	cam->vbq.ops = &wcam_vb2_ops;
	cam->vbq.mem_ops = &vb2_vmalloc_memops;
	cam->vbq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	cam->vbq.lock = &cam->lock;
	cam->vbq.min_queued_buffers = 1;
	cam->vbq.dev = cam->v4l2_dev.dev;

	ret = vb2_queue_init(&cam->vbq);
	if (ret)
		goto err_v4l2_unregister;

	cam->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE |
				V4L2_CAP_STREAMING |
				V4L2_CAP_READWRITE;
	cam->vdev.v4l2_dev = &cam->v4l2_dev;
	cam->vdev.queue = &cam->vbq;
	cam->vdev.lock = &cam->lock;
	cam->vdev.fops = &wcam_fops;
	cam->vdev.ioctl_ops = &wcam_ioctl_ops;
	cam->vdev.release = video_device_release_empty;
	cam->vdev.vfl_dir = VFL_DIR_RX;
	strscpy(cam->vdev.name, WCAM_CARD_NAME, sizeof(cam->vdev.name));
	video_set_drvdata(&cam->vdev, cam);

	ret = video_register_device(&cam->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_v4l2_unregister;

	dev_info(&cam->intf->dev,
		 "registered /dev/video%d for %s (%ux%u MJPG @ %u fps)\n",
		 cam->vdev.num, WCAM_CARD_NAME, cam->width, cam->height, WCAM_FPS);
	return 0;

err_v4l2_unregister:
	v4l2_device_unregister(&cam->v4l2_dev);
	return ret;
}

static void wcam_unregister_video(struct wcam_device *cam)
{
	vb2_video_unregister_device(&cam->vdev);
	v4l2_device_unregister(&cam->v4l2_dev);
}

static int wcam_probe(struct usb_interface *intf,
		      const struct usb_device_id *id)
{
	struct wcam_device *cam;
	int ret;

	cam = devm_kzalloc(&intf->dev, sizeof(*cam), GFP_KERNEL);
	if (!cam)
		return -ENOMEM;

	cam->udev = usb_get_dev(interface_to_usbdev(intf));
	cam->intf = intf;
	usb_set_intfdata(intf, cam);
	usb_disable_autosuspend(cam->udev);

	ret = wcam_detect_endpoints(cam);
	if (ret)
		goto err_put_dev;

	ret = wcam_parse_streaming_descriptors(cam);
	if (ret)
		goto err_put_dev;

	ret = wcam_register_video(cam);
	if (ret)
		goto err_put_dev;

	dev_info(&intf->dev,
		 "bound to %04x:%04x interface %u alt %u (uvc %04x)\n",
		 le16_to_cpu(cam->udev->descriptor.idVendor),
		 le16_to_cpu(cam->udev->descriptor.idProduct),
		 intf->cur_altsetting->desc.bInterfaceNumber,
		 intf->cur_altsetting->desc.bAlternateSetting,
		 WCAM_UVC_VERSION);
	return 0;

err_put_dev:
	usb_set_intfdata(intf, NULL);
	usb_put_dev(cam->udev);
	return ret;
}

static void wcam_disconnect(struct usb_interface *intf)
{
	struct wcam_device *cam = usb_get_intfdata(intf);

	if (!cam)
		return;

	usb_set_intfdata(intf, NULL);

	mutex_lock(&cam->lock);
	cam->disconnected = true;
	cam->streaming = false;
	mutex_unlock(&cam->lock);

	wcam_kill_urbs(cam);
	wcam_free_urbs(cam);
	wcam_return_all_buffers(cam, VB2_BUF_STATE_ERROR);
	wcam_unregister_video(cam);
	usb_put_dev(cam->udev);
	dev_info(&intf->dev, "disconnected\n");
}

static const struct usb_device_id wcam_id_table[] = {
	{ USB_DEVICE_AND_INTERFACE_INFO(WCAM_USB_VENDOR_ID,
					      WCAM_USB_PRODUCT_ID,
					      USB_CLASS_VIDEO, 2, 0) },
	{ }
};
MODULE_DEVICE_TABLE(usb, wcam_id_table);

static struct usb_driver wcam_usb_driver = {
	.name = WCAM_NAME,
	.probe = wcam_probe,
	.disconnect = wcam_disconnect,
	.id_table = wcam_id_table,
};

module_usb_driver(wcam_usb_driver);

MODULE_DESCRIPTION("Custom V4L2 MJPEG bulk webcam driver for Web Camera 5843:e515");
MODULE_AUTHOR("OpenAI");
MODULE_LICENSE("GPL");
