# uvc_camera - Custom V4L2 USB Webcam Driver

A custom out-of-tree Linux kernel driver that replaces the generic UVC driver for a specific USB webcam (`5843:e515`), exposing it as a standard V4L2 video capture device. Includes a userspace test application for frame capture verification.

## Overview

This project converts a UVC-compliant USB camera into a device managed by a custom V4L2 kernel driver. Instead of relying on the in-kernel `uvcvideo` module, this driver implements its own UVC bulk-streaming path, probe/commit negotiation, and V4L2 interface — giving full control over how the hardware is driven.

## Files

| File | Description |
|------|-------------|
| `usb_v4l2_webcam.c` | Kernel module: custom V4L2 driver for the webcam |
| `capture_v4l2_frame.c` | Userspace test app: captures a single frame via V4L2 |
| `Makefile` | Builds the kernel module against the running kernel |

## Driver Details (`usb_v4l2_webcam.c`)

- **Target device**: USB `5843:e515` (Web Camera), interface 1, bulk endpoint `0x83`
- **Capture mode**: MJPEG 640x360 @ 30 fps
- **Streaming method**: Bulk transfer with UVC payload header parsing
- **V4L2 capabilities**: `VIDEO_CAPTURE | STREAMING | READWRITE`
- **Buffer management**: videobuf2 with vmalloc backend, mmap/dmabuf/read I/O modes
- **UVC negotiation**: Full probe/commit control exchange with fallback path

The driver registers as `/dev/videoN` upon binding and supports standard V4L2 ioctls: `QUERYCAP`, `ENUM_FMT`, `G/S/TRY_FMT`, `ENUM_FRAMESIZES`, `ENUM_FRAMEINTERVALS`, and all vb2-backed buffer operations.

## Build

Requires kernel headers for the running kernel.

```bash
cd uvc_camera
make
```

This produces `usb_v4l2_webcam.ko`.

## Usage

### Load the driver

Before loading, unbind the camera from the default `uvcvideo` driver if it is already claimed:

```bash
# Unload generic UVC driver (if loaded for this device)
sudo modprobe -r uvcvideo

# Load the custom driver
sudo insmod usb_v4l2_webcam.ko
```

Check `dmesg` for registration messages — the driver logs the assigned `/dev/videoN` node.

### Capture a test frame

Build and run the test application:

```bash
gcc -o capture_v4l2_frame capture_v4l2_frame.c
./capture_v4l2_frame /dev/videoN frame.jpg
```

- **Arg 1**: V4L2 device node (default: `/dev/video0`)
- **Arg 2**: Output file path (default: `frame.jpg`)

The application opens the device, queries its format, memory-maps 4 buffers, starts streaming, captures one frame via `select()` + `DQBUF`, and writes it to disk. If the device is outputting MJPEG, the saved file is a valid JPEG image viewable directly.

### Unload

```bash
sudo rmmod usb_v4l2_webcam
```

## Troubleshooting

- **Device not claimed**: Verify the camera is plugged in and shows `5843:e515` in `lsusb`. The driver only binds to interface 1 (Video Streaming, subclass 2) with bulk endpoint `0x83`.
- **Timeout waiting for frame**: The test app waits 5 seconds. Check `dmesg` for URB submission errors or negotiation failures.
- **STREAMON fails**: The driver logs probe/commit exchange details to `dmesg` — look for `uvc ctrl failed` or `negotiate stream` messages.

## License

The kernel module is licensed under GPL-2.0.
