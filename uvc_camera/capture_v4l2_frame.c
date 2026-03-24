#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#define BUFFER_COUNT 4

struct buffer {
    void *start;
    size_t length;
};

static int xioctl(int fd, unsigned long request, void *arg)
{
    int ret;
    do {
        ret = ioctl(fd, request, arg);
    } while (ret == -1 && errno == EINTR);
    return ret;
}

static const char *fourcc_to_str(uint32_t fmt, char out[5])
{
    out[0] = fmt & 0xff;
    out[1] = (fmt >> 8) & 0xff;
    out[2] = (fmt >> 16) & 0xff;
    out[3] = (fmt >> 24) & 0xff;
    out[4] = '\0';
    return out;
}

int main(int argc, char **argv)
{
    const char *dev = argc > 1 ? argv[1] : "/dev/video0";
    const char *out_path = argc > 2 ? argv[2] : "frame.jpg";
    int fd = -1;
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    struct buffer buffers[BUFFER_COUNT] = {0};
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    struct v4l2_buffer buf;
    fd_set fds;
    struct timeval tv;
    int ret;
    FILE *out = NULL;
    char fourcc[5];
    bool streaming = false;

    fd = open(dev, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    memset(&cap, 0, sizeof(cap));
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
        perror("VIDIOC_QUERYCAP");
        close(fd);
        return 1;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) &&
        !(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "%s is not a video capture device\n", dev);
        close(fd);
        return 1;
    }

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_G_FMT, &fmt) < 0) {
        perror("VIDIOC_G_FMT");
        close(fd);
        return 1;
    }

    printf("Device : %s\n", dev);
    printf("Driver : %s\n", cap.driver);
    printf("Card   : %s\n", cap.card);
    printf("Format : %s %ux%u sizeimage=%u\n",
           fourcc_to_str(fmt.fmt.pix.pixelformat, fourcc),
           fmt.fmt.pix.width,
           fmt.fmt.pix.height,
           fmt.fmt.pix.sizeimage);

    memset(&req, 0, sizeof(req));
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        close(fd);
        return 1;
    }

    if (req.count < 2) {
        fprintf(stderr, "insufficient buffer memory\n");
        close(fd);
        return 1;
    }

    for (unsigned int i = 0; i < req.count; ++i) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF");
            goto cleanup;
        }

        buffers[i].length = buf.length;
        buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, buf.m.offset);
        if (buffers[i].start == MAP_FAILED) {
            perror("mmap");
            buffers[i].start = NULL;
            goto cleanup;
        }

        if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            goto cleanup;
        }
    }

    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        goto cleanup;
    }
    streaming = true;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    tv.tv_sec = 5;
    tv.tv_usec = 0;

    ret = select(fd + 1, &fds, NULL, NULL, &tv);
    if (ret < 0) {
        perror("select");
        goto cleanup;
    }
    if (ret == 0) {
        fprintf(stderr, "timeout waiting for frame\n");
        goto cleanup;
    }

    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
        perror("VIDIOC_DQBUF");
        goto cleanup;
    }

    out = fopen(out_path, "wb");
    if (!out) {
        perror("fopen");
        goto cleanup;
    }

    if (fwrite(buffers[buf.index].start, 1, buf.bytesused, out) != buf.bytesused) {
        perror("fwrite");
        goto cleanup;
    }

    fclose(out);
    out = NULL;

    printf("Saved %u bytes to %s\n", buf.bytesused, out_path);
    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG)
        printf("The output file should be viewable as a JPEG image.\n");
    else
        printf("Note: output is raw frame data, not a directly viewable JPEG.\n");

cleanup:
    if (out)
        fclose(out);

    if (streaming)
        xioctl(fd, VIDIOC_STREAMOFF, &type);

    for (unsigned int i = 0; i < req.count && i < BUFFER_COUNT; ++i) {
        if (buffers[i].start)
            munmap(buffers[i].start, buffers[i].length);
    }

    if (fd >= 0)
        close(fd);

    return 0;
}
