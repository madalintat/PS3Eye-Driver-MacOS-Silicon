#include <stdio.h>
#include <stdlib.h>
#include <libusb.h>
#include <string.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// OpenCV global
cv::Mat frame_mat;

// PS3 Eye constants
#define PS3EYE_VID 0x1415
#define PS3EYE_PID 0x2000
#define VIDEO_ENDPOINT 0x81
#define TRANSFER_SIZE 65536
#define NUM_TRANSFERS 5

#define OV_GAIN      0x00
#define OV_EXPOSURE  0x04

// OV534 register addresses
#define OV534_REG_ADDRESS   0xf1
#define OV534_REG_SUBADDR   0xf2
#define OV534_REG_WRITE     0xf3
#define OV534_REG_READ      0xf4
#define OV534_REG_OPERATION 0xf5
#define OV534_REG_STATUS    0xf6

#define OV534_OP_WRITE_3    0x37
#define OV534_OP_WRITE_2    0x33
#define OV534_OP_READ_2     0xf9

// OV534 initialization data
static const uint8_t ov534_reg_initdata[][2] = {
    { 0xe7, 0x3a },
    { OV534_REG_ADDRESS, 0x42 }, // select OV772x sensor
    { 0x92, 0x01 },
    { 0x93, 0x18 },
    { 0x94, 0x10 },
    { 0x95, 0x10 },
    { 0xE2, 0x00 },
    { 0xE7, 0x3E },
    { 0x96, 0x00 },
    { 0x97, 0x20 },
    { 0x97, 0x20 },
    { 0x97, 0x20 },
    { 0x97, 0x0A },
    { 0x97, 0x3F },
    { 0x97, 0x4A },
    { 0x97, 0x20 },
    { 0x97, 0x15 },
    { 0x97, 0x0B },
    { 0x8E, 0x40 },
    { 0x1F, 0x81 },
    { 0xC0, 0x50 },
    { 0xC1, 0x3C },
    { 0xC2, 0x01 },
    { 0xC3, 0x01 },
    { 0x50, 0x89 },
    { 0x88, 0x08 },
    { 0x8D, 0x00 },
    { 0x8E, 0x00 },
    { 0x1C, 0x00 }, // video data start (V_FMT)
    { 0x1D, 0x00 }, // RAW8 mode
    { 0x1D, 0x02 }, // payload size 0x0200 * 4 = 2048 bytes
    { 0x1D, 0x00 }, // payload size
    { 0x1D, 0x01 }, // frame size = 0x012C00 * 4 = 307200 bytes (640 * 480 @ 8bpp)
    { 0x1D, 0x2C }, // frame size
    { 0x1D, 0x00 }, // frame size
    { 0x1C, 0x0A }, // video data start (V_CNTL0)
    { 0x1D, 0x08 }, // turn on UVC header
    { 0x1D, 0x0E },
    { 0x34, 0x05 },
    { 0xE3, 0x04 },
    { 0x89, 0x00 },
    { 0x76, 0x00 },
    { 0xE7, 0x2E },
    { 0x31, 0xF9 },
    { 0x25, 0x42 },
    { 0x21, 0xF0 },
    { 0xE5, 0x04 }
};

// OV772x initialization data
static const uint8_t ov772x_reg_initdata[][2] = {
    { 0x12, 0x80 }, // reset
    { 0x3D, 0x00 },
    { 0x12, 0x01 }, // Processed Bayer RAW (8bit)
    { 0x11, 0x01 },
    { 0x14, 0x40 },
    { 0x15, 0x00 },
    { 0x63, 0xAA }, // AWB
    { 0x64, 0x87 },
    { 0x66, 0x00 },
    { 0x67, 0x02 },
    { 0x17, 0x26 },
    { 0x18, 0xA0 },
    { 0x19, 0x07 },
    { 0x1A, 0xF0 },
    { 0x29, 0xA0 },
    { 0x2A, 0x00 },
    { 0x2C, 0xF0 },
    { 0x20, 0x10 },
    { 0x4E, 0x0F },
    { 0x3E, 0xF3 },
    { 0x0D, 0x41 },
    { 0x32, 0x00 },
    { 0x13, 0xF0 }, // COM8
    { 0x22, 0x7F },
    { 0x23, 0x03 },
    { 0x24, 0x40 },
    { 0x25, 0x30 },
    { 0x26, 0xA1 },
    { 0x2A, 0x00 },
    { 0x2B, 0x00 },
    { 0x13, 0xF7 },
    { 0x0C, 0xC0 },
    { 0x11, 0x00 },
    { 0x0D, 0x41 },
    { 0x8E, 0x00 } // De-noise threshold
};

// Bridge start QVGA
static const uint8_t bridge_start_qvga[][2] = {
    {0x1c, 0x00},
    {0x1d, 0x00},
    {0x1d, 0x01}, // payload size 0x0100 * 4 = 1024 bytes
    {0x1d, 0x00},
    {0x1d, 0x00}, // frame size = 0x004B00 * 4 = 76800 bytes (320 * 240 @ 8bpp)
    {0x1d, 0x4B}, // frame size
    {0x1d, 0x00}, // frame size
    {0xc0, 0x50},
    {0xc1, 0x3c},
};

// Sensor start QVGA
static const uint8_t sensor_start_qvga[][2] = {
    {0x12, 0x01},
    {0x17, 0x3f},
    {0x18, 0x50},
    {0x19, 0x03},
    {0x1a, 0x78},
    {0x29, 0xa0},
    {0x2c, 0xf0},
    {0x65, 0x20},
};

struct libusb_transfer *global_transfers[NUM_TRANSFERS];
unsigned char *transfer_buffers[NUM_TRANSFERS];
uint8_t *frame_buffer;
uint32_t frame_buffer_size = 320 * 240;
uint32_t cur_frame_data_len = 0;
uint8_t *cur_frame_start;

// UVC header flags
#define UVC_STREAM_EOH (1 << 7)
#define UVC_STREAM_ERR (1 << 6)
#define UVC_STREAM_STI (1 << 5)
#define UVC_STREAM_RES (1 << 4)
#define UVC_STREAM_SCR (1 << 3)
#define UVC_STREAM_PTS (1 << 2)
#define UVC_STREAM_EOF (1 << 1)
#define UVC_STREAM_FID (1 << 0)

enum gspca_packet_type {
    DISCARD_PACKET,
    FIRST_PACKET,
    INTER_PACKET,
    LAST_PACKET
};

enum gspca_packet_type last_packet_type = DISCARD_PACKET;
uint32_t last_pts = 0;
uint16_t last_fid = 0;

void print_device_info(libusb_device_handle *dev_handle) {
    unsigned char str_desc[256];
    libusb_get_string_descriptor_ascii(dev_handle, 1, str_desc, sizeof(str_desc));
    printf("Manufacturer: %s\n", str_desc);
    libusb_get_string_descriptor_ascii(dev_handle, 2, str_desc, sizeof(str_desc));
    printf("Product: %s\n", str_desc);
    printf("Connected to PS3 Eye camera successfully!\n");
}

void print_endpoints(libusb_device_handle *dev_handle) {
    libusb_device *dev = libusb_get_device(dev_handle);
    struct libusb_config_descriptor *config;
    int ret = libusb_get_config_descriptor(dev, 0, &config);
    if (ret < 0) {
        printf("Failed to get config descriptor: %s\n", libusb_error_name(ret));
        return;
    }
    for (int i = 0; i < config->bNumInterfaces; i++) {
        const struct libusb_interface *interface = &config->interface[i];
        printf("Interface %d has %d alternate settings\n", i, interface->num_altsetting);
        for (int j = 0; j < interface->num_altsetting; j++) {
            const struct libusb_interface_descriptor *alt = &interface->altsetting[j];
            printf("  Altsetting %d: %d endpoints\n", alt->bAlternateSetting, alt->bNumEndpoints);
            for (int k = 0; k < alt->bNumEndpoints; k++) {
                const struct libusb_endpoint_descriptor *ep = &alt->endpoint[k];
                printf("    Endpoint 0x%02x: Type %d, Dir %s, MaxPacketSize %d\n",
                       ep->bEndpointAddress,
                       ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK,
                       (ep->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) ? "IN" : "OUT",
                       ep->wMaxPacketSize);
            }
        }
    }
    libusb_free_config_descriptor(config);
}

int send_ov_reg(libusb_device_handle *dev_handle, uint8_t reg, uint8_t value) {
    int ret = libusb_control_transfer(
        dev_handle,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
        0x01, value, reg, NULL, 0, 1000
    );
    printf("OV reg 0x%02x = 0x%02x: %d bytes transferred (%s)\n", reg, value, ret, ret < 0 ? libusb_error_name(ret) : "OK");
    return ret;
}

uint8_t ov534_reg_read(libusb_device_handle *dev_handle, uint16_t reg) {
    unsigned char data[1];
    int ret = libusb_control_transfer(
        dev_handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
        0x01, 0x00, reg, data, 1, 1000
    );
    return (ret >= 0) ? data[0] : 0;
}

int sccb_check_status(libusb_device_handle *dev_handle) {
    uint8_t data;
    int i;
    for (i = 0; i < 5; i++) {
        data = ov534_reg_read(dev_handle, OV534_REG_STATUS);
        switch (data) {
        case 0x00: return 1;
        case 0x04: return 0;
        case 0x03: break;
        default: printf("SCCB status 0x%02x, attempt %d/5\n", data, i + 1);
        }
    }
    return 0;
}

void sccb_reg_write(libusb_device_handle *dev_handle, uint8_t reg, uint8_t val) {
    send_ov_reg(dev_handle, OV534_REG_SUBADDR, reg);
    send_ov_reg(dev_handle, OV534_REG_WRITE, val);
    send_ov_reg(dev_handle, OV534_REG_OPERATION, OV534_OP_WRITE_3);
    if (!sccb_check_status(dev_handle)) {
        printf("sccb_reg_write failed\n");
    }
}

uint8_t sccb_reg_read(libusb_device_handle *dev_handle, uint16_t reg) {
    send_ov_reg(dev_handle, OV534_REG_SUBADDR, (uint8_t)reg);
    send_ov_reg(dev_handle, OV534_REG_OPERATION, OV534_OP_WRITE_2);
    if (!sccb_check_status(dev_handle)) {
        printf("sccb_reg_read failed 1\n");
    }
    send_ov_reg(dev_handle, OV534_REG_OPERATION, OV534_OP_READ_2);
    if (!sccb_check_status(dev_handle)) {
        printf("sccb_reg_read failed 2\n");
    }
    return ov534_reg_read(dev_handle, OV534_REG_READ);
}

void reg_w_array(libusb_device_handle *dev_handle, const uint8_t (*data)[2], int len) {
    while (--len >= 0) {
        send_ov_reg(dev_handle, (*data)[0], (*data)[1]);
        data++;
    }
}

void sccb_w_array(libusb_device_handle *dev_handle, const uint8_t (*data)[2], int len) {
    while (--len >= 0) {
        if ((*data)[0] != 0xff) {
            sccb_reg_write(dev_handle, (*data)[0], (*data)[1]);
        } else {
            sccb_reg_read(dev_handle, (*data)[1]);
            sccb_reg_write(dev_handle, 0xff, 0x00);
        }
        data++;
    }
}

int initialize_camera(libusb_device_handle *dev_handle) {
    uint16_t sensor_id;

    // Reset bridge
    send_ov_reg(dev_handle, 0xe7, 0x3a);
    send_ov_reg(dev_handle, 0xe0, 0x08);
    usleep(100000); // 100ms

    // Initialize sensor address
    send_ov_reg(dev_handle, OV534_REG_ADDRESS, 0x42);

    // Reset sensor
    sccb_reg_write(dev_handle, 0x12, 0x80);
    usleep(10000); // 10ms

    // Probe sensor
    printf("Probing sensor...\n");
    sensor_id = sccb_reg_read(dev_handle, 0x0a) << 8;
    sensor_id |= sccb_reg_read(dev_handle, 0x0b);
    printf("Sensor ID: %04x\n", sensor_id);

    // Initialize
    reg_w_array(dev_handle, ov534_reg_initdata, sizeof(ov534_reg_initdata) / sizeof(ov534_reg_initdata[0]));
    sccb_w_array(dev_handle, ov772x_reg_initdata, sizeof(ov772x_reg_initdata) / sizeof(ov772x_reg_initdata[0]));
    send_ov_reg(dev_handle, 0xe0, 0x09);

    printf("Camera initialized for video (640x480)\n");
    return 0;
}

void frame_add(enum gspca_packet_type packet_type, const uint8_t *data, int len) {
    printf("frame_add: type=%d, len=%d, cur_frame_data_len=%d\n", packet_type, len, cur_frame_data_len);

    if (packet_type == FIRST_PACKET) {
        cur_frame_data_len = 0;
        printf("Starting new frame\n");
    } else {
        switch(last_packet_type) {
            case DISCARD_PACKET:
                if (packet_type == LAST_PACKET) {
                    last_packet_type = packet_type;
                    cur_frame_data_len = 0;
                }
                printf("Discarding packet after DISCARD state\n");
                return;
            case LAST_PACKET:
                printf("Ignoring packet after LAST_PACKET\n");
                return;
            default:
                break;
        }
    }

    if (len > 0) {
        if (cur_frame_data_len + len > frame_buffer_size) {
            printf("Frame buffer overflow: %d + %d > %d, discarding frame\n",
                   cur_frame_data_len, len, frame_buffer_size);
            packet_type = DISCARD_PACKET;
            cur_frame_data_len = 0;
        } else {
            memcpy(cur_frame_start + cur_frame_data_len, data, len);
            cur_frame_data_len += len;
            printf("Added %d bytes, total now %d/%d\n", len, cur_frame_data_len, frame_buffer_size);
        }
    }

    last_packet_type = packet_type;

    if (packet_type == LAST_PACKET) {
        // Frame complete, debayer and display
        if (cur_frame_data_len == frame_buffer_size) {
            printf("Frame complete! Displaying...\n");
            cv::Mat bayer(240, 320, CV_8UC1, cur_frame_start);
            cv::cvtColor(bayer, frame_mat, cv::COLOR_BayerGR2BGR);
            cv::imshow("PS3 Eye", frame_mat);
            cv::waitKey(1);
        } else {
            printf("Frame incomplete: got %d bytes, expected %d\n", cur_frame_data_len, frame_buffer_size);
        }
        cur_frame_data_len = 0;
        cur_frame_start = frame_buffer; // For simplicity, just overwrite
    }
}

void pkt_scan(uint8_t *data, int len) {
    uint32_t this_pts;
    uint16_t this_fid;
    int pos = 0;

    while (pos < len) {
        // Find next UVC header (starts with 0x0C)
        int header_pos = pos;
        while (header_pos < len && data[header_pos] != 12) {
            header_pos++;
        }
        if (header_pos >= len) {
            // No more headers
            break;
        }

        // Check if we have a complete header
        if (header_pos + 12 > len) {
            // Incomplete header at end of buffer
            break;
        }

        // Find the end of this packet (start of next header or end of buffer)
        int packet_end = header_pos + 12;
        while (packet_end < len && data[packet_end] != 12) {
            packet_end++;
        }

        int payload_len = packet_end - (header_pos + 12);

        // Verify UVC header
        uint8_t *header = data + header_pos;
        if (header[0] != 12) {
            printf("bad header length: %d at pos %d\n", header[0], header_pos);
            frame_add(DISCARD_PACKET, NULL, 0);
            pos = header_pos + 1;
            continue;
        }

        // Check errors
        if (header[1] & UVC_STREAM_ERR) {
            printf("payload error at pos %d\n", header_pos);
            frame_add(DISCARD_PACKET, NULL, 0);
            pos = packet_end;
            continue;
        }

        // Extract PTS and FID
        if (!(header[1] & UVC_STREAM_PTS)) {
            printf("PTS not present at pos %d\n", header_pos);
            frame_add(DISCARD_PACKET, NULL, 0);
            pos = packet_end;
            continue;
        }

        this_pts = (header[5] << 24) | (header[4] << 16) | (header[3] << 8) | header[2];
        this_fid = (header[1] & UVC_STREAM_FID) ? 1 : 0;

        printf("Packet at %d: PTS=%u, FID=%d, EOF=%d, payload_len=%d\n",
               header_pos, this_pts, this_fid, (header[1] & UVC_STREAM_EOF) ? 1 : 0, payload_len);

        // Process packet
        if (this_pts != last_pts || this_fid != last_fid) {
            if (last_packet_type == INTER_PACKET) {
                frame_add(DISCARD_PACKET, NULL, 0);
            }
            last_pts = this_pts;
            last_fid = this_fid;
            frame_add(FIRST_PACKET, header + 12, payload_len);
        } else if (header[1] & UVC_STREAM_EOF) {
            last_pts = 0;
            // For EOF, check if the accumulated data matches expected frame size
            if (cur_frame_data_len + payload_len == frame_buffer_size) {
                frame_add(LAST_PACKET, header + 12, payload_len);
            } else {
                printf("Frame size mismatch: expected %d, got %d + %d = %d\n",
                       frame_buffer_size, cur_frame_data_len, payload_len, cur_frame_data_len + payload_len);
                frame_add(DISCARD_PACKET, NULL, 0);
            }
        } else {
            frame_add(INTER_PACKET, header + 12, payload_len);
        }

        pos = packet_end;
    }
}

void LIBUSB_CALL transfer_callback(struct libusb_transfer *transfer) {
    if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
        printf("Received %d bytes of video data\n", transfer->actual_length);
        if (transfer->actual_length > 0) {
            pkt_scan(transfer->buffer, transfer->actual_length);
        }
    } else if (transfer->status == LIBUSB_TRANSFER_CANCELLED) {
        printf("Transfer cancelled\n");
    } else {
        printf("Transfer failed: %s (status %d)\n", libusb_error_name(transfer->status), transfer->status);
    }
    if (transfer->status != LIBUSB_TRANSFER_CANCELLED) {
        libusb_submit_transfer(transfer);
    }
}

int start_video_capture(libusb_device_handle *dev_handle) {
    frame_buffer = (uint8_t*)malloc(frame_buffer_size);
    if (!frame_buffer) {
        printf("Failed to allocate frame buffer\n");
        return -1;
    }
    cur_frame_start = frame_buffer;

    int ret = libusb_clear_halt(dev_handle, VIDEO_ENDPOINT);
    if (ret < 0) {
        printf("Failed to clear endpoint: %s\n", libusb_error_name(ret));
    } else {
        printf("Endpoint 0x%02x cleared\n", VIDEO_ENDPOINT);
    }

    for (int i = 0; i < NUM_TRANSFERS; i++) {
        transfer_buffers[i] = (unsigned char*)malloc(TRANSFER_SIZE);
        if (!transfer_buffers[i]) {
            printf("Failed to allocate transfer buffer %d\n", i);
            return -1;
        }

        global_transfers[i] = libusb_alloc_transfer(0);
        if (!global_transfers[i]) {
            printf("Failed to allocate transfer %d\n", i);
            return -1;
        }

        libusb_fill_bulk_transfer(
            global_transfers[i], dev_handle, VIDEO_ENDPOINT,
            transfer_buffers[i], TRANSFER_SIZE, transfer_callback, NULL, 0
        );

        printf("About to submit bulk transfer %d\n", i);
        int ret = libusb_submit_transfer(global_transfers[i]);
        printf("libusb_submit_transfer returned %d\n", ret);
        if (ret < 0) {
            printf("Failed to submit transfer %d: %s\n", i, libusb_error_name(ret));
            return ret;
        }
    }

    printf("Video capture started\n");
    return 0;
}

int main() {
    libusb_context *ctx = NULL;
    libusb_device_handle *dev_handle = NULL;
    int ret;

    ret = libusb_init(&ctx);
    if (ret < 0) {
        printf("Failed to initialize libusb: %s\n", libusb_error_name(ret));
        return 1;
    }

    libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_DEBUG);

    dev_handle = libusb_open_device_with_vid_pid(ctx, PS3EYE_VID, PS3EYE_PID);
    if (!dev_handle) {
        printf("Could not find/open PS3 Eye device\n");
        libusb_exit(ctx);
        return 1;
    }

    print_device_info(dev_handle);
    print_endpoints(dev_handle);

    // Detach kernel drivers from all interfaces
    struct libusb_config_descriptor *config;
    libusb_device *dev = libusb_get_device(dev_handle);
    libusb_get_config_descriptor(dev, 0, &config);
    for (int i = 0; i < config->bNumInterfaces; i++) {
        if (libusb_kernel_driver_active(dev_handle, i)) {
            printf("Kernel driver active on interface %d, attempting to detach...\n", i);
            ret = libusb_detach_kernel_driver(dev_handle, i);
            if (ret < 0) {
                printf("Failed to detach kernel driver from interface %d: %s\n", i, libusb_error_name(ret));
                libusb_free_config_descriptor(config);
                libusb_close(dev_handle);
                libusb_exit(ctx);
                return 1;
            }
            printf("Kernel driver detached from interface %d successfully\n", i);
        } else {
            printf("No kernel driver active on interface %d\n", i);
        }
    }
    libusb_free_config_descriptor(config);

    ret = libusb_claim_interface(dev_handle, 0);
    if (ret < 0) {
        printf("Failed to claim interface 0: %s\n", libusb_error_name(ret));
        goto cleanup;
    }
    printf("Interface 0 claimed\n");

    ret = libusb_claim_interface(dev_handle, 2);
    if (ret < 0) {
        printf("Failed to claim interface 2: %s\n", libusb_error_name(ret));
        goto cleanup;
    }
    printf("Interface 2 claimed\n");

    // Set alternate setting to enable video streaming endpoint on interface 2
    ret = libusb_set_interface_alt_setting(dev_handle, 2, 1);
    if (ret < 0) {
        printf("Failed to set alternate setting on interface 2: %s (continuing anyway)\n", libusb_error_name(ret));
        // Continue despite error - some devices work without explicit altsetting on macOS
    } else {
        printf("Alternate setting 1 set for interface 2\n");
    }

    ret = initialize_camera(dev_handle);
    if (ret < 0) {
        printf("Failed to initialize camera: %s\n", libusb_error_name(ret));
        goto cleanup;
    }

    // Set video mode (320x240 @ 30 FPS)
    {
        unsigned char mode[] = {0x06, 0x00, 0x00, 0x00}; // 0x06 for QVGA
        libusb_control_transfer(
            dev_handle,
            LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
            0x03, 0x0000, 0x0200, mode, sizeof(mode), 1000
        );
    }

    // Start QVGA
    reg_w_array(dev_handle, bridge_start_qvga, sizeof(bridge_start_qvga) / sizeof(bridge_start_qvga[0]));
    sccb_w_array(dev_handle, sensor_start_qvga, sizeof(sensor_start_qvga) / sizeof(sensor_start_qvga[0]));

    // Set frame rate (30 FPS for safe mode)
    sccb_reg_write(dev_handle, 0x11, 0x00); // CLKRC
    sccb_reg_write(dev_handle, 0x0d, 0x41); // COM4
    send_ov_reg(dev_handle, 0xe5, 0x04); // CAMERA_CLK

    // Start stream
    send_ov_reg(dev_handle, 0xe0, 0x00);

    // Turn on LED
    send_ov_reg(dev_handle, 0x21, 0x80);
    send_ov_reg(dev_handle, 0x23, 0x80);

    // Set some basic controls
    sccb_reg_write(dev_handle, 0x13, 0xF7); // COM8 - enable AGC, AWB, AEC
    sccb_reg_write(dev_handle, 0x00, 0x14); // GAIN
    sccb_reg_write(dev_handle, 0x04, 0x80); // EXPOSURE
    sccb_reg_write(dev_handle, 0x06, 0x14); // BRIGHTNESS
    sccb_reg_write(dev_handle, 0x07, 0x1E); // CONTRAST
    sccb_reg_write(dev_handle, 0x01, 0x80); // BLUE
    sccb_reg_write(dev_handle, 0x02, 0x80); // RED
    sccb_reg_write(dev_handle, 0x03, 0x80); // GREEN
    sccb_reg_write(dev_handle, 0x0C, 0x00); // COM3
    sccb_reg_write(dev_handle, 0x3E, 0x00); // COM14
    sccb_reg_write(dev_handle, 0x40, 0xD0); // COM15

    usleep(1000000); // 1s delay

    ret = start_video_capture(dev_handle);
    if (ret < 0) {
        printf("Failed to start video capture\n");
        goto cleanup;
    }

    for (int i = 0; i < 10; i++) {
        libusb_handle_events(ctx);
        sleep(1);
    }

 cleanup:
    for (int i = 0; i < NUM_TRANSFERS; i++) {
        if (global_transfers[i]) {
            libusb_cancel_transfer(global_transfers[i]);
        }
    }
    // Wait for cancellations
    for (int i = 0; i < 10; i++) {
        libusb_handle_events(ctx);
        usleep(100000);
    }
    for (int i = 0; i < NUM_TRANSFERS; i++) {
        if (global_transfers[i]) {
            libusb_free_transfer(global_transfers[i]);
        }
        if (transfer_buffers[i]) {
            free(transfer_buffers[i]);
        }
    }
    if (frame_buffer) {
        free(frame_buffer);
    }
    libusb_release_interface(dev_handle, 0);
    libusb_release_interface(dev_handle, 2);
    // Skip kernel driver reattach to avoid segfault on macOS
    // libusb_attach_kernel_driver(dev_handle, 0);
    // libusb_attach_kernel_driver(dev_handle, 2);
    libusb_close(dev_handle);
    libusb_exit(ctx);
    return ret < 0 ? 1 : 0;
}
