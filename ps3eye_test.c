#include <stdio.h>
#include <stdlib.h>
#include "/opt/homebrew/include/libusb-1.0/libusb.h"
#include <string.h>
#include <unistd.h>

// PS3 Eye constants
#define PS3EYE_VID 0x1415
#define PS3EYE_PID 0x2000

// OV7720 register addresses
#define OV_GAIN      0x00
#define OV_BLUE      0x01
#define OV_RED       0x02
#define OV_GREEN     0x03
#define OV_EXPOSURE  0x04
#define OV_FREQ      0x05

void print_device_info(libusb_device_handle *dev_handle) {
    unsigned char str_desc[256];
    
    // Get manufacturer string
    libusb_get_string_descriptor_ascii(dev_handle, 1, str_desc, sizeof(str_desc));
    printf("Manufacturer: %s\n", str_desc);
    
    // Get product string
    libusb_get_string_descriptor_ascii(dev_handle, 2, str_desc, sizeof(str_desc));
    printf("Product: %s\n", str_desc);
    
    printf("Connected to PS3 Eye camera successfully!\n");
}

// Send a control message to the OV7720 sensor
int send_ov_reg(libusb_device_handle *dev_handle, uint8_t reg, uint8_t value) {
    int ret;
    
    ret = libusb_control_transfer(
        dev_handle,
        LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
        0x01,              // Request type (example)
        value,             // Value
        reg,               // Index (register)
        NULL,              // Data buffer
        0,                 // Length
        1000               // Timeout in ms
    );
    
    return ret;
}

// Initialize the camera with default settings
int initialize_camera(libusb_device_handle *dev_handle) {
    int ret;
    
    // Set exposure
    ret = send_ov_reg(dev_handle, OV_EXPOSURE, 120);
    if (ret < 0) return ret;
    
    // Set gain
    ret = send_ov_reg(dev_handle, OV_GAIN, 20);
    if (ret < 0) return ret;
    
    // Set RGB gains
    ret = send_ov_reg(dev_handle, OV_RED, 128);
    if (ret < 0) return ret;
    
    ret = send_ov_reg(dev_handle, OV_GREEN, 128);
    if (ret < 0) return ret;
    
    ret = send_ov_reg(dev_handle, OV_BLUE, 128);
    if (ret < 0) return ret;
    
    printf("Camera initialized with default settings\n");
    return 0;
}

int main() {
    libusb_context *ctx = NULL;
    libusb_device_handle *dev_handle = NULL;
    int ret;
    
    // Initialize libusb
    ret = libusb_init(&ctx);
    if (ret < 0) {
        printf("Failed to initialize libusb: %s\n", libusb_error_name(ret));
        return 1;
    }
    
    // Set debug level (optional)
    libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
    
    // Try to open the PS3 Eye device
    dev_handle = libusb_open_device_with_vid_pid(ctx, PS3EYE_VID, PS3EYE_PID);
    if (dev_handle == NULL) {
        printf("Could not find/open PS3 Eye device. Check connection and permissions.\n");
        libusb_exit(ctx);
        return 1;
    }
    
    // Print basic device information
    print_device_info(dev_handle);
    
    // Detach kernel driver if active
    if (libusb_kernel_driver_active(dev_handle, 0)) {
        printf("Kernel driver active, detaching...\n");
        ret = libusb_detach_kernel_driver(dev_handle, 0);
        if (ret < 0) {
            printf("Failed to detach kernel driver: %s\n", libusb_error_name(ret));
            libusb_close(dev_handle);
            libusb_exit(ctx);
            return 1;
        }
    }
    
    // Claim interface
    ret = libusb_claim_interface(dev_handle, 0);
    if (ret < 0) {
        printf("Failed to claim interface: %s\n", libusb_error_name(ret));
        libusb_close(dev_handle);
        libusb_exit(ctx);
        return 1;
    }
    
    // Initialize camera with default settings
    ret = initialize_camera(dev_handle);
    if (ret < 0) {
        printf("Failed to initialize camera: %s\n", libusb_error_name(ret));
    } else {
        printf("Camera test successful!\n");
    }
    
    // Release interface
    libusb_release_interface(dev_handle, 0);
    
    // Re-attach kernel driver if necessary
    libusb_attach_kernel_driver(dev_handle, 0);
    
    // Clean up
    libusb_close(dev_handle);
    libusb_exit(ctx);
    
    return 0;
}
