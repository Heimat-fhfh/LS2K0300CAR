#include "network/seekfree_udp.h"
#include "drivers/zf_driver_udp.hpp"
#include <cstring>

extern zf_driver_udp udp_dev;

void seekfree_udp_send_frame(uint8_t* image_data, uint16_t width, uint16_t height,
                             uint8_t* left_x, uint8_t* left_y,
                             uint8_t* center_x, uint8_t* center_y,
                             uint8_t* right_x, uint8_t* right_y,
                             uint16_t dot_count)
{
    // camera packet: head + function + camera_type + length + width + height
    uint8_t cam_pkt[8] = {
        SEEKFREE_UDP_SEND_HEAD,
        SEEKFREE_UDP_CAMERA_FUNCTION,
        (uint8_t)((SEEKFREE_UDP_IMAGE_TYPE << 5) | (image_data ? 0 : (1 << 4)) | 3),
        8, // length
        (uint8_t)(width & 0xFF), (uint8_t)(width >> 8),
        (uint8_t)(height & 0xFF), (uint8_t)(height >> 8)
    };

    // boundary dot packet
    uint8_t dot_pkt[12] = {
        SEEKFREE_UDP_SEND_HEAD,
        SEEKFREE_UDP_CAMERA_DOT_FUNC,
        (uint8_t)((2 << 6) | (1 << 5) | 3),  // XY_BOUNDARY + 16bit coords + 3 boundaries
        12, // length
        (uint8_t)(dot_count & 0xFF), (uint8_t)(dot_count >> 8),
        0x07, // valid_flag: 3 boundaries all valid
        0x00  // reserve
    };

    // send camera header
    udp_dev.send_data(cam_pkt, sizeof(cam_pkt));

    // send image data (grayscale, width*height bytes)
    if (image_data) {
        uint32_t image_size = width * height;
        udp_dev.send_data(image_data, image_size);
    }

    // send boundary dot header
    udp_dev.send_data(dot_pkt, sizeof(dot_pkt));

    // send boundary XY coordinates (16-bit each)
    const uint32_t dot_bytes = dot_count * 2;  // uint16 per coordinate
    uint8_t* boundaries[] = {left_x, left_y, center_x, center_y, right_x, right_y};
    for (int i = 0; i < 6; i++) {
        if (boundaries[i]) {
            udp_dev.send_data(boundaries[i], dot_bytes);
        }
    }
}
