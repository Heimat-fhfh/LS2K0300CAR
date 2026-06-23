#ifndef SEEKFREE_UDP_H
#define SEEKFREE_UDP_H

#include <cstdint>

#define SEEKFREE_UDP_SEND_HEAD         0xAA
#define SEEKFREE_UDP_CAMERA_FUNCTION   0x02
#define SEEKFREE_UDP_CAMERA_DOT_FUNC   0x03
#define SEEKFREE_UDP_IMAGE_TYPE        2   // MT9V03X = grayscale

#define BOUNDARY_NUM  (60 * 4 / 2)  // 120 points per boundary

void seekfree_udp_init();
void seekfree_udp_send_frame(uint8_t* image_data, uint16_t width, uint16_t height,
                             uint8_t* left_x, uint8_t* left_y,
                             uint8_t* center_x, uint8_t* center_y,
                             uint8_t* right_x, uint8_t* right_y,
                             uint16_t dot_count);

#endif
