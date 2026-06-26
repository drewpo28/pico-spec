#pragma once
#include <stdint.h>

// 254x156 keyboard help bitmap, packed 4 bits/pixel (index 1-7, 0=skip).
#define KBD_IMG_W 254
#define KBD_IMG_H 156
extern const uint8_t kbd_img[19812];
