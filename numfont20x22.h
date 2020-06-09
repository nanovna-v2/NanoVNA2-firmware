#pragma once

#include <stdint.h>

extern const uint16_t numfont16x22[];
#define NUM_FONT_GET_DATA(ch)   (&numfont16x22[ch*22])
#define NUM_FONT_GET_WIDTH      16
#define NUM_FONT_GET_HEIGHT     22
