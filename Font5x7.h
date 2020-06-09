#pragma once

#include <stdint.h>
extern const uint8_t x5x7_bits [];


#define S_DELTA "\004"
#define S_PI    "\034"
#define S_MICRO "\035"
#define S_OHM   "\036"
#define S_DEGREE "\037"
#define S_SARROW "\030"
#define S_INFINITY "\031"
#define S_LARROW "\032"
#define S_RARROW "\033"

// Font definitions
#define FONT_GET_DATA(ch)   (&x5x7_bits[ch*7])
#define FONT_GET_WIDTH(ch)  (8-(x5x7_bits[ch*7]&7))
#define FONT_MAX_WIDTH      7
#define FONT_WIDTH          5
#define FONT_GET_HEIGHT     7
#define FONT_STR_HEIGHT     8
