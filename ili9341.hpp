#pragma once

#include <stdint.h>
#include <mculib/fastwiring.hpp>
#include <mculib/small_function.hpp>

using namespace mculib;

#define RGB565(r,g,b)     byteReverse16( (((r)<<8)&0b1111100000000000) | (((g)<<3)&0b0000011111100000) | (((b)>>3)&0b0000000000011111) )

#define DEFAULT_FG_COLOR            RGB565(255,255,255)
#define DEFAULT_BG_COLOR            RGB565(  0,  0,  0)
#define DEFAULT_GRID_COLOR          RGB565(128,128,128)
#define DEFAULT_MENU_COLOR          RGB565(230,230,230)
#define DEFAULT_MENU_TEXT_COLOR     RGB565(  0,  0,  0)
#define DEFAULT_MENU_ACTIVE_COLOR   RGB565(210,210,210)
#define DEFAULT_TRACE_1_COLOR       RGB565(255,255,  0)
#define DEFAULT_TRACE_2_COLOR       RGB565(  0,255,255)
#define DEFAULT_TRACE_3_COLOR       RGB565(  0,255,  0)
#define DEFAULT_TRACE_4_COLOR       RGB565(255,  0,255)
#define DEFAULT_CAL_ACTIVE_COLOR    RGB565(  0,255,160)
#define DEFAULT_CAL_INACTIVE_COLOR  RGB565(255,  0,  0)
#define DEFAULT_NORMAL_BAT_COLOR    RGB565( 31,227,  0)
#define DEFAULT_LOW_BAT_COLOR       RGB565(255,  0,  0)
#define DEFAULT_SPEC_INPUT_COLOR    RGB565(128,255,128);
#define DEFAULT_RISE_EDGE_COLOR     RGB565(255,255,255);
#define DEFAULT_FALLEN_EDGE_COLOR   RGB565(160,160,160);

typedef struct {
	uint16_t width;
	uint16_t height;
	uint16_t scaley;
	uint16_t slide;
	const uint8_t *bitmap;
} font_t;

extern uint16_t foreground_color;
extern uint16_t background_color;

// internal buffer space; may be repurposed
#define SPI_BUFFER_SIZE	1024
extern uint16_t ili9341_spi_buffers[SPI_BUFFER_SIZE * 2];

// the buffer that ili9341_bulk() transfers from
extern uint16_t* ili9341_spi_buffer;
extern Pad ili9341_conf_dc;

// ===== hooks =====

// select or deselect the ili9341 spi slave, called to start/stop a transaction
extern small_function<void(bool selected)> ili9341_spi_set_cs;

// write sdi onto spi bus while returning read value; does not affect cs pin
extern small_function<uint32_t(uint32_t sdi, int bits)> ili9341_spi_transfer;

// write spi_buffer to spi bus up to bytes without waiting for completion
extern small_function<void(uint32_t words)> ili9341_spi_transfer_bulk;

// wait for bulk transfers to complete
extern small_function<void()> ili9341_spi_wait_bulk;


static inline constexpr uint16_t byteReverse16(uint16_t x) {
    return (x << 8) | (x >> 8);
}

void ili9341_init(void);
void ili9341_test(int mode);
void ili9341_bulk(int x, int y, int w, int h);
void ili9341_set_flip(bool flipX, bool flipY);
void ili9341_clear_screen(void);
void ili9341_set_foreground(uint16_t fg);
void ili9341_set_background(uint16_t bg);
void blit8BitWidthBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *bitmap);
void blit16BitWidthBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *bitmap);
void ili9341_line(int, int, int, int);
void ili9341_fill(int x, int y, int w, int h, uint16_t color);
void ili9341_drawstring(const char *str, int x, int y);
void ili9341_drawstring(const char *str, int len, int x, int y);
//int ili9341_drawchar_size(uint8_t ch, int x, int y, uint8_t size);
void ili9341_drawstring_size(const char *str, int x, int y, uint8_t size);
void ili9341_drawfont(uint8_t ch, int x, int y);
void ili9341_read_memory(int x, int y, int w, int h, int len, uint16_t* out);
void ili9341_read_memory_continue(int len, uint16_t* out);
