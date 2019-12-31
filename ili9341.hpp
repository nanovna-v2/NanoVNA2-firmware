#pragma once

#include <stdint.h>
#include <mculib/fastwiring.hpp>
#include <mculib/small_function.hpp>

using namespace mculib;

#define RGB565(b,r,g)     ( (((b)<<8)&0xfc00) | (((r)<<2)&0x03e0) | (((g)>>3)&0x001f) )

typedef struct {
	uint16_t width;
	uint16_t height;
	uint16_t scaley;
	uint16_t slide;
	const uint8_t *bitmap;
} font_t;

extern const font_t NF20x22;

// internal buffer space; may be repurposed
constexpr int ili9341_bufferSize = 1024;
extern uint16_t ili9341_spi_buffers[ili9341_bufferSize * 2];

// the buffer that ili9341_bulk() transfers from
extern uint16_t* ili9341_spi_buffer;
extern Pad ili9341_conf_cs;
extern Pad ili9341_conf_dc;

// ===== hooks =====

// write sdi onto spi bus while returning read value; does not affect cs pin
extern small_function<uint32_t(uint32_t sdi, int bits)> ili9341_spi_transfer;

// write spi_buffer to spi bus up to bytes without waiting for completion
extern small_function<void(uint32_t words)> ili9341_spi_transfer_bulk;

// wait for bulk transfers to complete
extern small_function<void()> ili9341_spi_wait_bulk;

void ili9341_init(void);
void ili9341_test(int mode);
void ili9341_bulk(int x, int y, int w, int h);
void ili9341_set_flip(bool flipX, bool flipY);
void ili9341_line(int, int, int, int, int);
void ili9341_fill(int x, int y, int w, int h, int color);
void ili9341_drawchar_5x7(uint8_t ch, int x, int y, uint16_t fg, uint16_t bg);
void ili9341_drawstring_5x7_inv(const char *str, int x, int y, uint16_t fg, uint16_t bg, bool inv);
void ili9341_drawstring_5x7(const char *str, int x, int y, uint16_t fg, uint16_t bg);
void ili9341_drawstring_5x7(const char *str, int len, int x, int y, uint16_t fg, uint16_t bg);
void ili9341_drawchar_size(uint8_t ch, int x, int y, uint16_t fg, uint16_t bg, uint8_t size);
void ili9341_drawstring_size(const char *str, int x, int y, uint16_t fg, uint16_t bg, uint8_t size);
void ili9341_drawfont(uint8_t ch, const font_t *font, int x, int y, uint16_t fg, uint16_t bg);
void ili9341_read_memory(int x, int y, int w, int h, int len, uint16_t* out);
void ili9341_read_memory_continue(int len, uint16_t* out);
