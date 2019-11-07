/*
		ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

		Licensed under the Apache License, Version 2.0 (the "License");
		you may not use this file except in compliance with the License.
		You may obtain a copy of the License at

				http://www.apache.org/licenses/LICENSE-2.0

		Unless required by applicable law or agreed to in writing, software
		distributed under the License is distributed on an "AS IS" BASIS,
		WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
		See the License for the specific language governing permissions and
		limitations under the License.
*/

/*
	 Concepts and parts of this file have been contributed by Fabio Utzig,
	 chvprintf() added by Brent Roman.
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdint.h>
#include <string.h>


#define MAX_FILLER 11
#define FLOAT_PRECISION 9


static char *long_to_string_with_divisor(char *p, long num, unsigned radix, long divisor) {
	int i;
	char *q;
	long l, ll;

	l = num;
	if (divisor == 0) {
		ll = num;
	} else {
		ll = divisor;
	}

	q = p + MAX_FILLER;
	do {
		i = (int)(l % radix);
		i += '0';
		if (i > '9')
			i += 'A' - '0' - 10;
		*--q = i;
		l /= radix;
	} while ((ll /= radix) != 0);

	i = (int)(p + MAX_FILLER - q);
	do
		*p++ = *q++;
	while (--i);

	return p;
}

static char *ch_ltoa(char *p, long num, unsigned radix) {

	return long_to_string_with_divisor(p, num, radix, 0);
}


static const long pow10[FLOAT_PRECISION] = {
		10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000
};

static char *ftoa(char *p, double num, unsigned long precision) {
	long l;

	if ((precision == 0) || (precision > FLOAT_PRECISION))
		precision = FLOAT_PRECISION;
	precision = pow10[precision - 1];

	l = (long)num;
	p = long_to_string_with_divisor(p, l, 10, 0);
	*p++ = '.';
	l = (long)((num - l) * precision);
	return long_to_string_with_divisor(p, l, 10, precision / 10);
}


/**
 * @brief   System formatted output function.
 * @details This function implements a minimal @p vprintf()-like functionality
 *          with output on a @p BaseSequentialStream.
 *          The general parameters format is: %[-][width|*][.precision|*][l|L]p.
 *          The following parameter types (p) are supported:
 *          - <b>x</b> hexadecimal integer.
 *          - <b>X</b> hexadecimal long.
 *          - <b>o</b> octal integer.
 *          - <b>O</b> octal long.
 *          - <b>d</b> decimal signed integer.
 *          - <b>D</b> decimal signed long.
 *          - <b>u</b> decimal unsigned integer.
 *          - <b>U</b> decimal unsigned long.
 *          - <b>c</b> character.
 *          - <b>s</b> string.
 *          .
 *
 * @param[in] chp       pointer to a @p BaseSequentialStream implementing object
 * @param[in] fmt       formatting string
 * @param[in] ap        list of parameters
 * @return              The number of bytes that would have been
 *                      written to @p chp if no stream error occurs
 *
 * @api
 */

// void putChar(uint8_t c)
template<class putChar_t>
int chvprintf(const putChar_t& putChar, const char* fmt, va_list ap) {
	char* p;
	char* s;
	char c, filler;
	int i, precision, width;
	int n = 0;
	bool is_long, left_align;
	long l;
	float f;
	char tmpbuf[2*MAX_FILLER + 1];

	while (true) {
		c = *fmt++;
		if (c == 0)
			return n;
		if (c != '%') {
			putChar((uint8_t)c);
			n++;
			continue;
		}
		p = tmpbuf;
		s = tmpbuf;
		left_align = false;
		if (*fmt == '-') {
			fmt++;
			left_align = true;
		}
		filler = ' ';
		if (*fmt == '0') {
			fmt++;
			filler = '0';
		}
		width = 0;
		while (true) {
			c = *fmt++;
			if (c >= '0' && c <= '9')
				c -= '0';
			else if (c == '*')
				c = va_arg(ap, int);
			else
				break;
			width = width * 10 + c;
		}
		precision = 0;
		if (c == '.') {
			while (true) {
				c = *fmt++;
				if (c >= '0' && c <= '9')
					c -= '0';
				else if (c == '*')
					c = va_arg(ap, int);
				else
					break;
				precision *= 10;
				precision += c;
			}
		}
		/* Long modifier.*/
		if (c == 'l' || c == 'L') {
			is_long = true;
			if (*fmt)
				c = *fmt++;
		}
		else
			is_long = (c >= 'A') && (c <= 'Z');

		/* Command decoding.*/
		switch (c) {
		case 'c':
			filler = ' ';
			*p++ = va_arg(ap, int);
			break;
		case 's':
			filler = ' ';
			if ((s = va_arg(ap, char *)) == 0)
				s = "(null)";
			if (precision == 0)
				precision = 32767;
			for (p = s; *p && (--precision >= 0); p++)
				;
			break;
		case 'D':
		case 'd':
		case 'I':
		case 'i':
			if (is_long)
				l = va_arg(ap, long);
			else
				l = va_arg(ap, int);
			if (l < 0) {
				*p++ = '-';
				l = -l;
			}
			p = ch_ltoa(p, l, 10);
			break;
		case 'f':
			f = (float) va_arg(ap, double);
			if (f < 0) {
				*p++ = '-';
				f = -f;
			}
			p = ftoa(p, f, precision);
			break;
		case 'X':
		case 'x':
			c = 16;
			goto unsigned_common;
		case 'U':
		case 'u':
			c = 10;
			goto unsigned_common;
		case 'O':
		case 'o':
			c = 8;
unsigned_common:
			if (is_long)
				l = va_arg(ap, unsigned long);
			else
				l = va_arg(ap, unsigned int);
			p = ch_ltoa(p, l, c);
			break;
		default:
			*p++ = c;
			break;
		}
		i = (int)(p - s);
		if ((width -= i) < 0)
			width = 0;
		if (left_align == false)
			width = -width;
		if (width < 0) {
			if (*s == '-' && filler == '0') {
				putChar((uint8_t)*s++);
				n++;
				i--;
			}
			do {
				putChar((uint8_t)filler);
				n++;
			} while (++width != 0);
		}
		while (--i >= 0) {
			putChar((uint8_t)*s++);
			n++;
		}

		while (width) {
			putChar((uint8_t)filler);
			n++;
			width--;
		}
	}
}


extern "C" int chsnprintf(char* s, size_t size, const char* fmt, ...) {
	// write up to size-1 characters;
	// reserve one byte for null byte unless size is 0
	char* end = s + ((size == 0) ? 0 : size - 1);
	va_list ap;

	va_start(ap, fmt);
	int retval = chvprintf([&](char c) [[gnu::always_inline]] {
		if(s != end) {
			*s = c;
			s++;
		}
	}, fmt, ap);
	va_end(ap);

	// write null byte if buffer size is nonzero
	if(size > 0)
		*s = 0;

	return retval;
}
