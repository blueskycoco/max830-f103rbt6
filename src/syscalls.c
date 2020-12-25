#include <sys/stat.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>
#include "mymisc.h"
#define ITM_ENA   (*(volatile unsigned int*)0xE0000E00) // ITM Enable
#define ITM_TPR   (*(volatile unsigned int*)0xE0000E40) // Trace Privilege Register
#define ITM_TCR   (*(volatile unsigned int*)0xE0000E80) // ITM Trace Control Reg.
#define ITM_LSR   (*(volatile unsigned int*)0xE0000FB0) // ITM Lock Status Register
#define DHCSR     (*(volatile unsigned int*)0xE000EDF0) // Debug register
#define DEMCR     (*(volatile unsigned int*)0xE000EDFC) // Debug register
#define TPIU_ACPR (*(volatile unsigned int*)0xE0040010) // Async Clock presacler register
#define TPIU_SPPR (*(volatile unsigned int*)0xE00400F0) // Selected Pin Protocol Register
#define DWT_CTRL  (*(volatile unsigned int*)0xE0001000) // DWT Control Register
#define FFCR      (*(volatile unsigned int*)0xE0040304) // Formatter and flush Control Register

#define ITM_STIM_U32  (*(volatile unsigned int*)0xE0000000)
#define ITM_STIM_U8   (*(volatile char*)0xE0000000)

unsigned int ITM_PORT_BIT0 = 0;

unsigned int TargetDiv = 32;

void SWO_Enable( void )
{
	unsigned int StimulusRegs;
	DEMCR |= ( 1 << 24 );
	ITM_LSR = 0xC5ACCE55;
	StimulusRegs = ITM_ENA;
	StimulusRegs &= ~( 1 << ITM_PORT_BIT0 );
	ITM_ENA = StimulusRegs; 
	ITM_TCR = 0;            

	TPIU_SPPR = 0x00000002;     
	TPIU_ACPR = TargetDiv - 1;
	ITM_TPR = 0x00000000;
	DWT_CTRL = 0x400003FE;
	FFCR = 0x00000100;
	ITM_TCR = 0x1000D;
	ITM_ENA = StimulusRegs | ( 1 << ITM_PORT_BIT0 );
}

void SWO_PrintChar( char c )
{
	if ( ( DHCSR & 1 ) != 1 )
		return;

	if ( ( DEMCR & ( 1 << 24 ) ) == 0 )
		return;

	if ( ( ITM_TCR & ( 1 << 22 ) ) == 1 )
		return;

	if ( ( ITM_ENA & 1 ) == 0 )
		return;

	while ( ( ITM_STIM_U8 & 1 ) == 0 )
	{

	}

	ITM_STIM_U8 = c;
}

/************** nprintf **********************/

/* private function */
#define isdigit(c)  ((unsigned)((c) - '0') < 10)
#define rt_inline                   static __inline
#define RT_PRINTF_PRECISION
#ifdef RT_PRINTF_LONGLONG
rt_inline int divide(long long *n, int base)
{
	int res;

	/* optimized for processor which does not support divide instructions. */
	if (base == 10)
	{
		res = (int)(((unsigned long long)*n) % 10U);
		*n = (long long)(((unsigned long long)*n) / 10U);
	}
	else
	{
		res = (int)(((unsigned long long)*n) % 16U);
		*n = (long long)(((unsigned long long)*n) / 16U);
	}

	return res;
}
#else
rt_inline int divide(long *n, int base)
{
	int res;

	/* optimized for processor which does not support divide instructions. */
	if (base == 10)
	{
		res = (int)(((unsigned long)*n) % 10U);
		*n = (long)(((unsigned long)*n) / 10U);
	}
	else
	{
		res = (int)(((unsigned long)*n) % 16U);
		*n = (long)(((unsigned long)*n) / 16U);
	}

	return res;
}
#endif

rt_inline int skip_atoi(const char **s)
{
	register int i = 0;
	while (isdigit(**s))
		i = i * 10 + *((*s)++) - '0';

	return i;
}

#define ZEROPAD     (1 << 0)    /* pad with zero */
#define SIGN        (1 << 1)    /* unsigned/signed long */
#define PLUS        (1 << 2)    /* show plus */
#define SPACE       (1 << 3)    /* space if plus */
#define LEFT        (1 << 4)    /* left justified */
#define SPECIAL     (1 << 5)    /* 0x */
#define LARGE       (1 << 6)    /* use 'ABCDEF' instead of 'abcdef' */

#ifdef RT_PRINTF_PRECISION
static char *print_number(char *buf,
		char *end,
#ifdef RT_PRINTF_LONGLONG
		long long  num,
#else
		long  num,
#endif
		int   base,
		int   s,
		int   precision,
		int   type)
#else
static char *print_number(char *buf,
		char *end,
#ifdef RT_PRINTF_LONGLONG
		long long  num,
#else
		long  num,
#endif
		int   base,
		int   s,
		int   type)
#endif
{
	char c, sign;
#ifdef RT_PRINTF_LONGLONG
	char tmp[32];
#else
	char tmp[16];
#endif
	int precision_bak = precision;
	const char *digits;
	static const char small_digits[] = "0123456789abcdef";
	static const char large_digits[] = "0123456789ABCDEF";
	register int i;
	register int size;

	size = s;

	digits = (type & LARGE) ? large_digits : small_digits;
	if (type & LEFT)
		type &= ~ZEROPAD;

	c = (type & ZEROPAD) ? '0' : ' ';

	/* get sign */
	sign = 0;
	if (type & SIGN)
	{
		if (num < 0)
		{
			sign = '-';
			num = -num;
		}
		else if (type & PLUS)
			sign = '+';
		else if (type & SPACE)
			sign = ' ';
	}

#ifdef RT_PRINTF_SPECIAL
	if (type & SPECIAL)
	{
		if (base == 16)
			size -= 2;
		else if (base == 8)
			size--;
	}
#endif

	i = 0;
	if (num == 0)
		tmp[i++] = '0';
	else
	{
		while (num != 0)
			tmp[i++] = digits[divide(&num, base)];
	}

#ifdef RT_PRINTF_PRECISION
	if (i > precision)
		precision = i;
	size -= precision;
#else
	size -= i;
#endif

	if (!(type & (ZEROPAD | LEFT)))
	{
		if ((sign) && (size > 0))
			size--;

		while (size-- > 0)
		{
			if (buf < end)
				*buf = ' ';
			++ buf;
		}
	}

	if (sign)
	{
		if (buf < end)
		{
			*buf = sign;
		}
		-- size;
		++ buf;
	}

#ifdef RT_PRINTF_SPECIAL
	if (type & SPECIAL)
	{
		if (base == 8)
		{
			if (buf < end)
				*buf = '0';
			++ buf;
		}
		else if (base == 16)
		{
			if (buf < end)
				*buf = '0';
			++ buf;
			if (buf < end)
			{
				*buf = type & LARGE ? 'X' : 'x';
			}
			++ buf;
		}
	}
#endif

	/* no align to the left */
	if (!(type & LEFT))
	{
		while (size-- > 0)
		{
			if (buf < end)
				*buf = c;
			++ buf;
		}
	}

#ifdef RT_PRINTF_PRECISION
	while (i < precision--)
	{
		if (buf < end)
			*buf = '0';
		++ buf;
	}
#endif

	/* put number in the temporary buffer */
	while (i-- > 0 && (precision_bak != 0))
	{
		if (buf < end)
			*buf = tmp[i];
		++ buf;
	}

	while (size-- > 0)
	{
		if (buf < end)
			*buf = ' ';
		++ buf;
	}

	return buf;
}

int rt_vsnprintf(char       *buf,
		uint32_t   size,
		const char *fmt,
		va_list     args)
{
#ifdef RT_PRINTF_LONGLONG
	unsigned long long num;
#else
	uint32_t num;
#endif
	int i, len;
	char *str, *end, c;
	const char *s;

	uint8_t base;            /* the base of number */
	uint8_t flags;           /* flags to print number */
	uint8_t qualifier;       /* 'h', 'l', or 'L' for integer fields */
	int32_t field_width;     /* width of output field */

#ifdef RT_PRINTF_PRECISION
	int precision;      /* min. # of digits for integers and max for a string */
#endif

	str = buf;
	end = buf + size;

	/* Make sure end is always >= buf */
	if (end < buf)
	{
		end  = ((char *) - 1);
		size = end - buf;
	}

	for (; *fmt ; ++fmt)
	{
		if (*fmt != '%')
		{
			if (str < end)
				*str = *fmt;
			++ str;
			continue;
		}

		/* process flags */
		flags = 0;

		while (1)
		{
			/* skips the first '%' also */
			++ fmt;
			if (*fmt == '-') flags |= LEFT;
			else if (*fmt == '+') flags |= PLUS;
			else if (*fmt == ' ') flags |= SPACE;
			else if (*fmt == '#') flags |= SPECIAL;
			else if (*fmt == '0') flags |= ZEROPAD;
			else break;
		}

		/* get field width */
		field_width = -1;
		if (isdigit(*fmt)) field_width = skip_atoi(&fmt);
		else if (*fmt == '*')
		{
			++ fmt;
			/* it's the next argument */
			field_width = va_arg(args, int);
			if (field_width < 0)
			{
				field_width = -field_width;
				flags |= LEFT;
			}
		}

#ifdef RT_PRINTF_PRECISION
		/* get the precision */
		precision = -1;
		if (*fmt == '.')
		{
			++ fmt;
			if (isdigit(*fmt)) precision = skip_atoi(&fmt);
			else if (*fmt == '*')
			{
				++ fmt;
				/* it's the next argument */
				precision = va_arg(args, int);
			}
			if (precision < 0) precision = 0;
		}
#endif
		/* get the conversion qualifier */
		qualifier = 0;
#ifdef RT_PRINTF_LONGLONG
		if (*fmt == 'h' || *fmt == 'l' || *fmt == 'L')
#else
			if (*fmt == 'h' || *fmt == 'l')
#endif
			{
				qualifier = *fmt;
				++ fmt;
#ifdef RT_PRINTF_LONGLONG
				if (qualifier == 'l' && *fmt == 'l')
				{
					qualifier = 'L';
					++ fmt;
				}
#endif
			}

		/* the default base */
		base = 10;

		switch (*fmt)
		{
			case 'c':
				if (!(flags & LEFT))
				{
					while (--field_width > 0)
					{
						if (str < end) *str = ' ';
						++ str;
					}
				}

				/* get character */
				c = (uint8_t)va_arg(args, int);
				if (str < end) *str = c;
				++ str;

				/* put width */
				while (--field_width > 0)
				{
					if (str < end) *str = ' ';
					++ str;
				}
				continue;

			case 's':
				s = va_arg(args, char *);
				if (!s) s = "(NULL)";

				len = strlen(s);
#ifdef RT_PRINTF_PRECISION
				if (precision > 0 && len > precision) len = precision;
#endif

				if (!(flags & LEFT))
				{
					while (len < field_width--)
					{
						if (str < end) *str = ' ';
						++ str;
					}
				}

				for (i = 0; i < len; ++i)
				{
					if (str < end) *str = *s;
					++ str;
					++ s;
				}

				while (len < field_width--)
				{
					if (str < end) *str = ' ';
					++ str;
				}
				continue;

			case 'p':
				if (field_width == -1)
				{
					field_width = sizeof(void *) << 1;
					flags |= ZEROPAD;
				}
#ifdef RT_PRINTF_PRECISION
				str = print_number(str, end,
						(long)va_arg(args, void *),
						16, field_width, precision, flags);
#else
				str = print_number(str, end,
						(long)va_arg(args, void *),
						16, field_width, flags);
#endif
				continue;

			case '%':
				if (str < end) *str = '%';
				++ str;
				continue;

				/* integer number formats - set up the flags and "break" */
			case 'o':
				base = 8;
				break;

			case 'X':
				flags |= LARGE;
			case 'x':
				base = 16;
				break;

			case 'd':
			case 'i':
				flags |= SIGN;
			case 'u':
				break;

			default:
				if (str < end) *str = '%';
				++ str;

				if (*fmt)
				{
					if (str < end) *str = *fmt;
					++ str;
				}
				else
				{
					-- fmt;
				}
				continue;
		}

#ifdef RT_PRINTF_LONGLONG
		if (qualifier == 'L') num = va_arg(args, long long);
		else if (qualifier == 'l')
#else
			if (qualifier == 'l')
#endif
			{
				num = va_arg(args, uint32_t);
				if (flags & SIGN) num = (int32_t)num;
			}
			else if (qualifier == 'h')
			{
				num = (uint16_t)va_arg(args, int32_t);
				if (flags & SIGN) num = (int16_t)num;
			}
			else
			{
				num = va_arg(args, uint32_t);
				if (flags & SIGN) num = (int32_t)num;
			}
#ifdef RT_PRINTF_PRECISION
		str = print_number(str, end, num, base, field_width, precision, flags);
#else
		str = print_number(str, end, num, base, field_width, flags);
#endif
	}

	if (size > 0)
	{
		if (str < end) *str = '\0';
		else
		{
			end[-1] = '\0';
		}
	}

	/* the trailing null byte doesn't count towards the total
	 * ++str;
	 */
	return str - buf;
}
#define RT_CONSOLEBUF_SIZE 128
/**
 * This function will print a formatted string on system console
 *
 * @param fmt the format
 */
void nprintf(const char *fmt, ...)
{
	va_list args;
	int length;
	static char rt_log_buf[RT_CONSOLEBUF_SIZE];
	int i;
	va_start(args, fmt);
	/* the return value of vsnnprintf is the number of bytes that would be
	 * written to buffer had if the size of the buffer been sufficiently
	 * large excluding the terminating null byte. If the output string
	 * would be larger than the rt_log_buf, we have to adjust the output
	 * length. */
	length = rt_vsnprintf(rt_log_buf, sizeof(rt_log_buf) - 1, fmt, args);
	if (length > RT_CONSOLEBUF_SIZE - 1)
		length = RT_CONSOLEBUF_SIZE - 1;

	for (i = 0; i < length; i++) {
		PutChar(rt_log_buf[i]);
	}

	va_end(args);
}
