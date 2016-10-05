/*------------------------------------------------------------
 * Serial I/O
 * 8 bits, 1 stopbit, no parity,
 * 115,200 baud
 *------------------------------------------------------------
 */

#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <termios.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>

#include <stdint.h>

int serial_device;
int fd_RS232;

void rs232_open(void);
void rs232_close(void);
int	rs232_getchar_nb();
int rs232_getchar();
int rs232_putchar(char c);
uint8_t	rs232_getchar_nb_u8();

#endif  /* #ifndef _SERIAL_H_ */