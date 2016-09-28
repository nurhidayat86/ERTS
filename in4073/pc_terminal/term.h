/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
 
#ifndef _TERM_H_
#define _TERM_H_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>

struct termios savetty;
void term_initio();
void term_exitio();
void term_puts(char *s);
void term_putchar(char c);
int	term_getchar_nb();
int	term_getchar();

#endif  /* #ifndef _TERM_H_ */
