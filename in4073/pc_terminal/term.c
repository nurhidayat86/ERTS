/* 
* console I/O
 *------------------------------------------------------------
 */

#include "term.h"

struct termios 	savetty;

void	term_initio()
{
	struct termios tty;

	//tcgetattr - get the parameters associated with the terminal
	tcgetattr(0, &savetty);
	tcgetattr(0, &tty);

	tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	tcsetattr(0, TCSADRAIN, &tty); //If optional_actions is TCSADRAIN, the change will occur after all output written to fildes (in this case 0) is transmitted. This function should be used when changing parameters that affect output.
}

void	term_exitio()
{
	tcsetattr(0, TCSADRAIN, &savetty); //If optional_actions is TCSADRAIN, the change will occur after all output written to fildes (in this case 0) is transmitted. This function should be used when changing parameters that affect output.
}

void	term_puts(char *s)
{
	fprintf(stderr,"%s",s); //write s to stderr string
}

void  term_putchar(char c)
{
	putc(c,stderr); //write c to stderr character
}

int	term_getchar_nb()
{
        static unsigned char 	line [2];

        //The read() function shall attempt to read nbyte (1) bytes from the file associated with the open file descriptor, fildes (0), into the buffer pointed to by buf (line).
        if (read(0,line,1)) // note: destructive read
        		return (int) line[0];

        return -1;
}

int	term_getchar()
{
        int    c;

        while ((c = term_getchar_nb()) == -1)
                ;
        return c;
}
