/*------------------------------------------------------------
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

struct log_t {
  int16_t phi, theta, psi, sp, sq, sr, sax, say, saz, roll, pitch, yaw, bat_volt, ae[4];
  uint16_t thrust;
  uint8_t mode;
  uint32_t temperature, pressure;
}__attribute__((packed, aligned(1)));

void write_to_csv(char *filename, struct log_t msg_log) {
	if (fopen_stat == false) 
	{
		fp = fopen(filename, "w+");
		fopen_stat = true;
		fprintf(fp, "phi, theta, psi, sp, sq, sr, sax, say, saz, thrust, roll, pitch, yaw, bat_volt, ae[0], ae[1], ae[2], ae[3], mode, temperature, pressure\n")
		fprintf(fp, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
			msg_log->phi, msg_log->theta, msg_log->psi, msg_log->sp, 
			msg_log->sq, msg_log->sr, msg_log->sax, msg_log->say, msg_log->saz, 
			msg_log->thrust, msg_log->roll, msg_log->pitch, msg_log->yaw, msg_log->bat_volt,
			msg_log->ae[0], msg_log->ae[1], msg_log->ae[2], msg_log->ae[3], msg_log->mode, msg_log->temperature, msg_log->roll->pressure);
	}
	else 
	{
		fprintf(fp, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
			msg_log->phi, msg_log->theta, msg_log->psi, msg_log->sp, 
			msg_log->sq, msg_log->sr, msg_log->sax, msg_log->say, msg_log->saz, 
			msg_log->thrust, msg_log->roll, msg_log->pitch, msg_log->yaw, msg_log->bat_volt,
			msg_log->ae[0], msg_log->ae[1], msg_log->ae[2], msg_log->ae[3], msg_log->mode, msg_log->temperature, msg_log->roll->pressure);
	}

}

void leave_log_mode() {fopen_stat = false;}