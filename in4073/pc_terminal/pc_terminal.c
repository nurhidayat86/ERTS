/*------------------------------------------------------------
 * Simple pc terminal in C
 * 
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include "../logging_protocol.h"
#include "log_terminal.h"

/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
struct termios 	savetty;
FILE *kp;

void	term_initio()
{
	struct termios tty;

	tcgetattr(0, &savetty);
	tcgetattr(0, &tty);

	tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	tcsetattr(0, TCSADRAIN, &tty);
}

void	term_exitio()
{
	tcsetattr(0, TCSADRAIN, &savetty);
}

void	term_puts(char *s) 
{ 
	fprintf(stderr,"%s",s); 
}

void	term_putchar(char c) 
{ 
	putc(c,stderr); 
}

int	term_getchar_nb() 
{ 
        static unsigned char 	line [2];

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

/*------------------------------------------------------------
 * Serial I/O 
 * 8 bits, 1 stopbit, no parity, 
 * 115,200 baud
 *------------------------------------------------------------
 */
#include <termios.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>

int serial_device = 0;
int fd_RS232;

void rs232_open(void)
{
  	char 		*name;
  	int 		result;  
  	struct termios	tty;

       	fd_RS232 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);  // Hardcode your serial port here, or request it as an argument at runtime

	assert(fd_RS232>=0);

  	result = isatty(fd_RS232);
  	assert(result == 1);

  	name = ttyname(fd_RS232);
  	assert(name != 0);

  	result = tcgetattr(fd_RS232, &tty);	
	assert(result == 0);

	tty.c_iflag = IGNBRK; /* ignore break condition */
	tty.c_oflag = 0;
	tty.c_lflag = 0;

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8 bits-per-character */
	tty.c_cflag |= CLOCAL | CREAD; /* Ignore model status + read input */		

	cfsetospeed(&tty, B115200); 
	cfsetispeed(&tty, B115200); 

	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 1; // added timeout

	tty.c_iflag &= ~(IXON|IXOFF|IXANY);

	result = tcsetattr (fd_RS232, TCSANOW, &tty); /* non-canonical */

	tcflush(fd_RS232, TCIOFLUSH); /* flush I/O buffer */
}


void 	rs232_close(void)
{
  	int 	result;

  	result = close(fd_RS232);
  	assert (result==0);
}


int	rs232_getchar_nb()
{
	int 		result;
	unsigned char 	c;

	result = read(fd_RS232, &c, 1);

	if (result == 0) 
		return -1;
	
	else 
	{
		assert(result == 1);   
		return (int) c;
	}
}


int 	rs232_getchar()
{
	int 	c;

	while ((c = rs232_getchar_nb()) == -1) 
		;
	return c;
}


int 	rs232_putchar(char c)
{ 
	int result;

	do {
		result = (int) write(fd_RS232, &c, 1);
	} while (result == 0);   

	assert(result == 1);
	return result;
}


/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{
	uint8_t mode = 0;
	uint16_t count = 0;
	uint8_t decode_status;
	char	c;
	static struct msg_p_log msg_log_p;
	struct log_t *msg_log = malloc(sizeof(struct log_t));
	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	term_initio();
	rs232_open();

	term_puts("Type ^C to exit\n");
	msg_log_p.crc_fails=0;
	/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);
	
	/* send & receive
	 */
	for (;;) 
	{
		if ((c = term_getchar_nb()) != -1)
		{
			rs232_putchar(c);
			if(c == 'p'){mode = 1;}
			else if(c == 'k'){mode = 2;}
			else if(c == 'l'){mode = 0;}
			else {mode = mode;}
		} 
		
		if ((c = rs232_getchar_nb()) != -1){
			if(mode!=2) term_putchar(c);
			else if (mode==2){
				//term_puts("|");term_putchar(c);term_puts("|\n");
				decode_status = decode_log((uint8_t) c, &msg_log_p);
				if(msg_log_p.status == GOT_PACKAGE)
				{
					switch(msg_log_p.msg_ID){
						case INDEX_LOG:
							MSG_log_index = (uint16_t *)&msg_log_p.payload[0];
							break;

						case T_STAMP:
							MSG_time_stamp = (uint32_t *)&msg_log_p.payload[0];
							break;

						case MODE:
							MSG_time_mode = (uint8_t *)&msg_log_p.payload[0];
							break;

						case THRUST:
							MSG_time_thrust = (uint16_t *)&msg_log_p.payload[0];
							break;

						case ROLL:
							MSG_time_roll = (int16_t *)&msg_log_p.payload[0];
							break;

						case PITCH:
							MSG_time_pitch = (int16_t *)&msg_log_p.payload[0];
							break;

						case YAW:
							MSG_time_yaw = (int16_t *)&msg_log_p.payload[0];
							break;

						case AE_0:
							MSG_time_ae[0] = (int16_t *)&msg_log_p.payload[0];
							break;

						case AE_1:
							MSG_time_ae[1] = (int16_t *)&msg_log_p.payload[0];
							break;

						case AE_2:
							MSG_time_ae[2] = (int16_t *)&msg_log_p.payload[0];
							break;

						case AE_3:
							MSG_time_ae[3] = (int16_t *)&msg_log_p.payload[0];
							break;

						case PHI:
							MSG_time_phi = (int16_t *)&msg_log_p.payload[0];
							break;

						case THETA:
							MSG_time_theta = (int16_t *)&msg_log_p.payload[0];
							break;

						case PSI:
							MSG_time_psi = (int16_t *)&msg_log_p.payload[0];
							break;

						case SP:
							MSG_time_sp = (int16_t *)&msg_log_p.payload[0];
							break;

						case SQ:
							MSG_time_sq = (int16_t *)&msg_log_p.payload[0];
							break;

						case SR:
							MSG_time_sr = (int16_t *)&msg_log_p.payload[0];
							break;

						// case SAX:
						// 	MSG_time_sax = (int16_t *)&msg_log_p.payload[0];
						// 	break;

						// case SAY:
						// 	MSG_time_say = (int16_t *)&msg_log_p.payload[0];
						// 	break;

						// case SAZ:
						// 	MSG_time_saz = (int16_t *)&msg_log_p.payload[0];
						// 	break;

						case BAT_V:
							MSG_time_bat_volt = (uint16_t *)&msg_log_p.payload[0];
							break;

						case TEMP:
							MSG_time_temperature = (uint32_t *)&msg_log_p.payload[0];
							break;

    					case PRESS:
    						MSG_time_pressure = (uint32_t *)&msg_log_p.payload[0];
    						break;

						case ACK:
							printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", 
							MSG_time_index_log, MSG_time_time_stamp, MSG_time_mode, MSG_time_thrust, MSG_time_roll, MSG_time_pitch, MSG_time_yaw,
							MSG_time_ae[0], MSG_time_ae[1], MSG_time_ae[2], MSG_time_ae[3], MSG_time_phi, MSG_time_theta, MSG_time_psi,
							MSG_time_sp, MSG_time_sq, MSG_time_sr, MSG_time_bat_volt, MSG_time_temperature, MSG_time_pressure);
							break;

						default:
							break;

					}
					msg_log_p.status = UNITINIT;
				}
			}
		}

	}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");
  	
	return 0;
}


