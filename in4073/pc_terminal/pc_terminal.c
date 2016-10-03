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
	FILE *kp;
	uint8_t mode = 0;
	uint16_t count = 0;
	uint8_t decode_status;
	char	c;
	static struct msg_p_log msg_log_p;
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
							MSG_index_log = (uint16_t *)&msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_index_log = *MSG_index_log;
							//printf(" %d||", *MSG_index_log);
							break;

						case T_STAMP:
							MSG_time_stamp = (uint32_t *)&msg_log_p.payload[0];
							// MSG_time_stamp = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_time_stamp = *MSG_time_stamp;
							//printf(" %d||", *MSG_time_stamp);
							break;

						case MODE:
							MSG_mode = (uint8_t *)&msg_log_p.payload[0];
							// MSG_mode = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_mode = *MSG_mode;
							//printf(" %d||", *MSG_mode);
							break;

						case THRUST:
							MSG_thrust = (uint16_t *)&msg_log_p.payload[0];
							// MSG_thrust = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							//printf(" %d||", *MSG_thrust);
						 	break;

						case ROLL:
							MSG_roll = (int16_t *)&msg_log_p.payload[0];
							// MSG_roll = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_roll = *MSG_roll;
							//printf(" %d||", *MSG_roll);
							break;

						case PITCH:
							MSG_pitch = (int16_t *)&msg_log_p.payload[0];
							// MSG_pitch = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_pitch = *MSG_pitch;
							//printf(" %d||", *MSG_pitch);
							break;

						case YAW:
							MSG_yaw = (int16_t *)&msg_log_p.payload[0];
							// MSG_yaw = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_yaw = *MSG_yaw;
							//printf(" %d||", *MSG_yaw);
							break;

						case AE_0:
							MSG_ae_0 = (int16_t *)&msg_log_p.payload[0];
							// MSG_ae_0 = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_ae_0 = *MSG_ae_0;
							//printf(" %d||", *MSG_ae_0);
							break;

						case AE_1:
							MSG_ae_1 = (int16_t *)&msg_log_p.payload[0];
							// MSG_ae_1 = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_ae_1 = *MSG_ae_1;
							//printf(" %d||", *MSG_ae_1);
							break;

						case AE_2:
							MSG_ae_2 = (int16_t *)&msg_log_p.payload[0];
							// MSG_ae_2 = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_ae_2 = *MSG_ae_2;
							//printf(" %d||", *MSG_ae_2);
							break;

						case AE_3:
							MSG_ae_3 = (int16_t *)&msg_log_p.payload[0];
							// MSG_ae_3 = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_ae_3 = *MSG_ae_3;
							//printf(" %d||", *MSG_ae_3);
							break;

						case PHI:
							MSG_phi = (int16_t *)&msg_log_p.payload[0];
							// MSG_phi = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_phi = *MSG_phi;
							//printf(" %d||", *MSG_phi);
							break;

						case THETA:
							MSG_theta = (int16_t *)&msg_log_p.payload[0];
							// MSG_theta = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_theta = *MSG_theta;
							//printf(" %d||", *MSG_theta);
							break;

						case PSI:
							MSG_psi = (int16_t *)&msg_log_p.payload[0];
							// MSG_psi = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_psi = *MSG_psi;
							//printf(" %d||", *MSG_psi);
							break;

						case SP:
							MSG_sp = (int16_t *)&msg_log_p.payload[0];
							// MSG_sp = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_sp = *MSG_sp;
							//printf(" %d||", *MSG_sp);
							break;

						case SQ:
							MSG_sq = (int16_t *)&msg_log_p.payload[0];
							// MSG_sq = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_sq = *MSG_sq;
							//printf(" %d||", *MSG_sq);
							break;

						case SR:
							MSG_sr = (int16_t *)&msg_log_p.payload[0];
							// MSG_sr = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_sr = *MSG_sr;
							//printf(" %d||\n", *MSG_sr);
							break;

						// // case SAX:
						// // 	MSG_sax = (int16_t *)&msg_log_p.payload[0];
							// np_MSG_sax = *MSG_time_sax
						// // 	break;

						// // case SAY:
						// // 	MSG_say = (int16_t *)&msg_log_p.payload[0];
						// // 	break;

						// // case SAZ:
						// // 	MSG_saz = (int16_t *)&msg_log_p.payload[0];
						// // 	break;

						case BAT_V:
							MSG_bat_volt = (uint16_t *)&msg_log_p.payload[0];
							// MSG_sr = &msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_bat_volt = *MSG_bat_volt;
							//printf(" %d||\n", *MSG_bat_volt);
							break;

						case TEMP:
							MSG_temperature = (uint32_t *)&msg_log_p.payload[0];
							term_putchar(msg_log_p.payload[0]+60);
							np_MSG_temperature = *MSG_temperature;
							//printf(" %d||\n", *MSG_temperature);
							break;

    					case PRESS:
    						MSG_pressure = (uint32_t *)&msg_log_p.payload[0];
    						term_putchar(msg_log_p.payload[0]+60);
    						np_MSG_pressure = *MSG_pressure;
    						//printf(" %d||\n", *MSG_pressure);
    						break;

						 case ACK:
						 	MSG_ack = (uint8_t *)&msg_log_p.payload[0];
						 	np_MSG_ack = *MSG_ack;
						 	if (np_MSG_ack == INIT)
						 	{
						 		kp = fopen("logging.csv","w+");
						 		fprintf(kp,"index, time stamp, mode, thrust, roll, pitch, yaw, ae0, ae1, ae2, ae3, phi, theta, psi, sp, sq, sr, bat_volt, temperature, pressure\n");
						 		fprintf(kp,"%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", 
								np_MSG_index_log, np_MSG_time_stamp, np_MSG_mode, np_MSG_thrust, np_MSG_roll, np_MSG_pitch, np_MSG_yaw,
								np_MSG_ae_0, np_MSG_ae_1, np_MSG_ae_2, np_MSG_ae_3, np_MSG_phi, np_MSG_theta, np_MSG_psi,
								np_MSG_sp, np_MSG_sq, np_MSG_sr, np_MSG_bat_volt, np_MSG_temperature, np_MSG_pressure);
						 	}
						 	else if (np_MSG_ack == OK)
						 	{
						 		fprintf(kp,"%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", 
								np_MSG_index_log, np_MSG_time_stamp, np_MSG_mode, np_MSG_thrust, np_MSG_roll, np_MSG_pitch, np_MSG_yaw,
								np_MSG_ae_0, np_MSG_ae_1, np_MSG_ae_2, np_MSG_ae_3, np_MSG_phi, np_MSG_theta, np_MSG_psi,
								np_MSG_sp, np_MSG_sq, np_MSG_sr, np_MSG_bat_volt, np_MSG_temperature, np_MSG_pressure);
						 	}
							else if (np_MSG_ack == COMPLETE)
							{
								fprintf(kp,"%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", 
								np_MSG_index_log, np_MSG_time_stamp, np_MSG_mode, np_MSG_thrust, np_MSG_roll, np_MSG_pitch, np_MSG_yaw,
								np_MSG_ae_0, np_MSG_ae_1, np_MSG_ae_2, np_MSG_ae_3, np_MSG_phi, np_MSG_theta, np_MSG_psi,
								np_MSG_sp, np_MSG_sq, np_MSG_sr, np_MSG_bat_volt, np_MSG_temperature, np_MSG_pressure);
								fclose(kp);
							}
							flash_np();
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


