/*------------------------------------------------------------
 * Serial I/O
 * 8 bits, 1 stopbit, no parity,
 * 115,200 baud
 *------------------------------------------------------------
 */

#include "serial.h"

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

	result = tcsetattr (fd_RS232, TCSANOW, &tty); /* non-canonical */ //the change will occur immediately.

	tcflush(fd_RS232, TCIOFLUSH); /* flush I/O buffer */
}


void rs232_close(void)
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
	
	// static struct msg_p msg;

	// msg_parse(&msg, c);
	// if(msg.status == GOT_PACKET) {
	// 		// We got a valid packet
	// 		printf("Got a message %d\n", msg.msg_id);

	// 		switch(msg.msg_id) {
	// 			case MSG_JOYSTICK: {
	// 				struct msg_joystick_t *msg_js = (struct msg_joystick_t *)&msg.payload[0];
	// 				printf("Payload %d %d %d %d\n", msg_js->roll, msg_js->pitch, msg_js->yaw, msg_js->thrust);
	// 				break;
	// 			}

	// 			default:
	// 				break;
	// 		};

	// 		// Start to receive a new packet
	// 		msg.status = UNITINIT;
	// 	}

	do {
		result = (int) write(fd_RS232, &c, 1);
	} while (result == 0);

	assert(result == 1);
	return result;
}
