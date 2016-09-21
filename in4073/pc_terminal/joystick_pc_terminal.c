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

	result = tcsetattr (fd_RS232, TCSANOW, &tty); /* non-canonical */ //the change will occur immediately.

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

#define HDR 0x99
#define MAX_PAYLOAD 200
#define HDR_FTR_SIZE 5

enum msg_status {
	UNITINIT,
	GOT_HDR,
	GOT_LEN,
	GOT_ID,
	GOT_PAYLOAD,
	GOT_CRC1,
	GOT_PACKET
};

enum msg_ids {
	MSG_JOYSTICK
};

struct msg_joystick_t {
	uint8_t mode;
	uint16_t thrust;
	int16_t roll;
  int16_t pitch;
  int16_t yaw;
};

struct msg_p {
	enum msg_status status;
	uint8_t ck1, ck2;
	uint8_t msg_id;
	uint8_t payload_len;
	uint8_t payload_idx;
	uint8_t payload[MAX_PAYLOAD];
	uint8_t crc_fails;
};

void msg_parse(struct msg_p *msg, uint8_t c);

int 	rs232_putchar(char c)
{
	int result;
	static struct msg_p msg;

	msg_parse(&msg, c);
	if(msg.status == GOT_PACKET) {
			// We got a valid packet
			printf("Got a message %d\n", msg.msg_id);

			switch(msg.msg_id) {
				case MSG_JOYSTICK: {
					struct msg_joystick_t *msg_js = (struct msg_joystick_t *)&msg.payload[0];
					printf("Payload %d %d %d %d\n", msg_js->roll, msg_js->pitch, msg_js->yaw, msg_js->thrust);
					break;
				}

				default:
					break;
			};

			// Start to receive a new packet
			msg.status = UNITINIT;
		}

	do {
		result = (int) write(fd_RS232, &c, 1);
	} while (result == 0);

	assert(result == 1);
	return result;
}


/*------------------------------------------------------------
 * constume I/O
 *------------------------------------------------------------
 */

//void encode_packet(struct msg_joystick_t *joystick_msg, int16_t c)
void encode_packet(uint8_t *data, uint8_t len, uint8_t msg_id, uint8_t *output_data, uint8_t *output_size) {
	uint8_t i = 0;
	uint8_t checksum1 = 0;
  uint8_t checksum2 = 0;

  // Setting the header
	output_data[0] = 0x99;
	output_data[1] = len;
	checksum1 = checksum2 = len;
	output_data[2] = msg_id;
	checksum1 += msg_id;
	checksum2 += checksum1;

	// Encoding the data
	while (i<len)
	{
		output_data[i+3] = data[i];
		checksum1 += output_data[i+3];
		checksum2 += checksum1;
		i++;
	}

	// Setting the checksum
	output_data[i+3] = checksum1;
	output_data[i+4] = checksum2;

	// Set the output size
	*output_size = len + HDR_FTR_SIZE;
}

void msg_parse(struct msg_p *msg, uint8_t c) {
	switch (msg->status) {
		//waitng start byte
		case UNITINIT:
			if (c == HDR) {
				msg->status++;
			}
			break;

		case GOT_HDR:
			msg->ck1 = msg->ck2 = c;
			msg->payload_len = c;
			msg->status++;
			break;

		case GOT_LEN:
			msg->msg_id = c;
			msg->ck1 += c;
			msg->ck2 += msg->ck1;
			msg->payload_idx = 0;
			msg->status++;
			break;

		case GOT_ID:
			msg->payload[msg->payload_idx] = c;
			msg->payload_idx++;
			msg->ck1 += c;
			msg->ck2 += msg->ck1;
			if (msg->payload_idx == msg->payload_len)
				msg->status++;
			break;

		case GOT_PAYLOAD:
			if (c != msg->ck1) {
				msg->crc_fails++;
				msg->status = UNITINIT;
			}
			else {
				msg->status++;
			}
			break;

		case GOT_CRC1:
			if (c != msg->ck2) {
				msg->crc_fails++;
				msg->status = UNITINIT;
			}
			else {
				msg->status++;
			}
			break;

		default:
			break;
	}
}

/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
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

/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{
	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;
	uint8_t i = 0;
	struct msg_p msg;
	struct msg_joystick_t joystick_msg;
	joystick_msg.roll = 1;
	joystick_msg.pitch = 2;
	joystick_msg.yaw = 3;
	joystick_msg.thrust = 4;

	char	c;
	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	term_initio();
	rs232_open();

	term_puts("Type ^C to exit\n");

	/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);

	/* send & receive
	 */
	for (;;)
	{
		//press any keyboard will send data
		if ((c = term_getchar_nb()) != -1){
			//rs232_putchar(c);
			encode_packet((uint8_t *) &joystick_msg, sizeof(struct msg_joystick_t), MSG_JOYSTICK, output_data, &output_size);

			//sending packet
			printf("Packet: ");
			for (i=0; i<output_size; i++)
			{
				printf("0x%X ", output_data[i]);
				rs232_putchar((char) output_data[i]);
			}
			printf("\n");
		}

		if ((c = rs232_getchar_nb()) != -1)
			//term_putchar(c);
			msg_parse(&msg, c);
			if(msg.status == GOT_PACKET) {
				// We got a valid packet

				// Start to receive a new packet
				msg.status = UNITINIT;
			}
	}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}



