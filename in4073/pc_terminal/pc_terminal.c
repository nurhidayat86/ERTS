/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include "../protocol.h"
#include "term.h"
#include "serial.h"
#include "keyboard.h"
#include "joystick_interface.h"
#include "command.h"
//#include <ncurses.h>

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
	struct msg_telemetry_t *msg_tele;	

	struct msg_joystick_t joystick_msg;
	joystick_msg.mode = 0;
	joystick_msg.thrust = 0;
	joystick_msg.roll = 0;
	joystick_msg.pitch = 0;
	joystick_msg.yaw = 0;
	joystick_msg.update = FALSE;

	struct msg_keyboard_t keyboard_msg;
	keyboard_msg.mode = 0;
	keyboard_msg.thrust = 0;
	keyboard_msg.roll = 0;
	keyboard_msg.pitch = 0;
	keyboard_msg.yaw = 0;
	keyboard_msg.update = FALSE;

	struct msg_combine_t combine_msg;
	combine_msg.mode = 0;
	combine_msg.thrust = 0;
	combine_msg.roll = 0;
	combine_msg.pitch = 0;
	combine_msg.yaw = 0;
	combine_msg.update = FALSE;

	// joystick
	int fd = 0;
	struct js_event js;
	init_joystick(&fd);

	// special character
	// initscr();
	// raw();
	// keypad(stdscr, TRUE);
	// noecho();

	char c;
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
		//JoystickCommand(fd, js, &joystick_msg);
		int16_t axis[4];
		int16_t button[8];
		while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
			button[js.number] = js.value;
			axis[js.number] = js.value;
		
			//mapping to output
			joystick_msg.roll = axis[0]>>5; //>>7;
			joystick_msg.pitch = axis[1]>>5; //>>7;
			joystick_msg.yaw = axis[2]>>5; //>>7;
			joystick_msg.thrust = (JOY_THRUST_OFF - axis[3])>>6; //>>8;
			//joystick_msg.thrust = axis[3]>>5; //>>8;			
			//joystick_msg.mode = (button[0] << 7) | (button[1] << 6) | (button[2] << 5) | (button[3] << 4) | (button[4] << 3) | (button[5] << 2) | (button[6] << 1) | (button[7]);
			//joystick_msg.mode = button[3];
			joystick_msg.update = true;
		}

		if ((c = term_getchar_nb()) != -1){
			//printf("%d",c);
			KeyboardCommand(c, &keyboard_msg);
			keyboard_msg.update=TRUE;
		}

		// combine keyboard and joystick
		combine_msg.update = (keyboard_msg.update || joystick_msg.update);
		if(combine_msg.update){
			CombineCommand(&combine_msg, &keyboard_msg, &joystick_msg);
			
			// encode the message
			encode_packet((uint8_t *) &combine_msg, sizeof(struct msg_combine_t), MSG_COMBINE, output_data, &output_size);

			// send the message
			for (i=0; i<output_size; i++) {	
				rs232_putchar((char) output_data[i]);
				//printf("0x%X ", output_data[i]);
			}
			printf("\n");
		}

		// receive data from board
		if ((c = rs232_getchar_nb()) != -1){			
			// print the message sent by the board to the terminal
			//printf("0x%X \n", (uint8_t)c);
			msg_parse(&msg, (uint8_t)c);
			if(msg.status == GOT_PACKET) {
				// We got a valid packet
				switch(msg.msg_id) {
					case MSG_TELEMETRY: 
					{
						msg_tele = (struct msg_telemetry_t *)&msg.payload[0];
						printf("%d %d %d %d %d |", msg_tele->mode, msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
						printf("%d %d %d %d \n",msg_tele->engine[0],msg_tele->engine[1],msg_tele->engine[2],msg_tele->engine[3]);
						//printf("%d %d %d |",phi,ae[1],ae[2],ae[3]);
						break;
					}

				default:
					break;
				};

				// Start to receive a new packet
				msg.status = UNITINIT;
			}
		}
	}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}



