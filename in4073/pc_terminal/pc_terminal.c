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

#define Bound(_x, _min, _max) { if (_x > (_max)) _x = (_max); else if (_x < (_min)) _x = (_min); }


/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{
	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;
	uint8_t i = 0;
	// struct msg_p msg;
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

	// struct msg_combine_t combine_msg;
	// combine_msg.mode = 0;
	// combine_msg.thrust = 0;
	// combine_msg.roll = 0;
	// combine_msg.pitch = 0;
	// combine_msg.yaw = 0;
	// combine_msg.update = FALSE;

	// joystick
	int fd = 0;
	struct js_event js;
	init_joystick(&fd);

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
		//JoystickCommand(fd, js, &joystick_msg);
		int axis[4];
		char button[8];
		while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
			button[js.number] = js.value;
			axis[js.number] = js.value;
		
			//mapping to output
			joystick_msg.roll = axis[0]>>7;
			joystick_msg.pitch = axis[1]>>7;
			joystick_msg.yaw = axis[2]>>7;
			joystick_msg.thrust = (MAX_ATTITUDE - axis[3])>>8;
			//joystick_msg.mode = (button[0] << 7) | (button[1] << 6) | (button[2] << 5) | (button[3] << 4) | (button[4] << 3) | (button[5] << 2) | (button[6] << 1) | (button[7]);
			//joystick_msg.mode = button[3];
			joystick_msg.update = true;
		}
		
		if(joystick_msg.update)
		{
			encode_packet((uint8_t *) &joystick_msg, sizeof(struct msg_joystick_t), MSG_JOYSTICK, output_data, &output_size);
		
			//printf("Axes: %s Output data:", &axes);
			for (i=0; i<output_size; i++) {
			rs232_putchar((char) output_data[i]);
			//printf("0x%X ", output_data[i]);
			}
			joystick_msg.update=FALSE;
		}

		if ((c = term_getchar_nb()) != -1){
			KeyboardCommand(c, &keyboard_msg);
			keyboard_msg.update=TRUE;
			encode_packet((uint8_t *) &keyboard_msg, sizeof(struct msg_keyboard_t), MSG_KEYBOARD, output_data, &output_size);
	
			//printf("Packet: ");
			for (i=0; i<output_size; i++)
			{
				//printf("0x%X ", output_data[i]);
				rs232_putchar((char) output_data[i]);
			}
			//printf("\n");
			// send the pressed key to the board 
			// rs232_putchar(c);
		}

		// combine_msg.update = (joystick_msg.update || keyboard_msg.update);
			
		// if(combine_msg.update)
		// {
		// 	encode_packet((uint8_t *) &combine_msg, sizeof(struct msg_combine_t), MSG_COMBINE, output_data, &output_size);
			
		// 	combine_msg.thrust = joystick_msg.thrust + keyboard_msg.thrust;	
		// 	combine_msg.roll = joystick_msg.roll + keyboard_msg.roll;
		// 	combine_msg.pitch = joystick_msg.pitch + keyboard_msg.pitch;
		// 	combine_msg.yaw = joystick_msg.yaw + keyboard_msg.yaw;

		// 	//printf("Axes: %s Output data:", &axes);
		// 	for (i=0; i<output_size; i++) {
		// 		rs232_putchar((char) output_data[i]);
		// 		//printf("0x%X ", output_data[i]);
		// 	}
		// 	combine_msg.update=FALSE;
		// 	joystick_msg.update=FALSE;
		// 	keyboard_msg.update=FALSE;
		// }

		//sending packet
		if ((c = rs232_getchar_nb()) != -1){
			
			// print the message sent by the board to the terminal
			printf("%c", c);
			//msg_parse(&msg, c);
			//if(msg.status == GOT_PACKET) {
				// We got a valid packet

				// Start to receive a new packet
			//	msg.status = UNITINIT;
			//}
		}
	}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}



