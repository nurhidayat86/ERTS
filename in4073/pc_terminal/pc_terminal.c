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

#include <time.h>
#include <assert.h>

unsigned int mon_time_ms(void)
{
    unsigned int    ms;
    struct timeval  tv;
    struct timezone tz;

    gettimeofday(&tv, &tz);
    ms = 1000 * (tv.tv_sec % 65); // 65 sec wrap around
    ms = ms + tv.tv_usec / 1000;
    return ms;
}

void mon_delay_ms(unsigned int ms)
{
    struct timespec req, rem;

    req.tv_sec = ms / 1000;
    req.tv_nsec = 1000000 * (ms % 1000);
    assert(nanosleep(&req,&rem) == 0);
}

void send_command(struct msg_combine_t* combine_msg)
{
	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;
	uint8_t i = 0;

	encode_packet((uint8_t *) combine_msg, sizeof(struct msg_combine_t), MSG_COMBINE, output_data, &output_size);
	//printf("encode message \n");
	// printf("\n");
	// printf("%d |", (int16_t)(joystick_msg.thrust + keyboard_msg.thrust));
	// send the message
	for (i=0; i<output_size; i++) {	
		rs232_putchar((char) output_data[i]);
		//printf("0x%X ", output_data[i]);
	}
	//printf("message sent \n");
	//printf("\n");
}

int main(int argc, char **argv)
{
	uint32_t start, end, panic_start = 0;

	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;
	uint8_t i = 0;
	uint16_t log_counter = 0;
	
	struct msg_p msg;
	struct msg_telemetry_t *msg_tele;

	struct log_t *msg_log; 	

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
	uint32_t counter = 0;
			
	char c;
	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	term_initio();
	rs232_open();

	term_puts("Type ^C to exit\n");

	/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
	//	fputc(c,stderr);

	/* send & receive
	 */	
	for (;;)
	{
		
		end = mon_time_ms();
		// printf("%d \n", start);
		// printf("%d \n", end);
		// printf("%d \n", end-start);
		
		// peridocally send the command to the board
		// check panic time as well, do not send anything2
		if(((end-start) > PERIODIC_COM) && ((mon_time_ms() - panic_start) > 2000))
		{
			send_command(&combine_msg);
			// check if panic_mode happened
			if(combine_msg.mode == MODE_PANIC) 
			{
				// start panic start
				panic_start = mon_time_ms();
				keyboard_msg.mode = joystick_msg.mode = combine_msg.mode = MODE_SAFE;
			}
			start = mon_time_ms();
		}

		//JoystickCommand(fd, js, &joystick_msg);
		int16_t axis[4];
		int16_t button[8];
		while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
			button[js.number] = js.value;
			axis[js.number] = js.value;
		
			//mapping to output
			// play at 11 bit (2048 from 8192 (13BIT))
			// we might need to compress it a little bit more
			joystick_msg.roll = axis[0]>>4; 		//15>>4 = 11
			joystick_msg.pitch = axis[1]>>4; 		//15>>4 = 11
			joystick_msg.yaw = axis[2]>>4; 			//15>>4 = 11
			joystick_msg.thrust = (JOY_THRUST_OFF - axis[3])>>5; //>>8;
			//joystick_msg.thrust = axis[3]>>5; //>>8;			
			//joystick_msg.mode = (button[0] << 7) | (button[1] << 6) | (button[2] << 5) | (button[3] << 4) | (button[4] << 3) | (button[5] << 2) | (button[6] << 1) | (button[7]);
			//if(button[0]) joystick_msg.mode = MODE_PANIC;
			joystick_msg.update = TRUE;
			//printf("update joystick \n");
			
		}

		if ((c = term_getchar_nb()) != -1){
			//printf("%d",c);
			KeyboardCommand(c, &keyboard_msg);
			keyboard_msg.update = TRUE;
			//printf("update keyboard \n");
		}

		// combine keyboard and joystick
		combine_msg.update = (keyboard_msg.update || joystick_msg.update);
		if(combine_msg.update){
			CombineCommand(&combine_msg, &keyboard_msg, &joystick_msg);
			//printf("update combine \n");
			// encode the message
		}

		// receive data from board
		if ((c = rs232_getchar_nb()) != -1){			
			// print the message sent by the board to the terminal
			// printf("get message from board \n");
			// if ((uint8_t)c == 0x99) printf("\n");
			// printf("0x%X ", (uint8_t)c);
			
			#ifdef ENCODE
				msg_parse(&msg, (uint8_t)c);
				//printf("0x%X \n", (uint8_t)c);
				if(msg.status == GOT_PACKET) {
					// We got a valid packet
					printf("got packet\n");
					switch(msg.msg_id) {
						case MSG_TELEMETRY: 
						{
							//printf("tele \n");
							msg_tele = (struct msg_telemetry_t *)&msg.payload[0];
							// printf("%d %d %d %d %d \n", msg.crc_fails, msg_tele->mode, msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
							printf("%d %d \n", msg.crc_fails, msg_tele->mode);
							// printf("%d %d %d %d \n", msg_tele->engine[0],msg_tele->engine[1],msg_tele->engine[2],msg_tele->engine[3]);
							// printf("%d %d %d |",phi,ae[1],ae[2],ae[3]);
							break;
						}

						case MSG_LOG: 
						{
							msg_log = (struct msg_log_t *)&msg.payload[0];
							printf("%d %d\n", msg_log->time_stamp, msg_log->mode);
							// printf("%d %d %d %d %d %d\n", msg_log->time_stamp, msg_log->mode, msg_log->thrust, msg_log->roll, msg_log->pitch, msg_log->yaw);
							// printf("%d %d %d %d \n", msg_log->ae[0],msg_log->ae[1],msg_log->ae[2],msg_log->ae[3]);
							
							// printf("%d %d %d %d \n", msg_tele->engine[0],msg_tele->engine[1],msg_tele->engine[2],msg_tele->engine[3]);
							// log_counter++;
							break;
						}
						default:
							break;
					};

					// Start to receive a new packet
					msg.status = UNITINIT;
				}
				//else{printf("0x%X \n", (uint8_t)c);}
			#else
				printf("%c", (uint8_t)c);
			#endif			
		}
	}

	term_puts("\n<exit>\n");
	term_exitio();
	rs232_close();

	return 0;
}



