/*------------------------------------------------------------
 * Simple pc terminal in C
 * 
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include <pthread.h>
#include "term.h"			// adjusted to ERTS
#include "serial.h"			// adjusted to ERTS
#include "joystick_interface.h"
#include "../packet.h"
#include "../crc_calc.h"

#define REFRESH_TIME 	500


pthread_mutex_t lock_send;
int end_communication = 0;


void *heartbeat(void* x_void_ptr)
{
    // int idboard = *( (int*)(board) );
    // in idboard = *(  (int*)(fd_RS232) ); 
    int c;

    while(!end_communication)
    {
		int *x_ptr = (int *)x_void_ptr;
		while(++(*x_ptr) < 100);
		
		packet_t p;
		p.sflag = 0x99;
        p.msgtype = 0x83;
        p.value = 0x01;

        crc_calc(&p);

        pthread_mutex_lock( &lock_send );

        send_packet(fd_RS232, p);

		// c = 'h';
		// rs232_putchar(c);

		// result = (int) write(fd_RS232, &p, 3);


        pthread_mutex_unlock( &lock_send );

        usleep(REFRESH_TIME*1000); //50ms
    }
    /* the function must return something - NULL will do */
    return NULL;
}


/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 *
 * adjusted to ERTS
 *
 */


/*------------------------------------------------------------
 * Serial I/O 
 * 8 bits, 1 stopbit, no parity, 
 * 115,200 baud
 *------------------------------------------------------------
 *
 * adjusted to ERTS
 *
 */


/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{
	/*
	 * output
	 * msg
	 * msg_joystick
	 * msg_keyboard
	 * msg_combine
	 *
	 */

	/*----------------------------------------------------------------
	 * ERTS -- joystick handling
	 *----------------------------------------------------------------
	 */
	int fd = 0;
	struct js_event js;
	init_joystick(&fd);
	
	int iomode = 0;
	
	int status = 0;
	
	int x = 0;
	
	/* this variable is our reference to the second thread */
    pthread_t heartbeat_thread;

	char c;
	packet_t p;

	
	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	term_initio();
	rs232_open();
	
	pthread_mutex_init(&lock_send, NULL);
	
	// status = pthread_create(&inc_x_thread, NULL, inc_x, &x);
	status = pthread_create(&heartbeat_thread, NULL, heartbeat, (void*)&x);




	term_puts("Type ^C to exit\n");

	/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);
	
	/* send & receive
	 */
	for (;;) 
	{
	/*----------------------------------------------------------------
	 * ERTS -- joystick handling
	 *----------------------------------------------------------------
	 */
		// int16_t axis[4];
		// int16_t button[8];
		int16_t axis[4];
		int16_t button[8];
		int nummer;
		int waarde;
		int x = 0;
		
		if (iomode == 1) {
		
			while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
				printf("Event: type %d, time %d, number %d, value %d\n",
						js.type, js.time, js.number, js.value);
				if (js.type==1 && js.number==11) iomode =0;
				nummer = js.number;
				waarde = js.value;
				button[nummer] = waarde;
				// axis[nummer] = waarde;
				
				// c = button[nummer];
				// rs232_putchar(c);
				
				/*
				//mapping to output
				joystick_msg.roll = axis[0]>>5; //>>7;
				joystick_msg.pitch = axis[1]>>5; //>>7;
				joystick_msg.yaw = axis[2]>>5; //>>7;
				joystick_msg.thrust = (JOY_THRUST_OFF - axis[3])>>6; //>>8;
				//joystick_msg.thrust = axis[3]>>5; //>>8;			
				//joystick_msg.mode = (button[0] << 7) | (button[1] << 6) | (button[2] << 5) | (button[3] << 4) | (button[4] << 3) | (button[5] << 2) | (button[6] << 1) | (button[7]);
				//joystick_msg.mode = button[3];

				joystick_msg.update = true;

				*/

				//	printf ("Joystick button: %c, Joystick axis: %c \n", button[js.value], axis[js.value]);
			
			}
		usleep(1000);
				
		}

	
	
		if (iomode == 0) {

			if ((c = term_getchar_nb()) != -1) {
				// rs232_putchar(c);
				p.sflag = 0x99;
				p.msgtype = c;
				p.value = 0x00;

				crc_calc(&p);

				send_packet(fd_RS232, p);

				// printf("term_getchar: %c\n",c);
				if (c == 'j') {
					iomode = 1;
				}
			}
	/*----------------------------------------------------------------
	 * ERTS -- combining keyboard and joystick input
	 *----------------------------------------------------------------
	 */
		
		
		
		
		
		
		
			if ((c = rs232_getchar_nb()) != -1) 
				term_putchar(c);

	/*----------------------------------------------------------------
	 * ERTS -- receiving data from board
	 *----------------------------------------------------------------
	 */
		
		
		
		
		
		}
		
	}

	end_communication = 1;
	
	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");
  	
	return 0;
}


