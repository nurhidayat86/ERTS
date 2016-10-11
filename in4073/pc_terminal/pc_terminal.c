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
#include <pthread.h>

// #include "../logging_protocol.h"
// #include "log_terminal.h"

/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */

#include <time.h>
#include <assert.h>

#define REFRESH_TIME 200
pthread_mutex_t lock_send;
bool heartbeat_flag;
struct msg_combine_t combine_msg;
void *heartbeat(void* x_void_ptr);
uint32_t hb_timer;

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

uint32_t mon_time_us(void)
{
    unsigned int    us;
    struct timeval  tv;
    struct timezone tz;

    gettimeofday(&tv, &tz);
   
    us = tv.tv_usec;
    return us;
}

void *heartbeat(void* x_void_ptr)
{
    while(combine_msg.mode != MODE_LOG)
    {
        if(mon_time_ms() - hb_timer > 500)
        {
	        pthread_mutex_lock( &lock_send );
	        if(!heartbeat_flag) combine_msg.mode = MODE_PANIC;
	        // printf("heartbeat failed ");
	        pthread_mutex_unlock( &lock_send );
		}
        // usleep(REFRESH_TIME*1000); // 200ms
        heartbeat_flag = false;
    }
    /* the function must return something - NULL will do */
    return NULL;
}

int main(int argc, char **argv)
{
	
	/* this variable is our reference to the second thread */
    pthread_t heartbeat_thread;
    uint8_t thread_status;
 	int x = 0;

    // status = pthread_create(&inc_x_thread, NULL, inc_x, &x);
	thread_status = pthread_create(&heartbeat_thread, NULL, heartbeat, (void*)&x);
	
	// periodic command timer 
	uint32_t start, end, panic_start = 0;
	
	// profile timer
	#ifdef PC_PROFILE 
	uint32_t start_profile, end_profile = 0;
	uint32_t proc_send = 0;
	uint32_t proc_joy = 0;
	uint32_t proc_key = 0;
	uint32_t proc_comb = 0;
	uint32_t proc_read = 0;
	#endif

	// encode variable
	// uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	// uint8_t output_size;
	// uint8_t i = 0;

	// uint16_t log_counter = 0;
	// struct log_t *msg_log; 
	// uint32_t counter = 0;	

	#ifdef ENCODE_PC_RECEIVE
	struct msg_p msg;
	struct msg_telemetry_t *msg_tele;
	struct msg_log_t *msg_logging;
		#ifdef DRONE_PROFILE
		struct msg_profile_t *msg_profile;
		#endif
	#endif

	//struct msg_profile_t *msg_profile;

	// message struck
	struct msg_joystick_t joystick_msg;
	struct msg_keyboard_t keyboard_msg;
	//struct msg_combine_t combine_msg;
	struct msg_tuning_t tuning_msg;
	
	// initialize the message by zeroing all value
	InitCommand(&combine_msg, &keyboard_msg, &joystick_msg, &tuning_msg);

	// logging variable
	FILE *kp;
	uint8_t decode_status;
 	// static struct msg_p_log msg_log_p;
	bool log_start = TRUE;

	// joystick initialization
	int fd = 0;
	struct js_event js;
	init_joystick(&fd);
	int16_t axis[4];
	int16_t button[8];
	bool warning = FALSE;
		
	// communication initialization	
	char c;
	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	term_initio();
	rs232_open();

	term_puts("Type ^C to exit\n");

	/* discard any incoming text
	 */
	//while ((c = rs232_getchar_nb()) != -1)
	//	fputc(c,stderr);

	// check joystick command when start up the system
	while((!joystick_msg.update) || (joystick_msg.thrust != 0) || joystick_msg.roll != 0 || joystick_msg.pitch != 0 || joystick_msg.yaw != 0)
	{
		if (read(fd_RS232, &c, 1)){printf("%c", (uint8_t)c);}

		while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
			
			switch(js.type & ~JS_EVENT_INIT) {
				case JS_EVENT_BUTTON:
					button[js.number] = js.value;
					break;
				case JS_EVENT_AXIS:
					axis[js.number] = js.value;
					break;
			}
		
			// scale it down to 9 bit signed integer (-255 to 255), make it less sensitive
			joystick_msg.roll = axis[0]>>7; 
			joystick_msg.pitch = axis[1]>>7;
			joystick_msg.yaw = axis[2]>>6;
			
			// scale it down from U16 to U12 (4096) we might need to compress it a little bit more
			joystick_msg.thrust = (JOY_THRUST_OFF - axis[3])>>4;
			joystick_msg.update = TRUE;
			warning = TRUE;	
		}

		if(warning)		// print a warning to terminal
		{
			printf("WARNING! thrust %d roll %d pitch %d yaw %d \n", joystick_msg.thrust, joystick_msg.roll, joystick_msg.pitch, joystick_msg.yaw);
			warning = FALSE;	
		}

	}

	/* send & receive */	
	while(combine_msg.mode != MODE_LOG)		// while loop for the mission phase
	{
		#ifdef PC_PROFILE 
			start_profile = mon_time_us();
		#endif
		// peridocally send the command to the board
		// check panic time as well, do not send anything if we are in the panic time interval
		end = mon_time_ms();
		if(((end-start) > PERIODIC_COM) && ((mon_time_ms() - panic_start) > PANIC_TIME_MS) && (combine_msg.mode != MODE_LOG))
		{
			// send gain tuning message
			// the attitude command wont be sent if the gain tuning updated
			if(tuning_msg.update)
			{
				SendCommandTuning(&tuning_msg);
				tuning_msg.update = FALSE;
			}
			else // send thrust and attitude command
			{
				SendCommand(&combine_msg);
				// combine_msg.update = FALSE;			
			}
			
			// check if panic_mode happened
			#ifdef ENCODE_PC_RECEIVE
			if(combine_msg.mode == MODE_PANIC || (msg.crc_fails > 4)) 
			#else
			if(combine_msg.mode == MODE_PANIC) 
			#endif
			{
				// start panic start, reset the mode to the safe mode
				panic_start = mon_time_ms();
				keyboard_msg.mode = joystick_msg.mode = combine_msg.mode = MODE_SAFE;
				combine_msg.thrust = keyboard_msg.thrust = 0;	
				combine_msg.roll = keyboard_msg.roll = 0;
				combine_msg.pitch = keyboard_msg.pitch = 0;
				combine_msg.yaw = keyboard_msg.yaw = 0;
				tuning_msg.P = 0;
				tuning_msg.P1 = 0;
				tuning_msg.P2 = 0;
			}
			start = mon_time_ms();

			// change the mode to log mode, if we abort the mission
			if(combine_msg.mode == ESCAPE) combine_msg.mode = MODE_LOG;	
		}
		#ifdef PC_PROFILE
			end_profile = mon_time_us();
			proc_send = end_profile - start_profile;
			//printf("s %d ", proc_send);
		#endif

		// JoystickCommand(fd, js, &joystick_msg);
		// Read the joystick event
		#ifdef PC_PROFILE 
			start_profile = mon_time_us();
		#endif
		while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
			
			switch(js.type & ~JS_EVENT_INIT) {
				case JS_EVENT_BUTTON:
					button[js.number] = js.value;
					break;
				case JS_EVENT_AXIS:
					axis[js.number] = js.value;
					break;
			}
		
			// scale it down to 9 bit signed integer (-255 to 255)
			joystick_msg.roll = axis[0]>>7; 
			joystick_msg.pitch = axis[1]>>7;
			joystick_msg.yaw = axis[2]>>6; // make it more sensitive than the others 
			
			// scale it down from U16 to U12 (4096) we might need to compress it a little bit more
			joystick_msg.thrust = (JOY_THRUST_OFF - axis[3])>>4;
			
			if(button[0]) joystick_msg.mode = MODE_SAFE;	// assign the fire button
			joystick_msg.update = TRUE;
		}
		#ifdef PC_PROFILE
			end_profile = mon_time_us();
			proc_joy = end_profile - start_profile;
			//printf("j %d ", proc_joy);
		#endif

		
		// read the keyboard
		// it is also used to update the tuning gain
		#ifdef PC_PROFILE 
			start_profile = mon_time_us();
		#endif
		if ((c = term_getchar_nb()) != -1){
			KeyboardCommand(c, &keyboard_msg, &tuning_msg, &joystick_msg);
			keyboard_msg.update = TRUE;
		}
		#ifdef PC_PROFILE
			end_profile = mon_time_us();
			proc_key = end_profile - start_profile;
			//printf("k %d ", proc_key);
		#endif

		// combine keyboard and joystick
		#ifdef PC_PROFILE 
			start_profile = mon_time_us();
		#endif
		combine_msg.update = (keyboard_msg.update || joystick_msg.update);
		if(combine_msg.update){
			CombineCommand(&combine_msg, &keyboard_msg, &joystick_msg, &tuning_msg);
		}
		#ifdef PC_PROFILE
			end_profile = mon_time_us();
			proc_comb = end_profile - start_profile;
			//printf("c %d ", proc_comb);
		#endif

		// read the uart data from drone
		#ifdef PC_PROFILE 
			start_profile = mon_time_us();
		#endif
		
		if (read(fd_RS232, &c, 1)){

		#ifdef PC_PROFILE
			end_profile = mon_time_us();
			proc_read = end_profile - start_profile;	
		#endif
				
			#ifdef ENCODE_PC_RECEIVE
			msg_parse(&msg, (uint8_t)c);
			if(msg.status == GOT_PACKET) { // We got a valid packet
				switch(msg.msg_id) {
					case MSG_TELEMETRY: 
					{
						heartbeat_flag = true;
						hb_timer = mon_time_ms();
						msg_tele = (struct msg_telemetry_t *)&msg.payload[0];
						printf("%d %d %d %d %d %d| ", msg.crc_fails, msg_tele->mode, msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
						printf("%d %d %d %d| ", msg_tele->engine[0],msg_tele->engine[1],msg_tele->engine[2],msg_tele->engine[3]);
						printf("%d %d %d| ",msg_tele->phi, msg_tele->theta, msg_tele->psi);
						printf("%d %d %d| ",msg_tele->sp, msg_tele->sq, msg_tele->sr);
						// printf("%d %d %d| ",msg_tele->sax, msg_tele->say, msg_tele->saz);
						printf("%d %d %d %d\n ",msg_tele->bat_volt, msg_tele->P, msg_tele->P1, msg_tele->P2);
						// printf("s:%d j:%d k:%d c:%d r:%d\n ",proc_send, proc_joy, proc_key, proc_comb, proc_read);
						break;
					}

					#ifdef DRONE_PROFILE
					case MSG_PROFILE: 
					{
						msg_profile = (struct msg_profile_t *)&msg.payload[0];
						printf("\n%d %d %d %d %d %d %d\n", msg_profile->proc_read, msg_profile->proc_adc, msg_profile->proc_send, msg_profile->proc_log, msg_profile->proc_dmp, msg_profile->proc_control,  msg_profile->time_all);
						//mon_delay_ms(2);
						break;
					}
					#endif

					default:
						break;
				};
				msg.status = UNITINIT;	// Start to receive a new packet
				msg.crc_fails = 0;		// reset the crc fail number
			}

			// decode_status = decode_log((uint8_t) c, &msg_log_p);
			// if(msg_log_p.status == GOT_PACKAGE)
			// {
			// 	switch(msg_log_p.msg_ID)
			// 	{
			// 		case ACK:
			// 			MSG_ack = (uint8_t *)&msg_log_p.payload[0];
			// 			np_MSG_ack = *MSG_ack;
			// 			if (np_MSG_ack == NOK)
			// 			{
			// 				printf("error reading fifo error\n");
			// 			}
			// 			break;

			// 		default:
			// 			break;
			// 	}
			// 	msg_log_p.status = UNITINIT;
			// }

			#else
				#ifndef PC_PROFILE
					printf("%c", (uint8_t)c);
				#endif // PC_PROFILE
			#endif		
		}

		// if(msg_tele->bat_volt<1100){printf("The BATTERY is LOW")}
		// if(msg_tele->bat_volt<1050){combine_msg.mode = MODE_PANIC;}
	}

	// printf("mode %d ", combine_msg.mode);
	
	while(true) // start logging 
	{
		if (read(fd_RS232, &c, 1)){ 				// if ((c = rs232_getchar_nb()) != -1){		
			#ifdef ENCODE_PC_RECEIVE				//#ifdef ENCODE
				msg_parse(&msg, (uint8_t)c);
				if(msg.status == GOT_PACKET) { 		// We got a valid packet
					// printf("got packet");
					switch(msg.msg_id) {
						case MSG_TELEMETRY: 
						{
							msg_tele = (struct msg_telemetry_t *)&msg.payload[0];
							printf("%d %d %d %d %d %d| ", msg.crc_fails, msg_tele->mode, msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
							printf("%d %d %d %d| ", msg_tele->engine[0],msg_tele->engine[1],msg_tele->engine[2],msg_tele->engine[3]);
							printf("%d %d %d| ",msg_tele->phi, msg_tele->theta, msg_tele->psi);
							printf("%d %d %d| ",msg_tele->sp, msg_tele->sq, msg_tele->sr);
							printf("%d %d %d| ",msg_tele->sax, msg_tele->say, msg_tele->saz);
							printf("%d\n ",msg_tele->bat_volt);
							// printf("s:%d j:%d k:%d c:%d r:%d\n ",proc_send, proc_joy, proc_key, proc_comb, proc_read);
							break;
						}

						case MSG_LOG: 
						{
							msg_logging = (struct msg_log_t *)&msg.payload[0];
							printf("%d %d | %d | %d %d %d %d | ", msg_logging->index_log, msg_logging->time_stamp, msg_logging->mode, msg_logging->thrust, msg_logging->roll, msg_logging->pitch, msg_logging->yaw);
							printf("%d %d %d %d | ", msg_logging->ae[0], msg_logging->ae[1], msg_logging->ae[2], msg_logging->ae[3]);
							printf("%d %d %d | ", msg_logging->phi, msg_logging->theta, msg_logging->psi);
							printf("%d %d %d | ", msg_logging->sp, msg_logging->sq, msg_logging->sr); 
							printf("%d %d %d %d |%d %d\n", msg_logging->bat_volt, msg_logging->P, msg_logging->P1, msg_logging->P2, msg_logging->temperature, msg_logging->pressure);
							break;
						}

						default:
							break;
					};
					msg.status = UNITINIT;	// Start to receive a new packet
					msg.crc_fails = 0;
				}
				//if(c == 0x99) printf("\n");
				//printf("0x%X ", (uint8_t)c);	
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




