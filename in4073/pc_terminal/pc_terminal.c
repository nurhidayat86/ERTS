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
struct msg_combine_all_t combine_msg;
void *heartbeat(void* x_void_ptr);
uint32_t hb_timer, print_warning_start;
bool stop_sending = FALSE;

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
	        if(mon_time_ms() - print_warning_start > 1000) 
	        {
	        	printf("Communication link is lost, try to reconnect\n ");
	        	print_warning_start = mon_time_ms();	
	        }
	        // usleep(REFRESH_TIME*1000); // 200ms
        	heartbeat_flag = false;
	        pthread_mutex_unlock( &lock_send );
		}
        
    }
    /* the function must return something - NULL will do */
    return NULL;
}

int main(int argc, char **argv)
{
	/****************************************************************************************
	* Variable initiator
	*****************************************************************************************/
	// periodic command timer 
	uint32_t start, end, panic_start, start_batt = 0;
	
	// profile timer
	#ifdef PC_PROFILE 
	uint32_t start_profile, end_profile = 0;
	uint32_t proc_send = 0;
	uint32_t proc_joy = 0;
	uint32_t proc_key = 0;
	uint32_t proc_comb = 0;
	uint32_t proc_read = 0;
	#endif

	#ifdef ENCODE_PC_RECEIVE
	static struct msg_p msg;
	msg.crc_fails = 0;

	//pointer to receive
	struct msg_telemetry_t *msg_tele;
	struct msg_log_t *msg_logging;

		#ifdef DRONE_PROFILE
		//pointer to receive
		struct msg_profile_t *msg_profile;
		#endif
	#endif

	// message struck
	struct msg_combine_all_t combine_msg_all;
	
	// initialize the message by zeroing all value
	InitCommandUpdate(&combine_msg_all);
	CommandModeSafe(&combine_msg_all);

	// logging variable
	FILE *kp;
	uint8_t decode_status;
	bool log_start = TRUE;
	/****************************************************************************************
	* Variable initiator end
	*****************************************************************************************/

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
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);

	/****************************************************************************************
	* check joystick command when start up the system
	*****************************************************************************************/
	while((!combine_msg_all.update) || (combine_msg_all.thrust != 0) || combine_msg_all.roll != 0 || combine_msg_all.pitch != 0 || combine_msg_all.yaw != 0)
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
			combine_msg_all.roll = axis[0]>>7; 
			combine_msg_all.pitch = axis[1]>>7;
			combine_msg_all.yaw = axis[2]>>6;
			
			// scale it down from U16 to U12 (4096) we might need to compress it a little bit more
			combine_msg_all.thrust = (JOY_THRUST_OFF - axis[3])>>4;
			if(combine_msg_all.thrust > 30) combine_msg_all.thrust = (((combine_msg_all.thrust)*11)>>4)+1280;
			combine_msg_all.update = true;
			warning = TRUE;	
		}

		if(warning)		// print a warning to terminal
		{
			printf("WARNING! ILLEGAL JOYSTICK POSITION!\nthrust %d roll %d pitch %d yaw %d \n", combine_msg_all.thrust, combine_msg_all.roll, combine_msg_all.pitch, combine_msg_all.yaw);
			warning = FALSE;	
		}

	}
	/****************************************************************************************
	* End of joystick check end reset
	*****************************************************************************************/

	/* this variable is our reference to the second thread */
    pthread_t heartbeat_thread;
    uint8_t thread_status;
 	int x = 0;

 	//raw var start
 	initraw_stat();
 	//========================================================================================//
 	// ================================ACTIVATE THE HEARTBEAT=================================//
 	//========================================================================================//
	// thread_status = pthread_create(&heartbeat_thread, NULL, heartbeat, (void*)&x);

	/* send & receive */

	/****************************************************************************************
	* Mission phase
	*****************************************************************************************/
	while((combine_msg_all.mode != MODE_LOG) && (combine_msg_all.mode != MODE_FINISH))		// while loop for the mission phase
	{
		
		#ifdef PC_PROFILE 
			start_profile = mon_time_us();
		#endif
		// periodically send the command to the board
		// check panic time as well, do not send anything if we are in the panic time interval
		end = mon_time_ms();
		if((((end-start) > PERIODIC_COM) && ((mon_time_ms() - panic_start) > PANIC_TIME_MS) && (combine_msg_all.mode != MODE_LOG)) && (!stop_sending))
		{
			SendCommandAll(&combine_msg_all);

			// check if panic_mode happened
			#ifdef ENCODE_PC_RECEIVE
			if(combine_msg_all.mode == MODE_PANIC || (msg.crc_fails > 4)) 
			#else
			if(combine_msg_all.mode == MODE_PANIC) 
			#endif
			{
				// start panic start, reset the mode to the safe mode
				panic_start = mon_time_ms();
				CommandModeSafe(&combine_msg_all);
			}
			start = mon_time_ms();
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
			combine_msg_all.roll = axis[0]>>7; 
			combine_msg_all.pitch = axis[1]>>7;
			combine_msg_all.yaw = axis[2]>>6; // make it more sensitive than the others 
			
			// scale it down from U16 to U12 (4096) we might need to compress it a little bit more
			combine_msg_all.thrust = (JOY_THRUST_OFF - axis[3])>>4;
			if(combine_msg_all.thrust > 30) combine_msg_all.thrust = (((combine_msg_all.thrust)*11)>>4)+1280;
			// assign the fire button
			if(button[0]) 
			{
				if((combine_msg_all.mode != MODE_SAFE) && (combine_msg_all.mode != MODE_PANIC)) combine_msg_all.mode = MODE_PANIC;
				else if (combine_msg_all.mode == MODE_SAFE) combine_msg_all.mode = ESCAPE;
			}	
			combine_msg_all.update = true;
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
			KeyboardCommand(c, &combine_msg_all);
			combine_msg_all.update = true;
		}
		#ifdef PC_PROFILE
			end_profile = mon_time_us();
			proc_key = end_profile - start_profile;
			//printf("k %d ", proc_key);
		#endif
		CombineCommand(&combine_msg_all);
		// combine keyboard and joystick
		#ifdef PC_PROFILE 
			start_profile = mon_time_us();
		#endif



		#ifdef PC_PROFILE
			end_profile = mon_time_us();
			proc_comb = end_profile - start_profile;
			//printf("c %d ", proc_comb);
		#endif

		// read the uart data from drone
		#ifdef PC_PROFILE 
			start_profile = mon_time_us();
		#endif
		
		if (read(fd_RS232, &c, 1))
		{
			#ifdef PC_PROFILE
				end_profile = mon_time_us();
				proc_read = end_profile - start_profile;	
			#endif
				
			#ifdef ENCODE_PC_RECEIVE
			// msg_parse(&msg, (uint8_t) c)
			msg_parse(&msg, (uint8_t) c);
				#ifdef ENCODE_DEBUG
					if (msg.status == UNITINIT) printf("UNITINIT\n");
					if (msg.status == GOT_HDR) printf("GOT_HDR\n");
					if (msg.status == GOT_LEN) printf("GOT_LEN\n");
					if (msg.status == GOT_ID) printf("GOT_ID\n");
					if (msg.status == GOT_PAYLOAD) printf("GOT_PAYLOAD\n");
					if (msg.status == GOT_PACKET) printf("GOT_PACKET\n");
				#endif
			if(msg.status == GOT_PACKET) { 
				// We got a valid packet
				switch(msg.msg_id) {
					case MSG_TELEMETRY: 
					{
						msg_tele = (struct msg_telemetry_t *)&msg.payload[0];
						printf("%d %d %d %d %d %d| ", msg.crc_fails, msg_tele->mode, msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
						printf("%d %d %d %d| ", msg_tele->engine[0],msg_tele->engine[1],msg_tele->engine[2],msg_tele->engine[3]);
						printf("%d %d %d| ",msg_tele->phi, msg_tele->theta, msg_tele->psi);
						printf("%d %d %d| ",msg_tele->sp, msg_tele->sq, msg_tele->sr);
						printf("%d %d %d| ",msg_tele->sax, msg_tele->say, msg_tele->saz);
						printf("%d %d %d %d\n ",msg_tele->bat_volt, msg_tele->P, msg_tele->P1, msg_tele->P2);
						#ifdef PC_PROFILE
							printf("s:%d j:%d k:%d c:%d r:%d\n ",proc_send, proc_joy, proc_key, proc_comb, proc_read);
						#endif

						if((msg_tele->bat_volt<1070) && ((mon_time_ms() - start_batt) > 1000)) 
						{
							// printf("\n == The BATTERY is LOW == \n \n");
							// if(msg_tele->bat_volt<1050){combine_msg.mode = MODE_PANIC;}
							start_batt = mon_time_ms();
						}	
						break;
					}

					#ifdef DRONE_PROFILE
					case MSG_PROFILE: 
					{
						msg_profile = (struct msg_profile_t *)&msg.payload[0];
						msg_profile_np = *msg_profile;
						printf("\n%d %d %d %d %d %d %d\n", msg_profile_np.proc_read, msg_profile_np.proc_adc, msg_profile_np.proc_send, msg_profile_np.proc_log, msg_profile_np.proc_dmp, msg_profile_np.proc_control,  msg_profile_np.time_all);
						break;
					}
					#endif

					case MSG_LOG:
					{
						msg_logging = (struct msg_log_t *)&msg.payload[0];
						kp = fopen("logging.csv","w+");
						fprintf(kp,"index_log, time_stamp, mode, thrust, roll, pitch, yaw, ae[0], ae[1], ae[2], ae[3], phi, theta, psi, sp, sq, sr, sax, say, saz, bat_volt, P, P1, P2, temperature, pressure\n");
						printf("%d %d | %d | %d %d %d %d | ", msg_logging->index_log, msg_logging->time_stamp, msg_logging->mode, msg_logging->thrust, msg_logging->roll, msg_logging->pitch, msg_logging->yaw);
						fprintf(kp, "%d, %d, %d, %d, %d, %d, %d, ", msg_logging->index_log, msg_logging->time_stamp, msg_logging->mode, msg_logging->thrust, msg_logging->roll, msg_logging->pitch, msg_logging->yaw);
						printf("%d %d %d %d | ", msg_logging->ae[0], msg_logging->ae[1], msg_logging->ae[2], msg_logging->ae[3]);
						fprintf(kp, "%d, %d, %d, %d, ", msg_logging->ae[0], msg_logging->ae[1], msg_logging->ae[2], msg_logging->ae[3]);
						printf("%d %d %d | ", msg_logging->phi, msg_logging->theta, msg_logging->psi);
						fprintf(kp,"%d, %d, %d, ", msg_logging->phi, msg_logging->theta, msg_logging->psi);
						printf("%d %d %d | ", msg_logging->sp, msg_logging->sq, msg_logging->sr);
						fprintf(kp,"%d, %d, %d, ", msg_logging->sp, msg_logging->sq, msg_logging->sr); 
						printf("%d %d %d | ", msg_logging->sax, msg_logging->say, msg_logging->saz);
						fprintf(kp,"%d, %d, %d, ", msg_logging->sax, msg_logging->say, msg_logging->saz); 
						printf("%d %d %d %d |%d %d\n", msg_logging->bat_volt, msg_logging->P, msg_logging->P1, msg_logging->P2, msg_logging->temperature, msg_logging->pressure);
						fprintf(kp,"%d, %d, %d, %d, %d, %d\n", msg_logging->bat_volt, msg_logging->P, msg_logging->P1, msg_logging->P2, msg_logging->temperature, msg_logging->pressure);
						combine_msg_all.mode = MODE_LOG;	// change the mode to mode log, so we do not need to send message anymore	
						break;
					}

					case MSG_ACK:
					{
						if (msg.payload[0] == ACK_FIRED)
						{
							//separated with ESCAPE, because ESCAPE is to acknowledge board.
							//This way the program will exit safely without writing log if there is no og to be written
							printf("exit\n");
							combine_msg_all.mode = MODE_FINISH; 
						}
						else if(msg.payload[0] == ACK_RAW_INIT)
						{
							printf("enter raw init\n");
						}
						else if (msg.payload[0] == ACK_BAT_LOW)
						{
							printf("Battery Low\n");
						}
						else if(msg.payload[0]==ACK_BAT_LOW_EMERGENCY)
						{
							printf("Battery low, uplink connection will be disconnected now!\n");
							stop_sending = true;
						}
						
						break; 
					}

					default:
						break;
				};
				msg.status = UNITINIT;	// Start to receive a new packet
				msg.crc_fails = 0;		// reset the crc fail number
			}
			
			#else
				#ifndef PC_PROFILE
					
				#endif // PC_PROFILE
			#endif		
		}
		if(combine_msg_all.mode == MODE_FINISH)
			break;	
	}
	/****************************************************************************************
	* End of Mission phase
	*****************************************************************************************/

	/****************************************************************************************
	* Writing log phase
	*****************************************************************************************/
	
	while((combine_msg_all.mode == MODE_LOG) && (combine_msg_all.mode != MODE_FINISH))// start logging 
	//while(true)
	{
		if (read(fd_RS232, &c, 1)){ 				// if ((c = rs232_getchar_nb()) != -1){		
			#ifdef ENCODE_PC_RECEIVE				//#ifdef ENCODE
				msg_parse(&msg, (uint8_t)c);
				if(msg.status == GOT_PACKET) { 		// We got a valid packet
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
							fprintf(kp, "%d, %d, %d, %d, %d, %d, %d, ", msg_logging->index_log, msg_logging->time_stamp, msg_logging->mode, msg_logging->thrust, msg_logging->roll, msg_logging->pitch, msg_logging->yaw);
							printf("%d %d %d %d | ", msg_logging->ae[0], msg_logging->ae[1], msg_logging->ae[2], msg_logging->ae[3]);
							fprintf(kp, "%d, %d, %d, %d, ", msg_logging->ae[0], msg_logging->ae[1], msg_logging->ae[2], msg_logging->ae[3]);
							printf("%d %d %d | ", msg_logging->phi, msg_logging->theta, msg_logging->psi);
							fprintf(kp,"%d, %d, %d, ", msg_logging->phi, msg_logging->theta, msg_logging->psi);
							printf("%d %d %d | ", msg_logging->sp, msg_logging->sq, msg_logging->sr);
							fprintf(kp,"%d, %d, %d, ", msg_logging->sp, msg_logging->sq, msg_logging->sr); 
							printf("%d %d %d | ", msg_logging->sax, msg_logging->say, msg_logging->saz);
							fprintf(kp,"%d, %d, %d, ", msg_logging->sax, msg_logging->say, msg_logging->saz); 
							printf("%d %d %d %d |%d %d\n", msg_logging->bat_volt, msg_logging->P, msg_logging->P1, msg_logging->P2, msg_logging->temperature, msg_logging->pressure);
							fprintf(kp,"%d, %d, %d, %d, %d, %d\n", msg_logging->bat_volt, msg_logging->P, msg_logging->P1, msg_logging->P2, msg_logging->temperature, msg_logging->pressure);
							break;
						}

						case MSG_ACK:
						{
							if (msg.payload[0] == ACK_RCV)
							{
								//finish log, file closed
								printf("Finished\n");
								fclose(kp);
								combine_msg_all.mode = MODE_FINISH;
							}
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
		//if(combine_msg_all.mode == MODE_FINISH)
		//	break;
	}
	

	/****************************************************************************************
	* End of Writing log phase
	*****************************************************************************************/
	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");
	
	return 0;
}




