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

int main(int argc, char **argv)
{
	/****************************************************************************************
	* Variable initiator
	*****************************************************************************************/
	// periodic command timer 
	uint32_t start, end, panic_start = 0; 
	bool resend_flag = false;

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
		struct msg_profile_t msg_profile_np;
		#endif
	#endif

	// message struck
	struct msg_combine_all_t combine_msg_all;
	struct msg_joystick_t joystick_msg;
	struct msg_keyboard_t keyboard_msg;

	// initialize the message by zeroing all value
	// InitCommandUpdate(&combine_msg_all);
	InitCommandAll(&joystick_msg, &keyboard_msg, &combine_msg_all);
	// CommandModeSafe(&combine_msg_all);
	CommandModeSafeAll(&joystick_msg, &keyboard_msg, &combine_msg_all);

	// logging variable
	FILE *kp;
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
			combine_msg_all.roll = axis[0]>>8; 
			combine_msg_all.pitch = axis[1]>>8;
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

 	//raw var start
 	initraw_stat();
 	
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
		
		if((((end-start) > PERIODIC_COM) && (combine_msg_all.mode != MODE_LOG))) //&& (!stop_sending)
		{
			
			// print to terminal to indicate the panic button was succesfully pressed
			if(combine_msg_all.mode == MODE_PANIC) printf("\nPANIC MODE from PC\n");
			
			// stop sending a command if the we are still in panic period
			if(!stop_sending) SendCommandAll(&combine_msg_all);
			// check the panic time, to reset the stop flag so we can reestablish the communication
			else if((mon_time_ms() - panic_start) > PANIC_TIME_MS)
			{
				stop_sending = false;
			}

			// check if panic_mode happened and the mode of drone also already in panic mode
			// it is indicated by resend flag
			// if we dont need to resend the flag, reset the mode to safe mode
			#ifdef ENCODE_PC_RECEIVE
			if( (combine_msg_all.mode == MODE_PANIC && (!resend_flag) ) || (msg.crc_fails > 4)) 
			#else
			if(combine_msg_all.mode == MODE_PANIC) 
			#endif
			{
				// start panic start, reset the mode to the safe mode
				panic_start = mon_time_ms();
				// CommandModeSafe(&combine_msg_all);
				CommandModeSafeAll(&joystick_msg, &keyboard_msg, &combine_msg_all); 
				// stop sending the command
				stop_sending = true;
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
			joystick_msg.roll = axis[0]>>8; 
			joystick_msg.pitch = axis[1]>>8;
			joystick_msg.yaw = axis[2]>>6; // make it more sensitive than the others 
			
			// scale it down from U16 to U12 (4096) we might need to compress it a little bit more
			joystick_msg.thrust = (JOY_THRUST_OFF - axis[3])>>4;
			if(joystick_msg.thrust > 30) joystick_msg.thrust = (((joystick_msg.thrust)*11)>>4)+1280;
			// assign the fire button
			if(button[0]) 
			{
				if((joystick_msg.mode != MODE_SAFE) && (joystick_msg.mode != MODE_PANIC)) joystick_msg.mode = MODE_PANIC;
				else if (joystick_msg.mode == MODE_SAFE) joystick_msg.mode = ESCAPE;
			}	
			joystick_msg.update = true;
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
			KeyboardCommandSplit(c, &joystick_msg, &keyboard_msg);
			keyboard_msg.update = true;
		}
		#ifdef PC_PROFILE
			end_profile = mon_time_us();
			proc_key = end_profile - start_profile;
			//printf("k %d ", proc_key);
		#endif
		
		#ifdef PC_PROFILE 
			start_profile = mon_time_us();
		#endif

		// combine keyboard and joystick
		combine_msg_all.update = (joystick_msg.update || keyboard_msg.update); 
		if(combine_msg_all.update)
		{
			CombineCommandAll(&joystick_msg, &keyboard_msg, &combine_msg_all);
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
		
		if (read(fd_RS232, &c, 1))
		{	
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
					
					#ifndef DRONE_PROFILE
					case MSG_TELEMETRY: 
					{
						msg_tele = (struct msg_telemetry_t *)&msg.payload[0];
						// printf("%d %d %d %d %d %d| ", msg.crc_fails, msg_tele->mode, msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
						// printf("%d %d %d %d| ", msg_tele->engine[0],msg_tele->engine[1],msg_tele->engine[2],msg_tele->engine[3]);
						// printf("%d %d %d| ",msg_tele->phi, msg_tele->theta, msg_tele->psi);
						// printf("%d %d %d| ",msg_tele->sp, msg_tele->sq, msg_tele->sr);
						// printf("%d %d %d| ",msg_tele->sax, msg_tele->say, msg_tele->saz);
						// printf("%d %d %d %d\n ",msg_tele->bat_volt, msg_tele->P, msg_tele->P1, msg_tele->P2);
						
						printf("\r%d %4d %4d %4d %4d| ", msg_tele->mode, msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
						printf("%4d %4d %4d %4d| ", msg_tele->engine[0],msg_tele->engine[1],msg_tele->engine[2],msg_tele->engine[3]);
						printf("%6d %6d %6d| ",msg_tele->phi, msg_tele->theta, msg_tele->psi);
						printf("%6d %6d %6d| ",msg_tele->sp, msg_tele->sq, msg_tele->sr);
						printf("%6d %6d %6d| ",msg_tele->sax, msg_tele->say, msg_tele->saz);
						printf("%4d %2d %2d %2d\n ",msg_tele->bat_volt, msg_tele->P, msg_tele->P1, msg_tele->P2);
						
						// check drone mode, resend if it is still not in the panic mode
						if(combine_msg_all.mode == MODE_PANIC && msg_tele->mode != MODE_PANIC) resend_flag = true;
						// neither on the panic mode or the panic mode succesfully sent anc accepted by the drone
						else resend_flag = false;
						// #ifdef PC_PROFILE
						// 	printf("s:%d j:%d k:%d c:%d r:%d\n ",proc_send, proc_joy, proc_key, proc_comb, proc_read);
						// #endif
						break;
					}
					#endif

					#ifdef DRONE_PROFILE
					case MSG_PROFILE: 
					{
						msg_profile = (struct msg_profile_t *)&msg.payload[0];
						msg_profile_np = *msg_profile;
						printf("%d %d %d %d %d %d %d\n", msg_profile_np.proc_read, msg_profile_np.proc_adc, msg_profile_np.proc_send, msg_profile_np.proc_log, msg_profile_np.proc_dmp, msg_profile_np.proc_control,  msg_profile_np.time_all);
						break;
					}
					#endif

					case MSG_LOG:
					{
						msg_logging = (struct msg_log_t *)&msg.payload[0];
						kp = fopen("logging.csv","w+");
						fprintf(kp,"index_log, time_stamp, mode, thrust, roll, pitch, yaw, ae[0], ae[1], ae[2], ae[3], phi, theta, psi, sp, sq, sr, esp, esq, esr, sax, say, saz, bat_volt, P, P1, P2, temperature, pressure\n");
						printf("%4d %9d | %d | %4d %4d %4d %4d | ", msg_logging->index_log, msg_logging->time_stamp, msg_logging->mode, msg_logging->thrust, msg_logging->roll, msg_logging->pitch, msg_logging->yaw);
						fprintf(kp, "%d, %d, %d, %d, %d, %d, %d, ", msg_logging->index_log, msg_logging->time_stamp, msg_logging->mode, msg_logging->thrust, msg_logging->roll, msg_logging->pitch, msg_logging->yaw);
						printf("%4d %4d %4d %4d | ", msg_logging->ae[0], msg_logging->ae[1], msg_logging->ae[2], msg_logging->ae[3]);
						fprintf(kp, "%d, %d, %d, %d, ", msg_logging->ae[0], msg_logging->ae[1], msg_logging->ae[2], msg_logging->ae[3]);
						printf("%6d %6d %6d | ", msg_logging->phi, msg_logging->theta, msg_logging->psi);
						fprintf(kp,"%d, %d, %d, ", msg_logging->phi, msg_logging->theta, msg_logging->psi);
						
						printf("%6d %6d %6d | ", msg_logging->sp, msg_logging->sq, msg_logging->sr);
						fprintf(kp,"%d, %d, %d, ", msg_logging->sp, msg_logging->sq, msg_logging->sr); 
						printf("%6d %6d %6d | ", msg_logging->esp, msg_logging->esq, msg_logging->esr);
						fprintf(kp,"%d, %d, %d, ", msg_logging->esp, msg_logging->esq, msg_logging->esr); 
						
						printf("%6d %6d %6d | ", msg_logging->sax, msg_logging->say, msg_logging->saz);
						fprintf(kp,"%d, %d, %d, ", msg_logging->sax, msg_logging->say, msg_logging->saz); 
						printf("%4d %2d %2d %2d |%4d %4d\n", msg_logging->bat_volt, msg_logging->P, msg_logging->P1, msg_logging->P2, msg_logging->temperature, msg_logging->pressure);
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
							printf("warning battery low\n");
						}
						else if(msg.payload[0]==ACK_BAT_LOW_EMERGENCY)
						{
							printf("Battery low, uplink connection will be disconnected now!\n");
							// stop_sending = true;
							combine_msg_all.mode = MODE_PANIC;
						}
						else if (msg.payload[0] == ACK_BAT_LOW_EMER_SAFE)
						{
							printf("Battery low, change the battery now!\n");
						}
						else if(msg.payload[0]==ACK_CON)
						{
							printf("fifo error\n");
						}

						else if(msg.payload[0]==ACK_FLASH)
						{
							printf("flash is full\n");
						}
						else if(msg.payload[0]==ACK_LOST_COM)
						{
							printf("communication is lost\n");
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

		#ifdef PC_PROFILE
			end_profile = mon_time_us();
			proc_read = end_profile - start_profile;	
		#endif

		if(combine_msg_all.mode == MODE_FINISH)
			break;

		#ifdef PC_PROFILE
			printf("per: %d s:%d j:%d k:%d c:%d r:%d\n ",periodic ,proc_send, proc_joy, proc_key, proc_comb, proc_read);
		#endif	
	}
	/****************************************************************************************
	* End of Mission phase
	*****************************************************************************************/

	/****************************************************************************************
	* Writing log phase
	*****************************************************************************************/
	
	while((combine_msg_all.mode == MODE_LOG) && (combine_msg_all.mode != MODE_FINISH))// start logging 
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
							// printf("%d %d | %d | %d %d %d %d | ", msg_logging->index_log, msg_logging->time_stamp, msg_logging->mode, msg_logging->thrust, msg_logging->roll, msg_logging->pitch, msg_logging->yaw);
							// fprintf(kp, "%d, %d, %d, %d, %d, %d, %d, ", msg_logging->index_log, msg_logging->time_stamp, msg_logging->mode, msg_logging->thrust, msg_logging->roll, msg_logging->pitch, msg_logging->yaw);
							// printf("%d %d %d %d | ", msg_logging->ae[0], msg_logging->ae[1], msg_logging->ae[2], msg_logging->ae[3]);
							// fprintf(kp, "%d, %d, %d, %d, ", msg_logging->ae[0], msg_logging->ae[1], msg_logging->ae[2], msg_logging->ae[3]);
							// printf("%d %d %d | ", msg_logging->phi, msg_logging->theta, msg_logging->psi);
							// fprintf(kp,"%d, %d, %d, ", msg_logging->phi, msg_logging->theta, msg_logging->psi);
							// printf("%d %d %d | ", msg_logging->sp, msg_logging->sq, msg_logging->sr);
							// fprintf(kp,"%d, %d, %d, ", msg_logging->sp, msg_logging->sq, msg_logging->sr); 
							// printf("%d %d %d | ", msg_logging->sax, msg_logging->say, msg_logging->saz);
							// fprintf(kp,"%d, %d, %d, ", msg_logging->sax, msg_logging->say, msg_logging->saz); 
							// printf("%d %d %d %d |%d %d\n", msg_logging->bat_volt, msg_logging->P, msg_logging->P1, msg_logging->P2, msg_logging->temperature, msg_logging->pressure);
							// fprintf(kp,"%d, %d, %d, %d, %d, %d\n", msg_logging->bat_volt, msg_logging->P, msg_logging->P1, msg_logging->P2, msg_logging->temperature, msg_logging->pressure);
							printf("%4d %9d | %d | %4d %4d %4d %4d | ", msg_logging->index_log, msg_logging->time_stamp, msg_logging->mode, msg_logging->thrust, msg_logging->roll, msg_logging->pitch, msg_logging->yaw);
							fprintf(kp, "%d, %d, %d, %d, %d, %d, %d, ", msg_logging->index_log, msg_logging->time_stamp, msg_logging->mode, msg_logging->thrust, msg_logging->roll, msg_logging->pitch, msg_logging->yaw);
							printf("%4d %4d %4d %4d | ", msg_logging->ae[0], msg_logging->ae[1], msg_logging->ae[2], msg_logging->ae[3]);
							fprintf(kp, "%d, %d, %d, %d, ", msg_logging->ae[0], msg_logging->ae[1], msg_logging->ae[2], msg_logging->ae[3]);
							printf("%6d %6d %6d | ", msg_logging->phi, msg_logging->theta, msg_logging->psi);
							fprintf(kp,"%d, %d, %d, ", msg_logging->phi, msg_logging->theta, msg_logging->psi);
							printf("%6d %6d %6d | ", msg_logging->sp, msg_logging->sq, msg_logging->sr);
							fprintf(kp,"%d, %d, %d, ", msg_logging->sp, msg_logging->sq, msg_logging->sr); 
							
							printf("%6d %6d %6d | ", msg_logging->esp, msg_logging->esq, msg_logging->esr);
							fprintf(kp,"%d, %d, %d, ", msg_logging->esp, msg_logging->esq, msg_logging->esr); 
							
							printf("%6d %6d %6d | ", msg_logging->sax, msg_logging->say, msg_logging->saz);
							fprintf(kp,"%d, %d, %d, ", msg_logging->sax, msg_logging->say, msg_logging->saz); 
							printf("%4d %2d %2d %2d |%4d %4d\n", msg_logging->bat_volt, msg_logging->P, msg_logging->P1, msg_logging->P2, msg_logging->temperature, msg_logging->pressure);
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




