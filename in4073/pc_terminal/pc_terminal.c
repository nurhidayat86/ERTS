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

#include "../logging_protocol.h"
#include "log_terminal.h"

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

int main(int argc, char **argv)
{
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
	uint32_t counter = 0;	

	#ifdef ENCODE_PC_RECEIVE
	struct msg_p msg;
	struct msg_telemetry_t *msg_tele;
	#endif

	// message struck
	struct msg_joystick_t joystick_msg;
	struct msg_keyboard_t keyboard_msg;
	struct msg_combine_t combine_msg;
	struct msg_tuning_t tuning_msg;
	
	// initialize the message by zeroing all value
	InitCommand(&combine_msg, &keyboard_msg, &joystick_msg, &tuning_msg);

	// logging variable
	FILE *kp;
	uint8_t decode_status;
 	static struct msg_p_log msg_log_p;
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
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);

	// check joystick command when start up the system
	while((!joystick_msg.update) || (joystick_msg.thrust != 0) || joystick_msg.roll != 0 || joystick_msg.pitch != 0 || joystick_msg.yaw != 0)
	{
		while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
			button[js.number] = js.value;
			axis[js.number] = js.value;
		
			// scale it down to 9 bit signed integer (-255 to 255), make it less sensitive
			joystick_msg.roll = axis[0]>>7; 
			joystick_msg.pitch = axis[1]>>7;
			joystick_msg.yaw = axis[2]>>7;
			
			// scale it down from U16 to U12 (4096) we might need to compress it a little bit more
			joystick_msg.thrust = (JOY_THRUST_OFF - axis[3])>>4;
			
			// assign the fire button
			// if(button[0]) joystick_msg.mode = MODE_SAFE;
			joystick_msg.update = TRUE;
			warning = TRUE;	
		}

		if(warning) 
		{
			printf("WARNING! thrust %d roll %d pitch %d yaw %d \n", joystick_msg.thrust, joystick_msg.roll, joystick_msg.pitch, joystick_msg.yaw);
			warning = FALSE;	
		}

	}

	/* send & receive
	 */	
	while(combine_msg.mode != MODE_LOG)
	{
		#ifdef PC_PROFILE 
			start_profile = mon_time_ms();
		#endif
		// peridocally send the command to the board
		// check panic time as well, do not send anything if we are in the panic time
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
			}
			// check if panic_mode happened
			if(combine_msg.mode == MODE_PANIC) 
			{
				// start panic start, reset the mode to the panic mode
				panic_start = mon_time_ms();
				keyboard_msg.mode = joystick_msg.mode = combine_msg.mode = MODE_SAFE;
				combine_msg.thrust = keyboard_msg.thrust = 0;	
				combine_msg.roll = keyboard_msg.roll = 0;
				combine_msg.pitch = keyboard_msg.pitch = 0;
				combine_msg.yaw = keyboard_msg.yaw = 0;
			}
			start = mon_time_ms();

			if(combine_msg.mode == ESCAPE) combine_msg.mode = MODE_LOG;	
		}
		#ifdef PC_PROFILE
			end_profile = mon_time_ms();
			proc_send = end_profile - start_profile;
			printf("s %d ", proc_send);
		#endif

		// JoystickCommand(fd, js, &joystick_msg);
		// Read the joystick event
		#ifdef PC_PROFILE 
			start_profile = mon_time_ms();
		#endif
		while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){
			button[js.number] = js.value;
			axis[js.number] = js.value;
		
			// scale it down to 9 bit signed integer (-255 to 255)
			joystick_msg.roll = axis[0]>>7; 
			joystick_msg.pitch = axis[1]>>7;
			joystick_msg.yaw = axis[2]>>7;
			
			// scale it down from U16 to U12 (4096) we might need to compress it a little bit more
			joystick_msg.thrust = (JOY_THRUST_OFF - axis[3])>>4;
			
			// assign the fire button
			// if(button[0]) joystick_msg.mode = MODE_SAFE;
			joystick_msg.update = TRUE;
			// printf("update joystick \n");				
		}
		#ifdef PC_PROFILE
			end_profile = mon_time_ms();
			proc_joy = end_profile - start_profile;
			printf("j %d ", proc_joy);
		#endif

		#ifdef PC_PROFILE 
			start_profile = mon_time_ms();
		#endif
		// read the keyboard
		// it also is used to update the tuning gain
		if ((c = term_getchar_nb()) != -1){
			KeyboardCommand(c, &keyboard_msg, &tuning_msg, &joystick_msg);
			keyboard_msg.update = TRUE;
		}
		#ifdef PC_PROFILE
			end_profile = mon_time_ms();
			proc_key = end_profile - start_profile;
			printf("k %d ", proc_key);
		#endif

		#ifdef PC_PROFILE 
			start_profile = mon_time_ms();
		#endif
		// combine keyboard and joystick
		combine_msg.update = (keyboard_msg.update || joystick_msg.update);
		if(combine_msg.update){
			CombineCommand(&combine_msg, &keyboard_msg, &joystick_msg);
			// printf("update combine \n");
		}
		#ifdef PC_PROFILE
			end_profile = mon_time_ms();
			proc_comb = end_profile - start_profile;
			printf("c %d ", proc_comb);
		#endif

		#ifdef PC_PROFILE 
			start_profile = mon_time_ms();
		#endif
		// receive data from board
		if ((c = rs232_getchar_nb()) != -1){			
			// print the message sent by the board to the terminal
			// printf("get message from board \n");
			// if ((uint8_t)c == 0x99) printf("\n");
			//if(counter++%14==0) printf("\n");
			// printf("0x%X ", (uint8_t)c);
			// c = (uint8_t)c;
			// printf("0x%X 	", (char) c);
			#ifdef ENCODE_PC_RECEIVE
				msg_parse(&msg, c);
				if(msg.status == GOT_PACKET) {
					// We got a valid packet
					printf("got packet ");
					switch(msg.msg_id) {
						case MSG_TELEMETRY: 
						{
							msg_tele = (struct msg_telemetry_t *)&msg.payload[0];
							printf("%d %d %d %d %d %d \n", msg.crc_fails, msg_tele->mode, msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
							// printf("%d %d \n", msg.crc_fails, msg_tele->mode);
							// printf("%d %d %d %d \n", msg_tele->engine[0],msg_tele->engine[1],msg_tele->engine[2],msg_tele->engine[3]);
							// printf("%d %d %d |",phi,ae[1],ae[2],ae[3]);
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
				#ifndef PC_PROFILE
					printf("%c", (uint8_t)c);
				#endif // PC_PROFILE
			#endif		
		}
		#ifdef PC_PROFILE
			end_profile = mon_time_ms();
			proc_read = end_profile - start_profile;	
			printf("r %d\n", proc_read);
		#endif
	}

	while(true)
	{
		if ((c = rs232_getchar_nb()) != -1){				
			//#ifdef ENCODE
			#ifdef ENCODE_PC_RECEIVE
				decode_status = decode_log((uint8_t) c, &msg_log_p);
				if(msg_log_p.status == GOT_PACKAGE)
				{
					switch(msg_log_p.msg_ID){
						case INDEX_LOG:
							MSG_index_log = (uint16_t *)&msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_index_log = *MSG_index_log;
							//printf(" %d||", *MSG_index_log);
							break;

						case T_STAMP:
							MSG_time_stamp = (uint32_t *)&msg_log_p.payload[0];
							// MSG_time_stamp = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_time_stamp = *MSG_time_stamp;
							//printf(" %d||", *MSG_time_stamp);
							break;

						case MODE:
							MSG_mode = (uint8_t *)&msg_log_p.payload[0];
							// MSG_mode = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_mode = *MSG_mode;
							//printf(" %d||", *MSG_mode);
							break;

						case THRUST:
							MSG_thrust = (uint16_t *)&msg_log_p.payload[0];
							// MSG_thrust = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_thrust = *MSG_thrust;
							//printf(" %d||", *MSG_thrust);
					 		break;

						case ROLL:
							MSG_roll = (int16_t *)&msg_log_p.payload[0];
							// MSG_roll = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_roll = *MSG_roll;
							//printf(" %d||", *MSG_roll);
							break;

						case PITCH:
							MSG_pitch = (int16_t *)&msg_log_p.payload[0];
							// MSG_pitch = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_pitch = *MSG_pitch;
							//printf(" %d||", *MSG_pitch);
							break;

						case YAW:
							MSG_yaw = (int16_t *)&msg_log_p.payload[0];
							// MSG_yaw = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_yaw = *MSG_yaw;
							//printf(" %d||", *MSG_yaw);
							break;

						case AE_0:
							MSG_ae_0 = (int16_t *)&msg_log_p.payload[0];
							// MSG_ae_0 = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_ae_0 = *MSG_ae_0;
							//printf(" %d||", *MSG_ae_0);
							break;

						case AE_1:
							MSG_ae_1 = (int16_t *)&msg_log_p.payload[0];
							// MSG_ae_1 = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_ae_1 = *MSG_ae_1;
							//printf(" %d||", *MSG_ae_1);
							break;

						case AE_2:
							MSG_ae_2 = (int16_t *)&msg_log_p.payload[0];
							// MSG_ae_2 = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_ae_2 = *MSG_ae_2;
							//printf(" %d||", *MSG_ae_2);
							break;

						case AE_3:
							MSG_ae_3 = (int16_t *)&msg_log_p.payload[0];
							// MSG_ae_3 = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_ae_3 = *MSG_ae_3;
							//printf(" %d||", *MSG_ae_3);
							break;

						case PHI:
							MSG_phi = (int16_t *)&msg_log_p.payload[0];
							// MSG_phi = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_phi = *MSG_phi;
							//printf(" %d||", *MSG_phi);
							break;

						case THETA:
							MSG_theta = (int16_t *)&msg_log_p.payload[0];
							// MSG_theta = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_theta = *MSG_theta;
							//printf(" %d||", *MSG_theta);
							break;

						case PSI:
							MSG_psi = (int16_t *)&msg_log_p.payload[0];
							// MSG_psi = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_psi = *MSG_psi;
							//printf(" %d||", *MSG_psi);
							break;

						case SP:
							MSG_sp = (int16_t *)&msg_log_p.payload[0];
							// MSG_sp = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_sp = *MSG_sp;
							//printf(" %d||", *MSG_sp);
							break;

						case SQ:
							MSG_sq = (int16_t *)&msg_log_p.payload[0];
							// MSG_sq = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_sq = *MSG_sq;
							//printf(" %d||", *MSG_sq);
							break;

						case SR:
							MSG_sr = (int16_t *)&msg_log_p.payload[0];
							// MSG_sr = &msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
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
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_bat_volt = *MSG_bat_volt;
							//printf(" %d||\n", *MSG_bat_volt);
							break;

						case TEMP:
							MSG_temperature = (uint32_t *)&msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
							np_MSG_temperature = *MSG_temperature;
							//printf(" %d||\n", *MSG_temperature);
							break;

						case PRESS:
							MSG_pressure = (uint32_t *)&msg_log_p.payload[0];
							//term_putchar(msg_log_p.payload[0]+60);
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
								printf("complete");
							}
							flash_np();
							break;

						default:
							break;

					}
					msg_log_p.status = UNITINIT;
				}	
			#else
				// if(log_start)
				// {
				// 	kp = fopen("logging.csv","w+");
				// 	log_start = FALSE;
				// }

				printf("%c", (uint8_t)c);
				// fprintf(kp, "%c", (uint8_t)c);
				
				// if((uint8_t)c == 0x0A) fclose(kp);

			#endif			
		}
	}
	
	term_puts("\n<exit>\n");
	term_exitio();
	rs232_close();

	return 0;
}



