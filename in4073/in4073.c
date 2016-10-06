/*------------------------------------------------------------------
 *  in4073.c -- test QR engines and sensors
 *
 *  reads ae[0-3] uart rx queue
 *  (q,w,e,r increment, a,s,d,f decrement)
 *
 *  prints timestamp, ae[0-3], sensors to uart tx queue
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  June 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include "logging.h"
#include "logging_protocol.h"

bool loop;

static void process_bytes(uint8_t byte) {
	
	static struct msg_p msg;
	struct msg_combine_t *msg_com;	
	struct msg_tuning_t *msg_tune;	
	// static bool disable_mode = FALSE;
	// static struct msg_combine_t msg_com_val;
	// static struct msg_tuning_t msg_tune_val;	
	// static uint8_t mmode;
	// static uint16_t mthrust;
	// static int16_t mroll;
	// static int16_t mpitch;
	// static int16_t myaw;
	
	msg_parse(&msg, byte);
	if(msg.status == GOT_PACKET) {
		nrf_gpio_pin_toggle(RED);
		// We got a valid packet
		switch(msg.msg_id) {
			case MSG_COMBINE: 
			{
				msg_com = (struct msg_combine_t *)&msg.payload[0];
				mmode = msg_com->mode;
				mthrust = msg_com->thrust;
				mroll = msg_com->roll;
				mpitch = msg_com->pitch;
				myaw = msg_com->yaw;
				
				msg_tele = (struct msg_telemetry_t *)&msg.payload[0];
				msg_tele->engine[0] = ae[0];
				msg_tele->engine[1] = ae[1];
				msg_tele->engine[2] = ae[2];
				msg_tele->engine[3] = ae[3];
				// msg_tele->update = TRUE;
				msg_tele->phi = phi;
				msg_tele->theta = theta;
				msg_tele->psi = -psi;

				msg_tele->sp = sp-cp;
				msg_tele->sq = -(sq-cq); 
				msg_tele->sr = -(sr-cr);

				msg_tele->bat_volt = bat_volt;

				if(mmode == ESCAPE)
				{loop = false;}
				// disable_mode = FALSE;
				break;
			}

			case MSG_TUNE: 
			{
				msg_tune = (struct msg_tuning_t *)&msg.payload[0];
				P = msg_tune->P;
				// disable_mode = TRUE;
				break;
			}

			default:
				break;

		};
		// Start to receive a new packet
		msg.status = UNITINIT;
	}
	// receive the mode
	// put semaphore 
	// if(!disable_mode) set_control_mode(msg_com->mode);
	set_control_mode(mmode);
	
	// set control command 
	// set_control_command(msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
	// set_control_command(msg_com->thrust, msg_com->roll, msg_com->pitch, msg_com->yaw);
	set_control_command(mthrust, mroll, mpitch, myaw);

	// set yaw gain
	// set_control_gains(msg_tune->P); //msg_tune->P		
	set_control_gains(P);
}

/*------------------------------------------------------------------
 * main -- do the test
 *------------------------------------------------------------------
 */
int main(void)
{
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);
	baro_init();
	spi_flash_init();
	uint32_t start_all = 0, end_all = 0, time_all = 0;
	//int8_t dmp_status =0;
//	ble_init();
	
	#ifdef ENCODE_PC_RECEIVE
	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;
	uint8_t j,i = 0;
	#endif

    // command_init();
	// msg_tele->update = FALSE;
	msg_tele->mode = 0;
	msg_tele->thrust = 0;
	msg_tele->roll = 0;
 	msg_tele->pitch = 0;
 	msg_tele->yaw = 0;
					
 	cphi = ctheta = cpsi = 0;        ///< Calibration values of phi, theta, psi
	cp = cq = cr = 0;                ///< Calibration valies of p, q and r

	bool status = true;
	uint32_t counter = 0;
	uint32_t counter_log = 0;
	//uint32_t counter_link = 0;
	
	// profiling
	#ifdef DRONE_PROFILE
	uint32_t start = 0;
	uint32_t end = 0;
	uint32_t proc_read = 0;
	uint32_t  proc_adc = 0;
	uint32_t  proc_send = 0;
	uint32_t  proc_log = 0;
	uint32_t  proc_dmp = 0;
	uint32_t proc_control = 0;
	uint32_t counter_prof = 0;
	
	msg_profile.proc_read = 0;
	msg_profile.proc_adc = 0;
	msg_profile.proc_send = 0;
	msg_profile.proc_log = 0;
	msg_profile.proc_dmp = 0;
	msg_profile.proc_control = 0;
	#endif
	
	loop = true;
	
	while (loop)
	{
		//if (counter_link > PERIODIC_LINK_S) printf("link is missing \n");  
		if (rx_queue.count) 
		{
			process_bytes( dequeue(&rx_queue) ) ;
			//counter_link = 0;
		}
		
		#ifdef DRONE_PROFILE
		end = get_time_us();
		proc_read = end - start;
		#endif

		if (check_timer_flag())
		{
			if (counter++%20 == 0) 
			{
				nrf_gpio_pin_toggle(BLUE);
				//counter_link++;		
			}

			#ifdef DRONE_PROFILE
			if (counter_prof++%20 == 0) 
			{
				#ifdef ENCODE_PC_RECEIVE
					encode_packet((uint8_t *) &msg_profile, sizeof(struct msg_profile_t), MSG_PROFILE, output_data, &output_size);
					for (i=0; i<output_size; i++) {uart_put(output_data[i]);}
				#else 
					printf("%ld %ld %ld %ld %ld %ld %ld\n", proc_read, proc_adc, proc_send, proc_log, proc_dmp, proc_control, time_all);
				#endif
			}
			#endif
			
			#ifdef DRONE_PROFILE
			start = get_time_us();
			#endif
			adc_request_sample();
			//read_baro();
			#ifdef DRONE_PROFILE
			end = get_time_us();
			proc_adc = end - start;
			#endif

			if (counter_log++%2 == 0)
			{
				#ifdef DRONE_PROFILE
				start = get_time_us();
				#endif
				#ifdef ENCODE_PC_RECEIVE
					encode_packet((uint8_t *) msg_tele, sizeof(struct msg_telemetry_t), MSG_TELEMETRY, output_data, &output_size);	
					for (i=0; i<output_size; i++) {uart_put(output_data[i]);}
				#else
					printf("%d %d %d %d %d %d| ", mmode, control_mode, mthrust, mroll, mpitch, myaw);
					printf("%d %d %d %d| ", ae[0],ae[1],ae[2],ae[3]);
					// printf("%d %d %d| ", phi-cphi, theta-ctheta, psi-cpsi);
					printf("%d %d %d| ", phi, theta, -psi);
					printf("%d %d %d| ", sp-cp, -(sq-cq), -(sr-cr));
					// printf("%d %d %d| ", sp, -sq, -sr);					
					printf("%d %d \n", bat_volt, P);
				#endif
				#ifdef DRONE_PROFILE
				end = get_time_us();
				proc_send = end - start;
				#endif
			}

			// write to FLASH EVERY 50 ms
			#ifdef DRONE_PROFILE
			start = get_time_us();
			#endif
			if (status == true)
			{
				status = flash_data() ;
 				if((status = write_log()) == false) {printf("failed to write log");}	
			}
			#ifdef DRONE_PROFILE
			end = get_time_us();
			proc_log = end - start;
			#endif
			
			clear_timer_flag();
		}

		if (check_sensor_int_flag())
		{
			#ifdef DRONE_PROFILE
			start = get_time_us();
			#endif
			get_dmp_data();
			// dmp_status = get_dmp_data();
			// if(dmp_status == -1)
			// {
			// 	#ifdef DRONE_PROFILE
			// 	encode_ack(NOK);
			// 	for(j=0;j<encodedlog_size;j++)
			// 	{
			// 		uart_put(encodedlog[j]);
			// 	}
			// 	#endif
			// }
			// else
			// {
			// 	#ifdef DRONE_PROFILE
			// 	encode_ack(OK);
			// 	for(j=0;j<encodedlog_size;j++)
			// 	{
			// 		uart_put(encodedlog[j]);
			// 	}
			// 	#endif

			// }
			#ifdef DRONE_PROFILE
			end = get_time_us();
			proc_dmp = end - start;
			#endif
			
			#ifdef DRONE_PROFILE
			start = get_time_us();
			#endif
			run_filters_and_control();
			#ifdef DRONE_PROFILE
			end = get_time_us();
			proc_control = end - start;
			#endif
			
			clear_sensor_int_flag();
		}

		#ifdef DRONE_PROFILE
		start = get_time_us();
		end_all = get_time_us();
		time_all = end_all - start_all;
		start_all = get_time_us();
		#endif

		#ifdef DRONE_PROFILE
		msg_profile.proc_read = (uint16_t)proc_read;
		msg_profile.proc_adc = (uint16_t)proc_adc;
		msg_profile.proc_send = (uint16_t)proc_send;
		msg_profile.proc_log = (uint16_t)proc_log;
		msg_profile.proc_dmp = (uint16_t)proc_dmp;
		msg_profile.proc_control = (uint16_t)proc_control;
		msg_profile.time_all = time_all;
		#endif				
	}

	while(control_mode == ESCAPE)
	{
		if ((status = read_log())==true)
		{
			printf("Finished reading log flash");
			control_mode = MODE_SAFE;
		}	
	}

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);
	NVIC_SystemReset();
}

				