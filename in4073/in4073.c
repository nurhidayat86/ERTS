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
#include "kalman.h"
// #include "logging_protocol.h"

bool loop;

static void process_bytes(uint8_t byte) {
	
	static struct msg_p msg;		// struct to parse the message
	// static struct msg_combine_t *msg_com;	// struct to capture the combined command from PC
	// static struct msg_combine_t com_msg;	// struct to capture the combined command from PC
	struct msg_tuning_t *msg_tune;	// struct to capture the gain tuning message
	//static bool disable_mode = false;
	msg_parse(&msg, byte);
	if(msg.status == GOT_PACKET) { 	// got the packet
		nrf_gpio_pin_toggle(RED);	// toggle the RED led to indicate the command from PC received
		switch(msg.msg_id) {		// capture the message based on the message id
			case MSG_COMBINE: 		// the message is command message
			{
				// capture the payload of the message
				// msg_com = (struct msg_combine_t *)&msg.payload[0];
				msg_tele = (struct msg_telemetry_t *)&msg.payload[0];
				
				// assign to coresponding element of struct
				// mmode = msg_com->mode;
				// mthrust = msg_com->thrust;
				// mroll = msg_com->roll;
				// mpitch = msg_com->pitch;
				// myaw = msg_com->yaw;
				
				mmode = msg_tele->mode;
				// control_mode = msg_tele->mode;
				//com_msg.mode = msg_tele->mode;
				// msg_com->mode = mmode;

				if(mmode == ESCAPE)
				{loop = false;}

				mthrust = msg_tele->thrust;
				mroll = msg_tele->roll;
				mpitch = msg_tele->pitch;
				myaw = msg_tele->yaw;
				
				//disable_mode = false;
				break;
			}

			case MSG_TUNE: 
			{
				msg_tune = (struct msg_tuning_t *)&msg.payload[0];
				P = msg_tune->P;
				P1 = msg_tune->P1;
				P2 = msg_tune->P2;
				
				//disable_mode = true;
				break;
			}

			default:
				break;

		};
		msg.status = UNITINIT; // Start to receive a new packet	
	}
	// put semaphore 
	// if(!disable_mode) set_control_mode(msg_tele->mode);
	set_control_mode(mmode);		// set the mode
	// set_control_mode(control_mode);		// set the mode

	// set_control_command(msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
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
	//imu_init(true, 100);
	imu_init(false, 512);

	baro_init();
	spi_flash_init();
	//	ble_init();

	unsigned short gfsr[0];
	unsigned char afsr[0];
	unsigned short srfsr[0];

	mpu_get_gyro_fsr(gfsr);
    mpu_get_accel_fsr(afsr);
    mpu_get_sample_rate(srfsr);  

    printf("%d %d %d", gfsr[0], afsr[0], srfsr[0]);

    uint16_t c1phi = 10;
	uint16_t c1theta = 10;
	uint16_t c2phi = c1phi<<10;
	uint16_t c2theta = c1theta<10;
	int16_t estimated_p = 0;
	int16_t estimated_q = 0;
	int16_t estimated_phi = 0;
	int16_t estimated_theta = 0;
	int16_t bp = 0;
	int16_t bq = 0;
	
	#ifdef ENCODE_PC_RECEIVE
	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;
	uint8_t i = 0;
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
	uint32_t proc_adc = 0;
	uint32_t proc_send = 0;
	uint32_t proc_log = 0;
	uint32_t proc_dmp = 0;
	uint32_t proc_control = 0;
	// uint32_t counter_prof = 0;
	uint32_t start_all = 0, end_all = 0, time_all = 0;
	uint32_t start_send = 0;
	
	msg_profile.proc_read = 0;
	msg_profile.proc_adc = 0;
	msg_profile.proc_send = 0;
	msg_profile.proc_log = 0;
	msg_profile.proc_dmp = 0;
	msg_profile.proc_control = 0;
	#endif
	
	// loop = true;
	// control_mode = ESCAPE;
	// printf(" mode %d\n", control_mode);
	// while(control_mode == ESCAPE) // wait until the PC safe to start up 
	// {
	// 	// printf("count %d\n", rx_queue.count);
	// 	// printf("mode %d\n", control_mode);
	// 	control_mode = ESCAPE;
	// 	printf("mode %d\n", control_mode);
	// 	if (rx_queue.count) {
	// 		process_bytes( dequeue(&rx_queue) ) ;
	// 		break;
	// 	}
	// 	// if (rx_queue.count) 
	// 	// {
	// 	// 	control_mode = MODE_SAFE;
	// 	// 	// printf("mode %d\n", control_mode);
	// 	// }	
	// } 
	
	// while (loop)
	while(control_mode != ESCAPE)
	{
		// if (counter_link > PERIODIC_LINK_S) printf("link is missing \n");  
		#ifdef DRONE_PROFILE
		start = get_time_us();
		#endif

		if (rx_queue.count) 
		{
			process_bytes( dequeue(&rx_queue) ) ;
			// counter_link = 0;
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
				// counter_link++;		
			}

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
					msg_tele->mode = mmode;

					msg_tele->thrust = mthrust;
					msg_tele->roll = mroll;
					msg_tele->pitch = mpitch;
					msg_tele->yaw = myaw;

					msg_tele->engine[0] = ae[0];
					msg_tele->engine[1] = ae[1];
					msg_tele->engine[2] = ae[2];
					msg_tele->engine[3] = ae[3];

					// msg_tele->phi = phi-cphi;
					// msg_tele->theta = theta-ctheta;
					// msg_tele->psi = -(psi-cpsi);

					msg_tele->phi = estimated_phi;
					msg_tele->theta = estimated_theta;
					msg_tele->psi = -(psi-cpsi);
	
					msg_tele->sp = sp-cp;
					msg_tele->sq = -(sq-cq); 
					msg_tele->sr = -(sr-cr);

					msg_tele->sax = sax;
					msg_tele->say = say; 
					msg_tele->saz = saz;

					msg_tele->bat_volt = bat_volt;

					msg_tele->P = P;
					msg_tele->P1 = P1;
					msg_tele->P2 = P2;

					if(control_mode != MODE_HEIGHT)
					{
						encode_packet((uint8_t *) msg_tele, sizeof(struct msg_telemetry_t), MSG_TELEMETRY, output_data, &output_size);	
						for(i=0; i<output_size; i++) {uart_put(output_data[i]);}
					}	
					
				#else
					printf("%d %d %d %d %d %d| ", mmode, control_mode, mthrust, mroll, mpitch, myaw);
					printf("%d %d %d %d| ", ae[0],ae[1],ae[2],ae[3]);
					printf("%d %d %d| ", phi, theta, -psi);
					printf("%d %d %d| ", phi-cphi, theta-ctheta, -(psi-cpsi));
					printf("%d %d %d| ", sp-cp, -(sq-cq), -(sr-cr));
					printf("%d %d %d| ", sax, say, saz);
					// printf("%d %d %d| ", sp, -sq, -sr);					
					printf("%d %d %d %d \n", bat_volt, P, P1, P2);
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
			// if (status == true)
			// {
			// 	status = flash_data() ;
 			// 	if((status = write_log()) == false) {printf("failed to write log");}	
			// }
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
			//get_dmp_data();
			get_raw_sensor_data();
			kalman((sp-cp), -(sq-cq), sax, say, c1phi, c2phi, c1theta, c2theta, &estimated_p, &estimated_q, &estimated_phi, &estimated_theta, &bp, &bq);
			if (status == true)
			{
				status = flash_data() ;
 				if((status = write_log()) == false) {printf("failed to write log");}	
			}
						
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
		msg_profile.time_all = (uint16_t)time_all;
		#endif	

		#ifdef DRONE_PROFILE
		if ( get_time_us() - start_send > 100000) 
		{
			#ifdef ENCODE_PC_RECEIVE
				encode_packet((uint8_t *) &msg_profile, sizeof(struct msg_profile_t), MSG_PROFILE, output_data, &output_size);
				for (i=0; i<output_size; i++) {uart_put(output_data[i]);}
			#else 
				printf("%ld %ld %ld %ld %ld %ld %ld\n", proc_read, proc_adc, proc_send, proc_log, proc_dmp, proc_control, time_all);
			#endif
		}
		start_send = get_time_us();	
		#endif
				
	}

	// control_mode = ESCAPE;
	// while(control_mode == ESCAPE)
	// while(control_mode != MODE_SAFE)	
	while(true)
	{
		// if ((status = read_logs())==true)
		// {
		// 	printf("Finished reading log flash");
		// 	control_mode = MODE_SAFE;

		// }
		read_logs();
		printf("Finished");
		break;	
	}

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);
	NVIC_SystemReset();
}

				