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
uint8_t msc_flag = LOG_NO_USE;
bool update_flag = true;

static void process_bytes(uint8_t byte) {
	#ifdef DRONE_DEBUG
		printf("process_bytes()\n");
	#endif
	
	static struct msg_p msg;		// struct to parse the message
	struct msg_combine_all_t *msg_com_all; 
	msg_parse(&msg, byte);
	if(msg.status == GOT_PACKET) { 	// got the packet
		nrf_gpio_pin_toggle(RED);	// toggle the RED led to indicate the command from PC received
		switch(msg.msg_id) {		// capture the message based on the message id
			case MSG_COMBINE_ALL: 		// the message is command message
			{
				msg_com_all = (struct msg_combine_all_t *)&msg.payload[0];
				//update_flag = msg_com_all->update;
				mmode = msg_com_all->mode;
				mthrust = msg_com_all->thrust;
				mroll = msg_com_all->roll;
				mpitch = msg_com_all->pitch;
				myaw = msg_com_all->yaw;

				P = msg_com_all->P;
				P1 = msg_com_all->P1;
				P2 = msg_com_all->P2;
				msc_flag = msg_com_all->msc_flag;
				break;
			}

			default:
				break;

		};
		msg.status = UNITINIT; // Start to receive a new packet	
		lost_flag = false;
	}

	//separate toggle status from miscelanous flag
	if (msc_flag == RAW_USE)
		raw_status = true;
	else if(msc_flag == RAW_NO_USE)
		raw_status = false;
	else if(msc_flag == LOG_NO_USE)
		log_status = false;
	else if(msc_flag == LOG_USE)
		log_status = true;

	set_control_mode(mmode);							// set the mode
	set_control_command(mthrust, mroll, mpitch, myaw);	// set the control command
	
	if ((P<=MAX_P)&&(P1<=MAX_P1)&&(P2<=MAX_P2)) //add safety constraint, to prevent use pointer
		set_control_gains(P, P1, P2);
}

/*------------------------------------------------------------------
 * main -- do the test
 *------------------------------------------------------------------
 */
int main(void)
{
	///raw toggle variable
	init_raw = false;
	raw_status = false;
	log_status = false;

	#ifdef DRONE_DEBUG
		printf("int_main()\n");
	#endif
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);
	//baro_init();
	spi_flash_init();
	//	ble_init();

	unsigned short gfsr[0];
	unsigned char afsr[0];
	unsigned short srfsr[0];

	mpu_get_gyro_fsr(gfsr);
    mpu_get_accel_fsr(afsr);
    mpu_get_sample_rate(srfsr);  
    printf("%d %d %d", gfsr[0], afsr[0], srfsr[0]);

    //*****************************************************************************/
	//kalman variable
	//best performance when c1 phi btween 64 -256.
	//bigger, more damping ratio.
	//consider to change to bitwise operation after choosing c
	//*****************************************************************************/
    c1phi = 7;
	c1theta = 7;
	c2phi = 12;
	c2theta = 12;
	bp = 0;
	bq = 0;
	estimated_p = 0;
	estimated_q = 0;
	//*****************************************************************************/
	
	#ifdef ENCODE_PC_RECEIVE
	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;
	uint8_t i = 0;
	#endif

    // command_init();
	msg_tele.update = FALSE;
	msg_tele.mode = 0;
	msg_tele.thrust = 0;
	msg_tele.roll = 0;
 	msg_tele.pitch = 0;
 	msg_tele.yaw = 0;
					
 	cphi = ctheta = cpsi = 0;        ///< Calibration values of phi, theta, psi
	cp = cq = cr = 0;                ///< Calibration valies of p, q and r

	bool status = true;
	uint32_t counter = 0;
	uint32_t counter_log = 0;
	// uint32_t counter_link = 0;
	
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

	//communication lost
	uint32_t comm_start = 0;
	uint32_t comm_end = 0;
	uint16_t comm_duration = 0;
	uint32_t comm_duration_total = 0;
	uint32_t threshold = 20000000;
	lost_flag = false;
	#if DRONE_DEBUG
		uint8_t ackfired = ACK_FIRED;
	#endif
	uint8_t acklog = ACK_RCV;

	//to cut communication from the pc in case of emergency panic mode
	pc_link = true;

	/****************************************************************************
	* battery check variable
	*****************************************************************************/
	bat_flag = false;
	uint16_t BAT_THRESHOLD = 0; //demo
	uint8_t bat_counter = 0;
	uint8_t bat_counter_test = 0;
	/****************************************************************************
	* End of battery check variable
	*****************************************************************************/


	//=============================== MODE_START ===================================//
	set_control_mode(MODE_START);
	printf(" mode start: %d\n", control_mode);
	#ifdef DRONE_DEBUG
		printf("mode_start()\n");
	#endif
	while(control_mode == MODE_START) // wait until the PC safe to start up 
	{
		uart_put(0); 	// there is still a bug here
		if (rx_queue.count) 				// the count is not detected if the print is commented
		{
			process_bytes( dequeue(&rx_queue) );
			set_control_mode(MODE_SAFE);
			break;
		}
	} 
	//=============================== END MODE_START ===================================//


	#ifdef DRONE_DEBUG
		printf("mode_nonescape()\n");
	#endif


	//=============================== MODE_NONESCAPE ===================================//
	while((control_mode != ESCAPE))
	{
		
		#ifdef DRONE_PROFILE
		start = get_time_us();
		#endif


		//=============================== COMM ROBUST CHECK ==================================//
		comm_end = get_time_us();
		if (pc_link != false)
		if (rx_queue.count) 
		{
			process_bytes( dequeue(&rx_queue) ) ;
			comm_duration_total = 0;
			update_flag = true;
		}

		if (lost_flag == false)
			comm_duration = (comm_end - comm_start); //--> prevent loop forever in panic mode
		
		if(comm_duration > 700000)
		{
			comm_check(comm_duration, &comm_duration_total, &update_flag);
			if ((comm_duration_total >= threshold) && !lost_flag)
				{
					if(control_mode != MODE_SAFE)
					{
						set_control_mode(MODE_PANIC);
						pc_link = false;
					}
					// #ifdef DRONE_DEBUG
					// 	#ifdef ENCODE_PC_RECEIVE
					// 		encode_packet((uint8_t *) &ackfired, sizeof(uint8_t), MSG_ACK, output_data, &output_size);

					// 		for(i=0; i<output_size; i++) {uart_put(output_data[i]);}
					// 	#else
					// 		printf("communication_fail()\n");
					// 	#endif
					// #endif
					// set_control_command(400, 0, 0, 0); //--> bug solved due to this dont remove
					comm_duration_total = 0; // --> to prevent MODE_PANIC triger forever without going to mode_safe.
				}
		}
		//=============================== END COMM ROBUST CHECK ==============================//
		comm_start = get_time_us();
		
		#ifdef DRONE_PROFILE
		end = get_time_us();
		proc_read = end - start;
		#endif

		if (check_timer_flag())
		{
			if (counter++%20 == 0) 
			{
				nrf_gpio_pin_toggle(BLUE);
				bat_counter_test++;
				/****************************************************************************
				* battery check, it has to be 2 sec of bellow any speccified BAT_THRESHOLD
				*****************************************************************************/
				if ((bat_volt <= BAT_THRESHOLD)&&(!bat_flag))
				{
					if ((bat_counter >=1))
					{
						if(control_mode != MODE_SAFE)
						{

							set_control_mode(MODE_PANIC);
							#ifdef ENCODE_PC_RECEIVE
								acklog = ACK_BAT_LOW_EMERGENCY;
								encode_packet((uint8_t *) &acklog, sizeof(uint8_t), MSG_ACK, output_data, &output_size);
								for(i=0; i<output_size; i++) {uart_put(output_data[i]);}
							#else
								printf("Battery low disconnecting now!\n");
							#endif
							pc_link = false;
						}
						else
						{
							//send acknowledge to pc terminal
							#ifdef ENCODE_PC_RECEIVE
								acklog = ACK_BAT_LOW;
								encode_packet((uint8_t *) &acklog, sizeof(uint8_t), MSG_ACK, output_data, &output_size);
								for(i=0; i<output_size; i++) {uart_put(output_data[i]);}
							#else
								printf("Battery low\n");
							#endif
						}
						BAT_THRESHOLD = 0; //redundant
					}
					else if(bat_counter <1)
					{
						bat_counter++;
						#ifdef ENCODE_PC_RECEIVE
							acklog = ACK_BAT_LOW;
							encode_packet((uint8_t *) &acklog, sizeof(uint8_t), MSG_ACK, output_data, &output_size);
							for(i=0; i<output_size; i++) {uart_put(output_data[i]);}
						#else
							printf("Battery low\n");
						#endif
					}
				}
				/****************************************************************************
				* End of battery check
				*****************************************************************************/
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

			if ((counter_log++%2 == 0));
			{
				#ifdef DRONE_PROFILE
				start = get_time_us();
				#endif
				#ifdef ENCODE_PC_RECEIVE
					msg_tele.mode = control_mode;

					msg_tele.thrust = mthrust;
					msg_tele.roll = mroll;
					msg_tele.pitch = mpitch;
					msg_tele.yaw = myaw;

					msg_tele.engine[0] = ae[0];
					msg_tele.engine[1] = ae[1];
					msg_tele.engine[2] = ae[2];
					msg_tele.engine[3] = ae[3];

					if(init_raw == true)
					{
						// msg_tele.sp = (estimated_p-cp);
						// msg_tele.sq = (estimated_q-cq);
						msg_tele.sp = (estimated_p-cp);
						msg_tele.sq = (estimated_q-cq); 
					}
					else
					{
						msg_tele.sp = sp-cp;
						msg_tele.sq = -(sq-cq); 
					}

					msg_tele.phi = phi-cphi;
					msg_tele.theta = theta-ctheta;
					msg_tele.psi = -(psi-cpsi);
					msg_tele.sr = -(sr-cr);				
					
					msg_tele.sax = sax;
					msg_tele.say = say; 
					msg_tele.saz = saz;

					msg_tele.bat_volt = bat_volt;
					msg_tele.P = P;
					msg_tele.P1 = P1;
					msg_tele.P2 = P2;

					encode_packet((uint8_t *) &msg_tele, sizeof(struct msg_telemetry_t), MSG_TELEMETRY, output_data, &output_size);	
					for(i=0; i<output_size; i++) {uart_put(output_data[i]);}
					// for(i=0; i<output_size; i++) {printf("0x%x||\n",output_data[i]);}
					
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

			/***********************battery check debbuging purpose only*****************************/
			// if(bat_counter_test >= 10)
			// {
			// 	BAT_THRESHOLD = 1024;
			// }
			/***********************End of battery check debbuging purpose only**********************/

			//profiling
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
			
			if ((raw_status == false)&&(init_raw == false))
			{
				get_dmp_data();
			}

			//=============================== RAW  ==============================//
			if((raw_status == true) &&(init_raw == true))
			{
				get_raw_sensor_data();
				// kalman((sp-cp), -(sq-cq), sax, say, c1phi, c2phi, c1theta, c2theta, &estimated_p, &estimated_q, &phi, &theta, &bp, &bq);
				kalman(sp, -sq, sax, say, c1phi, c2phi, c1theta, c2theta, &estimated_p, &estimated_q, &phi, &theta, &bp, &bq);
				// kalman(sp-cq, -(sq-cq), sax-csax, say-csay, c1phi, c2phi, c1theta, c2theta, &estimated_p, &estimated_q, &phi, &theta, &bp, &bq);
			}
			//=============================== END RAW =================================//

			//=============================== TOGGLE RAW  HERE ==============================//
			if((raw_status == true) && (init_raw == false))
			{
				acklog = ACK_RAW_INIT;
				threshold = 700000; //1 sec for raw;
				// encode_ack(acklog, output_data, &output_size);
				// for(i=0;i<output_size;i++)
				// 	uart_put(output_data[i]);
				imu_init(false, 256);
				init_raw = true;
			}

			if((raw_status == false) && (init_raw == true))
			{
				imu_init(true, 100);
				init_raw = false;
				threshold = 700000; //700ms for dmp
			}
			//=============================== END TOGGLE RAW =================================//


			if ((status == true) && (log_status == true))
			{
				nrf_gpio_pin_clear(GREEN);
				status = flash_data() ;
 				if((status = write_log()) == false) {printf("failed to write log");}	
			}
			else {nrf_gpio_pin_set(GREEN);}
						
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
	//=============================== END MODE_NONESCAPE ===================================//

	//=============================== MODE_LOG ===================================//
	//this block is changed due to the previous version does not really exit the program
	//==========================================================================//

	if((status = read_logs())==true) 
	{
		acklog = ACK_RCV;
	}
	else
		acklog = ACK_FIRED;
	
	#ifdef ENCODE_PC_RECEIVE
	encode_ack(acklog, output_data, &output_size);
	for(i=0;i<output_size;i++)
	{
		uart_put(output_data[i]);
	}
	#else
		uart_put(0x77);
	#endif
	//=============================== END MODE_LOG ===================================//

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);
	NVIC_SystemReset();
}

				