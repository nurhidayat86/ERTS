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
//#include "control.h" 

bool loop;
static void process_bytes(uint8_t byte) {
	
	static struct msg_p msg;
	struct msg_combine_t *msg_com;	
	msg_parse(&msg, byte);
	if(msg.status == GOT_PACKET) {
		nrf_gpio_pin_toggle(RED);
		// We got a valid packet
		switch(msg.msg_id) {
			case MSG_COMBINE: 
			{
				msg_com = (struct msg_combine_t *)&msg.payload[0];
				msg_tele = (struct msg_telemetry_t *)&msg.payload[0];
				// msg_tele->mode = 2;
				// printf("%d \n", msg.crc_fails);
				// printf("%d %d %d %d %d \n", msg_tele->mode, msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
				// //printf("%d %d %d %d\n", ae[0], ae[1], ae[2], ae[3]);
				// msg_tele->engine[0] = ae[0];
				// msg_tele->engine[1] = ae[1];
				// msg_tele->engine[2] = ae[2];
				// msg_tele->engine[3] = ae[3];
				// msg_tele->update = TRUE;
				
				if(msg_com->mode == ESCAPE)
				{loop = false;}
		
				break;
			}

			default:
				break;

		};
		// Start to receive a new packet
		msg.status = UNITINIT;
	}

	// receive the mode
	set_control_mode(msg_com->mode);
	
	// set control command 
	// set_control_command(msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
	set_control_command(msg_com->thrust, msg_com->roll, msg_com->pitch, msg_com->yaw);
	
	// set_control_gains(yaw_d)			
}

void send_telemetry(void)
{
	// encode_packet((uint8_t *) msg_tele, sizeof(struct msg_telemetry_t), MSG_TELEMETRY, output_data, &output_size);	
	// for (i=0; i<output_size; i++) {uart_put(output_data[i]);}				
	// printf("%10ld | ", get_time_us());
	// printf("%d %d %d %d %d |", control_mode, msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
	printf("%d %d %d %d | ", ae[0],ae[1],ae[2],ae[3]);
	printf("%d %d %d | ", phi, theta, psi);
	printf("%d %d %d \n", sp, sq, sr);
	//printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);
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
//	ble_init();
	
	// uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	// uint8_t output_size;
	// uint8_t i = 0;
	
    // command_init();
	msg_tele->update = FALSE;
	msg_tele->mode = 0;
	msg_tele->thrust = 0;
	msg_tele->roll = 0;
 	msg_tele->pitch = 0;
 	msg_tele->yaw = 0;

 	cphi = ctheta = cpsi = 0;           ///< Calibration values of phi, theta, psi
	cp = cq = cr = 0;                ///< Calibration valies of p, q and r

	bool status = true;
	uint32_t counter = 0;
	uint32_t counter_log = 0;
	
	loop = true;
	
	while (loop)
	{
		if (rx_queue.count) process_bytes( dequeue(&rx_queue) );

		if (check_timer_flag())
		{
			if (counter++%20 == 0) 
			{
				nrf_gpio_pin_toggle(BLUE);
				
			}
			
			adc_request_sample();
			read_baro();
			
			// write to FLASH EVERY 100 ms
			if (counter_log++%2 == 0)
			{
				flash_data();
				write_log();
			} 
			
			// send telemetry on 50 ms			
			#ifdef ENCODE
				encode_packet((uint8_t *) msg_tele, sizeof(struct msg_telemetry_t), MSG_TELEMETRY, output_data, &output_size);	
				for (i=0; i<output_size; i++) {uart_put(output_data[i]);}
			#else
				// 	printf("%d %d %d %d %d \n", control_mode, msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
				//  send_telemetry();
				printf("%d %d %d %d %d| ", control_mode, msg_tele->thrust, msg_tele->roll, msg_tele->pitch, msg_tele->yaw);
				printf("%d %d %d %d| ", ae[0],ae[1],ae[2],ae[3]);
				//printf("%d %d %d| ", phi-cphi, theta-ctheta, psi-cpsi);
				printf("%d %d %d| ", phi, theta, -psi);
				printf("%d %d %d| ", sp-cp, -(sq-cq), -(sr-cr));
				printf("%d\n", bat_volt);
			#endif
			
			clear_timer_flag();
		}

		if (check_sensor_int_flag())
		{
			get_dmp_data();
			run_filters_and_control();
			clear_sensor_int_flag();
		}

	}

	while(control_mode == ESCAPE)
	{
		if ((status = read_log())==true)
		{
			printf("Reading Log Data Finished");
			control_mode = MODE_SAFE;
		}	
	}

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);
	NVIC_SystemReset();
}

				