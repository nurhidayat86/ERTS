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
				
				//printf("co %d %d %d %d %d\n", msg_com->mode, msg_com->thrust, msg_com->roll, msg_com->pitch, msg_com->yaw);
				//printf("%d %d %d %d\n", ae[0], ae[1], ae[2], ae[3]);
				msg_tele->update = TRUE;
				msg_tele->engine[0] = ae[0];
				msg_tele->engine[1] = ae[1];
				msg_tele->engine[2] = ae[2];
				msg_tele->engine[3] = ae[3];

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
	set_control_command(msg_com->thrust, msg_com->roll, msg_com->pitch, msg_com->yaw);
	
	// set_control_gains(yaw_d)			
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

	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;
	uint8_t i = 0;
	
	uint32_t counter = 0;
	
	loop = true;
	
	while (loop)
	{
		if (rx_queue.count) process_bytes( dequeue(&rx_queue) );

		if (check_timer_flag())
		{
			if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);

			adc_request_sample();
			read_baro();

			// printf("%10ld | ", get_time_us());
			// printf("ae %d %d %d %d \n",ae[0],ae[1],ae[2],ae[3]);
			// printf("%6d %6d %6d | ", phi, theta, psi);
			// printf("%6d %6d %6d | ", sp, sq, sr);
			// printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);
			
			if(msg_tele->update)
			{
				encode_packet((uint8_t *) msg_tele, sizeof(struct msg_telemetry_t), MSG_TELEMETRY, output_data, &output_size);	
				for (i=0; i<output_size; i++) {uart_put(output_data[i]);}	
				msg_tele->update = FALSE;			
			}

			clear_timer_flag();
		}

		if (check_sensor_int_flag())
		{
			//get_dmp_data();
			run_filters_and_control();

			clear_sensor_int_flag();
		}
	}

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);
	NVIC_SystemReset();
}
