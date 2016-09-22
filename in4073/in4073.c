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
#include "protocol.h"
//#include "control.h" 

bool loop;

static void process_bytes(uint8_t byte) {
	
	static struct msg_p msg;
	struct msg_keyboard_t *msg_key;
	msg_parse(&msg, byte);
	if(msg.status == GOT_PACKET) {
		nrf_gpio_pin_toggle(RED);
		// We got a valid packet
		switch(msg.msg_id) {
			case MSG_JOYSTICK: {
				struct msg_joystick_t *msg_js = (struct msg_joystick_t *)&msg.payload[0];
				printf("%d %d %d %d %d\n", msg_js->mode, msg_js->thrust, msg_js->roll, msg_js->pitch, msg_js->yaw);
				break;
			}
			case MSG_KEYBOARD: {
				msg_key = (struct msg_keyboard_t *)&msg.payload[0];
				printf("%d %d %d %d %d\n", msg_key->mode, msg_key->thrust, msg_key->roll, msg_key->pitch, msg_key->yaw);
				
				if(msg_key->mode == ESCAPE)
				{loop = false;}
		
				break;
			}

			default:
				break;
		};
		// Start to receive a new packet
		msg.status = UNITINIT;
	}

	// escape : exit from main loop to prevent the need to unplug the serial 
	// switch (byte) 
	// {
	// 	case 27:
	// 		loop = false;
	// 		break;
	// 	default:
	// 		nrf_gpio_pin_toggle(RED);
	// }
	
	//set_control_mode(MODE_...)
	set_control_mode(msg_key->mode);
	set_control_from_js(msg_key->thrust, msg_key->roll, msg_key->pitch, msg_key->yaw);
	//set_control_from_js(msg_thrust, msg_key.roll, msg_key.pitch, msg_key.yaw);
	//set_control_gains(yaw_d)			
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
			// printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
			// printf("%6d %6d %6d | ", phi, theta, psi);
			// printf("%6d %6d %6d | ", sp, sq, sr);
			// printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);

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
