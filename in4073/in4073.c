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

#include "logging.h"
#include "logging_protocol.h"

/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */

uint8_t mode;

void process_key(uint8_t c)
{
	switch (c) 
	{
		case 'q':
			ae[0] += 10;
			break;
		case 'a':
			ae[0] -= 10;
			if (ae[0] < 0) ae[0] = 0;
			break;
		case 'w':
			ae[1] += 10;
			break;
		case 's':
			ae[1] -= 10;
			if (ae[1] < 0) ae[1] = 0;
			break;
		case 'e':
			ae[2] += 10;
			break;
		case 'd':
			ae[2] -= 10;
			if (ae[2] < 0) ae[2] = 0;
			break;
		case 'r':
			ae[3] += 10;
			break;
		case 'f':
			ae[3] -= 10;
			if (ae[3] < 0) ae[3] = 0;
			break;
		case 27:
			demo_done = true;
			break;
		case 'p':
		{
			mode = 1;
			break;
		}
		case 'l':
		{
			mode = 0;
			break;
		}
		case 'k':
		{
			mode = 2;
			break;
		}
		case 'm':
		{
			uart_put(0x7a);
			break;
		}
		default:
			nrf_gpio_pin_toggle(RED);
	}
}

/*------------------------------------------------------------------
 * main -- do the test
 *------------------------------------------------------------------
 */
int main(void)
{
	mode=0;

	index_logging = 0;
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);	
	baro_init();
	spi_flash_init();
	bool status;
//	ble_init();
	if((status = flash_chip_erase())==false) {printf("failed to erase data");}

	uint32_t counter = 0;

	while (!demo_done)
	{	
		if (rx_queue.count) process_key( dequeue(&rx_queue) ); 

		if (check_timer_flag()) 
		{
			if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);

			adc_request_sample();
			read_baro();
			if(mode == 0) {
				printf("%10ld | ", get_time_us());
				printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
				printf("%6d %6d %6d | ", phi, theta, psi);
				printf("%6d %6d %6d | ", sp, sq, sr);
				printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);
				printf("mode: %d \n", mode);
			}
			
			else if(mode ==1) {
				if(counter++%5 == 0) // write log every 1/5 secs;
				{
					flash_data();
					if((status = write_log()) == false) {printf("failed to write log");}
					else printf("index_logging: %6d log_msg |%10ld |%6d |%6d |\n",index_logging, log_msg.time_stamp, log_msg.mode, log_msg.thrust);
				} 
			}
			else if (mode == 2) {
				//status = read_log();
				demo_done = true;
			}

			clear_timer_flag();
		}

		if (check_sensor_int_flag()) 
		{
			if (mode != 2) {get_dmp_data();
			run_filters_and_control();
			clear_sensor_int_flag();}
		}

	}
	status = read_log();
	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);
	NVIC_SystemReset();
}
