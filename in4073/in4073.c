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
#include "protocol.h"

/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */

uint8_t mode = 0;

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
	// uint8_t output_data[255];
	// uint8_t output_size;

	index_logging = 0;
	// uint8_t j;
	uint i_log = 0;
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
				if (counter++%5 == 0)
				{
					if(i_log<index_logging) {
						if(i_log == 0){printf("time_stamp, mode, thrust, roll, pitch, yaw, ae[0], ae[1], ae[2], ae[3], phi, theta, psi, sp, sq, sr, bat_volt, temperature, pressure \n\r");}
						if ((status = flash_read_bytes((uint32_t) i_log*sizeof(struct log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct log_t)))==true)
						{
							printf("%10ld, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d, %10ld, %10ld \n\r",
							log_msg.time_stamp, log_msg.mode, log_msg.thrust, log_msg.roll, log_msg.pitch, log_msg.yaw, 
							log_msg.ae[0], log_msg.ae[1], log_msg.ae[2], log_msg.ae[3], log_msg.phi, log_msg.theta, log_msg.psi, log_msg.sp, log_msg.sq, log_msg.sr, 
							log_msg.bat_volt, log_msg.temperature, pressure);
						}
						i_log+=1;
					}
					
				}
			}

			clear_timer_flag();
		}

		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
			run_filters_and_control();
			clear_sensor_int_flag();
		}

	}
	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
