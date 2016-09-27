#include "in4073.h"
#include "protocol.h"

bool write_log() {
	bool status;
	status = flash_write_bytes((uint32_t) index_logging*sizeof(struct log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct log_t));
	// printf("%c", HDR);
	// printf("%c", index_logging);
	// printf("%d ", log_msg.phi);
	// printf("%d ", log_msg.theta);
	// printf("%d \n", log_msg.psi);
	index_logging+=1;
	return status;
}

bool read_log() {
	
	uint8_t output_data[205];
 	uint8_t output_size;
	bool status = true;
	uint8_t i = 0;
	uint8_t j = 0;

	for (i=0; i<index_logging; i+=1) 
	{
		// if (status == false)
		// {
		// 	break;
		// }
		// else
		// {
			
		// }
		//printf("%d |", i);
		status = flash_read_bytes((uint32_t) i*sizeof(struct log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct log_t));
		//printf("%d | %d | %d \n", log_msg.phi, log_msg.theta, log_msg.psi);
		encode_packet((uint8_t *) &log_msg, sizeof(struct log_t), MSG_LOG, output_data, &output_size);
		for (j=0;j<output_size;j++) uart_put(output_data[j]);
		nrf_delay_ms(100);	
	}
	return status;
}

// bool read_log (struct log_t *log_read, uint8_t *log_index) {
// 	bool status = true;
// 	uint8_t index = 0;
// 	while (((index*sizeof(log_t)) <= 125000) && (status == true))
// 	{
// 		status = flash_read_bytes(index*sizeof(struct log_t), (uint8_t *) &log_read, sizeof(struct log_t));
// 		if (status == true)
// 		{
// 			//send encoded data to the pc here here
// 		}
// 		else
// 		{
// 			status = false;
// 			break;
// 		}
// 	}
// 	return status;
// }

bool flash_data() {
	log_msg.phi = phi;
	log_msg.theta = theta;
	log_msg.psi = psi;
	log_msg.sp = sp;
	log_msg.sq = sq;
	log_msg.sr = sr;
	log_msg.sax = sax;
	log_msg.say = say;
	log_msg.saz = saz;
	log_msg.roll = 255;
	log_msg.pitch = 233;
	log_msg.yaw = 122;
	log_msg.bat_volt = bat_volt;
	log_msg.ae[0] = ae[0];
	log_msg.ae[1] = ae[1];
	log_msg.ae[2] = ae[2];
	log_msg.ae[3] = ae[3];
	log_msg.thrust = 134;
	log_msg.mode = 1;
	log_msg.temperature = temperature;
	log_msg.pressure = pressure;
	return true;
}