#include "in4073.h"
#include "logging.h"
// #include "logging_protocol.h"

struct msg_log_t *msg_logging;
struct msg_log_t log_msg;
static uint16_t index_logging;


/*------------------------------------------------------------
 * Author		: Arif Nurhidayat
 * Adapted from : 
 * Funtionalty	: Write to flash memory, gives false status if it failes.
 *------------------------------------------------------------*/
bool write_log() {
	bool status;
	status = flash_write_bytes((uint32_t) index_logging*sizeof(struct msg_log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct msg_log_t));
	index_logging+=1;
	return status;
}
/*------------------------------------------------------------*/



/*------------------------------------------------------------
 * Author		: Arif Nurhidayat
 * Adapted from : 
 * Funtionalty	: Read from flash memory, gives false status if it failes.
 *------------------------------------------------------------*/
bool read_logs() {
	
	#ifdef ENCODE_PC_RECEIVE
	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;			
	uint16_t j;	
	#endif
	uint16_t i;
	bool status = false;
	
	// printf("%d \n", index_logging);
	for (i=0; i<index_logging; i++) 
	{
		#ifdef ENCODE_PC_RECEIVE
			// The log mmessage is encoded before sending
			status = flash_read_bytes((uint32_t) i*sizeof(struct msg_log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct msg_log_t));
			if(status == true)
			{
				encode_packet((uint8_t *) &log_msg, sizeof(struct msg_log_t), MSG_LOG, output_data, &output_size);
				for (j=0; j<output_size; j++) 
				{
					uart_put(output_data[j]);
					nrf_delay_ms(1);
				}	
			}
			
		#else
			// The log mmessage is sent without encoding
			status = flash_read_bytes((uint32_t) i*sizeof(struct msg_log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct msg_log_t));
			printf("%d %ld | %d | %d %d %d %d | ", i, log_msg.time_stamp, log_msg.mode, log_msg.thrust, log_msg.roll, log_msg.pitch, log_msg.yaw);
			printf("%d %d %d %d | ", log_msg.ae[0], log_msg.ae[1], log_msg.ae[2], log_msg.ae[3] );
			printf("%d %d %d | ", log_msg.phi, log_msg.theta, log_msg.psi);
			printf("%d %d %d | ", log_msg.sp, log_msg.sq, log_msg.sr); 
			printf("%d %d %d | ", log_msg.sax, log_msg.say, log_msg.saz); 
			printf("%d %ld %ld\n", log_msg.bat_volt, log_msg.temperature, log_msg.pressure);
			nrf_delay_ms(10);
		#endif		
	}
	return status;
}
/*------------------------------------------------------------*/



/*------------------------------------------------------------
 * Author		: Reggie
 * Adapted from : 
 * Funtionalty	: Writes all needed parameters (sensors, cotrollers, etc) to the logging variable.
 *------------------------------------------------------------*/
bool flash_data() {
	//Save sensors value to log message structure, before this message structure is written to the memory.
	log_msg.index_log = index_logging;
	log_msg.time_stamp = get_time_us();
	log_msg.mode = control_mode;
	log_msg.thrust = mthrust;
	
	log_msg.roll = mroll;
	log_msg.pitch = mpitch;
	log_msg.yaw = myaw;
	
	log_msg.ae[0] = ae[0];
	log_msg.ae[1] = ae[1];
	log_msg.ae[2] = ae[2];
	log_msg.ae[3] = ae[3];
		
	if(init_raw == true)
	{
		log_msg.phi = phi;
		log_msg.theta = theta;
	}
	else
	{
		log_msg.phi = phi-cphi;
		log_msg.theta = theta-ctheta;
	}
	log_msg.psi = -(psi-cpsi);
	
	log_msg.sp = sp-cp;
	log_msg.sq = -(sq-cq);
	log_msg.sr = -(sr-cr);

	log_msg.esp = estimated_p;
	log_msg.esq = estimated_q;
	log_msg.esr = r_butter;

	log_msg.sax = sax-csax;
	log_msg.say = say-csay;
	log_msg.saz = saz;
	
	log_msg.bat_volt = bat_volt;
	
	log_msg.P = P;
	log_msg.P1 = P1;
	log_msg.P2 = P2;
	
	log_msg.temperature = temperature;
	log_msg.pressure = pressure;
	
	return true;
}
/*------------------------------------------------------------*/