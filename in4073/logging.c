/*------------------------------------------------------------------
 *  logging.c
 *
 *
 *
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include "logging.h"
// #include "logging_protocol.h"

struct msg_log_t *msg_logging;
struct msg_log_t log_msg;
static uint16_t index_logging;

bool write_log() {
	bool status;
	// status = flash_write_bytes((uint32_t) index_logging*sizeof(struct log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct log_t));
	// status = flash_write_bytes((uint32_t) index_logging*sizeof(struct msg_log_t), (uint8_t *) msg_logging, (uint32_t) sizeof(struct msg_log_t));
	status = flash_write_bytes((uint32_t) index_logging*sizeof(struct msg_log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct msg_log_t));
	
	// printf("%c", HDR);
	// printf("%c", index_logging);
	// printf("%d ", log_msg.phi);
	// printf("%d ", log_msg.theta);
	// printf("%d \n", log_msg.psi);
	index_logging+=1;
	return status;
}

bool read_logs() {
	
	#ifdef ENCODE_PC_RECEIVE
	uint8_t output_data[MAX_PAYLOAD+HDR_FTR_SIZE];
	uint8_t output_size;			
	uint16_t j;	
	#endif
	uint16_t i;
	bool status = true;
	
	// printf("%d \n", index_logging);
	for (i=0; i<index_logging; i++) 
	{
		#ifdef ENCODE_PC_RECEIVE
			// if(flash_read_bytes((uint32_t) i*sizeof(struct log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct log_t)) == true)
			if(flash_read_bytes((uint32_t) i*sizeof(struct msg_log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct msg_log_t)) == true)
			{
				encode_packet((uint8_t *) &log_msg, sizeof(struct msg_log_t), MSG_LOG, output_data, &output_size);
				for (j=0; j<output_size; j++) 
				{
					uart_put(output_data[j]);
					nrf_delay_ms(1);
				}	
			}
			
		#else
			// status = flash_read_bytes((uint32_t) i*sizeof(struct msg_log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct log_t));
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

bool flash_data() {
	
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
	
	log_msg.phi = phi-cphi;
	log_msg.theta = theta-ctheta;
	log_msg.psi = -(psi-cpsi);
	log_msg.sp = sp-cp;
	log_msg.sq = (sq-cq);
	log_msg.sr = -(sr-cr);

	log_msg.sax = sax;
	log_msg.say = say;
	log_msg.saz = saz;
	
	log_msg.bat_volt = bat_volt;
	
	log_msg.P = P;
	log_msg.P1 = P1;
	log_msg.P2 = P2;
	
	log_msg.temperature = temperature;
	log_msg.pressure = pressure;
	
	return true;
}

	// msg_logging->time_stamp = get_time_us();
	// msg_logging->mode = control_mode;
	// msg_logging->thrust = mthrust;
	
	// msg_logging->roll = mroll;
	// msg_logging->pitch = mpitch;
	// msg_logging->yaw = myaw;
	
	// msg_logging->ae[0] = ae[0];
	// msg_logging->ae[1] = ae[1];
	// msg_logging->ae[2] = ae[2];
	// msg_logging->ae[3] = ae[3];
	
	// msg_logging->phi = phi-cphi;
	// msg_logging->theta = theta-ctheta;
	// msg_logging->psi = -(psi-cpsi);
	// msg_logging->sp = sp-cp;
	// msg_logging->sq = (sq-cq);
	// msg_logging->sr = -(sr-cr);

	// msg_logging->sax = sax;
	// msg_logging->say = say;
	// msg_logging->saz = saz;
	
	// msg_logging->bat_volt = bat_volt;
	
	// msg_logging->P = P;
	// msg_logging->P1 = P1;
	// msg_logging->P2 = P2;
	
	// msg_logging->temperature = temperature;
	// msg_logging->pressure = pressure;

// bool read_log() {
	
// 	#ifdef ENCODE_PC_RECEIVE
// 	uint8_t j;
// 	#endif
// 	uint16_t i;
// 	bool status = true;
	
// 	printf("%d \n", index_logging);
// 	for (i=0; i<index_logging; i++) 
// 	{
// 		#ifdef ENCODE_PC_RECEIVE
// 		//#ifdef ENCODE
// 			if(flash_read_bytes((uint32_t) i*sizeof(struct log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct log_t)) == true)
// 			{
// 				encode_log((uint8_t *)&i,INDEX_LOG);for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.time_stamp,T_STAMP);for (j=0; j<encodedlog_size; j++) {uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.mode,MODE); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.thrust,THRUST); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.roll,ROLL); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.pitch,PITCH); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.yaw,YAW); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.ae[0],AE_0); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.ae[1],AE_1); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.ae[2],AE_2); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.ae[3],AE_3); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.phi,PHI); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.theta,THETA); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.psi,PSI); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.sp,SP); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.sq,SQ); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.sr,SR); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				//encode_log((uint8_t *)&log_msg.sax,SAX); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
// 				//encode_log((uint8_t *)&log_msg.say,SAY); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
// 				//encode_log((uint8_t *)&log_msg.saz,SAZ); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
// 				encode_log((uint8_t *)&log_msg.bat_volt,BAT_V); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.temperature,TEMP); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				encode_log((uint8_t *)&log_msg.pressure,PRESS); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 				if (i == 0)
// 					{
// 						encode_ack(INIT);
// 						for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 					}
// 				else if(i == (index_logging-1))
// 					{
// 						encode_ack(COMPLETE); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 					}
// 				else
// 					{
// 						encode_ack(OK); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(LOG_DELAY);}
// 					}
// 			}
			
// 		#else
// 			status = flash_read_bytes((uint32_t) i*sizeof(struct msg_log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct msg_log_t));
// 			printf("%d %ld | %d | %d %d %d %d | ", i, log_msg.time_stamp, log_msg.mode, log_msg.thrust, log_msg.roll, log_msg.pitch, log_msg.yaw);
// 			printf("%d %d %d %d | ", log_msg.ae[0], log_msg.ae[1], log_msg.ae[2], log_msg.ae[3] );
// 			printf("%d %d %d | ", log_msg.phi, log_msg.theta, log_msg.psi);
// 			printf("%d %d %d | ", log_msg.sp, log_msg.sq, log_msg.sr); 
// 			printf("%d %d %d | ", log_msg.sax, log_msg.say, log_msg.saz); 
// 			printf("%d %ld %ld\n", log_msg.bat_volt, log_msg.temperature, log_msg.pressure);
// 			nrf_delay_ms(10);
// 		#endif		
// 	}
// 	return status;
// }