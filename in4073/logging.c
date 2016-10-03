#include "logging.h"
#include "logging_protocol.h"

#define ENCODE

bool write_log() {
	bool status;
	status = flash_write_bytes((uint32_t) index_logging*sizeof(struct log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct log_t));
	index_logging+=1;
	return status;
}

bool read_log() {
	
	#ifdef ENCODE
	uint8_t j;
	#endif
	uint16_t i;
	bool status = true;
	
	printf("%d \n", index_logging);
	for (i=0; i<index_logging; i++) 
	{
		#ifdef ENCODE
			if(flash_read_bytes((uint32_t) i*sizeof(struct log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct log_t)) == true)
			{
				encode_log((uint8_t *)&i,INDEX_LOG);for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_time_stamp,T_STAMP);for (j=0; j<encodedlog_size; j++) {uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_mode,MODE); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_thrust,THRUST); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_roll,ROLL); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_pitch,PITCH); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_yaw,YAW); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_ae[0],AE_0); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_ae[1],AE_1); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_ae[2],AE_2); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_ae[3],AE_3); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_phi,PHI); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_theta,THETA); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_psi,PSI); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_sp,SP); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_sq,SQ); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_sr,SR); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				//encode_log((uint8_t *)&log_msg.sax,SAX); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				//encode_log((uint8_t *)&log_msg.say,SAY); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				//encode_log((uint8_t *)&log_msg.saz,SAZ); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_bat_volt,BAT_V); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_temperature,TEMP); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&MSG_pressure,PRESS); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_ack(OK); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
			}
			
		#else
			status = flash_read_bytes((uint32_t) i*sizeof(struct log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct log_t));
			printf("%ld %d | %d | %d %d %d %d | ", MSG_time_stamp, i, MSG_mode, MSG_thrust, MSG_roll, MSG_pitch, MSG_yaw);
			printf("%d %d %d %d | ", MSG_ae[0], MSG_ae[1], MSG_ae[2], MSG_ae[3] );
			printf("%d %d %d | ", MSG_phi, MSG_theta, MSG_psi);
			printf("%d %d %d | ", MSG_sp, MSG_sq, MSG_sr); 
			printf("%d %ld %ld\n", MSG_bat_volt, MSG_temperature, MSG_pressure); 
			nrf_delay_ms(10);
		#endif		
	}
	return status;
}

bool flash_data() {
	log_msg.time_stamp = get_time_us();
	log_msg.mode = 1;
	log_msg.thrust = 3;
	
	log_msg.roll = 300;
	log_msg.pitch = 200;
	log_msg.yaw = 100;
	
	log_msg.ae[0] = ae[0];
	log_msg.ae[1] = ae[1];
	log_msg.ae[2] = ae[2];
	log_msg.ae[3] = ae[3];
	
	log_msg.phi = phi;
	log_msg.theta = theta;
	log_msg.psi = psi;
	log_msg.sp = sp;
	log_msg.sq = sq;
	log_msg.sr = sr;

	// log_msg.sax = sax;
	// log_msg.say = say;
	// log_msg.saz = saz;
	
	log_msg.bat_volt = bat_volt;
	log_msg.temperature = temperature;
	log_msg.pressure = pressure;
	return true;
}

bool flash_individual_data() {
	MSG_time_stamp = get_time_us();
	MSG_index_mode = 1;
	MSG_index_thrust = 3;
	
	MSG_index_roll = 300;
	MSG_index_pitch = 200;
	MSG_index_yaw = 100;
	
	MSG_index_ae[0] = ae[0];
	MSG_index_ae[1] = ae[1];
	MSG_index_ae[2] = ae[2];
	MSG_index_ae[3] = ae[3];
	
	MSG_index_phi = phi;
	MSG_index_theta = theta;
	MSG_index_psi = psi;
	MSG_index_sp = sp;
	MSG_index_sq = sq;
	MSG_index_sr = sr;

	// MSG_index_sax = sax;
	// MSG_index_say = say;
	// MSG_index_saz = saz;
	
	MSG_index_bat_volt = bat_volt;
	MSG_index_temperature = temperature;
	MSG_index_pressure = pressure;
	return true;
}