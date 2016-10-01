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
				encode_log((uint8_t *)&i,INDEX_LOG);
				for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.time_stamp,T_STAMP);for (j=0; j<encodedlog_size; j++) {uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.mode,MODE); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.thrust,THRUST); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.roll,ROLL); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.pitch,PITCH); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.yaw,YAW); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.ae[0],AE_0); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.ae[1],AE_1); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.ae[2],AE_2); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.ae[3],AE_3); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.phi,PHI); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.theta,THETA); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.psi,PSI); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.sp,SP); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.sq,SQ); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.sr,SR); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				//encode_log((uint8_t *)&log_msg.sax,SAX); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				//encode_log((uint8_t *)&log_msg.say,SAY); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				//encode_log((uint8_t *)&log_msg.saz,SAZ); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.bat_volt,BAT_V); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.temperature,TEMP); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_log((uint8_t *)&log_msg.pressure,PRESS); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
				encode_ack(OK); for (j=0; j<encodedlog_size; j++){uart_put(encodedlog[j]); nrf_delay_ms(100);}
			}
			
		#else
			status = flash_read_bytes((uint32_t) i*sizeof(struct log_t), (uint8_t *) &log_msg, (uint32_t) sizeof(struct log_t));
			printf("%ld %d | %d | %d %d %d %d | ", log_msg.time_stamp, i, log_msg.mode, log_msg.thrust, log_msg.roll, log_msg.pitch, log_msg.yaw);
			printf("%d %d %d %d | ", log_msg.ae[0], log_msg.ae[1], log_msg.ae[2], log_msg.ae[3] );
			printf("%d %d %d | ", log_msg.phi, log_msg.theta, log_msg.psi);
			printf("%d %d %d | ", log_msg.sp, log_msg.sq, log_msg.sr); 
			printf("%d %ld %ld\n", log_msg.bat_volt, log_msg.temperature, log_msg.pressure); 
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