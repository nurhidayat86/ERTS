#ifndef _LOGGING_H_
#define _LOGGING_H_

// struct log_t {
//   uint16_t index_log;
//   uint32_t time_stamp;
//   uint8_t mode;
//   uint16_t thrust;
//   int16_t roll, pitch, yaw; 
//   int16_t ae[4];
//   int16_t phi, theta, psi; 
//   int16_t sp, sq, sr; 
//   int16_t sax, say, saz;
//   uint16_t bat_volt;
//   int32_t temperature, pressure;
// }__attribute__((packed, aligned(1)));

// struct msg_log_t log_msg;
// struct msg_log_t* msg_logging;
// uint16_t index_logging;

bool write_log(void);
bool read_log(void);
bool read_logs(void);
bool flash_data(void);
bool flash_individual_data(uint32_t time_stamp, uint8_t mode, uint16_t thrust, int16_t *ae, int16_t phi, int16_t theta, 
  int16_t psi, int16_t sp, int16_t sq, int16_t sr, uint16_t bat_volt, int32_t temperature, int32_t pressure);

#endif  /* #ifndef _LOGGING_H_ */
