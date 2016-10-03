#include "in4073.h"

struct log_t {
  uint16_t index_log;
  uint32_t time_stamp;
  uint8_t mode;
  uint16_t thrust;
  int16_t roll, pitch, yaw; 
  int16_t ae[4];
  int16_t phi, theta, psi; 
  int16_t sp, sq, sr; 
  //int16_t sax, say, saz;
  uint16_t bat_volt;
  int32_t temperature, pressure;
}__attribute__((packed, aligned(1)));


struct log_t log_msg;
uint16_t index_logging;

bool write_log(void);
bool read_log(void);
bool flash_data(void);
bool flash_individual_data(uint32_t time_stamp, uint8_t mode, uint16_t thrust, int16_t *ae, int16_t phi, int16_t theta, 
  int16_t psi, int16_t sp, int16_t sq, int16_t sr, uint16_t bat_volt, int32_t temperature, int32_t pressure);

//Individual variable
  uint16_t MSG_index_log;
  uint32_t MSG_time_stamp;
  uint8_t MSG_mode;
  uint16_t MSG_thrust;
  int16_t MSG_roll, MSG_pitch, MSG_yaw; 
  int16_t MSG_ae[4];
  int16_t MSG_phi, MSG_theta, MSG_psi; 
  int16_t MSG_sp, MSG_sq, MSG_sr; 
  //int16_t MSG_sax, MSG_say, MSG_saz;
  uint16_t MSG_bat_volt;
  int32_t MSG_temperature, MSG_pressure;
