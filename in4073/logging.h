#include "in4073.h"

struct log_t {
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

