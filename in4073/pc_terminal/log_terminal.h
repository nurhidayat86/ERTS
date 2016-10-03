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

//individual variable
  uint16_t *MSG_index_log;
  uint32_t *MSG_time_stamp;
  uint8_t *MSG_mode;
  uint16_t *MSG_thrust;
  int16_t *MSG_roll, *MSG_pitch, *MSG_yaw; 
  int16_t *MSG_ae[4];
  int16_t *MSG_phi, *MSG_theta, *MSG_psi; 
  int16_t *MSG_sp, *MSG_sq, *MSG_sr; 
  //int16_t *MSG_sax, *MSG_say, *MSG_saz;
  uint16_t *MSG_bat_volt;
  int32_t *MSG_temperature, *MSG_pressure;

//individual_var
// uint16_t msg_log_p-index_log;
// uint32_t msg_log_p-time_stamp;
// uint8_t msg_log_p-mode;
// uint16_t msg_log_p-thrust;
// int16_t msg_log_p-roll, msg_log_p-pitch, yaw; 
// int16_t msg_log_p-ae[4];
// int16_t msg_log_p-phi, msg_log_p-theta, msg_log_p-psi; 
// int16_t msg_log_p-sp, msg_log_p-sq, msg_log_p-sr; 
// //int16_t msg_log_p-sax, msg_log_p-say, msg_log_p-saz;
// uint16_t msg_log_p-bat_volt;
// int32_t msg_log_p-temperature, pressure;
