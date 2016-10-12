// struct log_t {
//   uint16_t index_log;
//   uint32_t time_stamp;
//   uint8_t mode;
//   uint16_t thrust;
//   int16_t roll, pitch, yaw; 
//   int16_t ae[4];
//   int16_t phi, theta, psi; 
//   int16_t sp, sq, sr; 
//   //int16_t sax, say, saz;
//   uint16_t bat_volt;
//   int32_t temperature, pressure;
// }__attribute__((packed, aligned(1)));

// //individual variable
//   uint16_t *MSG_index_log;
//   uint32_t *MSG_time_stamp;
//   uint8_t *MSG_mode;
//   uint16_t *MSG_thrust;
//   int16_t *MSG_roll;
//   int16_t *MSG_pitch;
//   int16_t *MSG_yaw;
//   int16_t *MSG_ae_0;
//   int16_t *MSG_ae_1;
//   int16_t *MSG_ae_2;
//   int16_t *MSG_ae_3;
//   int16_t *MSG_phi;
//   int16_t *MSG_theta;
//   int16_t *MSG_psi;
//   int16_t *MSG_sp;
//   int16_t *MSG_sq;
//   int16_t *MSG_sr;
//   //int16_t *MSG_sax, *MSG_say, *MSG_saz = malloc(sizeof(int16_t));
//   uint16_t *MSG_bat_volt;
//   uint32_t *MSG_temperature;
//   uint32_t *MSG_pressure;
//   uint8_t *MSG_ack;

// //individual variable
//   uint16_t np_MSG_index_log;
//   uint32_t np_MSG_time_stamp;
//   uint8_t np_MSG_mode;
//   uint16_t np_MSG_thrust;
//   int16_t np_MSG_roll;
//   int16_t np_MSG_pitch;
//   int16_t np_MSG_yaw;
//   int16_t np_MSG_ae_0;
//   int16_t np_MSG_ae_1;
//   int16_t np_MSG_ae_2;
//   int16_t np_MSG_ae_3;
//   int16_t np_MSG_phi;
//   int16_t np_MSG_theta;
//   int16_t np_MSG_psi;
//   int16_t np_MSG_sp;
//   int16_t np_MSG_sq;
//   int16_t np_MSG_sr;
//   //int16_t *MSG_sax, *MSG_say, *MSG_saz = malloc(sizeof(int16_t));
//   uint16_t np_MSG_bat_volt;
//   uint32_t np_MSG_temperature;
//   uint32_t np_MSG_pressure;
//   uint8_t np_MSG_ack;

// void initialize_pointer()
// {
//   MSG_index_log = malloc(sizeof(uint16_t));
//   MSG_time_stamp = malloc(sizeof(uint32_t));
//   MSG_mode = malloc(sizeof(uint8_t));
//   MSG_thrust = malloc(sizeof(uint16_t));
//   MSG_roll= malloc(sizeof(int16_t)); 
//   MSG_pitch = malloc(sizeof(int16_t)); 
//   MSG_yaw = malloc(sizeof(int16_t)); 
//   MSG_ae_0 = malloc(sizeof(int16_t));
//   MSG_ae_1 = malloc(sizeof(int16_t));
//   MSG_ae_2 = malloc(sizeof(int16_t));
//   MSG_ae_3 = malloc(sizeof(int16_t));
//   MSG_phi = malloc(sizeof(int16_t));
//   MSG_theta = malloc(sizeof(int16_t));
//   MSG_psi = malloc(sizeof(int16_t));
//   MSG_sp = malloc(sizeof(int16_t));
//   MSG_sq = malloc(sizeof(int16_t));
//   MSG_sr = malloc(sizeof(int16_t));
//   //int16_t *MSG_sax, *MSG_say, *MSG_saz = malloc(sizeof(int16_t));
//   MSG_bat_volt = malloc(sizeof(uint16_t));
//   MSG_temperature = malloc(sizeof(uint32_t));
//   MSG_pressure = malloc(sizeof(uint32_t));
// }

// void flash_np()
// {
//   np_MSG_index_log = 99;
//   np_MSG_time_stamp = 99;
//   np_MSG_mode = 99;
//   np_MSG_thrust = 99;
//   np_MSG_roll = 99;
//   np_MSG_pitch = 99;
//   np_MSG_yaw = 99;
//   np_MSG_ae_0 = 99;
//   np_MSG_ae_1 = 99;
//   np_MSG_ae_2 = 99;
//   np_MSG_ae_3 = 99;
//   np_MSG_phi = 99;
//   np_MSG_theta = 99;
//   np_MSG_psi = 99;
//   np_MSG_sp = 99;
//   np_MSG_sq = 99;
//   np_MSG_sr = 99;
//   //int16_t *MSG_sax, *MSG_say, *MSG_saz = malloc(sizeof(int16_t));
//   np_MSG_bat_volt = 99;
//   np_MSG_temperature = 99;
//   np_MSG_pressure = 99;
//   np_MSG_ack = 99;
// }
// //individual_var
// // uint16_t msg_log_p-index_log;
// // uint32_t msg_log_p-time_stamp;
// // uint8_t msg_log_p-mode;
// // uint16_t msg_log_p-thrust;
// // int16_t msg_log_p-roll, msg_log_p-pitch, yaw; 
// // int16_t msg_log_p-ae[4];
// // int16_t msg_log_p-phi, msg_log_p-theta, msg_log_p-psi; 
// // int16_t msg_log_p-sp, msg_log_p-sq, msg_log_p-sr; 
// // //int16_t msg_log_p-sax, msg_log_p-say, msg_log_p-saz;
// // uint16_t msg_log_p-bat_volt;
// // int32_t msg_log_p-temperature, pressure;
