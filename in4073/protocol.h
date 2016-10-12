#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define	FALSE 0
#define	TRUE 1
#define PERIODIC_COM 20
#define PANIC_TIME_MS 2000
#define PERIODIC_LINK_S PANIC_TIME_MS/1000

#define HDR 0x99
#define MAX_PAYLOAD 200
#define HDR_FTR_SIZE 5

#define ENCODE
// #define DRONE_PROFILE
// #define PC_PROFILE
#define ENCODE_PC_RECEIVE
// #define PC_DEBUG

enum msg_status {
	UNITINIT,
	GOT_HDR,
	GOT_LEN,
	GOT_ID,
	GOT_PAYLOAD,
	//GOT_CRC1,
	GOT_PACKET
};

enum msg_id{
	MSG_JOYSTICK,
	MSG_KEYBOARD,
	MSG_COMBINE,
	MSG_TELEMETRY,
	MSG_LOG,
	MSG_TUNE,
	MSG_PROFILE
};

// Control
enum control_mode_t {
  MODE_SAFE,
  MODE_PANIC,
  MODE_MANUAL,
  MODE_CALIBRATION,
  MODE_YAW,
  MODE_FULL,
  MODE_RAW,
  MODE_HEIGHT,
  ESCAPE,
  MODE_LOG
};

struct msg_joystick_t{
	bool update;
	uint8_t mode;
	uint16_t thrust;
	int16_t roll;
 	int16_t pitch;
 	int16_t yaw;
}__attribute__((packed));

struct msg_keyboard_t{
 	bool update;
	uint8_t mode;
	uint16_t thrust;
	int16_t roll;
 	int16_t pitch;
 	int16_t yaw;
}__attribute__((packed));

struct msg_combine_t{
 	bool update;
	uint8_t mode;
	uint16_t thrust;
	int16_t roll;
 	int16_t pitch;
 	int16_t yaw;
}__attribute__((packed));

struct msg_tuning_t{
 	bool update;
	uint8_t P;
	uint8_t P1;
	uint8_t P2;
	uint8_t log_flag;
}__attribute__((packed));

// need a bigger struct size
// we also have to include attitude and the rate as well
struct msg_telemetry_t{
	bool update;
	uint8_t mode;
	uint16_t thrust;
	int16_t roll;
 	int16_t pitch;
 	int16_t yaw;
 	int16_t engine[4];
 	int16_t phi, theta, psi; 
  	int16_t sp, sq, sr; 
  	int16_t sax, say, saz;
  	uint16_t bat_volt;
  	uint8_t P;
  	uint8_t P1;
  	uint8_t P2;
}__attribute__((packed));

// struct msg_profile_t{
// 	uint32_t proc_read;
// 	uint32_t proc_adc;
// 	uint32_t proc_send;
// 	uint32_t proc_log;
// 	uint32_t proc_dmp;
// 	uint32_t proc_control;
// }__attribute__((packed));

struct msg_profile_t{
	uint16_t proc_read;
	uint16_t proc_adc;
	uint16_t proc_send;
	uint16_t proc_log;
	uint16_t proc_dmp;
	uint16_t proc_control;
	uint16_t time_all;
}__attribute__((packed));

struct msg_log_t {
  uint16_t index_log;
  uint32_t time_stamp;
  uint8_t mode;
  uint16_t thrust;
  int16_t roll, pitch, yaw; 
  int16_t ae[4];
  int16_t phi, theta, psi; 
  int16_t sp, sq, sr; 
  int16_t sax, say, saz;
  uint16_t bat_volt;
  uint8_t P;
  uint8_t P1;
  uint8_t P2;
  int32_t temperature, pressure;
}__attribute__((packed));

struct msg_p {
	enum msg_status status;
	uint8_t ck1, ck2;
	uint8_t msg_id;
	uint8_t payload_len;
	uint8_t payload_idx;
	uint8_t payload[MAX_PAYLOAD];
	uint8_t crc_fails;
}__attribute__((packed));

void msg_parse(struct msg_p *msg, uint8_t c);
void encode_packet(uint8_t *data, uint8_t len, uint8_t msg_id, uint8_t *output_data, uint8_t *output_size);

#endif  /* #ifndef _PROTOCOL_H_ */