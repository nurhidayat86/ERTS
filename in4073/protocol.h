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
#define PERIODIC_LINK 1

#define HDR 0x99
#define MAX_PAYLOAD 200
#define HDR_FTR_SIZE 4

#define ENCODE
// #define DRONE_PROFILE
// #define PC_PROFILE
#define ENCODE_PC_RECEIVE
// #define PC_DEBUG
// #define DRONE_DEBUG
// #define ENCODE_DEBUG
// #define SIMULATE_BATTERY

#define HEART_BEAT 0x77

// #define MAX_P 16
#define MAX_P 26
#define MAX_P1 30
//#define MAX_P2 14
#define MAX_P2 26

#define ACK_OK 0x20
#define ACK_FIRED 0x22
#define ACK_RCV 0x24
#define ACK_NOK 0x26
#define ACK_RAW_INIT 0x28
#define ACK_BAT_LOW 0x2A
#define ACK_BAT_LOW_EMERGENCY 0x2C
#define ACK_CON 0x2D
#define ACK_FLASH 0x2E
#define ACK_LOST_COM 0x2F

#define ACK_BAT_LOW_EMER_SAFE 0x10

#define LOG_USE 1
#define LOG_NO_USE 0
#define RAW_USE 3
#define RAW_NO_USE 4

enum msg_status {
	UNITINIT,
	GOT_HDR,
	GOT_LEN,
	GOT_ID,
	GOT_PAYLOAD,
	GOT_PACKET
};

enum msg_id{
	MSG_JOYSTICK,
	MSG_KEYBOARD,
	MSG_COMBINE,
	MSG_TELEMETRY,
	MSG_LOG,
	MSG_TUNE,
	MSG_PROFILE,
	MSG_COMBINE_ALL,
	MSG_ACK
};

// Control
enum control_mode_t {
  MODE_SAFE = 0,
  MODE_PANIC,
  MODE_MANUAL,
  MODE_CALIBRATION,
  MODE_YAW,
  MODE_FULL,
  MODE_RAW,
  MODE_HEIGHT,
  ESCAPE,
  MODE_LOG,
  MODE_START,
  MODE_FINISH
};

struct msg_combine_all_t{
 	bool update;
	uint8_t mode;
	uint16_t thrust;
	int16_t roll;
 	int16_t pitch;
 	int16_t yaw;
	uint8_t P;
	uint8_t P1;
	uint8_t P2;
	uint8_t msc_flag;
}__attribute__((packed));

struct msg_joystick_t{
	bool update;
	uint8_t mode;
	uint16_t thrust;
	int16_t roll;
 	int16_t pitch;
 	int16_t yaw;
	uint8_t P;
	uint8_t P1;
	uint8_t P2;
	uint8_t msc_flag;
}__attribute__((packed));

struct msg_keyboard_t{
 	bool update;
	uint8_t mode;
	uint16_t thrust;
	int16_t roll;
 	int16_t pitch;
 	int16_t yaw;
	uint8_t P;
	uint8_t P1;
	uint8_t P2;
	uint8_t msc_flag;
}__attribute__((packed));

struct msg_combine_all_compact{
 	uint8_t mode;
	uint8_t thrust;
	int8_t roll;
 	int8_t pitch;
 	int8_t yaw;
 	uint8_t P;
	uint8_t P1;
	uint8_t P2;
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
  int16_t esp, esq, esr; 
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
void encode_ack(uint8_t data, uint8_t *output_data, uint8_t *output_size);

#endif  /* #ifndef _PROTOCOL_H_ */