#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define	FALSE 0
#define	TRUE 1
#define PERIODIC_COM 250
#define HDR 0x7a
#define MAX_PAYLOAD 200
#define HDR_FTR_SIZE 5
#define JOYSTICK_TYPE 0x10;
#define KEYBOARD_TYPE 0x20;

// #define ENCODE

//msg_status
#define	UNITINIT 0x10
#define GOT_HDR 0x20
#define GOT_LEN 0x30
#define GOT_ID 0x40
#define GOT_PAYLOAD 0x50
#define GOT_CRC1 0x60
#define GOT_PACKET 0x70


//msg_id
#define	MSG_JOYSTICK 0x15
#define MSG_KEYBOARD 0x25
#define MSG_COMBINE 0x35
#define MSG_TELEMETRY 0x45
#define MSG_LOG 0x55


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

// need a bigger struct size
// we also have to include attitude and the rate as well
struct msg_telemetry_t{
	bool update;
	uint8_t mode;
	uint16_t thrust;
	int16_t roll;
 	int16_t pitch;
 	int16_t yaw;
 	// int16_t engine[4];
}__attribute__((packed));

struct msg_p {
	uint8_t status;
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