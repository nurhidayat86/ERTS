#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define	FALSE 0
#define	TRUE 1

#define HDR 0x99
#define MAX_PAYLOAD 200
#define HDR_FTR_SIZE 5
#define JOYSTICK_TYPE 0x10;
#define KEYBOARD_TYPE 0x20;

enum msg_status {
	UNITINIT,
	GOT_HDR,
	GOT_LEN,
	GOT_ID,
	GOT_PAYLOAD,
	GOT_CRC1,
	GOT_PACKET
};

enum msg_ids {
	MSG_JOYSTICK,
	MSG_KEYBOARD,
	MSG_COMBINE
};

struct msg_joystick_t{
	uint8_t mode;
	uint16_t thrust;
	int16_t roll;
 	int16_t pitch;
 	int16_t yaw;
 	bool update;
}__attribute__((packed, aligned(1)));

struct msg_keyboard_t{
	uint8_t mode;
	uint16_t thrust;
	int16_t roll;
 	int16_t pitch;
 	int16_t yaw;
 	bool update;
}__attribute__((packed, aligned(1)));

struct msg_combine_t{
	uint8_t mode;
	uint16_t thrust;
	int16_t roll;
 	int16_t pitch;
 	int16_t yaw;
 	bool update;
}__attribute__((packed, aligned(1)));

struct msg_p {
	enum msg_status status;
	uint8_t ck1, ck2;
	uint8_t msg_id;
	uint8_t payload_len;
	uint8_t payload_idx;
	uint8_t payload[MAX_PAYLOAD];
	uint8_t crc_fails;
};

void msg_parse(struct msg_p *msg, uint8_t c);
void encode_packet(uint8_t *data, uint8_t len, uint8_t msg_id, uint8_t *output_data, uint8_t *output_size);

#endif  /* #ifndef _PROTOCOL_H_ */