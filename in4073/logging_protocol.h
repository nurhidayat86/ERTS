
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define PAYLOAD_SIZE 250
#define PAYLOAD_HDRFTR 4


enum msg_status_log {
	UNITINIT,
	GOT_HDR,
	GOT_ID,
	GOT_PAYLOAD,
	GOT_PACKAGE,
	GOT_ACK
};

enum msg_id_log {
	T_STAMP, MODE, THRUST, ROLL, PITCH, YAW,
 	AE_0, AE_1, AE_2, AE_3,
 	PHI, THETA, PSI, SP, SQ, SR, SAX, SAY, SAZ,
 	BAT_V, TEMP, PRESS, ACK
};

enum msg_ack {
	OK, NOK, PROCESS
};

struct msg_p_log {
	enum msg_status_log status;
	enum msg_id_log msg_ID;
	uint8_t payload_idx;
	uint8_t payload_len;
	uint8_t payload[PAYLOAD_SIZE+PAYLOAD_HDRFTR];
	uint8_t CRC1, CRC2;
	uint8_t crc_fails;
	enum msg_ack ack;
};

void encode_log(uint8_t *data, enum msg_id_log msg_ID, uint8_t *encodedlog, uint8_t *encodedlog_size);
enum msg_ack decode_log(struct msg_p_log *msg_log, uint8_t c);
