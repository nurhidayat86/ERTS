
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define HDR_LOG 0x77
#define PAYLOAD_SIZE_LOG 250
#define PAYLOAD_HDRFTR_LOG 3
uint8_t encodedlog[PAYLOAD_SIZE_LOG+PAYLOAD_HDRFTR_LOG];
uint8_t encodedlog_size;


//msg_status_log
#define	UNITINIT 0
#define	GOT_HDR 1
#define	GOT_ID 2
#define	GOT_PAYLOAD 3
#define	GOT_PACKAGE 4
#define	GOT_ACK 5

//msg_id_log {
#define	INDEX_LOG 0x18
#define	T_STAMP 0x21
#define MODE 0x22
#define THRUST 0x24
#define ROLL 0x26
#define PITCH 0x28
#define YAW 0x30
#define AE_0 0x32
#define AE_1 0x34 
#define AE_2 0x36
#define AE_3 0x38
#define PHI 0x40
#define THETA 0x42
#define PSI 0x44
#define SP 0x46
#define SQ 0x48
#define SR 0x50
#define SAX 0x52
#define SAY 0x54
#define SAZ 0x56
#define BAT_V 0x58
#define TEMP 0x60
#define PRESS 0x62 
#define ACK 0x64


//msg_ack
#define OK 0x41
#define NOK 0x43
#define PROCESS 0x45
#define INIT 0x47
#define COMPLETE 0x49

struct msg_p_log {
	uint8_t status;
	uint8_t msg_ID;
	uint8_t payload_idx;
	uint8_t payload_len;
	uint8_t payload[PAYLOAD_SIZE_LOG+PAYLOAD_HDRFTR_LOG];
	uint8_t CRC1, CRC2;
	uint8_t crc_fails;
	uint8_t ack;
};

uint8_t encodedlog[PAYLOAD_SIZE_LOG+PAYLOAD_HDRFTR_LOG];
uint8_t encodedlog_size;

void encode_log(uint8_t *data, uint8_t msg_ID);
uint8_t decode_log(uint8_t c, struct msg_p_log *msg_log_p);
void encode_ack(uint8_t ack_sent);
void flash_np(void);
