#include "logging_protocol.h"

void encode_log(uint8_t *data, enum msg_id_log msg_ID, uint8_t *encodedlog, uint8_t *encodedlog_size)
{
	int i=0;
	encodedlog[0] = 0xe3;
	encodedlog[1] = msg_ID;
	uint8_t CRC1, CRC2, size_encoded;
	switch (msg_ID)
	{
		case T_STAMP:
		{
			size_encoded = sizeof(uint32_t);
			break;
		}
		case MODE:
		{
			size_encoded = sizeof(uint8_t);
			break;
		}
		case THRUST:
		{
			size_encoded = sizeof(uint16_t);
			break;
		}
		case ROLL:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case PITCH:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case YAW:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case AE_0:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case AE_1:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case AE_2:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case AE_3:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case PHI:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case THETA:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case PSI:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case SP:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case SQ:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case SR:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case SAX:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case SAY:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case SAZ:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case BAT_V:
		{
			size_encoded = sizeof(int16_t);
			break;
		}
		case TEMP:
		{
			size_encoded = sizeof(int32_t);
			break;
		}
		case PRESS:
		{
			size_encoded = sizeof(int32_t);
			break;
		}
		case ACK:
		{
			size_encoded = sizeof(int8_t);
			break;
		}
 		default:
 		{
 			size_encoded = sizeof(int8_t);
			break;
 		}
	}
	*encodedlog_size = size_encoded;
	CRC1 = msg_ID + size_encoded;
	CRC2 = CRC1;
	while(i<size_encoded)
	{
		encodedlog[i+2] = data[i];
		CRC1 += data[i];
		CRC2 += CRC1;
		i++;
	}
	encodedlog[i+2] = CRC2;
}

enum msg_ack decode_log(struct msg_p_log *msg_log, uint8_t c) {
	uint8_t size_payload;
	switch (msg_log->status)
	{
		case UNITINIT:
		{
			if(c == 0xe3) {msg_log->status = GOT_HDR; msg_log->ack = PROCESS;}
			break;
		}

		case GOT_HDR:
		{
			if(c == T_STAMP) {msg_log->msg_ID = c; size_payload=sizeof(uint32_t);}
			else if(c == MODE) {msg_log->msg_ID = c; size_payload=sizeof(uint16_t);}
			else if(c == THRUST) {msg_log->msg_ID = c; size_payload=sizeof(uint16_t);}
			else if(c == ROLL) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == PITCH) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == YAW) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == AE_0) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == AE_1) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == AE_2) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == AE_3) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == PHI) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == THETA) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == PSI) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == SP) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == SQ) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == SR) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == SAX) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == SAY) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == SAZ) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == BAT_V) {msg_log->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == TEMP) {msg_log->msg_ID = c; size_payload=sizeof(uint32_t);}
			else if(c == PRESS) {msg_log->msg_ID = c; size_payload=sizeof(uint32_t);}
			else {msg_log->msg_ID = c; size_payload=sizeof(uint8_t);}
			msg_log->payload_len = size_payload;
			msg_log ->payload_idx = 0;
			msg_log->CRC1 = c + msg_log->payload_len;
			msg_log->CRC2 = msg_log ->CRC1;
			msg_log->status = GOT_ID;
			break;
		}

		case GOT_ID:
		{
			msg_log->payload[msg_log->payload_idx] = c;
			msg_log->payload_idx+=1;
			msg_log->CRC1+=c;
			msg_log->CRC2+=msg_log->CRC1;
			if(msg_log->payload_idx == msg_log->payload_len) msg_log->status = GOT_PAYLOAD;
			break;
		}

		case GOT_PAYLOAD:
		{
			if(c != msg_log->CRC2) {msg_log->ack = NOK; msg_log->status = UNITINIT; msg_log->crc_fails+=1;}
			else {msg_log->ack = OK; msg_log->status = GOT_PACKAGE;}
		}

		default:
			break;

	}
	return msg_log->ack;
}
