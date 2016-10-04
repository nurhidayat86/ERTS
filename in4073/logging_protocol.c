#include "logging_protocol.h"

void encode_log(uint8_t *data, uint8_t msg_ID)
{
	int i=0;
	uint8_t CRC1, CRC2, size_encoded;
	CRC2 = CRC1 = 0;
	encodedlog[0] = HDR_LOG;
	encodedlog[1] = msg_ID;
	switch (msg_ID)
	{
		case INDEX_LOG:
		{
			size_encoded = sizeof(uint16_t);
			break;
		}
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
			size_encoded = sizeof(uint16_t);
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
	encodedlog_size = size_encoded + PAYLOAD_HDRFTR_LOG;
	CRC1 = CRC2 = size_encoded;
	while(i<size_encoded)
	{
		encodedlog[i+2] = data[i];
		CRC1 += data[i];
		CRC2 += CRC1;
		i++;
	}
	encodedlog[i+2] = CRC2;
}

uint8_t decode_log(uint8_t c, struct msg_p_log *msg_log_p) {
	uint8_t size_payload;
	switch (msg_log_p->status)
	{
		case UNITINIT:
			if(c == HDR_LOG) {msg_log_p->status = GOT_HDR; msg_log_p->ack = PROCESS; msg_log_p->CRC2 = msg_log_p->CRC1 = 0;}
			break;

		case GOT_HDR:
			if(c == T_STAMP) {msg_log_p->msg_ID = c; size_payload=sizeof(uint32_t);}
			else if(c == INDEX_LOG) {msg_log_p->msg_ID = c; size_payload=sizeof(uint16_t);}
			else if(c == MODE) {msg_log_p->msg_ID = c; size_payload=sizeof(uint8_t);}
			else if(c == THRUST) {msg_log_p->msg_ID = c; size_payload=sizeof(uint16_t);}
			else if(c == ROLL) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == PITCH) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == YAW) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == AE_0) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == AE_1) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == AE_2) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == AE_3) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == PHI) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == THETA) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == PSI) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == SP) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == SQ) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == SR) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == SAX) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == SAY) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == SAZ) {msg_log_p->msg_ID = c; size_payload=sizeof(int16_t);}
			else if(c == BAT_V) {msg_log_p->msg_ID = c; size_payload=sizeof(uint16_t);}
			else if(c == TEMP) {msg_log_p->msg_ID = c; size_payload=sizeof(int32_t);}
			else if(c == PRESS) {msg_log_p->msg_ID = c; size_payload=sizeof(int32_t);}
			else {msg_log_p->msg_ID = c; size_payload=sizeof(uint8_t);}
			msg_log_p->payload_len = size_payload;
			msg_log_p->payload_idx = 0;
			msg_log_p->CRC2 = msg_log_p->CRC1 = size_payload;
			msg_log_p->status = GOT_ID;
			break;

		case GOT_ID:
			msg_log_p->payload[msg_log_p->payload_idx] = c;
			msg_log_p->CRC1+= c;
			msg_log_p->CRC2+=msg_log_p->CRC1;
			msg_log_p->payload_idx+=1;
			if(msg_log_p->payload_idx == msg_log_p->payload_len) msg_log_p->status = GOT_PAYLOAD;
			break;

		case GOT_PAYLOAD:
			if(c != msg_log_p->CRC2) {msg_log_p->ack = NOK; msg_log_p->status = UNITINIT; msg_log_p->crc_fails+=1;}
			else {msg_log_p->ack = OK; msg_log_p->status = GOT_PACKAGE;}
			break;

		default:
			break;

	}
	return msg_log_p->ack;
}

void encode_ack(uint8_t ack_sent)
{
	uint8_t CRC1, CRC2;
	encodedlog[0] = HDR_LOG;
	encodedlog[1] = ACK;
	CRC2 = CRC1 = sizeof(uint8_t);
	encodedlog[2] = ack_sent;
	CRC1 += ack_sent;
	CRC2 += CRC1;
	encodedlog[3] = CRC2;
	encodedlog_size = 1 + PAYLOAD_HDRFTR_LOG;
}

