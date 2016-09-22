/*------------------------------------------------------------
 * COMMUNICATION PROTOCOL
 * header, length, message id, data, checksum1, checksum2
 * data: roll pith yaw thrust
 *------------------------------------------------------------
 */

#include "protocol.h"

//void encode_packet(struct msg_joystick_t *joystick_msg, int16_t c)
void encode_packet(uint8_t *data, uint8_t len, uint8_t msg_id, uint8_t *output_data, uint8_t *output_size) {
	uint8_t i = 0;
	uint8_t checksum1 = 0;
  	uint8_t checksum2 = 0;

 	// Setting the header
	output_data[0] = 0x99;
	output_data[1] = len;
	checksum1 = checksum2 = len;
	output_data[2] = msg_id;
	checksum1 += msg_id;
	checksum2 += checksum1;

	// Encoding the data
	while (i<len)
	{
		output_data[i+3] = data[i];
		checksum1 += output_data[i+3];
		checksum2 += checksum1;
		i++;
	}

	// Setting the checksum
	output_data[i+3] = checksum1;
	output_data[i+4] = checksum2;

	// Set the output size
	*output_size = len + HDR_FTR_SIZE;
}

void msg_parse(struct msg_p *msg, uint8_t c) {
	switch (msg->status) {
		//waitng start byte
		case UNITINIT:
			if (c == HDR) {
				msg->status++;
			}
			break;

		case GOT_HDR:
			msg->ck1 = msg->ck2 = c;
			msg->payload_len = c;
			msg->status++;
			break;

		case GOT_LEN:
			msg->msg_id = c;
			msg->ck1 += c;
			msg->ck2 += msg->ck1;
			msg->payload_idx = 0;
			msg->status++;
			break;

		case GOT_ID:
			msg->payload[msg->payload_idx] = c;
			msg->payload_idx++;
			msg->ck1 += c;
			msg->ck2 += msg->ck1;
			if (msg->payload_idx == msg->payload_len)
				msg->status++;
			break;

		case GOT_PAYLOAD:
			if (c != msg->ck1) {
				msg->crc_fails++;
				msg->status = UNITINIT;
			}
			else {
				msg->status++;
			}
			break;

		case GOT_CRC1:
			if (c != msg->ck2) {
				msg->crc_fails++;
				msg->status = UNITINIT;
			}
			else {
				msg->status++;
			}
			break;

		default:
			break;
	}
}



