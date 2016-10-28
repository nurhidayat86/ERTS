/*------------------------------------------------------------
 * COMMUNICATION PROTOCOL
 * header, length, message id, data, checksum1, checksum2
 * data: roll pith yaw thrust
 *------------------------------------------------------------
 */

#include "protocol.h"

/*------------------------------------------------------------
 * void encode_packet(uint8_t *data, uint8_t len, uint8_t msg_id, uint8_t *output_data, uint8_t *output_size)
 * Author		: Arif Nurhidayat
 * Adapted from : Freeks van Tien previousely worked in group 13, he gave minor idea and examples. (adapted from MAV Link protocol).
 * Funtionalty	: Encode the data and write the encoded message to output_data variable which will be sent via serial communication.
 * 				  It contains header 1 byte, payload length 1 byte, message id 1 byte, payload up to 200 btes, and CRC 1 byte.
 *------------------------------------------------------------*/

void encode_packet(uint8_t *data, uint8_t len, uint8_t msg_id, uint8_t *output_data, uint8_t *output_size) {
	uint8_t i = 0;
	uint8_t checksum1 = 0;
  	uint8_t checksum2 = 0;

 	// Setting the header
	output_data[0] = (uint8_t)0x99;
	output_data[1] = len;
	checksum1 = checksum2 = len;
	output_data[2] = msg_id;
	checksum1 += msg_id;
	
	//checksum 2 is the sum of the sum of message.
	checksum2 += checksum1;
	
	// Encoding the data
	while (i<len)
	{
		output_data[i+3] = data[i];
		checksum1 += output_data[i+3];
		checksum2 += checksum1;
		i++;
	}
	//Only checksum 2 is used.
	output_data[i+3] = checksum2;

	// Set the output size
	*output_size = len + HDR_FTR_SIZE;
}
/*------------------------------------------------------------*/



/*------------------------------------------------------------
 * void msg_parse(struct msg_p *msg, uint8_t c)
 * Author		: Arif Nurhidayat
 * Adapted from : Freeks van Tien previousely works in group 13, he gave minor idea examples.
 * Funtionalty	: Decode the data, and save it to decoder structure, the payload can be accessed from sub variable (payload[]).
 *------------------------------------------------------------*/
void msg_parse(struct msg_p *msg, uint8_t c) {
	switch (msg->status) {
		//waitng start byte
		case UNITINIT:
			if (c == HDR) {
				msg->status++;
				#ifdef PC_DEBUG 
				printf("got status \n");
				#endif
			}
			break;

		//after the parser reads HDR byte, it waits for <payload length>.
		case GOT_HDR:
			msg->ck1 = msg->ck2 = c;
			msg->payload_len = c;
			msg->status++;
			#ifdef PC_DEBUG
			printf("got length %d \n", msg->ck1);
			#endif
			break;

		//after the parser reads <payload length>, it waits for <message id>.
		case GOT_LEN:
			msg->msg_id = c;
			msg->ck1 += c;
			msg->ck2 += msg->ck1;
			msg->payload_idx = 0;
			msg->status++;
			#ifdef PC_DEBUG
			printf("got ID %d \n", msg->ck1);
			#endif
			break;

		//after the parser reads <message id>, it waits for <payload>, stay in this process until it reach the end of the payload.
		case GOT_ID:
			msg->payload[msg->payload_idx] = c;
			msg->payload_idx++;
			if (c == 0xff)
				msg->ck1 += 255;
			else
				msg->ck1 += c;
			msg->ck2 += msg->ck1;
			if (msg->payload_idx == msg->payload_len) msg->status++;
			#ifdef PC_DEBUG
			printf("got payload %d \n", msg->ck1);
			#endif
			break;

		//after the <payload> is received, check the CRC. if it fails, discard the data.
		case GOT_PAYLOAD:
			if (c != msg->ck2) {
				msg->crc_fails++;
				msg->status = UNITINIT;
				printf("crc2 fail\n");
			}
			else {
				msg->status++;
	 			#ifdef PC_DEBUG
	 			printf("receive success\n");
				#endif 
			}
			break;

		default:
			break;
	}
}
/*------------------------------------------------------------*/



/*------------------------------------------------------------
 * void encode_ack(uint8_t data, uint8_t *output_data, uint8_t *output_size)
 * Author		: Arif Nurhidayat
 * Adapted from : adapted from MAV Link protocol.
 * Funtionalty	: Encode the ack message.
 *------------------------------------------------------------*/
void encode_ack(uint8_t data, uint8_t *output_data, uint8_t *output_size) {
	uint8_t checksum1 = 0;
  	uint8_t checksum2 = 0;

 	// Setting the header
	output_data[0] = (uint8_t)0x99;
	output_data[1] = sizeof(uint8_t);
	checksum1 = checksum2 = sizeof(uint8_t);
	output_data[2] = MSG_ACK;
	checksum1 += MSG_ACK;
	checksum2 += checksum1;
	output_data[3] = data;
	checksum1 += data;
	checksum2 += checksum1;
	output_data[4] = checksum2;
	*output_size = sizeof(uint8_t) + HDR_FTR_SIZE;
}
/*------------------------------------------------------------*/
