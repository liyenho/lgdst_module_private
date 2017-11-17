/*
 * MavLink.c
 *
 */
#include <compiler.h>
#include <string.h>
#include "MavLink.h"

#define X25_INIT_CRC 							0xffff
#define X25_VALIDATE_CRC 				0xf0b8

const uint8_t MAVLINK_MESSAGE_CRCS[] = {50, 124, 137,   0, 237, 217, 104, 119,   0,   0,\
									     0,  89,   0,   0,   0,   0,   0,   0,   0,   0,\
									   214, 159, 220, 168,  24,  23, 170, 144,  67, 115,\
									    39, 246, 185, 104, 237, 244, 222, 212,   9, 254,\
									   230,  28,  28, 132, 221, 232,  11, 153,  41,  39,\
									   214, 223, 141,  33,  15,   3, 100,  24, 239, 238,\
									    30, 200, 183,   0, 130,   0, 148,  21,   0,  52,\
									   124,   0,   0,   0,  20,   0, 152, 143,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0, 231,\
									   183,  63,  54,   0,   0,   0,   0,   0,   0,   0,\
									   175, 102, 158, 208,  56,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,\
									     0,   0,   0,   0,   0,   0,   0,   0,   0, 204,\
									    49, 170,  44,  83,  46,   0};

static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
        //Accumulate one byte of data into the CRC
        uint8_t tmp;
        tmp = data ^ (uint8_t)(*crcAccum &0xff);
        tmp ^= (tmp<<4);
        *crcAccum = (*crcAccum>>8) ^ ((uint16_t)tmp<<8) ^ ((uint16_t)tmp <<3) ^ (tmp>>4);
}


uint32_t Compute_Mavlink_Checksum(MavLinkPacket *packet){

	uint16_t checksum = X25_INIT_CRC;
	//compute checksum, excluding packet start sign
	for (int i =0; i< (packet->length+MAVLINK_HDR_LEN-1); i++){
		crc_accumulate(*(uint8_t *)((&packet->length)+i), &checksum);
	}
	//add CRC Extra per MavLink definition
#ifdef MAVLINK_V1
	int msg_id = (sizeof(MAVLINK_MESSAGE_CRCS)<=packet->message_ID)?0:packet->message_ID;
	crc_accumulate(MAVLINK_MESSAGE_CRCS[msg_id/*protect against msg corruption*/], &checksum);
#elif defined(MAVLINK_V2)
	int msg_id = 0;
	uint8_t *p_id = &msg_id, *p_msg_id= packet->message_ID;
	*p_id++ = *p_msg_id++;
	*p_id++ = *p_msg_id++;
	*p_id = *p_msg_id;
	if (sizeof(MAVLINK_MESSAGE_CRCS)<=msg_id) msg_id=0;
	crc_accumulate(MAVLINK_MESSAGE_CRCS[msg_id/*protect against msg corruption*/], &checksum);
#endif
	return checksum;
}


uint32_t MavLink_Total_Bytes_Used(MavLinkPacket *pkt){
	uint32_t bytes_used = MAVLINK_HDR_LEN + MAVLINK_CHKSUM_LEN + pkt->length;
	return bytes_used;
}
