/*
 * MavLink.h
 *
 */


#ifndef MAVLINK_H_
#define MAVLINK_H_


#define MAVLINK_CHKSUM_LEN							2  // two bytes
#ifdef MAVLINK_V1
  #define MAVLINK_START_SIGN					0x55
  #define MAVLINK_HDR_LEN						6
#elif defined(MAVLINK_V2)
  #define MAVLINK_START_SIGN					0xFD
  #define MAVLINK_HDR_LEN						10
#endif
#define MAVLINK_MAX_PAYLOAD_LEN				255
#define MAX_MAVLINK_SIZE		(MAVLINK_HDR_LEN+MAVLINK_CHKSUM_LEN+MAVLINK_MAX_PAYLOAD_LEN)
#define MAVLINK_ID_DATA			0x0

typedef struct
{
	uint8_t header;
	uint8_t length; //this describes how long the data payload is
 #ifdef MAVLINK_V2
	uint8_t comp_flags[2]; // compatibility flags
 #endif
	uint8_t sequence;
	uint8_t system_ID;
	uint8_t component_ID;
 #ifdef MAVLINK_V1
	uint8_t message_ID;
 #elif defined(MAVLINK_V2)
	uint8_t message_ID[3];
 #endif
	uint8_t data[MAVLINK_MAX_PAYLOAD_LEN]; //actual size determined by length parameter, 255 is max
	uint8_t checksum[MAVLINK_CHKSUM_LEN];
}MavLinkPacket;

uint32_t MavLink_Total_Bytes_Used(MavLinkPacket *pkt);
uint32_t Compute_Mavlink_Checksum(MavLinkPacket *packet);

#endif /* MAVLINK_H_ */