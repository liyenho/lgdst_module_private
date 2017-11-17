#ifndef REEDSOLOMON_H_
#define REEDSOLOMON_H_
																	// reduced parity overhead in order to fit in 1 ctl pkt, liyenho
#define CTRL_MAX_SYMBOL_ERR    /*8*/ 4
#define PARITY_LENGTH    (2*CTRL_MAX_SYMBOL_ERR)
																	// increase effective message length in order to fit in 1 ctl pkt, liyenho
#define CONTROL_MESSAGE_LENGTH   /*16*/ 24


void Encode_Control_Packet(uint8_t *source, uint8_t *destination);

void Decode_Control_Packet(uint8_t *source, uint8_t *destination);


#endif /* REEDSOLOMON_H_ */