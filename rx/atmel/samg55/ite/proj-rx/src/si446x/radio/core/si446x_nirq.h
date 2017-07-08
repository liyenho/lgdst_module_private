/*!
 * Silicon Laboratories Confidential
 * Copyright 2011 Silicon Laboratories, Inc.
 *
 * Public API for the Si446x nIRQ interface.
 */

#ifndef _SI446X_NIRQ_H
#define _SI446X_NIRQ_H
#include "bsp.h"

#ifdef RADIO_SI4463
  extern void si4463_radio_handler(const uint32_t id, const uint32_t index);
#endif

struct packet_handler_pend {
 U8		Si446xWUTPend : 1;
 U8		Si446xLowBattPend  : 1;
 U8		Si446xChipReadyPend  : 1;
 U8		Si446xCmdErrPend  : 1;
 U8		Si446xStateChangePend  : 1;
 U8		Si446xFifoUnderflowOverflowErrorPend : 1;
 U8		Si446xDummyPhPend6  : 1;
 U8		Si446xDummyPhPend7  : 1;
};

typedef union {
	U8                        		Si446xPhPend;
	struct packet_handler_pend        pkhdlr;
} si446x_pckt_hdlr_pend ;

struct modem_handler_pend {
 U8		Si446xSyncDetectPend   : 1;
 U8		Si446xPreambleDetectPend  : 1;
 U8		Si446xInvalidPreamblePend : 1;
 U8		Si446xRssiPend    : 1;
 U8		Si446xRssiJumpPend : 1;
 U8		Si446xInvalidSyncPend  : 1;
 U8		Si446xDummyModemPend6 : 1;
 U8		Si446xDummyModemPend7 : 1;
};

typedef union {
	U8                        		Si446xModemPend;
	struct modem_handler_pend        mdmhdlr;
} si446x_mdm_hdlr_pend ;

struct chip_handler_pend {
 U8		Si446xRxFifoAlmostFullPend  : 1;
 U8		Si446xTxFifoAlmostEmptyPend : 1;
 U8		Si446xCrc16ErrorPend   :  1;
 U8		Si446xCrc32ErrorPend   :  1;
 U8		Si446xPacketRxPend    :  1;
 U8		Si446xPacketSentPend  : 1;
 U8		Si446xFilterMissPend   : 1;
 U8		Si446xFilterMatchPend  : 1;
};

typedef union {
	U8                        		Si446xChipPend;
	struct chip_handler_pend        chiphdlr;
} si446x_chip_hdlr_pend ;
#endif //_SI446X_NIRQ_H
