#include <asf.h>
//#include <stdio.h>
#include <string.h>
//#include <unistd.h>
#include "rf2072_set.h"
#include "it9510.h"
#include "platform_it9517.h"

#include <math.h>
#include <delay.h>
#include "main.h"

extern volatile uint8_t spi_tgt_done;
static int pwr_attn=10000;
static int chsel_2072 = 0;

//function prototyping
extern void spi_tx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);
extern void spi_rx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
		uint32_t rsize, uint32_t ch);

void rffe_write_regs(dev_cfg* pregs) {
	uint16_t tmp, tmpw, *pth = &tmp;
	//RF2072_WRITE:
			while (spi_tgt_done) ; // flush any pending spi xfer
			spi_tgt_done = true;
			ACCESS_PROLOG_2072
			// setup spi to write addressed data
			*pth = 0x7f & pregs->addr; // write access
			spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
			while (spi_tgt_done) ;
			spi_tgt_done = true;
			*pth = 0xff&(pregs->data>>8); // high byte
			spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
			while (spi_tgt_done) ; spi_tgt_done = true;
			*pth = 0xff&pregs->data; // low byte
			spi_tx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/);
			while (spi_tgt_done) ;
		  //delay_us(1);
			pio_set(PIOA, CPLD_2072_TRIG);
}

uint16_t rffe_read_regs(dev_cfg* pregs) {
	uint8_t msg[80];
	dev_access *pr= (dev_access*)msg;
	uint16_t tmp, tmpw, *pth = &tmp;
	//RF2072_READ:
		while (spi_tgt_done) ; // flush any pending spi xfer
		spi_tgt_done = true;
		ACCESS_PROLOG_2072
		*pth = 0x80| (0x7f&pregs->addr); // read access
		spi_tx_transfer(pth, 1, &tmpw, 1, 0);
		while (spi_tgt_done);
		spi_tgt_done = true;
		READ_MID_PROC_2072
		spi_rx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); // high byte
		while (spi_tgt_done) ; spi_tgt_done = true;
		pr->data[1] = 0xff & tmpw;
		spi_rx_transfer(pth, 1, &tmpw, 1, 0/*ctrl/sts*/); // low byte
		while (spi_tgt_done) ;
		pr->data[0] = (0xff & tmpw);
		READ_END_REV_2072
		return *(uint16_t*)pr->data;
}

int set_frequency_rf2072(uint32_t f_lo_KHz)
{
	dev_cfg regs;
	uint16_t writeValue, P2_FREQ1,P2_FREQ2,P2_FREQ3;

	int i;
	uint32_t error;

	uint32_t dw_temp, dw_temp1000, dw_temp2, dw_ndiv1000,dw_fvco_KHz;
	uint8_t n_lo2, lodiv2,fbkdiv2, numlsb2,p2presc2,p2lodiv2;
	uint16_t n2 ,nummsb2 ;


	dw_temp = 5400000;
	dw_temp2 = dw_temp/(f_lo_KHz);
	dw_temp = log2(dw_temp2);
	n_lo2 = (uint8_t)dw_temp;
	p2lodiv2 = n_lo2;
	//lodiv =pow (2.0, n_lo);// 2^n_lo;
	lodiv2 = 1;
	for(i=0;i<n_lo2;i++)
		lodiv2 = lodiv2 * 2;


	dw_fvco_KHz = lodiv2 * f_lo_KHz;

	if(dw_fvco_KHz>3200000){
		fbkdiv2 = 4;

		p2presc2 = 2;

		//pllcpl to 3 to do ???
	}else{
		fbkdiv2 = 2;

		p2presc2 = 1;

	}
	dw_ndiv1000 = (dw_fvco_KHz*10)/fbkdiv2/26;

	n2 =  (uint16_t) (dw_fvco_KHz/fbkdiv2/26000);
	dw_temp1000 = (65536*(dw_ndiv1000-n2*10000));
	nummsb2 = (uint16_t)((65536*(dw_ndiv1000-n2*10000))/10000);
	numlsb2 = (uint8_t)((256*(dw_temp1000-nummsb2*10000))/10000);
	//-------------------------------------------


	P2_FREQ1 = n2<<7 | p2lodiv2<<4 | p2presc2<<2 | 0x02;
	P2_FREQ2 = nummsb2;
	P2_FREQ3 = numlsb2<<8;

	//printf("P2_FREQ1=%x,P2_FREQ2=%x,P2_FREQ3=%x\n",P2_FREQ1,P2_FREQ2,P2_FREQ3);

	regs.addr = 0x08;
	regs.data = (0xFC06 & 0x7FFE) | 0x8000;
	rffe_write_regs(&regs);

	delay_ms(10); 	// validate echo after 0.01 sec
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	regs.addr = 0x0F;
	regs.data = P2_FREQ1;
	rffe_write_regs(&regs);

	delay_ms(10); 	// validate echo after 0.01 sec
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	regs.addr = 0x10;
	regs.data = P2_FREQ2;
	rffe_write_regs(&regs);

	delay_ms(10); 	// validate echo after 0.01 sec
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	//libusb_control_transfer(devh,CTRL_OUT, USB_RQ,USB_HOST_MSG_TX_VAL,USB_HOST_MSG_IDX,(unsigned char*)acs, sizeof(*acs)+(acs->dcnt-1), 0);
	regs.addr = 0x11;
	regs.data = P2_FREQ3;
	rffe_write_regs(&regs);

	delay_ms(10); 	// validate echo after 0.01 sec
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	return 0;
}

int init_rf2072(void)
{
	//RF2072_RESET:
		pio_clear (PIOA, PIO_PA26);
		delay_ms(200);
		pio_set (PIOA, PIO_PA26);
	delay_ms(100);

	dev_cfg regs,*pregs=GET_ARRAY(chsel_2072);
		for (int32_t i=0; i<ARRAY_SIZE(chsel_2072); i++,pregs++)
	//RF2072_WRITE:
			rffe_write_regs(pregs);

	set_frequency_rf2072(LO_Frequency);
	delay_ms(100);

	regs.addr = 0x09;
	regs.data =((0x8224&0xFFF7) | 0x0008);
	rffe_write_regs(&regs);

	delay_ms(10);
	//printf("xxxxxx 0x%04x @ 0x%x written xxxxxx\n",regs.data,regs.addr);

	regs.addr = 0x1D;
	regs.data = 0x1001;
	rffe_write_regs(&regs);

	delay_ms(10);
	//printf("setup device control = 0x%04x\n",*(uint16_t*)acs->data);

	regs.addr = 0x1F;
	return 0!=(0x8000&rffe_read_regs(&regs));

	//printf("tx rffe is running...\n");
}

int init_video_subsystem()
{
	uint16_t bandwidth = 6000;
	uint16_t sawBandwidth = 8000;
	uint32_t error = ModulatorError_NO_ERROR;

	init_rf2072();

	error=it9517_initialize (Bus_I2C,SERIAL_TS_INPUT);
	if(error)goto exit;
	//error= it9517_loadIQ_calibration_table (const char*file_name);
	//if(error)goto exit;
	error = it9517_reset_pidfilter();
	if(error)goto exit;
	error= it9517_control_pidfilter(0,0);
	if(error)goto exit;
//	puts ("video subsystem initialized...");
 exit:
	if (error)  ;//printf("error=%x,%d\n",error,__LINE__);
	return error;
}

int start_video_subsystem()
{
	uint32_t error = ModulatorError_NO_ERROR;
	ChannelModulation      channel_Modulation;
	//	channel_Modulation.frequency=809000;
	//	channel_Modulation.bandwidth=6000;
	channel_Modulation.constellation=Constellation_QPSK;
	channel_Modulation.highCodeRate=CodeRate_1_OVER_2;
	channel_Modulation.interval=Interval_1_OVER_32;
	channel_Modulation.transmissionMode=TransmissionMode_2K;
	error=it9517_set_channel_modulation( channel_Modulation,2);
	if(error)goto exit;
	error=it9517_acquire_channel(809000,6000);
	if(error)goto exit;
	//error=it9517_get_output_gain();
	//if(error)goto exit;
	//error=it9517_get_output_gain_range(809000,6000);
	//if(error)goto exit;
	//error=it9517_adjust_output_gain(0);
	//if(error)goto exit;
	//	error = it9517_reset_pidfilter();
	//	if(error)goto exit;
	//	error= it9517_control_pidfilter(0,1);
	//	if(error)goto exit;
	//error=it9517_add_pidfilter(0, 0x100);
	//if(error)goto exit;
	//	error=it9517_pcr_restamp(PcrModeDisable,1);
	//	if(error)goto exit;

 exit:
	if (error)  ;//printf("error=%x,%d\n",error,__LINE__);
	return error;

}

