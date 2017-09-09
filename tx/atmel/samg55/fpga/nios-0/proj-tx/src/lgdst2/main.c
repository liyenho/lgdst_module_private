/***************************************************************************//**
 *   @file   main.c
 *   @brief  Implementation of Main Function.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

//#define CONSOLE_COMMANDS
//#define ALTERA_PLATFORM  // removed by liyenho
#define __INCLUDE_RF__

//#define __DVB_T_8MHZ__
#define __DVB_T_6MHZ__
//#define __DVB_T_1_7MHZ__
//#define __DVB_T_5MHZ__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "config.h"
#include "stdio.h"
#include "version_num.h"
#include "build_defs.h"
#include "ad9361_api.h"
#include "platform_altera/parameters.h"
#include "platform_altera/platform.h"
//#include "socal/socal.h"
#ifdef NOIS_EXTRACTED
  #include <asf.h>  // for atmel port
#endif
#ifdef CONSOLE_COMMANDS
#include "command.h"
#include "console.h"
#endif
#ifdef XILINX_PLATFORM
#include <xil_cache.h>
#endif
//#if defined XILINX_PLATFORM || defined LINUX_PLATFORM
#include "platform_altera/adc_core.h"
#include "platform_altera/dac_core.h"
//#endif

#include "altera_avalon_spi.h"
#include "altera_avalon_pio_regs.h"

#include <locale.h>

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
#ifdef CONSOLE_COMMANDS
extern command	  	cmd_list[];
extern char			cmd_no;
extern cmd_function	cmd_functions[11];
unsigned char		cmd				 =  0;
double				param[5]		 = {0, 0, 0, 0, 0};
char				param_no		 =  0;
int					cmd_type		 = -1;
char				invalid_cmd		 =  0;
char				received_cmd[30] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#endif

#ifdef __INCLUDE_RF__
//added by sid to get the version number
int num_1, num_2,num_3,num_4, num_5, num_6;
char tmp;

/*struct _reent;
struct lconv *_localeconv_r(struct _reent * r)
{
	return 0;
}*/ // removed by liyenho

/*
For 10 MHz - iclk = 80/7 * 16 = 182.85714 MHz;
For 8 MHz - iclk = 64/7 * 16 = 146.28571 MHz;
For 7 MHz - iclk = 64/8 * 16 = 128.00000 MHz;
For 6 MHz - iclk = 48/7 * 16 = 109.71429 MHz;
For 5 MHz - iclk = 40/7 * 16 = 91.42857 MHz;
For 1.7 MHz - iclk = 131/71 * 16 = 29.52113 MHz.
*/

AD9361_InitParam default_init_param = {
	/* Identification number */
	0,		//id_no;
	/* Reference Clock */
	40000000UL,	//reference_clk_rate
	/* Base Configuration */
	0,		//two_rx_two_tx_mode_enable *** adi,2rx-2tx-mode-enable
	1,		//one_rx_one_tx_mode_use_rx_num *** adi,1rx-1tx-mode-use-rx-num
	1,		//one_rx_one_tx_mode_use_tx_num *** adi,1rx-1tx-mode-use-tx-num
	1,		//frequency_division_duplex_mode_enable *** adi,frequency-division-duplex-mode-enable
	0,		//frequency_division_duplex_independent_mode_enable *** adi,frequency-division-duplex-independent-mode-enable
	0,		//tdd_use_dual_synth_mode_enable *** adi,tdd-use-dual-synth-mode-enable
	0,		//tdd_skip_vco_cal_enable *** adi,tdd-skip-vco-cal-enable
	0,		//tx_fastlock_delay_ns *** adi,tx-fastlock-delay-ns
	0,		//rx_fastlock_delay_ns *** adi,rx-fastlock-delay-ns
	0,		//rx_fastlock_pincontrol_enable *** adi,rx-fastlock-pincontrol-enable
	0,		//tx_fastlock_pincontrol_enable *** adi,tx-fastlock-pincontrol-enable
	0,		//external_rx_lo_enable *** adi,external-rx-lo-enable
	0,		//external_tx_lo_enable *** adi,external-tx-lo-enable
	5,		//dc_offset_tracking_update_event_mask *** adi,dc-offset-tracking-update-event-mask
	6,		//dc_offset_attenuation_high_range *** adi,dc-offset-attenuation-high-range
	5,		//dc_offset_attenuation_low_range *** adi,dc-offset-attenuation-low-range
	0x28,	//dc_offset_count_high_range *** adi,dc-offset-count-high-range
	0x32,	//dc_offset_count_low_range *** adi,dc-offset-count-low-range
	0,		//tdd_use_fdd_vco_tables_enable *** adi,tdd-use-fdd-vco-tables-enable
	0,		//split_gain_table_mode_enable *** adi,split-gain-table-mode-enable
	MAX_SYNTH_FREF,	//trx_synthesizer_target_fref_overwrite_hz *** adi,trx-synthesizer-target-fref-overwrite-hz
	0,		// qec_tracking_slow_mode_enable *** adi,qec-tracking-slow-mode-enable
	/* ENSM Control */
	0,		//ensm_enable_pin_pulse_mode_enable *** adi,ensm-enable-pin-pulse-mode-enable
	0,		//ensm_enable_txnrx_control_enable *** adi,ensm-enable-txnrx-control-enable
	/* LO Control */
	2400000000UL,	//rx_synthesizer_frequency_hz *** adi,rx-synthesizer-frequency-hz
	2400000000UL,	//tx_synthesizer_frequency_hz *** adi,tx-synthesizer-frequency-hz
	/* Rate & BW Control */
	//{983040000, 245760000, 122880000, 61440000, 30720000, 30720000},//uint32_t	rx_path_clock_frequencies[6] *** adi,rx-path-clock-frequencies
	//{983040000, 122880000, 122880000, 61440000, 30720000, 30720000},//uint32_t	tx_path_clock_frequencies[6] *** adi,tx-path-clock-frequencies

#ifdef __DVB_T_8MHZ__
	{1170285728, 146285716, 146285716, 73142858, 36571429, 36571429}, //uint32_t	rx_path_clock_frequencies[6]for 32.57 Msps, 8MHz bandwidth
	{1170285728, 146285716, 146285716, 73142858, 36571429, 36571429}, //uint32_t	tx_path_clock_frequencies[6] *** adi,tx-path-clock-frequencies
	64.0/7 * 1000000,//rf_rx_bandwidth_hz *** adi,rf-rx-bandwidth-hz
	64.0/7 * 1000000,//rf_tx_bandwidth_hz *** adi,rf-tx-bandwidth-hz
#endif

	//{731428576, 91428572, 91428572, 45714286, 22857143, 22857143}, //for 22.85 Msps, 5MHz bandwidth

#ifdef __DVB_T_6MHZ__
	{877714272, 109714284, 109714284, 27428571, 27428571, 27428571}, //for 27.42 Msps, 6MHz bandwidth
	{877714272, 109714284, 109714284, 27428571, 27428571, 27428571},
	48.0/7 * 1000000,
	48.0/7 * 1000000,
#endif

#ifdef __DVB_T_5MHZ__
	{731428576, 91428572, 45714286, 22857143, 22857143, 22857143}, //for 22.86 Msps, 5MHz bandwidth
	{731428576, 91428572, 45714286, 22857143, 22857143, 22857143},
	33620663,
	33620663,
#endif

#ifdef __DVB_T_1_7MHZ__
	{944676160, 59042260, 29521130, 14760565, 14760565, 14760565}, //for 14.76 Msps, 1.7MHz bandwidth
	{944676160, 59042260, 29521130, 14760565, 14760565, 14760565},
	131/71 * 1000000,
	131/71 * 1000000,
#endif

	//{1170285728, 292571432, 146285716, 73142858, 36571429, 18285713},
	//{983040000, 122880000, 122880000, 61440000, 30720000, 30720000},//uint32_t	tx_path_clock_frequencies[6] *** adi,tx-path-clock-frequencies
	//{(96 * 64.0/7 * 1000000), (8 * 64.0/7 * 1000000), (8 * 64.0/7 * 1000000), (4 * 64.0/7 * 1000000), (2*64.0/7 * 1000000), (2*64.0/7 * 1000000)},//uint32_t	tx_path_clock_frequencies[6] *** adi,tx-path-clock-frequencies
	//{1170285728, 146285716, 146285716, 73142858, 36571429, 36571429},//uint32_t	tx_path_clock_frequencies[6] *** adi,tx-path-clock-frequencies
	//{731428576, 91428572, 91428572, 45714286, 22857143, 22857143},
	//{877714272, 109714284, 109714284, 27428571, 27428571, 27428571},
	//{1170285696, 292571424, 146285712, 73142856, 36571428, 18285714},
	//{1170285728, 292571432, 146285716, 73142858, 36571429, 18285713},
	//18000000,//rf_rx_bandwidth_hz *** adi,rf-rx-bandwidth-hz
	//64.0/7 * 1000000,//rf_rx_bandwidth_hz *** adi,rf-rx-bandwidth-hz
	//40.0/7 * 1000000,
	//18000000,//rf_tx_bandwidth_hz *** adi,rf-tx-bandwidth-hz
	//64.0/7 * 1000000,//rf_tx_bandwidth_hz *** adi,rf-tx-bandwidth-hz
	//40.0/7 * 1000000,
	/* RF Port Control */
	0,		//rx_rf_port_input_select *** adi,rx-rf-port-input-select
	0,		//tx_rf_port_input_select *** adi,tx-rf-port-input-select
	/* TX Attenuation Control */
	10000,	//tx_attenuation_mdB *** adi,tx-attenuation-mdB
	0,		//update_tx_gain_in_alert_enable *** adi,update-tx-gain-in-alert-enable
	/* Reference Clock Control */
	0,		//xo_disable_use_ext_refclk_enable *** adi,xo-disable-use-ext-refclk-enable
	{8, 5920},	//dcxo_coarse_and_fine_tune[2] *** adi,dcxo-coarse-and-fine-tune
	CLKOUT_DISABLE,	//clk_output_mode_select *** adi,clk-output-mode-select
	/* Gain Control */
	2,		//gc_rx1_mode *** adi,gc-rx1-mode
	2,		//gc_rx2_mode *** adi,gc-rx2-mode
	58,		//gc_adc_large_overload_thresh *** adi,gc-adc-large-overload-thresh
	4,		//gc_adc_ovr_sample_size *** adi,gc-adc-ovr-sample-size
	47,		//gc_adc_small_overload_thresh *** adi,gc-adc-small-overload-thresh
	8192,	//gc_dec_pow_measurement_duration *** adi,gc-dec-pow-measurement-duration
	0,		//gc_dig_gain_enable *** adi,gc-dig-gain-enable
	800,	//gc_lmt_overload_high_thresh *** adi,gc-lmt-overload-high-thresh
	704,	//gc_lmt_overload_low_thresh *** adi,gc-lmt-overload-low-thresh
	24,		//gc_low_power_thresh *** adi,gc-low-power-thresh
	15,		//gc_max_dig_gain *** adi,gc-max-dig-gain
	/* Gain MGC Control */
	2,		//mgc_dec_gain_step *** adi,mgc-dec-gain-step
	2,		//mgc_inc_gain_step *** adi,mgc-inc-gain-step
	0,		//mgc_rx1_ctrl_inp_enable *** adi,mgc-rx1-ctrl-inp-enable
	0,		//mgc_rx2_ctrl_inp_enable *** adi,mgc-rx2-ctrl-inp-enable
	0,		//mgc_split_table_ctrl_inp_gain_mode *** adi,mgc-split-table-ctrl-inp-gain-mode
	/* Gain AGC Control */
	10,		//agc_adc_large_overload_exceed_counter *** adi,agc-adc-large-overload-exceed-counter
	2,		//agc_adc_large_overload_inc_steps *** adi,agc-adc-large-overload-inc-steps
	0,		//agc_adc_lmt_small_overload_prevent_gain_inc_enable *** adi,agc-adc-lmt-small-overload-prevent-gain-inc-enable
	10,		//agc_adc_small_overload_exceed_counter *** adi,agc-adc-small-overload-exceed-counter
	4,		//agc_dig_gain_step_size *** adi,agc-dig-gain-step-size
	3,		//agc_dig_saturation_exceed_counter *** adi,agc-dig-saturation-exceed-counter
	1000,	// agc_gain_update_interval_us *** adi,agc-gain-update-interval-us
	0,		//agc_immed_gain_change_if_large_adc_overload_enable *** adi,agc-immed-gain-change-if-large-adc-overload-enable
	0,		//agc_immed_gain_change_if_large_lmt_overload_enable *** adi,agc-immed-gain-change-if-large-lmt-overload-enable
	10,		//agc_inner_thresh_high *** adi,agc-inner-thresh-high
	1,		//agc_inner_thresh_high_dec_steps *** adi,agc-inner-thresh-high-dec-steps
	12,		//agc_inner_thresh_low *** adi,agc-inner-thresh-low
	1,		//agc_inner_thresh_low_inc_steps *** adi,agc-inner-thresh-low-inc-steps
	10,		//agc_lmt_overload_large_exceed_counter *** adi,agc-lmt-overload-large-exceed-counter
	2,		//agc_lmt_overload_large_inc_steps *** adi,agc-lmt-overload-large-inc-steps
	10,		//agc_lmt_overload_small_exceed_counter *** adi,agc-lmt-overload-small-exceed-counter
	5,		//agc_outer_thresh_high *** adi,agc-outer-thresh-high
	2,		//agc_outer_thresh_high_dec_steps *** adi,agc-outer-thresh-high-dec-steps
	18,		//agc_outer_thresh_low *** adi,agc-outer-thresh-low
	2,		//agc_outer_thresh_low_inc_steps *** adi,agc-outer-thresh-low-inc-steps
	1,		//agc_attack_delay_extra_margin_us; *** adi,agc-attack-delay-extra-margin-us
	0,		//agc_sync_for_gain_counter_enable *** adi,agc-sync-for-gain-counter-enable
	/* Fast AGC */
	64,		//fagc_dec_pow_measuremnt_duration ***  adi,fagc-dec-pow-measurement-duration
	260,	//fagc_state_wait_time_ns ***  adi,fagc-state-wait-time-ns
	/* Fast AGC - Low Power */
	0,		//fagc_allow_agc_gain_increase ***  adi,fagc-allow-agc-gain-increase-enable
	5,		//fagc_lp_thresh_increment_time ***  adi,fagc-lp-thresh-increment-time
	1,		//fagc_lp_thresh_increment_steps ***  adi,fagc-lp-thresh-increment-steps
	/* Fast AGC - Lock Level */
	10,		//fagc_lock_level ***  adi,fagc-lock-level
	1,		//fagc_lock_level_lmt_gain_increase_en ***  adi,fagc-lock-level-lmt-gain-increase-enable
	5,		//fagc_lock_level_gain_increase_upper_limit ***  adi,fagc-lock-level-gain-increase-upper-limit
	/* Fast AGC - Peak Detectors and Final Settling */
	1,		//fagc_lpf_final_settling_steps ***  adi,fagc-lpf-final-settling-steps
	1,		//fagc_lmt_final_settling_steps ***  adi,fagc-lmt-final-settling-steps
	3,		//fagc_final_overrange_count ***  adi,fagc-final-overrange-count
	/* Fast AGC - Final Power Test */
	0,		//fagc_gain_increase_after_gain_lock_en ***  adi,fagc-gain-increase-after-gain-lock-enable
	/* Fast AGC - Unlocking the Gain */
	0,		//fagc_gain_index_type_after_exit_rx_mode ***  adi,fagc-gain-index-type-after-exit-rx-mode
	1,		//fagc_use_last_lock_level_for_set_gain_en ***  adi,fagc-use-last-lock-level-for-set-gain-enable
	1,		//fagc_rst_gla_stronger_sig_thresh_exceeded_en ***  adi,fagc-rst-gla-stronger-sig-thresh-exceeded-enable
	5,		//fagc_optimized_gain_offset ***  adi,fagc-optimized-gain-offset
	10,		//fagc_rst_gla_stronger_sig_thresh_above_ll ***  adi,fagc-rst-gla-stronger-sig-thresh-above-ll
	1,		//fagc_rst_gla_engergy_lost_sig_thresh_exceeded_en ***  adi,fagc-rst-gla-engergy-lost-sig-thresh-exceeded-enable
	1,		//fagc_rst_gla_engergy_lost_goto_optim_gain_en ***  adi,fagc-rst-gla-engergy-lost-goto-optim-gain-enable
	10,		//fagc_rst_gla_engergy_lost_sig_thresh_below_ll ***  adi,fagc-rst-gla-engergy-lost-sig-thresh-below-ll
	8,		//fagc_energy_lost_stronger_sig_gain_lock_exit_cnt ***  adi,fagc-energy-lost-stronger-sig-gain-lock-exit-cnt
	1,		//fagc_rst_gla_large_adc_overload_en ***  adi,fagc-rst-gla-large-adc-overload-enable
	1,		//fagc_rst_gla_large_lmt_overload_en ***  adi,fagc-rst-gla-large-lmt-overload-enable
	0,		//fagc_rst_gla_en_agc_pulled_high_en ***  adi,fagc-rst-gla-en-agc-pulled-high-enable
	0,		//fagc_rst_gla_if_en_agc_pulled_high_mode ***  adi,fagc-rst-gla-if-en-agc-pulled-high-mode
	64,		//fagc_power_measurement_duration_in_state5 ***  adi,fagc-power-measurement-duration-in-state5
	/* RSSI Control */
	1,		//rssi_delay *** adi,rssi-delay
	1000,	//rssi_duration *** adi,rssi-duration
	3,		//rssi_restart_mode *** adi,rssi-restart-mode
	0,		//rssi_unit_is_rx_samples_enable *** adi,rssi-unit-is-rx-samples-enable
	1,		//rssi_wait *** adi,rssi-wait
	/* Aux ADC Control */
	256,	//aux_adc_decimation *** adi,aux-adc-decimation
	40000000UL,	//aux_adc_rate *** adi,aux-adc-rate
	/* AuxDAC Control */
	1,		//aux_dac_manual_mode_enable ***  adi,aux-dac-manual-mode-enable
	0,		//aux_dac1_default_value_mV ***  adi,aux-dac1-default-value-mV
	0,		//aux_dac1_active_in_rx_enable ***  adi,aux-dac1-active-in-rx-enable
	0,		//aux_dac1_active_in_tx_enable ***  adi,aux-dac1-active-in-tx-enable
	0,		//aux_dac1_active_in_alert_enable ***  adi,aux-dac1-active-in-alert-enable
	0,		//aux_dac1_rx_delay_us ***  adi,aux-dac1-rx-delay-us
	0,		//aux_dac1_tx_delay_us ***  adi,aux-dac1-tx-delay-us
	0,		//aux_dac2_default_value_mV ***  adi,aux-dac2-default-value-mV
	0,		//aux_dac2_active_in_rx_enable ***  adi,aux-dac2-active-in-rx-enable
	0,		//aux_dac2_active_in_tx_enable ***  adi,aux-dac2-active-in-tx-enable
	0,		//aux_dac2_active_in_alert_enable ***  adi,aux-dac2-active-in-alert-enable
	0,		//aux_dac2_rx_delay_us ***  adi,aux-dac2-rx-delay-us
	0,		//aux_dac2_tx_delay_us ***  adi,aux-dac2-tx-delay-us
	/* Temperature Sensor Control */
	256,	//temp_sense_decimation *** adi,temp-sense-decimation
	1000,	//temp_sense_measurement_interval_ms *** adi,temp-sense-measurement-interval-ms
	0xCE,	//temp_sense_offset_signed *** adi,temp-sense-offset-signed
	1,		//temp_sense_periodic_measurement_enable *** adi,temp-sense-periodic-measurement-enable
	/* Control Out Setup */
	0xFF,	//ctrl_outs_enable_mask *** adi,ctrl-outs-enable-mask
	0,		//ctrl_outs_index *** adi,ctrl-outs-index
	/* External LNA Control */
	0,		//elna_settling_delay_ns *** adi,elna-settling-delay-ns
	0,		//elna_gain_mdB *** adi,elna-gain-mdB
	0,		//elna_bypass_loss_mdB *** adi,elna-bypass-loss-mdB
	0,		//elna_rx1_gpo0_control_enable *** adi,elna-rx1-gpo0-control-enable
	0,		//elna_rx2_gpo1_control_enable *** adi,elna-rx2-gpo1-control-enable
	0,		//elna_gaintable_all_index_enable *** adi,elna-gaintable-all-index-enable
	/* Digital Interface Control */
	2,		//digital_interface_tune_skip_mode *** adi,digital-interface-tune-skip-mode, 2 = skip both RX & TX tuning
	0,		//digital_interface_tune_fir_disable *** adi,digital-interface-tune-fir-disable
	1,		//pp_tx_swap_enable *** adi,pp-tx-swap-enable
	1,		//pp_rx_swap_enable *** adi,pp-rx-swap-enable
	0,		//tx_channel_swap_enable *** adi,tx-channel-swap-enable
	0,		//rx_channel_swap_enable *** adi,rx-channel-swap-enable
	1,		//rx_frame_pulse_mode_enable *** adi,rx-frame-pulse-mode-enable
	0,		//two_t_two_r_timing_enable *** adi,2t2r-timing-enable
	0,		//invert_data_bus_enable *** adi,invert-data-bus-enable
	0,		//invert_data_clk_enable *** adi,invert-data-clk-enable
	0,		//fdd_alt_word_order_enable *** adi,fdd-alt-word-order-enable
	0,		//invert_rx_frame_enable *** adi,invert-rx-frame-enable
	0,		//fdd_rx_rate_2tx_enable *** adi,fdd-rx-rate-2tx-enable
	0,		//swap_ports_enable *** adi,swap-ports-enable
	0,		//single_data_rate_enable *** adi,single-data-rate-enable
	1,		//lvds_mode_enable *** adi,lvds-mode-enable
	0,		//half_duplex_mode_enable *** adi,half-duplex-mode-enable
	0,		//single_port_mode_enable *** adi,single-port-mode-enable
	0,		//full_port_enable *** adi,full-port-enable
	0,		//full_duplex_swap_bits_enable *** adi,full-duplex-swap-bits-enable
	0,		//delay_rx_data *** adi,delay-rx-data
	0,		//rx_data_clock_delay *** adi,rx-data-clock-delay
	4,		//rx_data_delay *** adi,rx-data-delay
	0,		//tx_fb_clock_delay *** adi,tx-fb-clock-delay
	7,		//tx_data_delay *** adi,tx-data-delay
	150,	//lvds_bias_mV *** adi,lvds-bias-mV
	1,		//lvds_rx_onchip_termination_enable *** adi,lvds-rx-onchip-termination-enable
	0,		//rx1rx2_phase_inversion_en *** adi,rx1-rx2-phase-inversion-enable
	0xFF,	//lvds_invert1_control *** adi,lvds-invert1-control
	0x0F,	//lvds_invert2_control *** adi,lvds-invert2-control
	/* GPO Control */
	0,		//gpo0_inactive_state_high_enable *** adi,gpo0-inactive-state-high-enable
	0,		//gpo1_inactive_state_high_enable *** adi,gpo1-inactive-state-high-enable
	0,		//gpo2_inactive_state_high_enable *** adi,gpo2-inactive-state-high-enable
	0,		//gpo3_inactive_state_high_enable *** adi,gpo3-inactive-state-high-enable
	0,		//gpo0_slave_rx_enable *** adi,gpo0-slave-rx-enable
	0,		//gpo0_slave_tx_enable *** adi,gpo0-slave-tx-enable
	0,		//gpo1_slave_rx_enable *** adi,gpo1-slave-rx-enable
	0,		//gpo1_slave_tx_enable *** adi,gpo1-slave-tx-enable
	0,		//gpo2_slave_rx_enable *** adi,gpo2-slave-rx-enable
	0,		//gpo2_slave_tx_enable *** adi,gpo2-slave-tx-enable
	0,		//gpo3_slave_rx_enable *** adi,gpo3-slave-rx-enable
	0,		//gpo3_slave_tx_enable *** adi,gpo3-slave-tx-enable
	0,		//gpo0_rx_delay_us *** adi,gpo0-rx-delay-us
	0,		//gpo0_tx_delay_us *** adi,gpo0-tx-delay-us
	0,		//gpo1_rx_delay_us *** adi,gpo1-rx-delay-us
	0,		//gpo1_tx_delay_us *** adi,gpo1-tx-delay-us
	0,		//gpo2_rx_delay_us *** adi,gpo2-rx-delay-us
	0,		//gpo2_tx_delay_us *** adi,gpo2-tx-delay-us
	0,		//gpo3_rx_delay_us *** adi,gpo3-rx-delay-us
	0,		//gpo3_tx_delay_us *** adi,gpo3-tx-delay-us
	/* Tx Monitor Control */
	37000,	//low_high_gain_threshold_mdB *** adi,txmon-low-high-thresh
	0,		//low_gain_dB *** adi,txmon-low-gain
	24,		//high_gain_dB *** adi,txmon-high-gain
	0,		//tx_mon_track_en *** adi,txmon-dc-tracking-enable
	0,		//one_shot_mode_en *** adi,txmon-one-shot-mode-enable
	511,	//tx_mon_delay *** adi,txmon-delay
	8192,	//tx_mon_duration *** adi,txmon-duration
	2,		//tx1_mon_front_end_gain *** adi,txmon-1-front-end-gain
	2,		//tx2_mon_front_end_gain *** adi,txmon-2-front-end-gain
	48,		//tx1_mon_lo_cm *** adi,txmon-1-lo-cm
	48,		//tx2_mon_lo_cm *** adi,txmon-2-lo-cm
	/* GPIO definitions */
	-1,		//gpio_resetb *** reset-gpios
	/* MCS Sync */
	-1,		//gpio_sync *** sync-gpios
	-1,		//gpio_cal_sw1 *** cal-sw1-gpios
	-1,		//gpio_cal_sw2 *** cal-sw2-gpios
	/* External LO clocks */
	NULL,	//(*ad9361_rfpll_ext_recalc_rate)()
	NULL,	//(*ad9361_rfpll_ext_round_rate)()
	NULL	//(*ad9361_rfpll_ext_set_rate)()
};

/*
AD9361_RXFIRConfig rx_fir_config = {	// BPF PASSBAND 3/20 fs to 1/4 fs
	3, // rx
	-6, // rx_gain
	1, // rx_dec

//	{-4, -6, -37, 35, 186, 86, -284, -315,
//	 107, 219, -4, 271, 558, -307, -1182, -356,
//	 658, 157, 207, 1648, 790, -2525, -2553, 748,
//	 865, -476, 3737, 6560, -3583, -14731, -5278, 14819,
//	 14819, -5278, -14731, -3583, 6560, 3737, -476, 865,
//	 748, -2553, -2525, 790, 1648, 207, 157, 658,
//	 -356, -1182, -307, 558, 271, -4, 219, 107,
//	 -315, -284, 86, 186, 35, -37, -6, -4,
//	 0, 0, 0, 0, 0, 0, 0, 0,
//	 0, 0, 0, 0, 0, 0, 0, 0,
//	 0, 0, 0, 0, 0, 0, 0, 0,
//	 0, 0, 0, 0, 0, 0, 0, 0,
//	 0, 0, 0, 0, 0, 0, 0, 0,
//	 0, 0, 0, 0, 0, 0, 0, 0,
//	 0, 0, 0, 0, 0, 0, 0, 0,
//	 0, 0, 0, 0, 0, 0, 0, 0}, // rx_coef[128]
	 {667,-577,720,-841,914,-914,812,-577,175,431,-1287,2464,-3984,5996,-9416,22051,22256,-8727,5745,-3926,2499,-1371,537,64,-472,719,-836,852,-794,687,-556,650,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	 //{28,160,-52,-136,334,-326,-46,659,-1063,715,571,-2234,3064,-1520,-4115,19428,20802,-2722,-1820,2978,-2078,456,756,-1046,619,-14,-336,327,-124,-60,161,33,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	 32, // rx_coef_size
	 {0, 0, 0, 0, 0, 0}, //rx_path_clks[6]
	 0 // rx_bandwidth
};
*/

AD9361_TXFIRConfig tx_fir_config = {	// BPF PASSBAND 3/20 fs to 1/4 fs
	3, // tx
	0, // tx_gain
	1, // tx_int

#ifdef __DVB_T_8MHZ__
	//for 8MHz bandwidth
	{-12,-83,-244,-438,-457,-67,695,1300,953,-686,-2771,-3348,-542,5805,13416,18623,18648,13478,5877,-486,-3321,-2770,-697,943,1297,697,-63,-454,-437,-244,-83,-12,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, // tx_coef[128];
#endif

	//for 5MHz bandwidth
	//{-71,-184,-71,523,1184,893,-568,-1546,-194,2357,2193,-2143,-5546,-591,12924,25070,25079,12945,-572,-5537,-2144,2189,2356,-193,-1545,-568,892,1183,523,-71,-183,-71,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
#ifdef __DVB_T_6MHZ__
	//for 6MHz bandwidth
	{-2,-54,-196,-398,-462,-132,610,1272,1026,-555,-2693,-3399,-685,5707,13473,18805,18805,13473,5707,-685,-3399,-2693,-555,1026,1272,610,-132,-462,-398,-196,-54,-2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
#endif

#ifdef __DVB_T_5MHZ__
	//for 6MHz bandwidth
	{-71,-184,-71,523,1184,893,-568,-1546,-194,2357,2193,-2143,-5546,-591,12924,25070,25079,12945,-572,-5537,-2144,2189,2356,-193,-1545,-568,892,1183,523,-71,-183,-71,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
#endif

#ifdef __DVB_T_1_7MHZ__
	//for 1.7MHz bandwidth
	{-10,-12,58,209,228,-161,-718,-535,820,1954,588,-3055,-4506,1215,13181,23322,23437,13440,1438,-4428,-3093,533,1942,841,-517,-718,-170,224,210,60,-11,-9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
#endif

	32, // tx_coef_size
	 {0, 0, 0, 0, 0, 0}, // tx_path_clks[6]
	 0 // tx_bandwidth
};
struct ad9361_rf_phy *ad9361_phy;
#ifdef FMCOMMS5
struct ad9361_rf_phy *ad9361_phy_b;
#endif

#define REG_MAP_BASEADDR 0x00054000

//bytes 64-67
#define REG_LOC_DVB_T_OUT_SELECT (31) //in big-endian convention, must be corrected for Nios Little Endianess
#define REG_LOC_CHASSIS_CTRL_INH (23)
#define REG_LOC_RF_REG_CHANGE_REQ (22)

//bytes 68-71
#define REG_LOC_TX_POWER_ON  (31)
#define REG_LOC_TX_ATTEN_SEL (28) //LSB
#define REG_LOC_TX_CHAN_SEL  (24) //LSB

//bytes 72-73
#define REG_LOC_TX_MIN_ATTEN (8) //byte loc

//reg page 4
#define REG_LOC_RF_CAL_DONE (0)

#define REG_LOC_CHAN_FREQ_OFF_BASE 80

#endif //__INCLUDE_RF__

#define MB_REG_PAGE_SIZE 256
#define MB_RF_CONF_PAGE 0
#define MB_RF_STAT_PAGE 4
#define MB_RF_CONF_OFFSET 64
#define MB_SYS_STAT_OFFSET 256
#define MB_RF_STAT_OFFSET (MB_RF_STAT_PAGE * MB_REG_PAGE_SIZE)  //relative to the base address

#define REG_MAP_mReadReg(BaseAddress, RegOffset) \
		IORD_ALTERA_AVALON_PIO_DATA((BaseAddress) + (RegOffset))
		//alt_read_word((BaseAddress) + (RegOffset))

#define REG_MAP_mWriteReg(BaseAddress, RegOffset, Data) \
		IOWR_ALTERA_AVALON_PIO_DATA((BaseAddress) + (RegOffset), (uint32_t)(Data))
		//alt_write_word((BaseAddress) + (RegOffset), (uint32_t)(Data))

#define SW_VERSION "1.0.0"

#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))
#ifndef NOIS_EXTRACTED
//read register and convert the value to big endian
uint32_t read_reg_bige(uint32_t BaseAddress, uint32_t RegOffset)
{
	uint32_t read_val = REG_MAP_mReadReg(BaseAddress, RegOffset);
	//return SWAP_UINT32(read_val);
	return read_val;
}

//convert data to big endian before writing
void write_reg_bige(uint32_t BaseAddress, uint32_t RegOffset, uint32_t Data)
{
	//REG_MAP_mWriteReg(BaseAddress, RegOffset, SWAP_UINT32(Data));
	REG_MAP_mWriteReg(BaseAddress, RegOffset, Data);
}
#else
#include "main.h"
extern volatile uint8_t spi_tgt_done ; // from atmel main.c
extern Pdc *g_p_spim_pdc [1+2+1]/*fpga/sms, video, radio ctrl/sts, dvbt-rf (ad9364), liyenho*/;
extern uint32_t gs_ul_spi_clock[2+1+1] ;
extern void spi_master_initialize(uint32_t ch, uint32_t base, uint32_t fxcom);
extern void spi_tx_transfer(void *p_tbuf, uint32_t tsize, void *p_rbuf,
			uint32_t rsize, uint32_t ch);
#ifdef  SPI0_FOR_SPI5
static void spi0_master_initialize()
{
//	puts("-I- Initialize SPI as master\r");
	pio_set(PIOA, PIO_PA24);  // setup spi0 in place of spi5
	/* Get pointer to SPI master PDC register base */
	g_p_spim_pdc [3] = spi_get_pdc_base(SPI0_MASTER_BASE);
#if (SAMG55)
	/* Enable the peripheral and set SPI mode. */
	flexcom_enable(BOARD_FLEXCOM_SPI0);
	flexcom_set_opmode(BOARD_FLEXCOM_SPI0, FLEXCOM_SPI);
#else
	/* Configure an SPI peripheral. */
	pmc_enable_periph_clk(SPI_ID);
#endif
	spi_disable(SPI0_MASTER_BASE);
	spi_reset(SPI0_MASTER_BASE);
	spi_set_lastxfer(SPI0_MASTER_BASE);
	spi_set_master_mode(SPI0_MASTER_BASE);
	spi_disable_mode_fault_detect(SPI0_MASTER_BASE);
	spi_set_peripheral_chip_select_value(SPI0_MASTER_BASE, 0);
	/*ctrl to ad9364*/ {
	  spi_set_clock_polarity(SPI0_MASTER_BASE, 0, 0/*clk idle state is low*/);
	  spi_set_clock_phase(SPI0_MASTER_BASE, 0, 0/*transit @ rising, captured @ falling*/); }
	/*ctrl to ad9364*/
	  spi_set_bits_per_transfer(SPI0_MASTER_BASE, 0,
			SPI_CSR_BITS_8_BIT);  // 8 bit spi xfer, liyenho
	spi_set_baudrate_div(SPI0_MASTER_BASE, 0,
			(sysclk_get_cpu_hz() / gs_ul_spi_clock [3]));
	/*ctrl to ad9364*/
	  spi_set_transfer_delay(SPI0_MASTER_BASE, 0,
	  		0/*delay between bytes, 2 clk cycles*/,
			2*sysclk_get_cpu_hz() / (gs_ul_spi_clock [3]*32)/*delay between spi xfer, 3 clk cycles*/);
	spi_enable(SPI0_MASTER_BASE);
	pdc_disable_transfer(g_p_spim_pdc [3], PERIPH_PTCR_RXTDIS |
			PERIPH_PTCR_TXTDIS);
}
#endif	 //SPI0_FOR_SPI5

static void write_regs_spi(uint32_t adr, uint32_t len, uint8_t *vals) {
	  uint32_t n, tmp, wtmp, *pth = &tmp;
	  spi_tgt_done = true;
	*pth = 0xc000 | (0xfff & adr);
	spi_tx_transfer(pth, 2/2, &wtmp, 2/2, 0/*ctrl/sts*/);
	while (spi_tgt_done) ;
	for (n=0; n<len; n++) {
		spi_tgt_done = true;
	  	wtmp = vals[n];  //request the change, to big endian
		*pth = 0xb000 | (0x0ff & wtmp);
		spi_tx_transfer(pth, 2/2, &wtmp, 2/2, 0/*ctrl/sts*/);
		while (spi_tgt_done) ;
	}
}
static void read_regs_spi(uint32_t adr, uint32_t len, uint8_t *vals) {
	 uint32_t n, tmp, wtmp, *pth = &tmp, *prh = &wtmp;
	 spi_tgt_done = true;
	// setup spi to address then read mem
	*pth = 0xd000 | (0xfff & adr);
	spi_tx_transfer(pth, 2/2, &wtmp, 2/2, 0/*ctrl/sts*/);
	while (spi_tgt_done) ;
	*pth = 0x30ff;  // write once
	for (n=0; n<len ;n++) {
		spi_tgt_done = true;
		spi_tx_transfer(pth, 2/2, prh, 2/2, 0/*ctrl/sts*/);
		while (spi_tgt_done) ;
		*vals++ = *(uint8_t*)prh;
	}
}
#endif

/***************************************************************************//**
 * @brief main
*******************************************************************************/
static uint8_t tx_power_on_prev;
static int8_t rf_tx_atten_select_prev,
							rf_tx_ch_sel_prev;
static uint32_t min_attenuation_mdb_prev,
								tone1_freq_prev,
								tone2_freq_prev,
								tx_ch_freq_min_prev,
								tx_bw_sel_prev,
								chassis_ctrl_prev;
int rf_dvbt_startup(void)
{
	int32_t ret;
	uint32_t reg_word0;
	uint32_t reg_word1;
	uint32_t reg_word2;
	uint32_t reg_word3;
	uint32_t cur_word;
	uint8_t chassis_ctrl_inhibit;
	uint8_t test_tone_out_select;
	uint8_t tx_power_on;
	uint32_t min_attenuation_mdb, cur_attenuation_mdb, attenuation_mdb;
	uint32_t tone1_freq;
	uint32_t tone2_freq;
	uint32_t tx_ch_freq_min;
	uint64_t lo_freq_hz, selected_tx_freq;
	uint8_t tx_bw_sel;
	int8_t rf_tx_atten_select, rf_tx_ch_sel;
	int32_t chassis_ctrl;
	uint32_t tx_pwr_atten[4]; //in mdB
	uint32_t tx_ch_freq[16];
	uint32_t freq_offset;
	uint8_t  atten_offset;
	uint8_t  rf_reg_change_req;

	int status;
	int i, j, k;
	int config_loaded;
	//lt_u8 spi_read_data[10];


	config_loaded = 0;
#ifdef NOIS_EXTRACTED
 #ifndef SPI0_FOR_SPI5
 	spi_master_initialize(3, SPI_ADI_MASTER_BASE, BOARD_FLEXCOM_SPI_ADI);// dvbt rf ctrl @ tx end
	spi_configure_cs_behavior(SPI_ADI_MASTER_BASE, 0, SPI_CS_KEEP_LOW);
	pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA11, 0); //ADI_NSEL
 #else // work around using spi0 in place of spi5
 	spi0_master_initialize();// dvbt rf ctrl @ tx end
	spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_KEEP_LOW);
	pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA25, 0); //ADI_NSEL
 #endif
#endif
	//printf("Hello!\n");
	//printf("lgdst software v.%s, %s %s\n\r", SW_VERSION, __DATE__, __TIME__);

	//alt_avalon_spi_command(SPI_BASEADDR, 0, 0, 0, 1, spi_read_data, 0);
	//alt_avl_gpio_write(0x00000000, 0x0000000e);
	//IOWR_ALTERA_AVALON_PIO_DATA(GPIO_BASEADDR, 0x0000000e);

	/*
	while(!config_loaded)
	{
		//printf("waiting for config load\n\r");
		//polling the registers for changes------------------------------------
		cur_word = REG_MAP_mReadReg(REG_MAP_BASEADDR, MB_SYS_STAT_OFFSET + 36); //bytes 36-39, sys stat page 1
		config_loaded = (cur_word >> 29) & 0x1;
	}*/

	rf_tx_atten_select_prev = -1;
	rf_tx_ch_sel_prev = -1;
	tx_power_on_prev = 100;
	tx_bw_sel_prev = 0;
	tone1_freq_prev = 0;
	tone2_freq_prev = 0;
	chassis_ctrl_prev = -1;
	min_attenuation_mdb_prev = 1000 * 1000;
	tx_ch_freq_min_prev = 100 * 1000;

	//to apply default settings
	//rf_reg_change_req = 1; // turn on request upon power on, changed by liyenho

#ifdef XILINX_PLATFORM
	Xil_ICacheEnable();
	Xil_DCacheEnable();
#endif

#ifdef ALTERA_PLATFORM
//	if (altera_bridge_init()) {
//		printf("Altera Bridge Init Error!\n");
//		return -1;
//	}
#endif


#ifdef __INCLUDE_RF__
	// NOTE: The user has to choose the GPIO numbers according to desired
	// carrier board.
	default_init_param.gpio_resetb = GPIO_RESET_PIN;
#ifdef FMCOMMS5
	default_init_param.gpio_sync = GPIO_SYNC_PIN;
	default_init_param.gpio_cal_sw1 = GPIO_CAL_SW1_PIN;
	default_init_param.gpio_cal_sw2 = GPIO_CAL_SW2_PIN;
#else
	default_init_param.gpio_sync = -1;
	default_init_param.gpio_cal_sw1 = -1;
	default_init_param.gpio_cal_sw2 = -1;
#endif

#ifdef LINUX_PLATFORM
	gpio_init(default_init_param.gpio_resetb);
#else
	//gpio_init(GPIO_DEVICE_ID); // removed by liyenho
#endif
	//gpio_direction(default_init_param.gpio_resetb, 1); // removed by liyenho

	//spi_init(SPI_DEVICE_ID, 1, 0); // removed by liyenho

#if defined FMCOMMS5 || defined PICOZED_SDR
	default_init_param.xo_disable_use_ext_refclk_enable = 1;
#endif

	//cur_word = (1 << REG_LOC_RF_CAL_DONE) | (0x43 << 24); //byte 3: usr tx control byte, bit 7: cnt select, bits 3:0: frame, bit 6: frame sel when cnt_sel = 0
	//write_reg_bige(REG_MAP_BASEADDR, MB_RF_STAT_OFFSET + 0, cur_word); //turn off the fabric transmitter
	//ad9361_bist_loopback(ad9361_phy, 1);

	ad9361_init(&ad9361_phy, &default_init_param);

	//cur_word = 9142857;
	//ad9361_set_rx_sampling_freq(ad9361_phy, cur_word);
	ad9361_get_rx_sampling_freq(ad9361_phy, &cur_word);


	ad9361_set_tx_fir_config(ad9361_phy, tx_fir_config);
	//ad9361_set_rx_fir_config(ad9361_phy, rx_fir_config);

	//Enable TX filter
	ret = ad9361_set_tx_fir_en_dis(ad9361_phy, 1);

#ifdef FMCOMMS5
#ifdef LINUX_PLATFORM
	gpio_init(default_init_param.gpio_sync);
#endif
	gpio_direction(default_init_param.gpio_sync, 1);
	default_init_param.id_no = 1;
	default_init_param.gpio_resetb = GPIO_RESET_PIN_2;
#ifdef LINUX_PLATFORM
	gpio_init(default_init_param.gpio_resetb);
#endif
	default_init_param.gpio_sync = -1;
	default_init_param.gpio_cal_sw1 = -1;
	default_init_param.gpio_cal_sw2 = -1;
	default_init_param.rx_synthesizer_frequency_hz = 2300000000UL;
	default_init_param.tx_synthesizer_frequency_hz = 2300000000UL;
	gpio_direction(default_init_param.gpio_resetb, 1);
	ad9361_init(&ad9361_phy_b, &default_init_param);

	ad9361_set_tx_fir_config(ad9361_phy_b, tx_fir_config);
	ad9361_set_rx_fir_config(ad9361_phy_b, rx_fir_config);
#endif

#ifndef AXI_ADC_NOT_PRESENT
#if defined XILINX_PLATFORM || defined LINUX_PLATFORM
#ifdef DAC_DMA
#ifdef FMCOMMS5
	dac_init(ad9361_phy_b, DATA_SEL_DMA, 0);
#endif
	dac_init(ad9361_phy, DATA_SEL_DMA, 1);
#else
#ifdef FMCOMMS5
	dac_init(ad9361_phy_b, DATA_SEL_DDS, 0);
#endif
	dac_init(ad9361_phy, DATA_SEL_DDS, 1);
#endif
#endif
#endif

#ifdef FMCOMMS5
	ad9361_do_mcs(ad9361_phy, ad9361_phy_b);
#endif

#ifndef AXI_ADC_NOT_PRESENT
#if defined XILINX_PLATFORM && defined CAPTURE_SCRIPT
    // NOTE: To prevent unwanted data loss, it's recommended to invalidate
    // cache after each adc_capture() call, keeping in mind that the
    // size of the capture and the start address must be alinged to the size
    // of the cache line.
	mdelay(1000);
    adc_capture(16384, ADC_DDR_BASEADDR);
    Xil_DCacheInvalidateRange(ADC_DDR_BASEADDR, 16384);
#endif
#endif

    //Signal init done
    //word 0: TX control
    //word 1: RF atten
    //word 2: [31:16]: attenuation in mdB
    //         [15:8]: tone1 freq in MHz
    //          [7:0]: tone2 freq in MHz
    //word 3:  [31:0]: LO frequency in kHz
    //REG_MAP_mWriteReg(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET, 0xdeadbeef);
    //REG_MAP_mWriteReg(REG_MAP_BASEADDR, 4, 0xb0a6e0ab);
    //REG_MAP_mWriteReg(REG_MAP_BASEADDR, 8, 0x11223344);
#ifdef NOIS_EXTRACTED
 #ifdef SPI0_FOR_SPI5
 extern void spi0_set_peripheral();
	spi0_set_peripheral();
	spi_master_initialize(0, SPI0_MASTER_BASE, BOARD_FLEXCOM_SPI0);// fpga ctrl pipe
	spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_RISE_NO_TX);
 #endif
#endif
    //default DVB-T settings: QPSK, rate 1/2, 1/32 guard interval
    cur_word = 1 << REG_LOC_RF_REG_CHANGE_REQ;  //request the change
#ifndef NOIS_EXTRACTED
    write_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 0, cur_word);
#else
		uint8_t *pv = ((uint8_t*)&cur_word)+2 ;
		write_regs_spi(MB_RF_CONF_OFFSET+1, 1, pv); // to big endian
#endif
    //cur_word = 1 << REG_LOC_TX_POWER_ON;
    cur_word = 0 << REG_LOC_TX_POWER_ON;    //default to power off
    cur_word |= 0x00024600; //default attenuation delta  default:
#ifndef NOIS_EXTRACTED
    write_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 4, cur_word); //power on
#else
		cur_word = (0xff&(cur_word>>24)) | (0xff000000&(cur_word<<24)) | (0xff00&(cur_word>>8)) | (0xff0000&(cur_word<<8));
		pv = (uint8_t*)&cur_word ;
		write_regs_spi(MB_RF_CONF_OFFSET+4, 4, pv); // to big endian
#endif
    //min TX attenuation
    cur_word = 10000 << 16; //default attenuation
#ifndef NOIS_EXTRACTED
    write_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + REG_LOC_TX_MIN_ATTEN, cur_word);
#else
		cur_word = (0xff&(cur_word>>24)) | (0xff00&(cur_word>>8));
		pv = (uint8_t*)&cur_word ;
		write_regs_spi(MB_RF_CONF_OFFSET+REG_LOC_TX_MIN_ATTEN, 2, pv); // to big endian
#endif
    //min TX LO freq
    cur_word = 9568;  /*666;*/ /*, or 2430*/
#ifndef NOIS_EXTRACTED
    write_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 12, cur_word); //freq in MHz
#else
		cur_word = (0xff000000&(cur_word<<24)) | (0xff0000&(cur_word<<8));
		pv = ((uint8_t*)&cur_word)+2 ;
		write_regs_spi(MB_RF_CONF_OFFSET+12+2, 2, pv); // to big endian
#endif
    //frequency change delta
#ifndef NOIS_EXTRACTED
    cur_word = 0x000a0a0a;
    write_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 16, cur_word);
    cur_word = 0x0a0a0a0a;
    write_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 20, cur_word);
    write_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 24, cur_word);
    write_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 28, cur_word);
#else
	cur_word = 0x0a0a0a00;
	pv = (uint8_t*)&cur_word ;
	write_regs_spi(MB_RF_CONF_OFFSET+16, 4, pv); // to big endian
    cur_word = 0x0a0a0a0a;
	pv = (uint8_t*)&cur_word ;
	write_regs_spi(MB_RF_CONF_OFFSET+20, 4, pv); // to big endian
	write_regs_spi(MB_RF_CONF_OFFSET+24, 4, pv); // to big endian
	write_regs_spi(MB_RF_CONF_OFFSET+28, 4, pv); // to big endian
#endif
    //loopback
    //ad9361_bist_loopback(ad9361_phy, 1);
#ifndef NOIS_EXTRACTED
    cur_word = (1 << REG_LOC_RF_CAL_DONE) | (0x43 << 24);
    write_reg_bige(REG_MAP_BASEADDR, MB_RF_STAT_OFFSET + 0, cur_word); //turn on the fabric transmitter
#else
  extern volatile uint32_t nios_done ;
  //cur_word = (1 << REG_LOC_RF_CAL_DONE) | (0x43 << 24);
  	 nios_done = 0x81; // emulate nois done
#endif
#ifndef NOIS_EXTRACTED
    //added by sid to get the version
    num_1 = atoi(&BUILD_YEAR_CH2);  //2nd digit of the year
    num_1 = (num_1/10);
    num_2 = atoi(&BUILD_YEAR_CH3);  //lsd digit of the year
    num_2 = (num_2 & 0xf);
    tmp = (BUILD_MONTH_CH0);
    num_3 = atoi(&tmp); //2nd digit of the month
    tmp = (BUILD_MONTH_CH1);
    num_4 = atoi(&tmp); //lsd digit of the month
    tmp = (BUILD_DAY_CH0);
    num_5 = atoi(&tmp); //2nd digit of the day
    tmp = (BUILD_DAY_CH1);
    num_6 = atoi(&tmp); //lsd digit of the day

    cur_word = (num_1 <<28) | (num_2 <<24) | (num_3 <<20) | (num_4 <<16) | (num_5 <<12) | (num_6 <<8);
    write_reg_bige(REG_MAP_BASEADDR, MB_RF_STAT_OFFSET + 4, cur_word);
#endif
#endif //__INCLUDE_RF__
    //printf("Init done.\r\n");
    //printf("griffin software v.%s, %s %s\n\r", SW_VERSION, __DATE__, __TIME__);
    //printf("-----------------------------------------------\n\r");

    return 0;
}
int rf_dvbt_control(void)
{
	int32_t ret;
	uint32_t reg_word0;
	uint32_t reg_word1;
	uint32_t reg_word2;
	uint32_t reg_word3;
	uint32_t cur_word;
	uint8_t chassis_ctrl_inhibit;
	uint8_t test_tone_out_select;
	uint8_t tx_power_on;
	uint32_t min_attenuation_mdb, cur_attenuation_mdb, attenuation_mdb;
	uint32_t tone1_freq;
	uint32_t tone2_freq;
	uint32_t tx_ch_freq_min;
	uint64_t lo_freq_hz, selected_tx_freq;
	uint8_t tx_bw_sel;
	int8_t rf_tx_atten_select, rf_tx_ch_sel;
	int32_t chassis_ctrl;
	uint32_t tx_pwr_atten[4]; //in mdB
	uint32_t tx_ch_freq[16];
	uint32_t freq_offset;
	uint8_t  atten_offset;
	uint8_t  rf_reg_change_req = 0;

	int status;
	int i, j, k;
	int config_loaded;

#ifdef __INCLUDE_RF__
#ifdef CONSOLE_COMMANDS
	get_help(NULL, 0);
#ifndef NOIS_EXTRACTED
	{
		console_get_command(received_cmd);
		invalid_cmd = 0;
		for(cmd = 0; cmd < cmd_no; cmd++)
		{
			param_no = 0;
			cmd_type = console_check_commands(received_cmd, cmd_list[cmd].name,
											  param, &param_no);
			if(cmd_type == UNKNOWN_CMD)
			{
				invalid_cmd++;
			}
			else
			{
				cmd_list[cmd].function(param, param_no);
			}
		}
		if(invalid_cmd == cmd_no)
		{
			;//console_print("Invalid command!\n");
		}
	}
#endif
#else

	chassis_ctrl_inhibit = 1; //no chassis control
	{
		//polling the registers for changes------------------------------------
#ifndef NOIS_EXTRACTED
		cur_word = read_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 0);
	    //chassis_ctrl_inhibit = (cur_word >> REG_LOC_CHASSIS_CTRL_INH) & 0x1;
	    test_tone_out_select = (cur_word >> REG_LOC_DVB_T_OUT_SELECT) & 0x1;
#else
		read_regs_spi(MB_RF_CONF_OFFSET, sizeof(cur_word), (uint8_t*)&cur_word);
	    //chassis_ctrl_inhibit = (cur_word >> REG_LOC_CHASSIS_CTRL_INH) & 0x1;
	    test_tone_out_select = (0x80 & cur_word) == 0x80;
#endif
#ifndef NOIS_EXTRACTED
	    if (!rf_reg_change_req)  // unintialized yet potential a bug, liyenho
	    {
	    	rf_reg_change_req = (cur_word >> REG_LOC_RF_REG_CHANGE_REQ) & 0x1;

	    	if (rf_reg_change_req)
	    	{
				cur_word &= ~(0x1 << REG_LOC_RF_REG_CHANGE_REQ);
				write_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 0, cur_word);
				//console_print("rf change request acknowledged.\n");
	    	}
	    }

	    //tx_power_on = (cur_word >> REG_LOC_TX_OUTPUT_DISABLE) & 0x1;

//	    if (rf_reg_change_req)
//		{
//			//clear the request bit
//			cur_word &= ~(0x1 << REG_LOC_RF_REG_CHANGE_REQ);
//			REG_MAP_mWriteReg(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 0, cur_word);
//		}

	    /*if (!chassis_ctrl_inhibit)
	    {
	    	//read from chassis control
			cur_word = REG_MAP_mReadReg(REG_MAP_BASEADDR, MB_SYS_STAT_OFFSET + 44); //bytes 44-47, sys stat page 1
			chassis_ctrl = (cur_word & 0xff);
			//rf_tx_on_btn = (cur_word >> 7) & 0x1;   //chassis control
			//rx_tf_pwr_ctrl = (cur_word >> 4) & 0x3; //chassis control
			//rf_tx_ch_ctrl = (cur_word >> 0) & 0xf;  //chassis control

			reg_word1 = REG_MAP_mReadReg(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 4);
			reg_word1 = (reg_word1 & (~0xff000000)) | (chassis_ctrl << 24); //use byte 0 of cur_word (stat byte 47)
			REG_MAP_mWriteReg(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 4, reg_word1);  //RMW

			if (chassis_ctrl != chassis_ctrl_prev)
			{
				rf_reg_change_req = 1;
				chassis_ctrl_prev = chassis_ctrl;
			}
	    }*/

	    //skip the rest of there is no change
	    if (!rf_reg_change_req)
	    	return 0;
#endif
	    //console_print("rf change request received.\n");
	    rf_reg_change_req = 0; //turn it off
#ifndef NOIS_EXTRACTED
	    reg_word1 = read_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 4);
	    reg_word2 = read_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 8);
	    reg_word3 = read_reg_bige(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 12);
	    tx_ch_freq_min = reg_word3;
#else
	    read_regs_spi(MB_RF_CONF_OFFSET + 4, sizeof(reg_word1), (uint8_t*)&reg_word1);
	    read_regs_spi(MB_RF_CONF_OFFSET + 8, sizeof(reg_word2), (uint8_t*)&reg_word2);
	    read_regs_spi(MB_RF_CONF_OFFSET + 12, sizeof(reg_word3), (uint8_t*)&reg_word3);
	    tx_ch_freq_min = (0xff&(reg_word3>>24))|(0xff00&(reg_word3>>8))|(0xff0000&(reg_word3<<8))|(0xff000000&(reg_word3<<24));
#endif
#ifndef NOIS_EXTRACTED
	    tx_power_on = (reg_word1 >> REG_LOC_TX_POWER_ON) & 0x1;
	    rf_tx_atten_select = (reg_word1 >> REG_LOC_TX_ATTEN_SEL) & 0x3;
	    rf_tx_ch_sel = (reg_word1 >> REG_LOC_TX_CHAN_SEL) & 0xf;
#else
	    tx_power_on = (0x80 & reg_word1) == 0x80;
	    rf_tx_atten_select = (reg_word1 >> 4) & 0x3;
	    rf_tx_ch_sel = (reg_word1) & 0xf;
#endif
#ifndef NOIS_EXTRACTED
	    /*
	    atten_change_req = (reg_word1 >> REG_LOC_ATTEN_CHANGE_REQ) & 0x1;
	    tx_lo_change_req = (reg_word1 >> REG_LOC_TX_LO_CHANGE_REQ) & 0x1;
	    tone1_change_req = (reg_word1 >> REG_LOC_TONE1_CHANGE_REQ) & 0x1;
	    tone2_change_req = (reg_word1 >> REG_LOC_TONE2_CHANGE_REQ) & 0x1;
	    tx_bw_change_req = (reg_word1 >> REG_LOC_TX_BW_CHANGE_REQ) & 0x1;
	    */
	    min_attenuation_mdb = reg_word2 >> 16;
	    tone1_freq = (reg_word2 >> 8) & 0xff;
	    tone2_freq = reg_word2 & 0xff;
	    tx_bw_sel = reg_word1 & 0x03; //00: 8 MHz, 01: 7 MHz, 10: 6 MHz
#else
	    min_attenuation_mdb = (0xff & (reg_word2>>8)) | ((0xff&reg_word2)<<8);
	    tone1_freq = (reg_word2 >> 16) & 0xff;
	    tone2_freq = (reg_word2&0xff000000) >> 24;
	    tx_bw_sel = (reg_word1&0x03000000) >> 24; //00: 8 MHz, 01: 7 MHz, 10: 6 MHz
#endif
	    for(i = 0; i < 4; i++)
	    {
#ifndef NOIS_EXTRACTED
	    	atten_offset = ((reg_word1 >> (20 - i * 4)) & 0xf);
#else
		static int pattern[4] = {12, 8, 20, 16};
			atten_offset = ((reg_word1 >> pattern[i]) & 0xf);
#endif
	    	tx_pwr_atten[i] = min_attenuation_mdb + atten_offset * 1000; //in mdB
	    }

	    tx_ch_freq[0] = tx_ch_freq_min;

	    for(i = 0; i < 4; i++)
	    {
#ifndef NOIS_EXTRACTED
	    	cur_word = read_reg_bige(REG_MAP_BASEADDR, REG_LOC_CHAN_FREQ_OFF_BASE + i * 4);
#else
	    	read_regs_spi(REG_LOC_CHAN_FREQ_OFF_BASE + i * 4, sizeof(cur_word), (uint8_t*)&cur_word);
#endif
	    	for(j = 0; j < 4; j++)
	    	{
	    		k = i * 4 + j;
#ifndef NOIS_EXTRACTED
	    		freq_offset = ((cur_word >> ((3 - j) * 8)) & 0xff);
#else
	    		freq_offset = ((cur_word >> (j * 8)) & 0xff);
#endif
	    		//console_print("k = %d, %d\n", k, freq_offset);

	    		if (k == 0)
	    		{
	    			tx_ch_freq[0] = tx_ch_freq_min + freq_offset;  //in MHz
	    		}
	    		else
	    		{
	    			tx_ch_freq[k] = tx_ch_freq[k - 1] + freq_offset;
	    		}
	    	}
	    }

	    /*
	    for(i = 0; i < 16; i++)
		{
			console_print("i = %d, %d\n", i, tx_ch_freq[i]);
		}*/

	    //Applying register changes---------------------------------------------
//		if ((rf_tx_ch_sel != rf_tx_ch_sel_prev))
//		{
//			tx_lo_change_req = 1;
//			lo_freq_hz = tx_ch_freq[rf_tx_ch_sel] * 1000000;
//			rf_tx_ch_sel_prev = rf_tx_ch_sel;
//		}
//		/*else
//		{
//			lo_freq_hz = tx_ch_freq_min * 1000000; //reg value in kHz
//		}*/

	    if (tx_power_on != tx_power_on_prev)
		{
			if (!tx_power_on)
			{
				//set deep attenuation
				cur_attenuation_mdb = 80 * 1000; //almost max atten (89.5)
				//console_print("TX power off: max attenuation set.\n");
			}
			else
			{
				cur_attenuation_mdb = tx_pwr_atten[rf_tx_atten_select];  //restore attenuation
				//console_print("TX power on: current attenuation restored (%d mdB).\n", cur_attenuation_mdb);
			}
 #ifdef SPI0_FOR_SPI5 // work around using spi0 in place of spi5
 		spi0_master_initialize();// dvbt rf ctrl @ tx end
		spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_KEEP_LOW);
		pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA25, 0); //ADI_NSEL
 #endif
			ad9361_set_tx_attenuation(ad9361_phy, 0, cur_attenuation_mdb);
			ad9361_get_tx_attenuation(ad9361_phy, 0, &attenuation_mdb); // for dbg brkpoint, liyenho
			tx_power_on_prev = tx_power_on;
		}

		if (((rf_tx_atten_select != rf_tx_atten_select_prev) || (min_attenuation_mdb != min_attenuation_mdb_prev)) && tx_power_on)
		{
 #ifdef SPI0_FOR_SPI5 // work around using spi0 in place of spi5
 		spi0_master_initialize();// dvbt rf ctrl @ tx end
		spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_KEEP_LOW);
		pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA25, 0); //ADI_NSEL
 #endif
			//atten_change_req = 1;
			cur_attenuation_mdb = tx_pwr_atten[rf_tx_atten_select];
			ad9361_set_tx_attenuation(ad9361_phy, 0, cur_attenuation_mdb);
			rf_tx_atten_select_prev = rf_tx_atten_select;
			min_attenuation_mdb_prev = min_attenuation_mdb;

			ad9361_get_tx_attenuation(ad9361_phy, 0, &attenuation_mdb); // for dbg brkpoint, liyenho
			//console_print("Attenuation changed: %d mdB.\n", cur_attenuation_mdb);
		}

//		//change attenuation
//		if ((rf_tx_atten_select != rf_tx_atten_select_prev) && (!tx_power_on))
//		{
//			ad9361_set_tx_attenuation(ad9361_phy, 0, min_attenuation_mdb);
//
//			console_print("Attenuation changed.\n");
//
//			//clear the request bit
//			//reg_word1 &= ~(0x1 << REG_LOC_ATTEN_CHANGE_REQ);
//		}



		/*
		if (rf_tx_on_btn != rf_tx_on_btn_prev)
		{
			cur_word = REG_MAP_mReadReg(REG_MAP_BASEADDR, MB_RF_STAT_OFFSET + 0);
			cur_word = (cur_word & (~0x2)) | (rf_tx_on_btn << 1);
			REG_MAP_mWriteReg(REG_MAP_BASEADDR, MB_RF_STAT_OFFSET + 0, cur_word);
			rf_tx_on_btn_prev = rf_tx_on_btn;
		}*/


		if ((rf_tx_ch_sel != rf_tx_ch_sel_prev) || (tx_ch_freq_min != tx_ch_freq_min_prev))
		{
 #ifdef SPI0_FOR_SPI5 // work around using spi0 in place of spi5
 		spi0_master_initialize();// dvbt rf ctrl @ tx end
		spi_configure_cs_behavior(SPI0_MASTER_BASE, 0, SPI_CS_KEEP_LOW);
		pio_configure(PIOA, PIO_OUTPUT_1, PIO_PA25, 0); //ADI_NSEL
 #endif
			/*for(i = 0; i < 16; i++)
			{
				console_print("i = %d, %d\n", i, tx_ch_freq[i]);
			}*/

			lo_freq_hz = (tx_ch_freq[rf_tx_ch_sel]) * 250000;
			selected_tx_freq = lo_freq_hz;
			selected_tx_freq /= 250000;
			//console_print("setting tx lo freq to %d\n", (uint32_t)(selected_tx_freq));

			rf_tx_ch_sel_prev = rf_tx_ch_sel;
			tx_ch_freq_min_prev = tx_ch_freq_min;

			//lo_freq_hz = reg_word3 * 250000; //reg value in MHz
			ad9361_set_tx_lo_freq(ad9361_phy, lo_freq_hz);

			//lo_freq_hz /= 250000;
			//console_print("tx_lo_freq=%d\n", (uint32_t)lo_freq_hz);
			//console_print("TX LO freq changed to %d.\n", (uint32_t)lo_freq_hz);

			ad9361_get_tx_lo_freq(ad9361_phy, &lo_freq_hz);
			lo_freq_hz /= 250000;
			//console_print("tx_lo_freq = %d MHz\n", (uint32_t)lo_freq_hz);

			//clear the request bit
			//reg_word1 &= ~(0x1 << REG_LOC_TX_LO_CHANGE_REQ);
		}

		/*if (tx_bw_sel != tx_bw_sel_prev)
		{
			if (tx_bw_sel == 0) //8 MHz
			{
				tx_path_clock_frequencies = tx_path_clock_frequencies_8mhz;
				rx_path_clock_frequencies = tx_path_clock_frequencies_8mhz;
			}
			else if (tx_bw_sel == 1) //7 MHz
			{
				tx_path_clock_frequencies = tx_path_clock_frequencies_7mhz;
				rx_path_clock_frequencies = tx_path_clock_frequencies_7mhz;
			}
			else
			{
				tx_path_clock_frequencies = tx_path_clock_frequencies_6mhz;
				rx_path_clock_frequencies = tx_path_clock_frequencies_6mhz;
			}

			//int32_t ad9361_set_trx_path_clks(struct ad9361_rf_phy *phy, uint32_t *rx_path_clks, uint32_t *tx_path_clks);
			ad9361_set_trx_path_clks(ad9361_phy, rx_path_clock_frequencies, tx_path_clock_frequencies);
			console_print("BW changed = %d MHz\n", (tx_bw_sel == 0)? 8 : (tx_bw_sel == 1)? 7 : 6);

			//clear the request bit
			//reg_word1 &= ~(0x1 << REG_LOC_TX_BW_CHANGE_REQ);
			tx_bw_sel_prev = tx_bw_sel;
		}*/

		//console_print("test_tone_out_select=%d\n", test_tone_out_select);
		if ((test_tone_out_select) && (tone1_freq != tone1_freq_prev))
		{
			//dds_set_frequency(DDS_CHAN_TX1_I_F1, tone1_freq);
			//dds_set_frequency(DDS_CHAN_TX1_Q_F1, tone1_freq);

			//console_print("Test tone1 freq changed: %d MHz.\n", tone1_freq);

			//clear the request bit
			//reg_word1 &= ~(0x1 << REG_LOC_TONE1_CHANGE_REQ);
			tone1_freq_prev = tone1_freq;
		}

		if ((test_tone_out_select) && (tone2_freq != tone2_freq_prev))
		{
			//dds_set_frequency(DDS_CHAN_TX1_I_F2, tone2_freq);
			//dds_set_frequency(DDS_CHAN_TX1_Q_F2, tone2_freq);

			//console_print("Test tone2 freq changed: %d MHz.\n", tone2_freq);

			//clear the request bit
			//reg_word1 &= ~(0x1 << REG_LOC_TONE2_CHANGE_REQ);
			tone2_freq_prev = tone2_freq;
		}

//		if (atten_change_req | tx_lo_change_req | tone1_change_req | tone2_change_req | tx_bw_change_req)
//		{
//			REG_MAP_mWriteReg(REG_MAP_BASEADDR, MB_RF_CONF_OFFSET + 4, reg_word1);
//		}
	}
#endif

#endif //__INCLUDE_RF__

	//printf("Done.\n");

#ifdef XILINX_PLATFORM
	Xil_DCacheDisable();
	Xil_ICacheDisable();
#endif

#ifdef ALTERA_PLATFORM
	if (altera_bridge_uninit()) {
		//printf("Altera Bridge Uninit Error!\n");
		return -1;
	}
#endif

	return 0;
}
