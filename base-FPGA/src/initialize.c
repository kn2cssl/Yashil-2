/*
 * initialize.c
 *
 * Created: 3/22/2013 1:12:24 PM
 *  Author: Milad
 */


#include "initialize.h"


void En_RC32M(void)
{
    // Start RC32M OSC
    OSC_CTRL |= OSC_RC32MEN_bm;
    while(!(OSC_STATUS & OSC_RC32MRDY_bm));

    // Select the system clock source: 32 MHz Internal RC Osc.
    CCP = CCP_IOREG_gc;
    CLK_CTRL = CLK_SCLKSEL_RC32M_gc;

    // Disable the unused oscillators: 2 MHz, internal 32 kHz, external clock/crystal oscillator, PLL
    OSC_CTRL &= ~(OSC_RC2MEN_bm | OSC_RC32KEN_bm | OSC_XOSCEN_bm | OSC_PLLEN_bm);
};

void PORT_init(void)
{   
	PORTA_DIRSET = CLK_par_bm | PARITY_bm; 
	//PORTB.7 SOOKHTE!
	PORTC_DIRSET =  KCK_DIR_PIN_bm | KCK_SH_PIN_bm | Gyro_SCL_PIN_bm | KCK_Chip_PIN_bm ;
			//PORTC_PIN0CTRL |= PORT_ISC_LEVEL_gc;  //vase chie??
			//PORTC_INTCTRL |= PORT_INT0LVL_LO_gc;
			//PORTC_INT0MASK |= PIN0_bm;
			
	PORTD_DIRSET = NRF24L01_L_CE_LINE | NRF24L01_L_CS_LINE | NRF24L01_L_MOSI_LINE | NRF24L01_L_SCK_LINE; //wireless module
	PORTD_PIN2CTRL=PORT_ISC_BOTHEDGES_gc;//IRQ PIN ?!
	PORTD_INTCTRL = PORT_INT0LVL_LO_gc;
	PORTD_INT0MASK = PIN2_bm;
	
	PORTE_DIRSET = SCK_CUR1_bm | MOSI_CUR1_bm | Buzzer_PIN_bm | TX_Data_PIN_bm | LED_Green_PIN_bm | LED_White_PIN_bm | LED_Red_PIN_bm;
	//PORTE.OUTSET = PIN3_bm;//TX pin on PORTE
	
	PORTF_DIRSET = FPGA_DATA0_bm | FPGA_DATA1_bm | FPGA_DATA2_bm | FPGA_DATA3_bm | FPGA_DATA4_bm | FPGA_DATA5_bm | FPGA_DATA6_bm | FPGA_DATA7_bm; 
	//PORTF_OUTSET = FPGA_DATA0_bm | FPGA_DATA1_bm | FPGA_DATA2_bm | FPGA_DATA3_bm | FPGA_DATA4_bm | FPGA_DATA5_bm | FPGA_DATA6_bm | FPGA_DATA7_bm;
	PORTR_DIRSET = MOTORNUM0_bm | MOTORNUM1_bm;
	//PORTR_OUTSET = MOTORNUM0_bm | MOTORNUM1_bm;
};

//#define TIMERD0_PER 0xE0
//void TimerD0_init(void)
//{
	//tc_write_clock_source(&TCD0,TC_CLKSEL_DIV4_gc);
	//tc_set_wgm(&TCD0,TC_WG_NORMAL);
	//tc_set_overflow_interrupt_level(&TCD0,TC_INT_LVL_LO);
	//tc_set_cca_interrupt_level(&TCD0,TC_INT_LVL_LO);
	//tc_write_period(&TCD0,TIMERD0_PER);
	//tc_write_cc(&TCD0, TC_CCA, 0x05);//05
	//tc_set_direction(&TCD0,TC_UP);
	////tc_enable_cc_channels(&TCD0,TC_CCAEN);
	//tc_enable(&TCD0);
//};
#define TIMERD0_PER 0xEF
void TimerD0_init(void)
{
	tc_write_clock_source(&TCD0,TC_CLKSEL_DIV8_gc);
	tc_set_wgm(&TCD0,TC_WG_NORMAL);
	tc_set_overflow_interrupt_level(&TCD0,TC_INT_LVL_MED);
	tc_set_cca_interrupt_level(&TCD0,TC_INT_LVL_MED);
	tc_write_period(&TCD0,TIMERD0_PER);
	tc_write_cc(&TCD0, TC_CCA, 0x05);//05
	tc_set_direction(&TCD0,TC_UP);
	//tc_enable_cc_channels(&TCD0,TC_CCAEN);
	tc_enable(&TCD0);
};

void TimerC0_init(void) //KICK_DIR -> OC0D(PORTC3) //SHG_PULSE -> OC0C(PORTC2)
{
    tc_write_clock_source(&TCC0,TC_CLKSEL_DIV64_gc);//1
    tc_set_wgm(&TCC0,TC_WG_SS);
    tc_write_period(&TCC0,0x77);//0x01DFF
    tc_set_direction(&TCC0,TC_UP);
    tc_enable_cc_channels(&TCC0,TC_CCCEN);
    tc_enable_cc_channels(&TCC0,TC_CCDEN);
    tc_enable(&TCC0);
	tc_write_cc(&TCC0,TC_CCC,0x5D);
};
void TimerC1_init(void) // KICK_CHIP -> OC1A (PORTC4)
{
	tc_write_clock_source(&TCC1,TC_CLKSEL_DIV64_gc);//1
	tc_set_wgm(&TCC1,TC_WG_SS);
	tc_write_period(&TCC1,0x77);
	tc_set_direction(&TCC1,TC_UP);
	tc_enable_cc_channels(&TCC1,TC_CCAEN);
	tc_enable(&TCC1);
	//tc_write_cc(&TCC1,TC_CCA,0x5D);
}
#define TIMERE1_PER 0x7C // per=0xD7,DIV256 => 1.728ms  // per=0x7c,DIV256 => 1ms
void TimerE1_init(void)
{
    tc_write_clock_source(&TCE1,TC_CLKSEL_DIV256_gc);
    tc_set_wgm(&TCE1,TC_WG_NORMAL);
    tc_set_overflow_interrupt_level(&TCE1,TC_INT_LVL_MED);
    tc_write_period(&TCE1,TIMERE1_PER);
    tc_set_direction(&TCE1,TC_UP);
    tc_enable(&TCE1);
};
void TimerE0_init(void)
{
	tc_write_clock_source(&TCE0,TC_CLKSEL_DIV256_gc);
	tc_set_wgm(&TCE0,TC_WG_SS);
	tc_write_period(&TCE0,0x00FF);
	tc_set_direction(&TCE0,TC_UP);
	tc_enable_cc_channels(&TCE0,TC_CCCEN);
	tc_enable_cc_channels(&TCE0,TC_CCDEN);
	tc_enable(&TCE0);
};
void SPI_Init(void)
{
	spi_xmega_set_baud_div(&NRF24L01_L_SPI,8000000UL,F_CPU);
	spi_enable_master_mode(&NRF24L01_L_SPI);
	spi_enable(&NRF24L01_L_SPI);
}

#define USARTE0_conf USARTE0
#define USARTE0_BUADRATE 9600
void USARTE0_init(void)
{
	usart_set_mode(&USARTE0_conf,USART_CMODE_ASYNCHRONOUS_gc);
	usart_format_set(&USARTE0_conf,USART_CHSIZE_8BIT_gc,USART_PMODE_DISABLED_gc,false);
	usart_set_rx_interrupt_level(&USARTE0_conf,USART_INT_LVL_MED);
	//usart_set_dre_interrupt_level(&USARTE0_conf,USART_INT_LVL_LO);
	usart_set_baudrate(&USARTE0_conf,USARTE0_BUADRATE,F_CPU);
	usart_tx_enable(&USARTE0_conf);
	//usart_rx_enable(&USARTE0_conf);
}
  //BATTERY FEEDBACK ----> PORTA3
#define CONFIG_ADC_INTLVL ADC_CH_INTLVL_LO_gc
void ADCA_init(void)
{
    struct adc_config adca_conf;
    struct adc_channel_config adca_ch_conf;
    //
    //// Initialize configuration structures.
    //adc_read_configuration(&ADCB, &adcb_conf);
    //
    ///* Configure the ADC module:
    //* - unsigned, 12-bit results
    //* - AREFA voltage reference
    //* - 8000 kHz clock rate
    //* - FreeRun Mode
    //*/
    adc_get_calibration_data(ADC_CAL_ADCA);
    adc_set_conversion_parameters(&adca_conf,ADC_SIGN_OFF,ADC_RES_12,ADC_REF_AREFA);
    adc_set_clock_rate(&adca_conf,125000UL);
    adc_set_conversion_trigger(&adca_conf,ADC_TRIG_FREERUN_SWEEP,1,0);//IN BARAYE CHIE??!
   // adc_set_config_compare_value(adcb_conf,KCK_MAX_CHARGE_AMP);
    adc_write_configuration(&ADCA,&adca_conf);
    //
    ///* Configure ADC channel 0:
    //* - Input: ADCB4
    //* - interrupts disable
    //*/
    adcch_read_configuration(&ADCA,1, &adca_ch_conf);
    adcch_set_input(&adca_ch_conf,ADCCH_POS_PIN3,ADCCH_NEG_NONE,ADC_CH_GAIN_1X_gc);
    adcch_write_configuration(&ADCA,1,&adca_ch_conf);
    
    ///* Configure ADC channel 1: darim az channel 0 estefade mikonim ehtemalan!
    //* - Input: ADCB5
    //* - Set Interrupt Mode: Below the threshold
    //* - interrupts disable
    ////*/
    //adcch_read_configuration(&ADCA,1, &adca_ch_conf);
    //adcch_set_input(&adcb_ch_conf,ADCCH_POS_PIN5,ADCCH_NEG_NONE,ADC_CH_GAIN_1X_gc);
	////adcch_set_interrupt_mode(&adcb_ch_conf,ADCCH_MODE_ABOVE);
	////adcch_enable_interrupt(&adcb_ch_conf);
    //adcch_write_configuration(&ADCA,1,&adca_ch_conf);
  
	//
    ///* Configure ADC channel 2:
    //* - Input: ADCB6
    //* - interrupts disable
    //*/
    //adcch_read_configuration(&ADCB,2, &adcb_ch_conf);
    //adcch_set_input(&adcb_ch_conf,ADCCH_POS_PIN6,ADCCH_NEG_NONE,ADC_CH_GAIN_1X_gc);
    ////adcch_disable_interrupt(&adcb_ch_conf);
    //adcch_write_configuration(&ADCB,2,&adcb_ch_conf);
    ////
    ///* Configure ADC channel 3:
    //* - Input: ADCB7
    //* - interrupts disable
    //*/
    //adcch_read_configuration(&ADCB,3, &adcb_ch_conf);
    //adcch_set_input(&adcb_ch_conf,ADCCH_POS_PIN7,ADCCH_NEG_NONE,ADC_CH_GAIN_1X_gc);
    //adcch_set_interrupt_mode(&adcb_ch_conf,ADCCH_MODE_ABOVE);
    //adcch_enable_interrupt(&adcb_ch_conf);
    //adcch_write_configuration(&ADCB,3,&adcb_ch_conf);
    //
    adc_enable(&ADCA);
    adc_start_conversion(&ADCA,ADC_CH0);
    //adc_start_conversion(&ADCB,ADC_CH1);
    //adc_start_conversion(&ADCB,ADC_CH2);
    ////adc_start_conversion(&ADCB,ADC_CH3);
}
//void ADCA_init(void)
//{
    //struct adc_config adca_conf;
    //struct adc_channel_config adca_ch_conf;
    ////
    ////// Initialize configuration structures.
    ////adc_read_configuration(&ADCB, &adcb_conf);
    ////
    /////* Configure the ADC module:
    ////* - unsigned, 12-bit results
    ////* - AREFA voltage reference
    ////* - 8000 kHz clock rate
    ////* - FreeRun Mode
    ////*/
    //adc_get_calibration_data(ADC_CAL_ADCA);
    //adc_set_conversion_parameters(&adca_conf,ADC_SIGN_OFF,ADC_RES_12,ADC_REF_AREFA);
    //adc_set_clock_rate(&adca_conf,125000UL);
    //adc_set_conversion_trigger(&adca_conf,ADC_TRIG_FREERUN_SWEEP,1,0);
   //// adc_set_config_compare_value(adcb_conf,KCK_MAX_CHARGE_AMP);
    //adc_write_configuration(&ADCA,&adca_conf);
    ////
    /////* Configure ADC channel 0:
    ////* - Input: ADCB4
    ////* - interrupts disable
    ////*/
    //adcch_read_configuration(&ADCA,1, &adca_ch_conf);
    //adcch_set_input(&adca_ch_conf,ADCCH_POS_PIN3,ADCCH_NEG_NONE,ADC_CH_GAIN_1X_gc);
    //adcch_write_configuration(&ADCA,1,&adca_ch_conf);
    //
    /////* Configure ADC channel 1: darim az channel 0 estefade mikonim ehtemalan!
    ////* - Input: ADCB5
    ////* - Set Interrupt Mode: Below the threshold
    ////* - interrupts disable
    //////*/
    ////adcch_read_configuration(&ADCA,1, &adca_ch_conf);
    ////adcch_set_input(&adcb_ch_conf,ADCCH_POS_PIN5,ADCCH_NEG_NONE,ADC_CH_GAIN_1X_gc);
	//////adcch_set_interrupt_mode(&adcb_ch_conf,ADCCH_MODE_ABOVE);
	//////adcch_enable_interrupt(&adcb_ch_conf);
    ////adcch_write_configuration(&ADCA,1,&adca_ch_conf);
  //
	////
    /////* Configure ADC channel 2:
    ////* - Input: ADCB6
    ////* - interrupts disable
    ////*/
    ////adcch_read_configuration(&ADCB,2, &adcb_ch_conf);
    ////adcch_set_input(&adcb_ch_conf,ADCCH_POS_PIN6,ADCCH_NEG_NONE,ADC_CH_GAIN_1X_gc);
    //////adcch_disable_interrupt(&adcb_ch_conf);
    ////adcch_write_configuration(&ADCB,2,&adcb_ch_conf);
    //////
    /////* Configure ADC channel 3:
    ////* - Input: ADCB7
    ////* - interrupts disable
    ////*/
    ////adcch_read_configuration(&ADCB,3, &adcb_ch_conf);
    ////adcch_set_input(&adcb_ch_conf,ADCCH_POS_PIN7,ADCCH_NEG_NONE,ADC_CH_GAIN_1X_gc);
    ////adcch_set_interrupt_mode(&adcb_ch_conf,ADCCH_MODE_ABOVE);
    ////adcch_enable_interrupt(&adcb_ch_conf);
    ////adcch_write_configuration(&ADCB,3,&adcb_ch_conf);
    ////
    //adc_enable(&ADCA);
    //adc_start_conversion(&ADCA,ADC_CH0);
    ////adc_start_conversion(&ADCB,ADC_CH1);
    ////adc_start_conversion(&ADCB,ADC_CH2);
    //////adc_start_conversion(&ADCB,ADC_CH3);
//}

void OUT_Bling(PORT_t *OUT_PORT,uint8_t OUT_PIN_bp,uint8_t Speed,uint32_t *Time_ON,uint32_t time_ms)
{
    if((*Time_ON) >0)
        (*Time_ON)--;
    if((Speed) != 0 && (*Time_ON) > 1)
        OUT_PORT->OUT = (OUT_PORT->OUT & ~(1<<OUT_PIN_bp)) | (((time_ms / (Speed)) & 0x00001) << OUT_PIN_bp);
    //if((Speed) == 0 && (*Time_ON) >  1)		///////Set OUT
        //OUT_PORT->OUTSET = 1<<OUT_PIN_bp;
    if((*Time_ON) == 1)						///////Clr OUT
        OUT_PORT->OUTCLR = 1<<OUT_PIN_bp;
}


