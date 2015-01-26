/*
 * initialize.h
 *
 * Created: 3/22/2012 12:50:44 AM
 *  Author: Milad
 */ 


#ifndef INITIALIZE_H_
#define INITIALIZE_H_
#include <asf.h>

#define F_CPU 32000000UL

#define ON 1
#define OFF 0
//ID1 ID0 ID3 ID2
#define RobotID ((((PORTB_IN & PIN5_bm) >> PIN5_bp) << 0)|(((PORTB_IN & PIN3_bm) >> PIN3_bp) << 1)|(((PORTB_IN & PIN6_bm) >> PIN6_bp) << 2)|(((PORTB_IN & PIN4_bm) >> PIN4_bp) << 3))     
//FPGA DATA
#define CLK_par_bm PIN0_bm 
#define CLK_par_bp PIN0_bp 
#define CLK_par_PORT PORTA
#define PARITY_bm PIN1_bm
#define PARITY_bp PIN1_bp
#define PARITY_PORT  PORTA 
#define FPGA_DATA0_bm	PIN0_bm	
#define FPGA_DATA1_bm	PIN1_bm	
#define FPGA_DATA2_bm	PIN2_bm	
#define FPGA_DATA3_bm	PIN3_bm	
#define FPGA_DATA4_bm	PIN4_bm	
#define FPGA_DATA5_bm	PIN5_bm	
#define FPGA_DATA6_bm	PIN6_bm	
#define FPGA_DATA7_bm	PIN7_bm	
#define FPGA_DATA_PORT  PORTF
#define MOTORNUM0_bm	PIN0_bm	
#define MOTORNUM1_bm	PIN1_bm	 
#define MOTORNUM_PORT   PORTR
//FPGA SPI
#define MOSI_CUR1_bm PIN5_bm
#define MISO_CUR1_bm PIN6_bm
#define SCK_CUR1_bm  PIN7_bm
#define MOSI_CUR2_bm PIN5_bm
//#define MISO_CUR2_bm PIN6_bm
#define SCK_CUR2_bm  PIN7_bm
//IR
#define KCK_Sens_PORT PORTD
#define KCK_Sens_PIN_bm		PIN1_bm
#define KCK_Sens_PIN_bp		PIN1_bp
#define KCK_Sens (((KCK_Sens_PORT.IN &KCK_Sens_PIN_bm) >> KCK_Sens_PIN_bp)?0x00:0xFF)
//SW_cap
#define KCK_DSH_SW_PORT PORTA
#define KCK_DSH_SW_PIN_bm	PIN2_bm
#define KCK_DSH_SW_PIN_bp	PIN2_bp
#define KCK_DSH_SW (((KCK_DSH_SW_PORT.IN & KCK_DSH_SW_PIN_bm) >> KCK_DSH_SW_PIN_bp)?0xFF:0x00)

#define KCK_Ch_Limit_PORT PORTC
#define KCK_Ch_Limit_PIN_bm		PIN4_bm
#define KCK_Ch_Limit_PIN_bp		PIN4_bp

#define KCK_DCh_Limit_PORT PORTC
#define KCK_DCh_Limit_PIN_bm		PIN6_bm
#define KCK_DCh_Limit_PIN_bp		PIN6_bp
#define KCK_DCh_Limit (((KCK_Ch_Limit_PORT.IN & KCK_Ch_Limit_PIN_bm) >> KCK_Ch_Limit_PIN_bp)?0xFF:0x00)
//ST_SHG
//#define KCK_Charge_PORT PORTC
//#define KCK_Charge_PIN_bm		PIN4_bm
//#define KCK_Charge_PIN_bp		PIN4_bp
//#define KCK_Charge(_A_) KCK_Charge_PORT.OUT = (KCK_Charge_PORT.OUT & (~KCK_Charge_PIN_bm)) | (_A_<<KCK_Charge_PIN_bp)

#define KCK_CHARGE_OFF 1
#define KCK_CHARGE_ON  0

#define KCK_SPEED_OFF 0x00
#define KCK_SPEED_LOW 0x64
#define KCK_SPEED_HI  0xFF
#define KCK_SPEED_RX Robot_D[RobotID].KCK
//#define KCK_Speed_CHIP(_A_) TCC0_CCC=_A_; // PORTC_OUT =(PORTC_OUT & (~KCK_DIR_PIN_bm)) | (_A_<<KCK_DIR_PIN_bp) // _delay_ms(100); TCC0_CCA=0; //KCK_Charge( KCK_CHARGE_ON) KCK_Charge( KCK_CHARGE_OFF) //PORTC_OUT =TCC0_CNT=_A_; ;
#define KCK_Speed_DIR(_A_) TCC0_CCC=_A_;

//#define KCK_Chip_PIN_bm		PIN2_bm //socket shoot 4pin --> chip nadare
//#define KCK_Chip_PIN_bp		PIN2_bp
#define KCK_SH_PIN_bm		PIN3_bm
#define KCK_SH_PIN_bp		PIN3_bp
#define KCK_DIR_PIN_bm		PIN2_bm
#define KCK_DIR_PIN_bp		PIN2_bp
//Vback
#define KCK_CAP_VFB_PIN_bm	PIN6_bm 
#define KCK_CAP_VFB_PIN_bp	PIN6_bp  //portesh chi??

//Gyroscope 
#define Gyro_SDA_PIN_bm			PIN0_bm //PORTC 
#define Gyro_SCL_PIN_bm			PIN1_bm

// BUZZER
#define Buzzer_PIN_bm		PIN4_bm
#define Buzzer_PIN_bp		PIN4_bp
#define Buzzer(_STATUS_)    PORTE_OUT = (PORTE_OUT & ~PIN4_bm) | (_STATUS_<<PIN4_bp)

//LED DONE
#define LED_Red_PIN_bm		PIN1_bm
#define LED_Red_PIN_bp		PIN1_bp
#define LED_White_PIN_bm	PIN2_bm
#define LED_White_PIN_bp	PIN2_bp
#define LED_Green_PIN_bm	PIN0_bm
#define LED_Green_PIN_bp	PIN0_bp

#define LED_Red(_STATUS_)   PORTE_OUT = (PORTE_OUT & ~PIN1_bm) | (_STATUS_<<PIN1_bp)
#define LED_White(_STATUS_) PORTE_OUT = (PORTE_OUT & ~PIN2_bm) | (_STATUS_<<PIN2_bp)
#define LED_Green(_STATUS_) PORTE_OUT = (PORTE_OUT & ~PIN0_bm) | (_STATUS_<<PIN0_bp)
#define LED_Red_PORT		PORTE
#define LED_White_PORT      PORTE
#define LED_Green_PORT      PORTE
#define Buzzer_PORT			PORTE

//USART
#define TX_Data_PIN_bm		PIN3_bm
#define RX_Data_PIN_bm		PIN2_bm

//TEST BUTTON 
#define SW_TEST_PIN_bm		PIN0_bm
#define SW_TEST_PIN_bp		PIN0_bp
#define SW_TEST (((PORTD_IN &SW_TEST_PIN_bm) >>SW_TEST_PIN_bp )?0xFF:0x00)

//WIRELESS
#define NRF24L01_L_SPI			SPID
#define NRF24L01_L_PORT			PORTD

#define NRF24L01_L_IRQ_LINE		PIN2_bm
#define NRF24L01_L_CE_LINE		PIN3_bm
#define NRF24L01_L_CS_LINE		PIN4_bm
#define NRF24L01_L_MOSI_LINE	PIN5_bm
#define NRF24L01_L_MISO_LINE	PIN6_bm
#define NRF24L01_L_SCK_LINE		PIN7_bm

#define PRX_L PORTD_INT0_vect

#define POWER_VFB_PIN_bm			PIN3_bm  // VFB=Vss * R28/(R28+R27) PORTA

#define _FILTER_FREQ 1.0
#define _FILTER_CONST .11 //(0.02/((1.0/(2.0*3.14*_FILTER_FREQ))+0.02))
#define LOWByte(_A_) ((_A_) & 0xff)
#define HIGHByte(_A_) (((_A_)>>8) & 0xff)

void En_RC32M(void);
void PORT_init(void);

//#define TIMERD0_PER 0x7D
void TimerD0_init(void);//timer 0.063ms
void TimerC0_init(void);//pwm?!?
void TimerE1_init(void);

void ADCA_init(void);
void SPI_Init(void);
void USARTE0_init(void);

void OUT_Bling(PORT_t *OUT_PORT,uint8_t OUT_PIN_bp,uint8_t Speed,uint32_t *Time_ON,uint32_t time_ms);

#endif /* INITIALIZE_H_ */

