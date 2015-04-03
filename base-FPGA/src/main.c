/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#define F_CPU 32000000UL
#include <util/delay.h>
#include "initialize.h"
#include "nrf24l01_L.h"
#include "transmitter.h" //warning mide az sendnewdata va senddata
#include <stdlib.h>


void NRF_init (void) ;
void data_transmission (void);

unsigned char Buf_Rx_L[_Buffer_Size] ;
char Buf_Tx_L[_Buffer_Size];
char Address[_Address_Width] = { 0x11, 0x22, 0x33, 0x44, 0x55};
int motor_num=0,test=0;
uint32_t kck_time_dir,time_test,kck_time_chip,charge_time=0,kck_time_sw=0,charge_count=0;
int free_wheel=0;
int free_wheel_flag = 0;
int adc_change = 0;
int wireless_reset=0;
float adc =0;
int flg_dir=0, flg_chip=0, charge_flg=0,flg_sw=0;


////////////////////////////////////////current defines
struct Driver
{
	unsigned int adc[4];
	float vol[4] , vol_last[4];
	unsigned long int offset[4];
	float cur[4];
	uint16_t count_H[4],count_L[4];
	int cur_alarm[4];
	};
struct Driver Driver;

int change_ADC,flg_change=0;


unsigned int buff_offset[320];
void read_DriverCurrent(int);
void chek_DriverCurrent(int);
void calc_offset(int);
int driver_adc(int);
uint32_t t_allow,t_1ms,shoot_alarm_time=0;
int flg_2min=0,flg_2min10ms=0;
int cur_allow=0;
int current_ov=0;
int flg_alarm=0,shoot_alarm_flg=0;
int t,t_10ms=0;
int bat_count=0,battery_alarm=0;
//////////////////////////////////////////////

int Robot_Select,Robot_Select_Last;
int Test_Data[8];
int time2sec=0;
char ctrlflg=0,full_charge=0;;
uint64_t flag2sec=0;
int i=0;
int parity_calc(signed int data);

int main (void)
{
	En_RC32M();

	//Enable LowLevel & HighLevel Interrupts
	PMIC_CTRL |= PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm |PMIC_MEDLVLEN_bm;

	change_ADC=6;
	PORT_init();
	TimerD0_init();
	TimerC0_init();
	TimerC1_init();
	//TimerE0_init();
	TimerE1_init();
	USARTE0_init();
	ADCA_init();
	ADCB_init();
	wdt_enable();
	wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_16CLK);
	
	// Globally enable interrupts
	sei();

	//Address[0]=Address[0] + RobotID ;

	//// NRF Initialize
	NRF_init () ;
	
	///////////////////voltage offset of current sensor
	calc_offset(0);
	calc_offset(1);
	calc_offset(2);
	calc_offset(3);
	
	while(1)
	  {  
		    asm("wdr");
			 wdt_reset();
		///////////////////////////////////////////////////////////////// motor current sensor
		
		if(t_10ms)
		{
			
			read_DriverCurrent(0);
			read_DriverCurrent(1);
			read_DriverCurrent(2);
			if(change_ADC==6)
			read_DriverCurrent(3);
			if(cur_allow)
			{
				chek_DriverCurrent(0);
				chek_DriverCurrent(1);
				chek_DriverCurrent(2);
				chek_DriverCurrent(3);
								
			}
		t_10ms=0;
		}
		current_ov = Driver.cur_alarm[0] || Driver.cur_alarm[1] || Driver.cur_alarm[2] || Driver.cur_alarm[3];
			
			Test_Data[0]=(int)adc;
			Test_Data[1]=(int)Driver.cur[1];
			Test_Data[2]=(int)Driver.cur[2]; 
			Test_Data[3]=(int)Driver.cur[3];			
			///////////////////////////////////////////////////////////////////////////BUZZER
			if (free_wheel_flag && adc_change)
			{
				change_ADC=3;
				ADCA_init();
				flg_2min=0;
				adc_change = 0 ;
			}
			if (!free_wheel_flag && !adc_change)
			{
				change_ADC=6;
				ADCA_init();
				flg_2min10ms=0;	
				adc_change = 1;			
			}
			if(change_ADC==3)
			{
				adc = adc + (0.2*((float)adc_get_unsigned_result(&ADCA,ADC_CH0)-adc));
				if (adc<=2470 && adc>=1600)//10 volt battery voltage feedback   2500-->9.3
				{
					bat_count++;
					if (bat_count>=100)
					{
						battery_alarm=1;
						bat_count=0;
					}
				}
				else
				{
					 battery_alarm=0;
					 bat_count=0;
				}
			} 
			
			if(current_ov && flg_alarm)
			Buzzer_PORT.OUTSET = Buzzer_PIN_bm;
			else if(battery_alarm)
			Buzzer_PORT.OUTSET = Buzzer_PIN_bm;	
			else if(shoot_alarm_flg && (charge_count>=30000))
			Buzzer_PORT.OUTSET = Buzzer_PIN_bm;		
			else
			Buzzer_PORT.OUTCLR = Buzzer_PIN_bm;

			////////////////////////////////////////////////////////////////////////////////SHOOT
			PORTC_OUTCLR=KCK_SH_PIN_bm;
			if(((KCK_Ch_Limit_PORT.IN & KCK_Ch_Limit_PIN_bm)>>KCK_Ch_Limit_PIN_bp))
			{
				full_charge=1;
				tc_disable_cc_channels(&TCC0,TC_CCCEN);
				charge_count=0;
			}
			else // if(((KCK_Ch_Limit_PORT.IN & KCK_Ch_Limit_PIN_bm)>>KCK_Ch_Limit_PIN_bp==0)) 
			{
				if(((flg_chip || flg_dir || flg_sw)==0))//(flg_dir==0)  //not tested!
					{
					tc_enable_cc_channels(&TCC0,TC_CCCEN);
					}
			}
			if (charge_flg)//full_charge ||
			{
				//LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
				if (Robot_D[RobotID].KCK )
				{
					if( KCK_Sens || (Robot_D[RobotID].KCK%2))
					{
					flg_dir = 1;	
					}
				}
				if (Robot_D[RobotID].CHP)
				{
					LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
					if(KCK_Sens || (Robot_D[RobotID].CHP%2))
					{
					flg_chip = 1;	
					}
				}
			}
			if (KCK_DSH_SW )
			{
				//flg_chip = 1;
// 				if(KCK_Sens)
// 				{
 					flg_sw = 1;
// 				}
			}
			//if (free_wheel >= 10000 )
			//{
				//NRF_init();
			//}
			if(wireless_reset>=50)
			{
			NRF_init();
			wireless_reset=0;
			}
			LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
			if(KCK_Sens)
			 LED_Green_PORT.OUTSET = LED_Green_PIN_bm;
			else
			 LED_Green_PORT.OUTCLR = LED_Green_PIN_bm;	
	  }
	   
}

ISR(PORTD_INT0_vect)////////////////////////////////////////PTX   IRQ Interrupt Pin
{  
	  uint8_t status_L = NRF24L01_L_WriteReg(W_REGISTER | STATUSe, _TX_DS|_MAX_RT|_RX_DR);
	  if((status_L & _RX_DR) == _RX_DR)
	  {
		  LED_White_PORT.OUTTGL = LED_White_PIN_bm;
		  wireless_reset=0;
		   wdt_reset();
		  //1) read payload through SPI,
		  NRF24L01_L_Read_RX_Buf(Buf_Rx_L, _Buffer_Size);
		  free_wheel=0 ;
		  if((Buf_Rx_L[0] == 0x0A && (RobotID < 3 || (RobotID<9 && RobotID>5)))|| (Buf_Rx_L[0] == 0xA0 && (RobotID > 8 || (RobotID<6 && RobotID>2))))
		  {
			  LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
			  Robot_D[RobotID].RID  = Buf_Rx_L[0];
			  Robot_D[RobotID].M0a  = Buf_Rx_L[1+ RobotID%3 * 10];
			  Robot_D[RobotID].M0b  = Buf_Rx_L[2+ RobotID%3 * 10];
			  Robot_D[RobotID].M1a  = Buf_Rx_L[3+ RobotID%3 * 10];
			  Robot_D[RobotID].M1b  = Buf_Rx_L[4+ RobotID%3 * 10];
			  Robot_D[RobotID].M2a  = Buf_Rx_L[5+ RobotID%3 * 10];
			  Robot_D[RobotID].M2b  = Buf_Rx_L[6+ RobotID%3 * 10];
			  Robot_D[RobotID].M3a  = Buf_Rx_L[7+ RobotID%3 * 10];
			  Robot_D[RobotID].M3b  = Buf_Rx_L[8+ RobotID%3 * 10];
			  Robot_D[RobotID].KCK  = Buf_Rx_L[9+ RobotID%3 * 10];
			  Robot_D[RobotID].CHP  = Buf_Rx_L[10+RobotID%3 * 10];
			  Robot_D[RobotID].ASK  = Buf_Rx_L[31];//0b00000000
			  
			  if (Robot_D[RobotID].ASK != Robot_Select)
			  {
				  Robot_Select = Robot_D[RobotID].ASK;
				  if (Robot_Select == RobotID)
				  {
					  NRF24L01_L_WriteReg(W_REGISTER | EN_AA, 0x01);
				  }
				  else
				  {
					  NRF24L01_L_WriteReg(W_REGISTER | EN_AA, 0x00);
				  }
			  }
			  
			  if (Robot_D[RobotID].ASK == RobotID)
			  {
				  data_transmission();
			  }

		  }
		  //2) clear RX_DR IRQ,
		  //NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _RX_DR );
		  //3) read FIFO_STATUS to check if there are more payloads available in RX FIFO,
		  //4) if there are more data in RX FIFO, repeat from step 1).
	  }
	  if((status_L&_TX_DS) == _TX_DS)
	  {
		  LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
		  wireless_reset=0;
	  }
	  
	  if ((status_L&_MAX_RT) == _MAX_RT)
	  {
		  NRF24L01_L_Flush_TX();
	  }
}

char timectrl;//,time2sec;
long int t_alarm;
ISR(TCE1_OVF_vect)//1ms
{
	wdt_reset();
	charge_count++;
		shoot_alarm_time++;
		
		if (shoot_alarm_time>=200)
		{
			shoot_alarm_flg = ~(shoot_alarm_flg);
			shoot_alarm_time=0;
		}
	
	t_allow++;
	if(t_allow>1000)   cur_allow=1;
	
	t++;
	if (t>=10)
	{
		t_10ms=1;
		t=0;
	}
	t_alarm++;
	if (t_alarm>=500)
	{
		flg_alarm = ~(flg_alarm);
		t_alarm=0;
	}
	
	t_1ms++;
	//if (t_1ms==60000)
	//{
		//flg_2min=1;				
	//}
	//else if (t_1ms>=61000)
	//{
		//flg_2min10ms=1;
		//t_1ms=0;
	//}
	
	timectrl++;
	wireless_reset++;
	free_wheel++;

	
	time2sec++;
	//if (time2sec>=10)
	//{
		//flag2sec++;
		//time2sec=0;
	//}
		
		if(((KCK_Ch_Limit_PORT.IN & KCK_Ch_Limit_PIN_bm)>>KCK_Ch_Limit_PIN_bp))
		{
		full_charge=1;
		tc_disable_cc_channels(&TCC0,TC_CCCEN);
		charge_count=0;
		}
		else
		{
		if((flg_chip || flg_dir || flg_sw)==0)//|| battery_low)==0)//(flg_dir==0)  //not tested!
		{
		tc_enable_cc_channels(&TCC0,TC_CCCEN);
		//LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
		}
		}
		if (charge_flg)//full_charge ||
		{
			//LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
			if (Robot_D[RobotID].KCK )
			{
				if( KCK_Sens || (Robot_D[RobotID].KCK%2))
				{
					flg_dir = 1;
				}
			}
			if (Robot_D[RobotID].CHP)
			{
				//LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
				if(KCK_Sens || (Robot_D[RobotID].CHP%2))
				{
					flg_chip = 1;
				}
			}
		}
		if (KCK_DSH_SW )
		{
			flg_sw = 1;
		}
		//
		//if(wireless_reset>=50)
		//{
			//NRF_init();
			//wireless_reset=0;
		//}
	
	charge_time++;
	if(charge_time>=3100)
	charge_flg=1;
			
	if ((flg_dir & flg_chip)==1)
	{
	flg_dir = 1;
	flg_chip = 0;
	}
	
	if(flg_sw)
	{
		if(kck_time_sw<3000)
		{
			kck_time_sw++;
			tc_disable_cc_channels(&TCC0,TC_CCCEN);
			if(((KCK_DCh_Limit_PORT.IN & KCK_DCh_Limit_PIN_bm)>>KCK_DCh_Limit_PIN_bp))
			tc_disable_cc_channels(&TCC0,TC_CCDEN);
			else
			{
			tc_enable_cc_channels(&TCC0,TC_CCDEN);
			KCK_Speed_DIR(KCK_SPEED_LOW);
			full_charge=0;
			charge_flg=0;
			charge_time=0;
			}
		}
	    else 
		{
			//if(battery_low==0)
			//{
			KCK_Speed_DIR(KCK_SPEED_OFF);
			tc_enable_cc_channels(&TCC0,TC_CCCEN);
		    kck_time_sw=0; kck_time_dir=0; flg_sw=0;
		}
	}
	if(flg_dir)
	{    
		if(kck_time_dir<100)
		{
			kck_time_dir++;
			tc_disable_cc_channels(&TCC0,TC_CCCEN);
			if(((KCK_DCh_Limit_PORT.IN & KCK_DCh_Limit_PIN_bm)>>KCK_DCh_Limit_PIN_bp))
				tc_disable_cc_channels(&TCC0,TC_CCDEN);
			else
			{
				if(charge_flg)//full_charge ||
				{
					tc_enable_cc_channels(&TCC0,TC_CCDEN);
					KCK_Speed_DIR(Robot_D[RobotID].KCK);
					full_charge=0;
					charge_flg=0;
					charge_time=0;
				}
			}
		}
		
		else {
			//if(battery_low==0)
			//{
			KCK_Speed_DIR(KCK_SPEED_OFF);
			tc_enable_cc_channels(&TCC0,TC_CCCEN);
			kck_time_dir=0; flg_dir=0;
		}
	}
	
	if(flg_chip)
	{
		if(kck_time_chip<100)
		{
			kck_time_chip++;
			tc_disable_cc_channels(&TCC0,TC_CCCEN);
			//LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
			if(((KCK_DCh_Limit_PORT.IN & KCK_DCh_Limit_PIN_bm)>>KCK_DCh_Limit_PIN_bp))
			tc_disable_cc_channels(&TCC1,TC_CCAEN);
			else
			{
				
			    if(full_charge==1)
				{
					tc_enable_cc_channels(&TCC1,TC_CCAEN);
					KCK_Speed_CHIP(Robot_D[RobotID].CHP);
					full_charge=0;
				}
			}
		}
		else {
			//if(battery_low==0)
			//{
			KCK_Speed_CHIP(KCK_SPEED_OFF);
			tc_enable_cc_channels(&TCC0,TC_CCCEN);
			kck_time_chip=0; flg_chip=0;
		}
	}
}
//ISR(TCE0_OVF_vect)//10s
//{
	//t_10s++;
	//if (t_10s>=12)
	//{
		//flg_2min=1;
		//t_10s=0;
	//}
//}

ISR(TCD0_OVF_vect)
{
	wdt_reset();
	CLK_par_PORT.OUTTGL = CLK_par_bm;	
};

ISR(TCD0_CCA_vect)
{   
	if ( free_wheel>100 || current_ov)// || battery_low )
	{
		Robot_D[RobotID].M0a = 1;
		Robot_D[RobotID].M0b = 2;
		Robot_D[RobotID].M1a = 3;
		Robot_D[RobotID].M1b = 4;
	}
	
	if (Robot_D[RobotID].M0a == 1  &&
		Robot_D[RobotID].M0b == 2  &&
		Robot_D[RobotID].M1a == 3  &&
		Robot_D[RobotID].M1b == 4)
		{
			free_wheel_flag = 1;
		}
		
	else
		free_wheel_flag = 0;
	
	if(SW_TEST)
	{
		//////Micro to FPGA communication test number 1 test number 2  (comment the data packet received from wireless)
		Robot_D[RobotID].M0b  = 0xB8;//low
		Robot_D[RobotID].M0a  = 0x0B;//high ---> speed=3000
		Robot_D[RobotID].M1b  = 0XB8;
		Robot_D[RobotID].M1a  = 0X0B;
		Robot_D[RobotID].M2b  = 0XB8;
		Robot_D[RobotID].M2a  = 0X0B;
		Robot_D[RobotID].M3b  = 0xB8;
		Robot_D[RobotID].M3a  = 0X0B;
		//wdt_reset_mcu();
	}
	switch (motor_num)
	{
		case 0 :
		//MOTORNUM_PORT.OUTCLR= (MOTORNUM0_bm | MOTORNUM1_bm);
		if(((CLK_par_PORT.IN && CLK_par_bm)))
		{
			FPGA_DATA_PORT.OUT = 0xAA;//low byte
			//PARITY_PORT.OUTCLR = PARITY_bm;
			STARTBIT_PORT.OUTSET = STARTBIT_bm;
			motor_num++;
			//PARITY_PORT.OUTSET =(parity_calc(Robot_D[RobotID].M0a)<<PARITY_bp);
		}
		break;
		case 1:
		{
			motor_num++;
			STARTBIT_PORT.OUTSET = STARTBIT_bm;
		}
		break;
		//case 2 :
		//MOTORNUM_PORT.OUTCLR= (MOTORNUM0_bm | MOTORNUM1_bm);
		//if(((CLK_par_PORT.IN && CLK_par_bm)))
		//{
			//FPGA_DATA_PORT.OUT = 0x55;
			//PARITY_PORT.OUTSET = PARITY_bm;
			//STARTBIT_PORT.OUTSET = STARTBIT_bm;
			//motor_num++;
			////PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M1a)<<PARITY_bp);
		//}
		//break;
		//case 3:
		//{
			//motor_num++;
		//}
		//break;
		case 2 :
			MOTORNUM_PORT.OUTCLR= (MOTORNUM0_bm | MOTORNUM1_bm);
			if(((CLK_par_PORT.IN && CLK_par_bm)))
			{
			FPGA_DATA_PORT.OUT = Robot_D[RobotID].M0b;//low byte
			PARITY_PORT.OUTCLR = PARITY_bm;
			STARTBIT_PORT.OUTCLR = STARTBIT_bm;
			motor_num++;
			//PARITY_PORT.OUTSET =(parity_calc(Robot_D[RobotID].M0a)<<PARITY_bp);
			}
		break;
		case 3:
		     {
				 motor_num++;
				 STARTBIT_PORT.OUTCLR = STARTBIT_bm;
			 }
		break;
		case 4 :
			MOTORNUM_PORT.OUTCLR= (MOTORNUM0_bm | MOTORNUM1_bm);
			if(((CLK_par_PORT.IN && CLK_par_bm)))
			{
			FPGA_DATA_PORT.OUT = Robot_D[RobotID].M0a;
			PARITY_PORT.OUTSET = PARITY_bm;
			STARTBIT_PORT.OUTCLR = STARTBIT_bm;
			motor_num++;
			//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M1a)<<PARITY_bp);
			}
		break;
		case 5:
			{
			motor_num++;
			STARTBIT_PORT.OUTCLR = STARTBIT_bm;
			}
			break;
		case 6 :
			MOTORNUM_PORT.OUT= MOTORNUM0_bm;
			if(((CLK_par_PORT.IN && CLK_par_bm)))
			{
			FPGA_DATA_PORT.OUT = Robot_D[RobotID].M1b;
			PARITY_PORT.OUTCLR = PARITY_bm;
			STARTBIT_PORT.OUTCLR = STARTBIT_bm;
			motor_num++;
			//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M2a)<<PARITY_bp);
			}
			break;
		case 7:
			{
			motor_num++;
			STARTBIT_PORT.OUTCLR = STARTBIT_bm;
			}
		    break;
		case 8 :
			MOTORNUM_PORT.OUT= MOTORNUM0_bm;
			if(((CLK_par_PORT.IN && CLK_par_bm)))
			{
			FPGA_DATA_PORT.OUT = Robot_D[RobotID].M1a;
			PARITY_PORT.OUTSET = PARITY_bm;
			STARTBIT_PORT.OUTCLR = STARTBIT_bm;
			motor_num++;
			//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M3a)<<PARITY_bp);
			}
			break;
		case 9:
			{
			motor_num++;
			STARTBIT_PORT.OUTCLR = STARTBIT_bm;
			}
		    break;
		case 10 :
			MOTORNUM_PORT.OUT= MOTORNUM1_bm;
			if(((CLK_par_PORT.IN && CLK_par_bm)))
			{
				FPGA_DATA_PORT.OUT = Robot_D[RobotID].M2b;
				PARITY_PORT.OUTCLR = PARITY_bm;
				STARTBIT_PORT.OUTCLR = STARTBIT_bm;
				motor_num++;
				//PARITY_PORT.OUTSET =(parity_calc(Robot_D[RobotID].M0a)<<PARITY_bp);
			}
		    break;
		case 11:
			{
				motor_num++;
				STARTBIT_PORT.OUTCLR = STARTBIT_bm;
			}
			break;
		case 12 :
			MOTORNUM_PORT.OUT= MOTORNUM1_bm;
			if(((CLK_par_PORT.IN && CLK_par_bm)))
			{
				FPGA_DATA_PORT.OUT = Robot_D[RobotID].M2a;
				PARITY_PORT.OUTSET = PARITY_bm;
				STARTBIT_PORT.OUTCLR = STARTBIT_bm;
				motor_num++;
				//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M1a)<<PARITY_bp);
			}
			break;
		case 13:
			{
			motor_num++;
			STARTBIT_PORT.OUTCLR = STARTBIT_bm;
			}
			break;
		case 14 :
			MOTORNUM_PORT.OUT= (MOTORNUM0_bm | MOTORNUM1_bm);
			if(((CLK_par_PORT.IN && CLK_par_bm)))
			{
				FPGA_DATA_PORT.OUT = Robot_D[RobotID].M3b;
				PARITY_PORT.OUTCLR = PARITY_bm;
				STARTBIT_PORT.OUTCLR = STARTBIT_bm;
				motor_num++;
				//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M2a)<<PARITY_bp);
			}
			break;
		case 15:
			{
				motor_num++;
				STARTBIT_PORT.OUTCLR = STARTBIT_bm;
			}
			break;
		case 16 :
			MOTORNUM_PORT.OUT= (MOTORNUM0_bm | MOTORNUM1_bm);
			if(((CLK_par_PORT.IN && CLK_par_bm)))
			{
				//LED_White_PORT.OUTTGL = LED_White_PIN_bm;
				FPGA_DATA_PORT.OUT = Robot_D[RobotID].M3a;
				PARITY_PORT.OUTSET = PARITY_bm;
				STARTBIT_PORT.OUTCLR = STARTBIT_bm;
				motor_num++;
				//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M3a)<<PARITY_bp);
			}
			break;
		case 17:
			{
				motor_num=0;
				STARTBIT_PORT.OUTCLR = STARTBIT_bm;
			}
			break;
		}
		
	  ////motor_num++;
	  ////if (motor_num>=16)
	  ////motor_num=0;
	}

int parity_calc(signed int data)
{
	int parity=0;
	parity = (data & PIN0_bm) ^ ((data & PIN1_bm)>>PIN1_bp) ^ ((data & PIN2_bm)>>PIN2_bp) ^ ((data & PIN3_bm)>>PIN3_bp)
	 ^ ((data & PIN4_bm)>>PIN4_bp) ^ ((data & PIN5_bm)>>PIN5_bp) ^ ((data & PIN6_bm)>>PIN6_bp) ^ ((data & PIN7_bm)>>PIN7_bp);
    return parity;
}

void NRF_init (void)
{
	NRF24L01_L_CE_LOW;       //disable transceiver modes
	SPI_Init();

	_delay_us(10);
	_delay_ms(11);      //power on reset delay needs 10.3ms//amin changed 100ms to 11ms
	NRF24L01_L_Clear_Interrupts();
	NRF24L01_L_Flush_TX();
	NRF24L01_L_Flush_RX();
	NRF24L01_L_CE_LOW;
	// 	    if (RobotID < 3)
	// 	    NRF24L01_L_Init_milad(_TX_MODE, _CH_0, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	// 	    else if(RobotID > 2 && RobotID < 6)
	// 	    NRF24L01_L_Init_milad(_TX_MODE, _CH_1, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	// 	    else if (RobotID > 5 && RobotID < 9)
	// 	    NRF24L01_L_Init_milad(_TX_MODE, _CH_2, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	// 	    else
	// 	    NRF24L01_L_Init_milad(_TX_MODE, _CH_3, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	if (RobotID < 6)
	NRF24L01_L_Init_milad(_RX_MODE, _CH_1, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	else if(RobotID > 5)
	NRF24L01_L_Init_milad(_RX_MODE, _CH_0, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	NRF24L01_L_WriteReg(W_REGISTER | DYNPD,0x01);
	NRF24L01_L_WriteReg(W_REGISTER | FEATURE,0x06);

	NRF24L01_L_CE_HIGH;
	_delay_us(130);
}

int driver_adc(int x)
{
	switch (x)
	{
		case 0: return adc_get_unsigned_result(&ADCB,ADC_CH0); break;
		case 1: return adc_get_unsigned_result(&ADCB,ADC_CH1); break;
		case 2: return adc_get_unsigned_result(&ADCA,ADC_CH1); break;
		case 3: return adc_get_unsigned_result(&ADCA,ADC_CH0); break;
	}
}
void calc_offset(int x)
{
	Driver.offset[x] =0;
	for (int i=0;i<50;i++)
	{
		Driver.adc[x]=(driver_adc(x));
		Driver.offset[x] = Driver.offset[x] + Driver.adc[x];
		
	}
	Driver.offset[x] = (Driver.offset[x])/50;
	Driver.offset[x] = ((float)Driver.offset[x]) * 0.86;
	
}
void read_DriverCurrent(int x)
{
	Driver.vol_last[x] = Driver.vol[x];	
	Driver.adc[x] = driver_adc(x);	
	Driver.vol[x] = ((float)Driver.adc[x]) * 0.86;//voltage khorujie sensor jaryan barhasbe milivolt   (3.3*1000/4096)/0.937   0.937 factore taghsim voltage
	Driver.vol[x] = Driver.vol_last[x] + (0.1*(float)(Driver.vol[x] - Driver.vol_last[x]));	
	Driver.cur[x] = (Driver.vol[x] - Driver.offset[x]) * 5.33;	
	if(Driver.cur[x]<5) Driver.cur[x]=0;
}
void chek_DriverCurrent(int x)
{
	if (Driver.cur[x]>=2000)
	{
		Driver.count_H[x] ++;
		if (Driver.count_H[x] >200)// && count<200)
		{
			Driver.cur_alarm[x] = 1;
			Driver.count_H[x] = 0;
		}
	}
	else if(Driver.cur[x]>=1000)
	{
		Driver.count_L[x] ++;
		if (Driver.count_L[x] >800)// && count<200)
		{
			Driver.cur_alarm[x] = 1;
			Driver.count_L[x] = 0;
		}
	}
	else	{Driver.count_H[x] = 0;Driver.count_L[x] = 0;}
		
}

void data_transmission (void)
{
	//transmitting data to wireless board/////////////////////////////////////////////////

			Test_Data[7] = adc*0.4761;//battery voltage
			Test_Data[0] = adc*0.4761;//battery voltage
			
	
// 	Buf_Tx_L[0]  = (Test_Data[0]>> 8) & 0xFF;	//drive test data
// 	Buf_Tx_L[1]  = Test_Data[0] & 0xFF;			//drive test data
// 	Buf_Tx_L[2]  = Robot_D[RobotID].M1a;//(Test_Data[1]>> 8) & 0xFF;	//drive test data
// 	Buf_Tx_L[3]  = Robot_D[RobotID].M1b;//Test_Data[1] & 0xFF;			//drive test data
// 	Buf_Tx_L[4]  = Robot_D[RobotID].M2a;//(Test_Data[2]>> 8) & 0xFF;	//drive test data
// 	Buf_Tx_L[5]  = Robot_D[RobotID].M2b;//Test_Data[2] & 0xFF;			//drive test data
// 	Buf_Tx_L[6]  = Robot_D[RobotID].M3a;//(Test_Data[3]>> 8) & 0xFF;	//drive test data
// 	Buf_Tx_L[7]  = Robot_D[RobotID].M3b;//Test_Data[3] & 0xFF;			//drive test data
// 	 		//Test_Data[0] = adc/12;
// 	 		//Test_Data[1] = time_test;
	//
	Buf_Tx_L[0]  = (Test_Data[0]>> 8) & 0xFF;//Robot_D[RobotID].M0a;//	//drive test data
	Buf_Tx_L[1]  = Test_Data[0] & 0xFF;//Robot_D[RobotID].M0b;//			//drive test data
	Buf_Tx_L[2]  = (Test_Data[1]>> 8) & 0xFF;//Robot_D[RobotID].M1a;//	//drive test data
	Buf_Tx_L[3]  = Test_Data[1] & 0xFF;	//Robot_D[RobotID].M1b;//		//drive test data
	Buf_Tx_L[4]  = (Test_Data[2]>> 8) & 0xFF;//Robot_D[RobotID].M2a;//	//drive test data
	Buf_Tx_L[5]  = Test_Data[2] & 0xFF;//Robot_D[RobotID].M2b;//			//drive test data
	Buf_Tx_L[6]  = (Test_Data[3]>> 8) & 0xFF;//Robot_D[RobotID].M3a;//	//drive test data
	Buf_Tx_L[7]  = Test_Data[3] & 0xFF;//Robot_D[RobotID].M3b;//			//drive test data


	//Buf_Tx_L[8]  = (Test_Data[4]>> 8) & 0xFF;	// unused
	//Buf_Tx_L[9]  = Test_Data[4] & 0xFF;			// unused
	//Buf_Tx_L[10] = (Test_Data[5]>> 8) & 0xFF;// unused
	//Buf_Tx_L[11] = Test_Data[5] & 0xFF;			// unused
	//Buf_Tx_L[12] = (Test_Data[6]>> 8) & 0xFF;	// unused
	//Buf_Tx_L[13] = Test_Data[6] & 0xFF;			// unused
	Buf_Tx_L[14] = (Test_Data[7]>> 8) & 0xFF;	// battery voltage
	Buf_Tx_L[15] = Test_Data[7] & 0xFF;			// battery voltage
	Buf_Tx_L[16] = adc/12;						//battery adc
	

	//LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
	NRF24L01_L_Write_TX_Buf(Buf_Tx_L, _Buffer_Size);
	//NRF24L01_L_RF_TX();
}