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

unsigned char Buf_Rx_L[_Buffer_Size] ;
char Buf_Tx_L[_Buffer_Size];
char Address[_Address_Width] = { 0x11, 0x22, 0x33, 0x44, 0x55};
int motor_num=0,test=0;
uint32_t kck_time;
int free_wheel=0;
int adc =0;
int flg=0;

char ctrlflg=0,full_charge=0;;
uint64_t flag2sec=0;
int i=0;
int parity_calc(signed int data);

int main (void)
{
	En_RC32M();

	//Enable LowLevel & HighLevel Interrupts
	PMIC_CTRL |= PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm |PMIC_MEDLVLEN_bm;

	PORT_init();
	TimerD0_init();
	TimerC0_init();
	TimerE1_init();
	USARTE0_init();
	ADCA_init();
	//wdt_enable();

	// Globally enable interrupts
	sei();

	Address[0]=Address[0] + RobotID ;

	///////////////////////////////////////////////////////////////////////////////////////////////Begin NRF Initialize
	NRF24L01_L_CE_LOW;       //disable transceiver modes

	SPI_Init();

	_delay_us(10);
	_delay_ms(100);      //power on reset delay needs 100ms
	NRF24L01_L_Clear_Interrupts();
	NRF24L01_L_Flush_TX();
	NRF24L01_L_Flush_RX();
	NRF24L01_L_CE_LOW;
	if (RobotID < 3)
	NRF24L01_L_Init_milad(_TX_MODE, _CH_0, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	else if(RobotID > 2 && RobotID < 6)
	NRF24L01_L_Init_milad(_TX_MODE, _CH_1, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
   else if(RobotID > 5 && RobotID < 9)
	NRF24L01_L_Init_milad(_TX_MODE, _CH_2, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	else
	NRF24L01_L_Init_milad(_TX_MODE, _CH_3, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	
	NRF24L01_L_WriteReg(W_REGISTER | DYNPD,0x01);
	NRF24L01_L_WriteReg(W_REGISTER | FEATURE,0x06);

	NRF24L01_L_CE_HIGH;
	_delay_us(130);
	/////////////////////////////////////////////////////////////////////////////////////////////END   NRF Initialize
	while(1)
	  {  
		  //if(((KCK_DCh_Limit_PORT.IN & KCK_DCh_Limit_PIN_bm)>>KCK_DCh_Limit_PIN_bp))
		  //{
			  //LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
		  //}
		   // BUZZER
		    adc = adc_get_unsigned_result(&ADCA,ADC_CH0);
		   //adc = 1200;
		    if (adc<=2250)//10 volt
		    {
			    Buzzer_PORT.OUTSET = Buzzer_PIN_bm;
		    }
		    else
		    {
			    Buzzer_PORT.OUTCLR = Buzzer_PIN_bm;
		    }
			
			//SHOOT
			PORTC_OUTCLR=KCK_SH_PIN_bm;
			//tc_enable_cc_channels(&TCC0,TC_CCDEN);
			if((KCK_Ch_Limit_PORT.IN & KCK_Ch_Limit_PIN_bm)>>KCK_Ch_Limit_PIN_bp)
			{
				full_charge=1;
				tc_disable_cc_channels(&TCC0,TC_CCDEN);
				//LED_White_PORT.OUTTGL = LED_White_PIN_bm;
			}
			else
			{
				if(flg==0)
				{
					tc_enable_cc_channels(&TCC0,TC_CCDEN);
				}
			}
			if (full_charge)
			{
				if (Robot_D[RobotID].KCK )
				{
					flg = 1;
				}
			}
			if (KCK_DSH_SW)//bazi vaghta begir nagir dare
			{
				flg = 1;
				//LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
			}
			
		   // //motor test
		   //switch(flag2sec)
			  //{ case 200:
			   //// M.Setpoint=1000;
			   //Robot_D[RobotID].M0b  = 0xE8;//low
			   //Robot_D[RobotID].M0a  = 0X03;//high
			   //break;
			   //
			   //case 400:
			   ////M.Setpoint=2000;
			   //Robot_D[RobotID].M0b  = 0xD0;//low
			   //Robot_D[RobotID].M0a  = 0X07;//high
			   //break;
			   //
			   //case 600:
			   ////M.Setpoint=500;
			   //Robot_D[RobotID].M0b  = 0xF4;//low
			   //Robot_D[RobotID].M0a  = 0X01;//high
			   //break;
			   //
			   //case 800:
			   ////M.Setpoint=4000;
			   //Robot_D[RobotID].M0b  = 0xA0;//low
			   //Robot_D[RobotID].M0a  = 0X0F;//high
			   //break;
			   //
			   //case 1000:
			   ////M.Setpoint=1000;
			   //Robot_D[RobotID].M0b  = 0xE8;//low
			   //Robot_D[RobotID].M0a  = 0X03;//high
			   //break;
			   //
			   //case 1200:
			   ////M.Setpoint=500;
			   //Robot_D[RobotID].M0b  = 0xF4;//low
			   //Robot_D[RobotID].M0a  = 0X01;//high
			   //break;
			   //
			   //case 1400:
			   ////M.Setpoint=-500;
			   //Robot_D[RobotID].M0b  = 0x0C;//low
			   //Robot_D[RobotID].M0a  = 0XFE;//high
			   //break;
			   //
			   //case 1600:
			   ////M.Setpoint=400;
			   //Robot_D[RobotID].M0b  = 0x90;//low
			   //Robot_D[RobotID].M0a  = 0X01;//high
			   //break;
			   //
			   //case 1800:
			   ////M.Setpoint=350;
			   //Robot_D[RobotID].M0b  = 0x5E;//low
			   //Robot_D[RobotID].M0a  = 0X01;//high
			   //break;
			   //
			   //case 2000:
			   ////M.Setpoint=340;
			   //Robot_D[RobotID].M0b  = 0x54;//low
			   //Robot_D[RobotID].M0a  = 0X01;//high
			   //break;
			   //
			   //case 2200:
			   ////M.Setpoint=330;
			   //Robot_D[RobotID].M0b  = 0x4A;//low
			   //Robot_D[RobotID].M0a  = 0X01;//high
			   //break;
			   //
			   //case 2600:
			   ////M.Setpoint=100;
			   //Robot_D[RobotID].M0b  = 0x64;//low
			   //Robot_D[RobotID].M0a  = 0X00;//high
			   //break;
			   //
			   //case 2800:
			   ////M.Setpoint=50;
			   //Robot_D[RobotID].M0b  = 0x32;//low
			   //Robot_D[RobotID].M0a  = 0X00;//high
			   //break;
			   //
			   //case 3000:
			   ////M.Setpoint=1000;
			   //Robot_D[RobotID].M0b  = 0xE8;//low
			   //Robot_D[RobotID].M0a  = 0X03;//high
			   //break;
			   //
			   //case 3200:
			   ////M.Setpoint=-50;
			   //Robot_D[RobotID].M0b  = 0xCE;//low
			   //Robot_D[RobotID].M0a  = 0XFF;//high
			   //flag2sec=0;
			   //break;
	  //
			   //}
		 // Robot_D[RobotID].M0b  = 0x00;//0X18;//-1000//01;//low37121
		 // Robot_D[RobotID].M0a  = 0xFF;//0XFC;//high
		  //Robot_D[RobotID].M1b  = 0XE8;//2000//ghalat17325
		  //Robot_D[RobotID].M1a  = 0X03;
		  //Robot_D[RobotID].M2b  = 0XE8;//1000//low13703
		  //Robot_D[RobotID].M2a  = 0X03;//high
		  //Robot_D[RobotID].M3b  = 0xB8;//3000//32;//ghalat30258
		  //Robot_D[RobotID].M3a  = 0X0B;//76;
		  
		  //LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
		 // (Robot_D[RobotID].M0a == 1) && (Robot_D[RobotID].M0b == 2) && (Robot_D[RobotID].M1a==3) && (Robot_D[RobotID].M1b == 4) ||
		//  if ( free_wheel>100)
		  //{  
			  //Robot_D[RobotID].M0a = 1;
			  //Robot_D[RobotID].M0b = 2;
			  //Robot_D[RobotID].M1a = 3;
			  //Robot_D[RobotID].M1b = 4;
		 // }
		  free_wheel++;
		  
		  
		    asm("wdr");
		    if (ctrlflg)
		    {
			    ctrlflg = 0;
				
				Buf_Tx_L[0] = Robot_D[RobotID].M0a;
				Buf_Tx_L[1] = Robot_D[RobotID].M0b;
				Buf_Tx_L[2] = Robot_D[RobotID].M1a;
				Buf_Tx_L[3] = Robot_D[RobotID].M1b;
				Buf_Tx_L[4] = Robot_D[RobotID].M2a;
				Buf_Tx_L[5] = Robot_D[RobotID].M2b;
				Buf_Tx_L[6] = Robot_D[RobotID].M3a;
				Buf_Tx_L[7] = Robot_D[RobotID].M3b;
			    NRF24L01_L_Write_TX_Buf(Buf_Tx_L,_Buffer_Size);
			    NRF24L01_L_RF_TX();
		    }
		    _delay_us(1);
			
			////SEND TEST DATA TO FT232
			//char str1[20];
		    //uint8_t count1 = sprintf(str1,"%d\r",adc);
			//
			//for (uint8_t i=0;i<count1;i++)
			//{
				//usart_putchar(&USARTE0,str1[i]);
				//
			//}
			//usart_putchar(&USARTE0,'a');
			
	  }
}

ISR(PORTD_INT0_vect)////////////////////////////////////////PTX   IRQ Interrupt Pin
{  
	uint8_t status_L = NRF24L01_L_WriteReg(W_REGISTER | STATUSe, _TX_DS|_MAX_RT|_RX_DR);
	  
	if((status_L & _RX_DR) == _RX_DR)
	{
		LED_White_PORT.OUTTGL = LED_White_PIN_bm;
		//1) read payload through SPI,
		NRF24L01_L_Read_RX_Buf(Buf_Rx_L, _Buffer_Size);
		free_wheel=0 ;
		if(Buf_Rx_L[0] == RobotID)
		{
			Robot_D[RobotID].RID  = Buf_Rx_L[0];
			Robot_D[RobotID].M0a  = Buf_Rx_L[1];
			Robot_D[RobotID].M0b  = Buf_Rx_L[2];
			Robot_D[RobotID].M1a  = Buf_Rx_L[3];
			Robot_D[RobotID].M1b  = Buf_Rx_L[4];
			Robot_D[RobotID].M2a  = Buf_Rx_L[5];
			Robot_D[RobotID].M2b  = Buf_Rx_L[6];
			Robot_D[RobotID].M3a  = Buf_Rx_L[7];
			Robot_D[RobotID].M3b  = Buf_Rx_L[8];
			Robot_D[RobotID].KCK  = Buf_Rx_L[9];
			Robot_D[RobotID].CHP  = Buf_Rx_L[10];
			Robot_D[RobotID].ASK  = Buf_Rx_L[11];
			Robot_D[RobotID].P    = Buf_Rx_L[12];
			Robot_D[RobotID].I    = Buf_Rx_L[13];
			Robot_D[RobotID].D    = Buf_Rx_L[14];
		}

		//2) clear RX_DR IRQ,
		//NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _RX_DR );
		//3) read FIFO_STATUS to check if there are more payloads available in RX FIFO,
		//4) if there are more data in RX FIFO, repeat from step 1).
	}
	if((status_L&_TX_DS) == _TX_DS)
	{   LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
		//NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _TX_DS);
	}
	if ((status_L&_MAX_RT) == _MAX_RT)
	{
		LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
		NRF24L01_L_Flush_TX();
		//NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _MAX_RT);
	}
}

char timectrl,time2sec;
ISR(TCE1_OVF_vect)//1ms
{
	timectrl++;
	if (timectrl>=32) 
	{
		ctrlflg=1;
		timectrl=0;
	}
	//time2sec++;
	//if (time2sec>=10)
	//{
		//flag2sec++;
		//time2sec=0;
	//}
	if(flg)
	{    
		if(kck_time<2000)
		{
			kck_time++;
			tc_disable_cc_channels(&TCC0,TC_CCDEN);
			if(((KCK_DCh_Limit_PORT.IN & KCK_DCh_Limit_PIN_bm)>>KCK_DCh_Limit_PIN_bp))
			//if(KCK_DCh_Limit) // chera moshkel dare?!
				tc_disable_cc_channels(&TCC0,TC_CCCEN);
				
				
			else
			{
				if(KCK_DSH_SW)
				{
					tc_enable_cc_channels(&TCC0,TC_CCCEN);
					KCK_Speed_DIR(KCK_SPEED_HI);
					full_charge=0;
				}
				else if(full_charge==1)
				{
					tc_enable_cc_channels(&TCC0,TC_CCCEN);
					KCK_Speed_DIR(Robot_D[RobotID].KCK);
					full_charge=0;
				}
			}
		}
		
		else {
			KCK_Speed_DIR(KCK_SPEED_OFF);
			tc_enable_cc_channels(&TCC0,TC_CCDEN);
			
		kck_time=0; flg=0;}
	}
}
//
//ISR(USARTE0_RXC_vect)       
//{
	//
//}

ISR(TCD0_OVF_vect)
{
	wdt_reset();
	CLK_par_PORT.OUTTGL = CLK_par_bm;	
};


ISR(TCD0_CCA_vect)
{   
	switch (motor_num)
	{
		case 0 :
			MOTORNUM_PORT.OUTCLR= (MOTORNUM0_bm | MOTORNUM1_bm);
			if((CLK_par_PORT.IN && CLK_par_bm)==1)
			{
			FPGA_DATA_PORT.OUT = Robot_D[RobotID].M0b;
			PARITY_PORT.OUTCLR = PARITY_bm;
			motor_num++;
			//PARITY_PORT.OUTSET =(parity_calc(Robot_D[RobotID].M0a)<<PARITY_bp);
			}
		break;
		case 1:
		     {
				 motor_num++;
			 }
		break;
		case 2 :
			MOTORNUM_PORT.OUTCLR= (MOTORNUM0_bm | MOTORNUM1_bm);
			if((CLK_par_PORT.IN && CLK_par_bm)==1)
			{
			FPGA_DATA_PORT.OUT = Robot_D[RobotID].M0a;
			PARITY_PORT.OUTSET = PARITY_bm;
			motor_num++;
			//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M1a)<<PARITY_bp);
			}
		break;
		case 3:
			{
			motor_num++;
			}
			break;
		case 4 :
			MOTORNUM_PORT.OUT= MOTORNUM0_bm;
			if((CLK_par_PORT.IN && CLK_par_bm)==1)
			{
			
			FPGA_DATA_PORT.OUT = Robot_D[RobotID].M1b;
			PARITY_PORT.OUTCLR = PARITY_bm;
			motor_num++;
			//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M2a)<<PARITY_bp);
			}
			break;
		case 5:
			{
			motor_num++;
			}
		    break;
		case 6 :
			MOTORNUM_PORT.OUT= MOTORNUM0_bm;
			if((CLK_par_PORT.IN && CLK_par_bm)==1)
			{
			//LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
			FPGA_DATA_PORT.OUT = Robot_D[RobotID].M1a;
			PARITY_PORT.OUTSET = PARITY_bm;
			motor_num++;
			//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M3a)<<PARITY_bp);
			}
			break;
		case 7:
			{
			motor_num++;
			}
		    break;
		case 8 :
			MOTORNUM_PORT.OUT= MOTORNUM1_bm;
			if((CLK_par_PORT.IN && CLK_par_bm)==1)
			{
				FPGA_DATA_PORT.OUT = Robot_D[RobotID].M2b;
				PARITY_PORT.OUTCLR = PARITY_bm;
				motor_num++;
				//PARITY_PORT.OUTSET =(parity_calc(Robot_D[RobotID].M0a)<<PARITY_bp);
			}
		    break;
		case 9:
			{
				motor_num++;
			}
			break;
		case 10 :
			MOTORNUM_PORT.OUT= MOTORNUM1_bm;
			if((CLK_par_PORT.IN && CLK_par_bm)==1)
			{
				FPGA_DATA_PORT.OUT = Robot_D[RobotID].M2a;
				PARITY_PORT.OUTSET = PARITY_bm;
				motor_num++;
				//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M1a)<<PARITY_bp);
			}
			break;
		case 11:
			{
			motor_num++;
			}
			break;
		case 12 :
			MOTORNUM_PORT.OUT= (MOTORNUM0_bm | MOTORNUM1_bm);
			if((CLK_par_PORT.IN && CLK_par_bm)==1)
			{
				FPGA_DATA_PORT.OUT = Robot_D[RobotID].M3b;
				PARITY_PORT.OUTCLR = PARITY_bm;
				motor_num++;
				//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M2a)<<PARITY_bp);
			}
			break;
		case 13:
			{
				motor_num++;
			}
			break;
		case 14 :
			MOTORNUM_PORT.OUT= (MOTORNUM0_bm | MOTORNUM1_bm);
			if((CLK_par_PORT.IN && CLK_par_bm)==1)
			{
				//LED_White_PORT.OUTTGL = LED_White_PIN_bm;
				FPGA_DATA_PORT.OUT = Robot_D[RobotID].M3a;
				PARITY_PORT.OUTSET = PARITY_bm;
				motor_num++;
				//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M3a)<<PARITY_bp);
			}
			break;
		case 15:
			{
				motor_num=0;
			}
			break;
		}
	  //motor_num++;
	  //if (motor_num>=16)
	  //motor_num=0;
	}
//ISR(TCD0_CCA_vect)
//{    
	//if (timeFPGA==5)
	  //{   
		  ////LED_White_PORT.OUTTGL = LED_White_PIN_bm;
	    //switch (motor_num)
	    //{
		    //case 0 :
				//MOTORNUM_PORT.OUTCLR= (MOTORNUM0_bm | MOTORNUM1_bm);
				//if((CLK_par_PORT.IN && CLK_par_bm)==0)
				//{   LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
					//FPGA_DATA_PORT.OUT = Robot_D[RobotID].M0a;
					//PARITY_PORT.OUTSET =(parity_calc(Robot_D[RobotID].M0a)<<PARITY_bp);
					//motor_num=0;
				//}
				//else if((CLK_par_PORT.IN && CLK_par_bm)==1)
				//{
					//FPGA_DATA_PORT.OUT = Robot_D[RobotID].M0b;
					//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M0b)<<PARITY_bp); 
					//LED_White_PORT.OUTTGL = LED_White_PIN_bm;
					//motor_num=1;
				//} 
		    //break;
		    //case 1 :
			   //MOTORNUM_PORT.OUT= MOTORNUM0_bm;
			   //if((CLK_par_PORT.IN && CLK_par_bm)==0)
			   //{
				   //FPGA_DATA_PORT.OUT = Robot_D[RobotID].M1a;
				   //PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M1a)<<PARITY_bp);
				   //motor_num=1;
			   //}
			   //else if((CLK_par_PORT.IN && CLK_par_bm)==1)
			   //{
				   //FPGA_DATA_PORT.OUT = Robot_D[RobotID].M1b;
				   //PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M1b)<<PARITY_bp);
				   //motor_num=2;
			   //}
		    //break;
		    //case 2 :
			   //MOTORNUM_PORT.OUT= MOTORNUM1_bm;
			   //if((CLK_par_PORT.IN && CLK_par_bm)==0)
			   //{
				   //FPGA_DATA_PORT.OUT = Robot_D[RobotID].M2a;
				   //PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M2a)<<PARITY_bp);
				   //motor_num=2;
			   //}
			   //else if((CLK_par_PORT.IN && CLK_par_bm)==1)
			   //{
				   //FPGA_DATA_PORT.OUT = Robot_D[RobotID].M2b;
				   //PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M2b)<<PARITY_bp);
				   //motor_num=3;
			   //}
		    //break;
		    //case 3 :
				//MOTORNUM_PORT.OUT= (MOTORNUM0_bm | MOTORNUM1_bm);
				//if((CLK_par_PORT.IN && CLK_par_bm)==0)
				//{
					//FPGA_DATA_PORT.OUT = Robot_D[RobotID].M3a;
					//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M3a)<<PARITY_bp);
					//motor_num=3;
				//}
				//else if((CLK_par_PORT.IN && CLK_par_bm)==1)
				//{
					//FPGA_DATA_PORT.OUT = Robot_D[RobotID].M3b;
					//PARITY_PORT.OUTSET = (parity_calc(Robot_D[RobotID].M3b)<<PARITY_bp);
					////LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
					//motor_num=0;
				//}
		    //break;
	    //}	
	//}
//}
int parity_calc(signed int data)
{
	int parity=0;
	parity = (data & PIN0_bm) ^ ((data & PIN1_bm)>>PIN1_bp) ^ ((data & PIN2_bm)>>PIN2_bp) ^ ((data & PIN3_bm)>>PIN3_bp)
	 ^ ((data & PIN4_bm)>>PIN4_bp) ^ ((data & PIN5_bm)>>PIN5_bp) ^ ((data & PIN6_bm)>>PIN6_bp) ^ ((data & PIN7_bm)>>PIN7_bp);
    return parity;
}