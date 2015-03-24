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
uint32_t kck_time_dir,time_test,kck_time_chip;
int free_wheel=0;
int wireless_reset=0;
////////////////////////////////////////current defines
struct Driver
{
	unsigned int adc[4];
	float vol[4] , vol_last[4];
	unsigned long int offset[4];
	float cur[4];
	uint16_t count[4];
	};
struct Driver Driver;

unsigned int buff_offset[320];
void calc_offset(int);
int driver_adc(int);
uint32_t t_allow,t_test;
int cur_allow=0;
int curr_alarm=0,curr_alarm0,curr_alarm1,curr_alarm2,curr_alarm3,current_ov;
int flg_alarm=0;
int t,t_10ms=0;
//////////////////////////////////////////////
int adc;
int flg_dir=0, flg_chip=0;
int Robot_Select,Robot_Select_Last;
int Test_Data[8];
int time2sec=0;
 struct rpm_ready
 {
	 uint8_t data_permit;
	 signed int Ma;
	 signed int Mb;
 };
 struct rpm_ready rpm_ready[4];

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
	TimerC1_init();
	TimerE1_init();
	USARTE0_init();
	ADCA_init();
	ADCB_init();
	//wdt_enable();

	// Globally enable interrupts
	sei();

	//Address[0]=Address[0] + RobotID ;

	//// NRF Initialize
	NRF_init () ;
	//offset_M1 = 2935.0;
	//offset_M2 = 2979.0;
	//offset_M3 = 2982.0;
	calc_offset(0);
	calc_offset(1);
	calc_offset(2);
	while(1)
	  {  
		    asm("wdr");
			if(t_10ms)
			{
			////////////////////////////////////////////////////////// motor current sensor
			Driver.vol_last[0] = Driver.vol[0];
			Driver.vol_last[1] = Driver.vol[1];
			Driver.vol_last[2] = Driver.vol[2];
			//Driver.vol_last[3] = Driver.vol[3];
			
			Driver.adc[0]=(adc_get_unsigned_result(&ADCB,ADC_CH0));
			Driver.adc[1]=(adc_get_unsigned_result(&ADCB,ADC_CH1));
			Driver.adc[2]=(adc_get_unsigned_result(&ADCA,ADC_CH1));
			//Driver.adc[3]=(adc_get_unsigned_result(&ADCA,ADC_CH2));
			
			Driver.vol[0] = ((float)Driver.adc[0]) * 0.86;//voltage khorujie sensor jaryan barhasbe milivolt   (3.3*1000/4096)/0.937   0.937 factore taghsim voltage
			Driver.vol[1] = ((float)Driver.adc[1]) * 0.86;
			Driver.vol[2] = ((float)Driver.adc[2]) * 0.86;
			//Driver.vol[3] = ((float)Driver.adc[3]) * 0.86;
			
			Driver.vol[0] = Driver.vol_last[0] + (0.1*(float)(Driver.vol[0] - Driver.vol_last[0]));
			Driver.vol[1] = Driver.vol_last[1] + (0.1*(float)(Driver.vol[1] - Driver.vol_last[1]));
			Driver.vol[2] = Driver.vol_last[2] + (0.1*(float)(Driver.vol[2] - Driver.vol_last[2]));
			//Driver.vol[3] = Driver.vol_last[3] + (0.02*(float)(Driver.vol[3] - Driver.vol_last[3]));
			
			Driver.cur[0] = (Driver.vol[0] - Driver.offset[0]) * 5.33;
			Driver.cur[1] = (Driver.vol[1] - Driver.offset[1]) * 5.33;
			Driver.cur[2] = (Driver.vol[2] - Driver.offset[2]) * 5.33;
			//Driver.cur[3] = (Driver.vol[3] - Driver.offset[3]) * 5.33;;
			if(Driver.cur[0]<5) Driver.cur[0]=0;
			if(Driver.cur[1]<5) Driver.cur[1]=0;
			if(Driver.cur[2]<5) Driver.cur[2]=0;
			//if(Driver.cur[3]) Driver.cur[3]=0;
			
			//switch
			//{
				//case Driver.cur[0]>=800 :
				//count ++;
				//break;
				//
			//}
			if(cur_allow)
			{
				if (Driver.cur[0]>=2000)
				{
					Driver.count[0] ++;
					if (Driver.count[0] >80)// && count<200)
					{
						
						
						curr_alarm0 = 1;
						Driver.count[0] = 0;
						
					}
				}
				else	Driver.count[0] = 0;
				
				if (Driver.cur[1]>=1800)
				{
					Driver.count[1] ++;
					if (Driver.count[1] >80)// && count<200)
					{
						
						
						curr_alarm1 = 1;
						Driver.count[1] = 0;
						
					}
				}
				else	Driver.count[1] = 0;
				
				if (Driver.cur[2]>=1800)
				{
					Driver.count[2] ++;
					if (Driver.count[2] >80)// && count<200)
					{
						
						
						curr_alarm2 = 1;
						Driver.count[2] = 0;
						
					}
				}
				else	Driver.count[2] = 0;
				
				//if (Driver.cur[3]>=800)
				//{
				//count[3] ++;
				//if (count[3] >5)// && count<200)
				//{
				//
				//
				//curr_alarm3 = 1;
				//count[3] = 0;
				//
				//}
				//}
				//else	count[3] = 0;
				
			}
			
			current_ov = curr_alarm0 || curr_alarm1 || curr_alarm2 || curr_alarm3;
			
			
			Test_Data[0]=(int)Driver.cur[0];
			Test_Data[1]=(int)Driver.cur[1];
			Test_Data[2]=(int)Driver.cur[2];
			 
			t_10ms=0;
			}
			if (current_ov)	Buzzer_PORT.OUTSET = (flg_alarm>>Buzzer_PIN_bp);
			//
			////cur = ((float)adc)*4.3;  // bar hasbe mA ---> be ezaye har 0.1875mv 1mA taghire jaryan darim!
			//
			////adc = adc_last + (0.2*(float)(adc-adc_last));
			 //
			//
			////adc_I=adc*8.65;//*34.732;	
			////adc_I = adc_I_1 + /*((0.01/(f+0.01))*/ (0.02*(float)(adc_I-adc_I_1));
			//
			//
			//////////////////////////////////////////////////// offset calculating of current sensor output
			////adc_offset=0;
			////for (int i=0;i<200;i++)
			////{
				////adc = adc_get_unsigned_result(&ADCA,ADC_CH0);
				////buff_offset[i]=adc_get_unsigned_result(&ADCA,ADC_CH0)-1985;
				////adc_offset += buff_offset[i];
			//
			////}
			////adc_offset = adc_offset/200;
			////adc_cur_M1 = adc_get_unsigned_result(&ADCB,ADC_CH0);
			////adc_cur_M2 = adc_get_unsigned_result(&ADCB,ADC_CH1);
			////adc_cur_M3 = adc_get_unsigned_result(&ADCA,ADC_CH1);
			////adc_cur_M4 = adc_get_unsigned_result(&ADCA,ADC_CH2);
			//
			//SEND TEST DATA TO FT232
			//char str1[20];
			//uint8_t count1 = sprintf(str1,"%d,%d,%d\r",(int)Driver.cur[0],(int)t_allow,(int)Driver.cur[2]);
			//
			//for (uint8_t i=0;i<count1;i++)
			//{
				//usart_putchar(&USARTE0,str1[i]);
			//}
			///////////////////////////////////////////////////////////
			
			
		    //BUZZER
		    adc = adc_get_unsigned_result(&ADCA,ADC_CH0);
		    if (adc<=2470 && adc>=1250)//10 volt battery voltage feedback
		    {
			    Buzzer_PORT.OUTSET = Buzzer_PIN_bm;
		    }
		    else
		    {
			    Buzzer_PORT.OUTCLR = Buzzer_PIN_bm;
		    }
			
			//SHOOT
			PORTC_OUTCLR=KCK_SH_PIN_bm;
			if((KCK_Ch_Limit_PORT.IN & KCK_Ch_Limit_PIN_bm)>>KCK_Ch_Limit_PIN_bp)
			{
				full_charge=1;
				tc_disable_cc_channels(&TCC0,TC_CCCEN);
			}
			else // if(((KCK_Ch_Limit_PORT.IN & KCK_Ch_Limit_PIN_bm)>>KCK_Ch_Limit_PIN_bp==0)) 
			{
				if((flg_chip & flg_dir)==0)//(flg_dir==0)  //not tested!
					{
					tc_enable_cc_channels(&TCC0,TC_CCCEN);
					}
			}
			if (full_charge)
			{
				if (Robot_D[RobotID].KCK )
				{
					if(KCK_Sens)
					{
					flg_dir = 1;	
					}
					
				}
				if (Robot_D[RobotID].CHP )
				{
					if(KCK_Sens)
					{
					flg_chip = 1;	
					}
					
				}
			}
			if (KCK_DSH_SW)
			{
				//flg_chip = 1;
				//if(KCK_Sens)
				//{
					flg_dir = 1;
				//}
			}
			
			if (free_wheel >= 500 )
			{
				NRF_init();
			}
			if(KCK_Sens)
			 LED_Green_PORT.OUTSET = LED_Green_PIN_bm;
			else
			 LED_Green_PORT.OUTCLR = LED_Green_PIN_bm;	
		 ////Micro to FPGA communication test number 1 (comment the data packet received from wireless)
		   //switch(flag2sec)//time2sec)//flag2sec
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
			   //case 2400:
			   ////M.Setpoint=100;
			   //Robot_D[RobotID].M0b  = 0x64;//low
			   //Robot_D[RobotID].M0a  = 0X00;//high
			   //break;
			   //
			   //case 2600:
			   ////M.Setpoint=50;
			   //Robot_D[RobotID].M0b  = 0x32;//low
			   //Robot_D[RobotID].M0a  = 0X00;//high
			   //break;
			   //
			   //case 2800:
			   ////M.Setpoint=1000;
			   //Robot_D[RobotID].M0b  = 0xE8;//low
			   //Robot_D[RobotID].M0a  = 0X03;//high
			   //break;
			   //
			   //case 3000:
			   ////M.Setpoint=-50;
			   //Robot_D[RobotID].M0b  = 0xCE;//low
			   //Robot_D[RobotID].M0a  = 0XFF;//high
			   //flag2sec=0;
			  //// time2sec=0;
			   //break;
			   //}
			    //switch(time2sec)//flag2sec
			    //{   case 10:
				    //// M.Setpoint=1000;
				    //Robot_D[RobotID].M0b  = 0xE8;//low
				    //Robot_D[RobotID].M0a  = 0X03;//high
				    //break;
				    //
				    //case 20:
				    ////M.Setpoint=2000;
				    //Robot_D[RobotID].M0b  = 0xD0;//low
				    //Robot_D[RobotID].M0a  = 0X07;//high
				    //break;
				    //
				    //case 30:
				    ////M.Setpoint=500;
				    //Robot_D[RobotID].M0b  = 0xF4;//low
				    //Robot_D[RobotID].M0a  = 0X01;//high
				    //break;
				    //
				    //case 40:
				    ////M.Setpoint=4000;
				    //Robot_D[RobotID].M0b  = 0xA0;//low
				    //Robot_D[RobotID].M0a  = 0X0F;//high
				    //break;
				    //
				    //case 50:
				    ////M.Setpoint=1000;
				    //Robot_D[RobotID].M0b  = 0xE8;//low
				    //Robot_D[RobotID].M0a  = 0X03;//high
				    //break;
				    //
				    //case 60:
				    ////M.Setpoint=500;
				    //Robot_D[RobotID].M0b  = 0xF4;//low
				    //Robot_D[RobotID].M0a  = 0X01;//high
				    //break;
				    //
				    //case 70:
				    ////M.Setpoint=-500;
				    //Robot_D[RobotID].M0b  = 0x0C;//low
				    //Robot_D[RobotID].M0a  = 0XFE;//high
				    //break;
				    //
				    //case 80:
				    ////M.Setpoint=400;
				    //Robot_D[RobotID].M0b  = 0x90;//low
				    //Robot_D[RobotID].M0a  = 0X01;//high
				    //break;
				    //
				    //case 90:
				    ////M.Setpoint=350;
				    //Robot_D[RobotID].M0b  = 0x5E;//low
				    //Robot_D[RobotID].M0a  = 0X01;//high
				    //break;
				    //
				    //case 100:
				    ////M.Setpoint=340;
				    //Robot_D[RobotID].M0b  = 0x54;//low
				    //Robot_D[RobotID].M0a  = 0X01;//high
				    //break;
				    //
				    //case 110:
				    ////M.Setpoint=330;
				    //Robot_D[RobotID].M0b  = 0x4A;//low
				    //Robot_D[RobotID].M0a  = 0X01;//high
				    //break;
				    //
				    //case 120:
				    ////M.Setpoint=100;
				    //Robot_D[RobotID].M0b  = 0x64;//low
				    //Robot_D[RobotID].M0a  = 0X00;//high
				    //break;
				    //
				    //case 130:
				    ////M.Setpoint=50;
				    //Robot_D[RobotID].M0b  = 0x32;//low
				    //Robot_D[RobotID].M0a  = 0X00;//high
				    //break;
				    //
				    //case 140:
				    ////M.Setpoint=1000;
				    //Robot_D[RobotID].M0b  = 0xE8;//low
				    //Robot_D[RobotID].M0a  = 0X03;//high
				    //break;
				    //
				    //case 150:
				    ////M.Setpoint=-50;
				    //Robot_D[RobotID].M0b  = 0xCE;//low
				    //Robot_D[RobotID].M0a  = 0XFF;//high
				    ////flag2sec=0;
				     //time2sec=0;
				    //break;
				    //
			    //}
		////Micro to FPGA communication test number 1 test number 2  (comment the data packet received from wireless)
		 //Robot_D[RobotID].M0b  = 0xE8;//low
		 //Robot_D[RobotID].M0a  = 0X03;//high
		 //Robot_D[RobotID].M1b  = 0x0;//low
		 //Robot_D[RobotID].M1a  = 0X00;//high
		  //Robot_D[RobotID].M1b  = 0X00;//2000//ghalat17325
		  //Robot_D[RobotID].M1a  = 0X00;
		  //Robot_D[RobotID].M2b  = 0X0;//1000//low13703
		  //Robot_D[RobotID].M2a  = 0X0;//high
		  //Robot_D[RobotID].M3b  = 0x0;//3000//32;//ghalat30258
		  //Robot_D[RobotID].M3a  = 0X0;//76;
		  
			////SEND TEST DATA TO FT232
			//char str1[20];
		    //uint8_t count1 = sprintf(str1,"%d,%d,%d,%d\r",FPGA_DATA_PORT.IN,MOTORNUM_PORT.IN,((PARITY_PORT.IN & PARITY_bm)>>PARITY_bp),((CLK_par_PORT.IN & CLK_par_bm)>>CLK_par_bp));
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
		  wireless_reset=0;
		  //1) read payload through SPI,
		  NRF24L01_L_Read_RX_Buf(Buf_Rx_L, _Buffer_Size);
		  free_wheel=0 ;
		  if(Buf_Rx_L[0] == 'L')//RobotID)
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
			  
			  //if (rpm_ready[0].data_permit)
			  //{
				  //rpm_ready[0].Ma = Robot_D[RobotID].M0a;
				  //rpm_ready[0].Mb = Robot_D[RobotID].M0b;
			  //}
			  //if (rpm_ready[1].data_permit)
			  //{
				  //rpm_ready[1].Ma = Robot_D[RobotID].M1a;
				  //rpm_ready[1].Mb = Robot_D[RobotID].M1b;
			  //}
			  //if (rpm_ready[2].data_permit)
			  //{
				  //rpm_ready[2].Ma = Robot_D[RobotID].M2a;
				  //rpm_ready[2].Mb = Robot_D[RobotID].M2b;
			  //}
			  //if (rpm_ready[3].data_permit)
			  //{
				  //rpm_ready[3].Ma = Robot_D[RobotID].M3a;
				  //rpm_ready[3].Mb = Robot_D[RobotID].M3b;
			  //}
			  
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
		  //LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
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
	t_allow++;
	if(t_allow>1000)   cur_allow=1;
	t++;
	if (t>=10)
	{
		t_10ms=1;
		t=0;
	}
	t_alarm++;
	time_test++;
	timectrl++;
	wireless_reset++;
	free_wheel++;
	if (t_alarm>=500)
	{
		flg_alarm = ~(flg_alarm);
		t_alarm=0;
		
	}
	if (timectrl>=32) 
	{
		ctrlflg=1;
		timectrl=0;
	}
	time2sec++;
	if (time2sec>=10)
	{
		flag2sec++;
		time2sec=0;
	}
	if ((flg_dir & flg_chip)==1)
	{
	flg_dir = 1;
	flg_chip = 0;
	}
	if(flg_dir)
	{    
		if(kck_time_dir<100)
		{
			kck_time_dir++;
			tc_disable_cc_channels(&TCC0,TC_CCCEN);
			//LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
			if(((KCK_DCh_Limit_PORT.IN & KCK_DCh_Limit_PIN_bm)>>KCK_DCh_Limit_PIN_bp))
				tc_disable_cc_channels(&TCC0,TC_CCDEN);
			else
			{
				if(KCK_DSH_SW)
				{
					tc_enable_cc_channels(&TCC0,TC_CCDEN);
					KCK_Speed_DIR(KCK_SPEED_HI);
					full_charge=0;
				}
				else if(full_charge==1)
				{
					tc_enable_cc_channels(&TCC0,TC_CCDEN);
					KCK_Speed_DIR(Robot_D[RobotID].KCK);
					full_charge=0;
				}
			}
		}
		
		else {
			KCK_Speed_DIR(KCK_SPEED_OFF);
			tc_enable_cc_channels(&TCC0,TC_CCCEN);
			//LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
			kck_time_dir=0; flg_dir=0;}
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
				if(KCK_DSH_SW)
				{
					tc_enable_cc_channels(&TCC1,TC_CCAEN);
					KCK_Speed_CHIP(KCK_SPEED_HI);
					full_charge=0;
				}
				else if(full_charge==1)
				{
					tc_enable_cc_channels(&TCC1,TC_CCAEN);
					KCK_Speed_CHIP(Robot_D[RobotID].CHP);
					full_charge=0;
				}
			}
		}
		else {
			KCK_Speed_CHIP(KCK_SPEED_OFF);
			tc_enable_cc_channels(&TCC0,TC_CCCEN);
			//LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
		kck_time_chip=0; flg_chip=0;}
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
	if ( free_wheel>100 || current_ov )
	{
		Robot_D[RobotID].M0a = 1;
		Robot_D[RobotID].M0b = 2;
		Robot_D[RobotID].M1a = 3;
		Robot_D[RobotID].M1b = 4;
	}
	switch (motor_num)
	{
		case 0 :
			MOTORNUM_PORT.OUTCLR= (MOTORNUM0_bm | MOTORNUM1_bm);
			if(((CLK_par_PORT.IN && CLK_par_bm)))
			{
			FPGA_DATA_PORT.OUT = Robot_D[RobotID].M0b;//low byte
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
			if(((CLK_par_PORT.IN && CLK_par_bm)))
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
			if(((CLK_par_PORT.IN && CLK_par_bm)))
			{
			FPGA_DATA_PORT.OUT = Robot_D[RobotID].M1b;
			rpm_ready[0].data_permit=1;
			rpm_ready[1].data_permit=0;
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
			if(((CLK_par_PORT.IN && CLK_par_bm)))
			{
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
			if(((CLK_par_PORT.IN && CLK_par_bm)))
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
			if(((CLK_par_PORT.IN && CLK_par_bm)))
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
			if(((CLK_par_PORT.IN && CLK_par_bm)))
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
			if(((CLK_par_PORT.IN && CLK_par_bm)))
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
	if (RobotID < 3)
	NRF24L01_L_Init_milad(_RX_MODE, _CH_1, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	else if(RobotID > 2 && RobotID < 6)
	NRF24L01_L_Init_milad(_RX_MODE, _CH_0, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	NRF24L01_L_WriteReg(W_REGISTER | DYNPD,0x01);
	NRF24L01_L_WriteReg(W_REGISTER | FEATURE,0x06);

	NRF24L01_L_CE_HIGH;
	_delay_us(130);
}
int driver_adc(int x)
{
	if (x==0)     return adc_get_unsigned_result(&ADCB,ADC_CH0);
	else if(x==1) return adc_get_unsigned_result(&ADCB,ADC_CH1);
	else if(x==2) return adc_get_unsigned_result(&ADCA,ADC_CH1);
}
void calc_offset(int x)
{
	Driver.offset[x] =0;
	for (int i=0;i<50;i++)
	{
		//Driver.adc[0] = adc_get_unsigned_result(&ADCA,ADC_CH0);
		Driver.adc[x]=(driver_adc(x));
		Driver.offset[x] = Driver.offset[x] + Driver.adc[x];
		
	}
	Driver.offset[x] = (Driver.offset[x])/50;
	Driver.offset[x] = ((float)Driver.offset[x]) * 0.86;
	
}

void data_transmission (void)
{
	//transmitting data to wireless board/////////////////////////////////////////////////
	 		//Test_Data[0] = adc/12;
	 		//Test_Data[1] = time_test;
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
	//Buf_Tx_L[14] = (Test_Data[7]>> 8) & 0xFF;	// unused
	//Buf_Tx_L[15] = Test_Data[7] & 0xFF;			// unused
	Buf_Tx_L[16] = adc/12;						//battery voltage
	

	//LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
	NRF24L01_L_Write_TX_Buf(Buf_Tx_L, _Buffer_Size);
	//NRF24L01_L_RF_TX();
}