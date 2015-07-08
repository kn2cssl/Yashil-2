/**
 * \file
 *
 * \brief Empty user application template
 *
 */

#include <asf.h>
#define F_CPU 32000000UL
#include <util/delay.h>
#include "initialize.h"
#include "nrf24l01_L.h"
#include <stdlib.h>


void NRF_init (void) ;
void data_transmission (void);
void every1ms(void);
void fpga_connection ( void ) ;

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
int flg_dir=0, flg_chip=0, charge_flg=0,flg_sw=0,SW_TEST_flg=0;
bool reset_setpoint_flag = true;

//================================
bool new_wireless_data = false ;
bool connection_permission = false ;
bool packing_pemission = false ;

struct Robot_Data
{
	//wireless data
	uint8_t RID;
	uint8_t Vxh;
	uint8_t Vxl;
	uint8_t Vyh;
	uint8_t Vyl;
	uint8_t Wh;
	uint8_t Wl;
	uint8_t alphah;
	uint8_t alphal;
	uint8_t KCK;
	uint8_t CHP;
	uint8_t ASK;
	
	//jyro data
	
	uint8_t JVxh;
	uint8_t JVxl;
	uint8_t JVyh;
	uint8_t JVyl;
	uint8_t JWh;
	uint8_t JWl;
	
}Robot;

uint8_t temp_data[20];
uint8_t received_data[20];
uint64_t send_packet0;
uint64_t send_packet1;
uint64_t receive_packet0;
uint64_t receive_packet1;
//================================

struct wireless_DATA
{
	uint8_t RID;
	signed int M0a;
	signed int M0b;
	signed int M1a;
	signed int M1b;
	signed int M2a;
	signed int M2b;
	signed int M3a;
	signed int M3b;
	uint8_t KCK;
	uint8_t CHP;
	uint8_t ASK;	
}Robot_D;



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
int current_ov=0,current_alarm;
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
	
	Robot_D.RID=0;
	Robot_D.M0a=1;
	Robot_D.M0b=2;
	Robot_D.M1a=3;
	Robot_D.M1b=4;
	Robot_D.M2a=0;
	Robot_D.M2b=0;
	Robot_D.M3a=0;
	Robot_D.M3b=0;
	Robot_D.KCK=0;
	Robot_D.CHP=0;
	Robot_D.ASK=0;
	
	


	En_RC32M();

	//Enable LowLevel & HighLevel Interrupts
	PMIC_CTRL |= PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm |PMIC_MEDLVLEN_bm;

	change_ADC=6;
	PORT_init();
	//TimerD0_init();
	//TimerC0_init();
	//TimerC1_init();
	//TimerE0_init();
	TimerE1_init();
	//USARTE0_init();
	//ADCA_init();
	//ADCB_init();
	
	// Globally enable interrupts


	//uint8_t temp;
	//temp = WDT_PER_2KCLK_gc | (1 << WDT_ENABLE_bp) | (1 << WDT_CEN_bp);
	//ccp_write_io((void *)&WDT.CTRL, temp);
	//wdt_wait_while_busy();
	
	sei();




	
	while(1)
	  {  

		if (connection_permission)
		{
			
			if (packing_pemission)
			{
				uint8_t check_sum0 = Robot.alphal + Robot.Wh + Robot.Wl + Robot.Vyh + Robot.Vyl + Robot.Vxh + Robot.Vxl ;
				uint8_t check_sum1 = Robot.alphah + Robot.JWh + Robot.JWl + Robot.JVyh + Robot.JVyl + Robot.JVxh + Robot.JVxl ;
				
				send_packet0 = check_sum0<<56 | Robot.alphal<<48 |Robot.Wh<<40 | Robot.Wl<<32 | Robot.Vyh<<24 | Robot.Vyl<<16 | Robot.Vxh<<8 | Robot.Vxl ; //MSB of check_sum won't be sent
				send_packet1 = check_sum1<<56 | Robot.alphah<<48 |Robot.JWh<<40 | Robot.JWl<<32 | Robot.JVyh<<24 | Robot.JVyl<<16 | Robot.JVxh<<8 | Robot.JVxl ; //MSB of check_sum won't be sent
				
				packing_pemission = false ;
			}
			

			fpga_connection();				

			
		}
		else
		{
			
		}
		
		


	  }
	   
}

int counter_10us=0,counter_1ms=0,counter3=0;

 ISR(TCE1_OVF_vect)//10us
 {
	 if (new_wireless_data==true)
	 {
		 counter_10us = 0;
		 counter_1ms = 0;
		 new_wireless_data = false;
		 connection_permission = true;
	 }

	counter_10us++;
	CLK_PORT.OUTTGL = CLK_PIN;
	
	if (counter_10us==100)//1ms
	{
		counter_1ms++;
		counter_10us=0;
	}

	if (counter_1ms==1000)//1s
	{
		counter3+=1;
		counter_1ms=0;
		LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;				
	}
 }


void fpga_connection ( void )
{
	switch (counter_10us)
	{
		case 0: //start bits1
		PORTF.OUT = ( PORTF.IN & 0x80 ) |  0b1010101 ;
		break ;
		
		case 1:
		//nothing
		break ;
		
		case 2: //start bits2
		PORTF.OUT = ( PORTF.IN & 0x80 ) |  0b1010101 ;
		break ;
		
		case 3:
		//nothing
		break ;
	}
		
	if (counter_10us < 22 && counter_10us > 3)
	{
		if (counter_10us % 2 == 0 )
		{
			PORTF.OUT = ( PORTF.IN & 0x80 ) | ( send_packet0 & 0x0000007F ) ;
			send_packet0 = send_packet0 >> 7 ;
		}
		else
		{
			receive_packet0 = receive_packet0 << 7 ;
			receive_packet0 = PORTX_IN | receive_packet0 ;
		}
	}
		
		
	if (counter_10us > 21)
	{
		if (counter_10us % 2 == 0 )
		{
			PORTF.OUT = ( PORTF.IN & 0x80 ) | ( send_packet1 & 0x0000007F ) ;
			send_packet1 = send_packet1 >> 7 ;
		}
		else
		{
			receive_packet1 = receive_packet1 << 7 ;
			receive_packet1 = PORTX_IN | receive_packet1 ;
		}
	}
	
	if (counter_10us == 39)
	{
		//unpacking data from FPGA
		for (int i =0 ; i ; i < 18 )
		{
			if(i<9)
			{
				temp_data[i] = receive_packet0 & 0x000000FF ;
				receive_packet0 = receive_packet0 >> 8 ;
			}
			else
			{
				temp_data[i] = receive_packet1 & 0x000000FF ;
				receive_packet1 = receive_packet1 >> 8 ;
			}
					 
			
		}
		
		//generating check_sum
		uint8_t check_sum_test0 = 0 ;
		uint8_t check_sum_test1 = 0 ;
		for (int i = 0 ; i++ ; i < 9)
		{
			check_sum_test0 +=  temp_data[i] ;
		}
		
		for (int i = 9 ; i++ ; i < 17)
		{
			check_sum_test1 +=  temp_data[i] ;
		}
		
		//saving checked data
		if( ( (check_sum_test0 & 0x7F) == temp_data[8] ) && ( (check_sum_test1 & 0x7F) == temp_data[17] ))
		{
			for (int i = 9 ; i++ ; i < 17)
			{
				received_data[i] = temp_data[i] ;
			}
		}


	}


}