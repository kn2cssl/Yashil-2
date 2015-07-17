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


void NRF_init ( void ) ;
void data_transmission ( void ) ;
void every1ms ( void ) ;
void fpga_connection ( void ) ;
void wireless_connection ( void ) ;
void data_packing ( void ) ;
void data_unpacking ( void ) ;


//================================
//bool new_wireless_data = false ;
//bool connection_permission = false ;
//bool packing_pemission = false ;

enum Data_Flow {new_wireless_data , new_jyro_data , packing_data , communication , unpacking_data , other_programs };
enum Data_Flow data;

enum data_counter {even=0 , odd=1};
enum data_counter d_counter;

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
	
	//gyro data
	
	uint8_t GVxh;
	uint8_t GVxl;
	uint8_t GVyh;
	uint8_t GVyl;
	uint8_t GWh;
	uint8_t GWl;
	
}Robot;

uint64_t send_packet[40];
uint64_t receive_packet [40];

uint8_t temp_data[20];
uint8_t received_data[20];

int counter_100ms=0,packet_counter=0,wl_counter=0,summer=0;
uint32_t cco=0;
int rate=0;
int rate_counter=0;
bool wireless_ok=false;
//================================

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
	
	//================================================================= Globally enable interrupts
	//uint8_t temp;
	//temp = WDT_PER_2KCLK_gc | (1 << WDT_ENABLE_bp) | (1 << WDT_CEN_bp);
	//ccp_write_io((void *)&WDT.CTRL, temp);
	//wdt_wait_while_busy();
	
	//// NRF Initialize
	NRF_init () ;
	
	
	sei();
	
	while(1)
	{
		
		//================================================WIRELESS DATA
		if (data == new_wireless_data)
		{
			wireless_connection();
			data = packing_data ;
			wl_counter ++;
		}
		_delay_us(1);
		//-------------------------------------------------------------

		//================================================GYRO DATA
		//if (data == new_jyro_data)
		//{
		//data = packing_data ;
		//}
		//_delay_us(1);
		//-------------------------------------------------------------

		if (data == packing_data)
		{
			data_packing () ;
			packet_counter = 0 ;
			summer=0;
			d_counter = even ;
			data = communication ;
		}
		
		if (data == communication)
		{
			fpga_connection () ;
			packet_counter++;
			summer += packet_counter;
		}
		
		if (data == unpacking_data)
		{
			data_unpacking () ;
			data = other_programs ;
		}
		
		if (data == other_programs)
		{
			Robot.Vxh = 0x0d;//3527
			Robot.Vxl = 0xc7;
			Robot.Vyh = 0x0b;//2891
			Robot.Vyl = 0x4b;
			Robot.Wh = 0x75;//30039
			Robot.Wl = 0x57;
			//other_programs
			//if (!wireless_ok)
			//{
				//data = packing_data ;//test
			//}
			
		}
		
	}
	
}


ISR(PORTD_INT0_vect)////////////////////////////////////////PTX   IRQ Interrupt Pin
{
	//if (wireless_ok)
	//{
		//data = new_wireless_data ;
	//}
	
	data = new_wireless_data ;
	
}


ISR(TCE1_OVF_vect)//0.1s
{
	//counter_100ms++;
	//if (counter_100ms==100)
	//{
		//tc_disable(&TCE1);
		//rate = rate_counter;
		//wireless_ok=true;
		//NRF_init () ;
	//}
}


void wireless_connection ( void )
{
	uint8_t status_L = NRF24L01_L_WriteReg(W_REGISTER | STATUSe, _TX_DS|_MAX_RT|_RX_DR);
	if((status_L & _RX_DR) == _RX_DR)
	{
		LED_White_PORT.OUTSET = LED_White_PIN_bm;
		wireless_reset=0;
		wdt_reset();
		//1) read payload through SPI,
		NRF24L01_L_Read_RX_Buf(Buf_Rx_L, _Buffer_Size);
		free_wheel=0 ;
		if((Buf_Rx_L[0] == 0x0A && (RobotID < 3 || (RobotID<9 && RobotID>5)))|| (Buf_Rx_L[0] == 0xA0 && (RobotID > 8 || (RobotID<6 &&      RobotID>2))))
		{
			LED_Red_PORT.OUTSET = LED_Red_PIN_bm;
			Robot_D.RID  = Buf_Rx_L[0];
			Robot_D.M0a  = Buf_Rx_L[1+ RobotID%3 * 10];
			Robot_D.M0b  = Buf_Rx_L[2+ RobotID%3 * 10];
			Robot_D.M1a  = Buf_Rx_L[3+ RobotID%3 * 10];
			Robot_D.M1b  = Buf_Rx_L[4+ RobotID%3 * 10];
			Robot_D.M2a  = Buf_Rx_L[5+ RobotID%3 * 10];
			Robot_D.M2b  = Buf_Rx_L[6+ RobotID%3 * 10];
			Robot_D.M3a  = Buf_Rx_L[7+ RobotID%3 * 10];
			Robot_D.M3b  = Buf_Rx_L[8+ RobotID%3 * 10];
			Robot_D.KCK  = Buf_Rx_L[9+ RobotID%3 * 10];
			Robot_D.CHP  = Buf_Rx_L[10+RobotID%3 * 10];
			Robot_D.ASK  = Buf_Rx_L[31];//0b00000000
			
			if (Robot_D.ASK != Robot_Select)
			{
				Robot_Select = Robot_D.ASK;
				if (Robot_Select == RobotID)
				{
					NRF24L01_L_WriteReg(W_REGISTER | EN_AA, 0x01);
				}
				else
				{
					NRF24L01_L_WriteReg(W_REGISTER | EN_AA, 0x00);
				}
			}
			
			if (Robot_D.ASK == RobotID)
			{
				data_transmission();
			}

		}
	}
	
	if ((status_L&_MAX_RT) == _MAX_RT)
	{
		NRF24L01_L_Flush_TX();
	}
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
	
	
	
	if (RobotID < 6)
	NRF24L01_L_Init_milad(_RX_MODE, _CH_1, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	else if(RobotID > 5)
	NRF24L01_L_Init_milad(_RX_MODE, _CH_0, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	NRF24L01_L_WriteReg(W_REGISTER | DYNPD,0x01);
	NRF24L01_L_WriteReg(W_REGISTER | FEATURE,0x06);

	NRF24L01_L_CE_HIGH;
	_delay_us(130);
}


void data_transmission (void)
{
	//transmitting data to wireless board/////////////////////////////////////////////////

	//Test_Data[4] = adc*0.4761;//battery voltage
	Test_Data[0] = (received_data[0] & 0x00ff) | ((received_data[1] & 0x00ff ) <<8);
	Test_Data[1] = rate;
	Test_Data[4] = summer;
	
	
	Buf_Tx_L[0]  = received_data[0];//(Test_Data[0]>> 8) & 0xFF;//Robot_D.M0a;//	//drive test data
	Buf_Tx_L[1]  = received_data[8];//Test_Data[0] & 0xFF;//Robot_D.M0b;//			//drive test data
	Buf_Tx_L[2]  = received_data[1];//(Test_Data[1]>> 8) & 0xFF;//Robot_D.M1a;//	//drive test data
	Buf_Tx_L[3]  = received_data[9];//Test_Data[1] & 0xFF;	//Robot_D.M1b;//		//drive test data
	Buf_Tx_L[4]  = received_data[2];//(Test_Data[2]>> 8) & 0xFF;//Robot_D.M2a;//	//drive test data
	Buf_Tx_L[5]  = received_data[10];//Test_Data[2] & 0xFF;//Robot_D.M2b;//			//drive test data
	Buf_Tx_L[6]  = received_data[36];//(Test_Data[3]>> 8) & 0xFF;//Robot_D.M3a;//	//drive test data
	Buf_Tx_L[7]  = received_data[38];//Test_Data[3] & 0xFF;//Robot_D.M3b;//			//drive test data
	Buf_Tx_L[8]  = (Test_Data[4]>> 8) & 0xFF;	// unused
	Buf_Tx_L[9]  = Test_Data[4] & 0xFF;			// unused
	Buf_Tx_L[10] = (Test_Data[5]>> 8) & 0xFF;// unused
	Buf_Tx_L[11] = Test_Data[5] & 0xFF;			// unused
	Buf_Tx_L[12] = (Test_Data[6]>> 8) & 0xFF;	// unused
	Buf_Tx_L[13] = Test_Data[6] & 0xFF;			// unused
	Buf_Tx_L[14] = (Test_Data[7]>> 8) & 0xFF;	// battery voltage
	Buf_Tx_L[15] = Test_Data[7] & 0xFF;			// battery voltage
	Buf_Tx_L[16] = adc/12;						//battery adc
// 	Buf_Tx_L[17] = 
// 	Buf_Tx_L[18] = 
// 	Buf_Tx_L[19] = 
// 	Buf_Tx_L[20] =
// 	Buf_Tx_L[21] =
// 	Buf_Tx_L[22] =
// 	Buf_Tx_L[23] =
// 	Buf_Tx_L[24] =
// 	Buf_Tx_L[25] =
// 	Buf_Tx_L[26] =
// 	Buf_Tx_L[27] =
// 	Buf_Tx_L[28] =
// 	Buf_Tx_L[29] =
// 	Buf_Tx_L[30] =
// 	Buf_Tx_L[31] =
	//LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
	NRF24L01_L_Write_TX_Buf(Buf_Tx_L, _Buffer_Size);
	//NRF24L01_L_RF_TX();
}


void data_packing ( void )
{
	uint16_t check_sum0 = Robot.alphah + Robot.GWh + Robot.GVyh + Robot.GVxh + Robot.Wh + Robot.Vyh + Robot.Vxh ;
	uint16_t check_sum1 = Robot.alphal + Robot.GWl + Robot.GVyl + Robot.GVxl + Robot.Wl + Robot.Vyl + Robot.Vxl ;
	
	//in even cases micro puts data on F0 to F6 and clear data_clk pin (F7) to 0 ,so micro puts '0'+'data' on port F
	//so there is no need for "CLK_PORT.OUTCLR = CLK_PIN ;"
	send_packet[0]  = 0b01010101 ;	//first start sign
	send_packet[2]  = 0b01010101 ;	//second start sign
	send_packet[4]  = Robot.Vxh    & 0b01111111 ;
	send_packet[6]  = Robot.Vyh    & 0b01111111 ;
	send_packet[8]  = Robot.Wh     & 0b01111111 ;
	send_packet[10] = Robot.GVxh   & 0b01111111 ;
	send_packet[12] = Robot.GVyh   & 0b01111111 ;
	send_packet[14] = Robot.GWh    & 0b01111111 ;
	send_packet[16] = Robot.alphah & 0b01111111 ;
	send_packet[18] = ( ((Robot.Vxh       & 0x80) >> 7) |
					    ((Robot.Vyh       & 0x80) >> 6) | 
						((Robot.Wh        & 0x80) >> 5) | 
						((Robot.GVxh      & 0x80) >> 4) | 
						((Robot.GVyh      & 0x80) >> 3) | 
						((Robot.GWh       & 0x80) >> 2) | 
						((Robot.alphah    & 0x80) >> 1) ) & 0b01111111;
	send_packet[20] = check_sum0 & 0b01111111 ;
	
	send_packet[22] = Robot.Vxl    & 0b01111111 ;
	send_packet[24] = Robot.Vyl    & 0b01111111 ;
	send_packet[26] = Robot.Wl     & 0b01111111 ;
	send_packet[28] = Robot.GVxl   & 0b01111111 ;
	send_packet[30] = Robot.GVyl   & 0b01111111 ;
	send_packet[32] = Robot.GWl    & 0b01111111 ;
	send_packet[34] = Robot.alphal & 0b01111111 ;
	send_packet[36] = ( ((Robot.Vxl       & 0x80) >> 7) |
				        ((Robot.Vyl       & 0x80) >> 6) |
				        ((Robot.Wl        & 0x80) >> 5) |
				        ((Robot.GVxl      & 0x80) >> 4) |
				        ((Robot.GVyl      & 0x80) >> 3) |
				        ((Robot.GWl       & 0x80) >> 2) |
				        ((Robot.alphal    & 0x80) >> 1) ) & 0b01111111;
	send_packet[38] = check_sum1 & 0b01111111 ;

}


void fpga_connection ( void )
{
	if (packet_counter % 2 == 0)//sending
	{
		PORTF_OUT = send_packet[packet_counter] ;
	} 
	else                       //receiving 
	{
		CLK_PORT.OUTSET = CLK_PIN ;
		receive_packet[packet_counter] = PORTX_IN ;
	}

	
	if (packet_counter == 39)
	{
				cco ++ ;
				if (cco == 1000)
				{
					LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;//test
					cco=0;
					rate_counter++;
				}
		data = unpacking_data ;
	}


}


void data_unpacking (void)
{
	//unpacking data from FPGA
	//High bytes
	temp_data[0]  = ( receive_packet[5]  & 0b01111111 ) | ( ( receive_packet[19] & 0b00000001 ) << 7 ) ;
	temp_data[1]  = ( receive_packet[7]  & 0b01111111 ) | ( ( receive_packet[19] & 0b00000010 ) << 6 ) ;
	temp_data[2]  = ( receive_packet[9]  & 0b01111111 ) | ( ( receive_packet[19] & 0b00000100 ) << 5 ) ;
	temp_data[3]  = ( receive_packet[11] & 0b01111111 ) | ( ( receive_packet[19] & 0b00001000 ) << 4 ) ;
	temp_data[4]  = ( receive_packet[13] & 0b01111111 ) | ( ( receive_packet[19] & 0b00010000 ) << 3 ) ;
	temp_data[5]  = ( receive_packet[15] & 0b01111111 ) | ( ( receive_packet[19] & 0b00100000 ) << 2 ) ;
	temp_data[6]  = ( receive_packet[17] & 0b01111111 ) | ( ( receive_packet[19] & 0b01000000 ) << 1 ) ;
	temp_data[7]  = ( receive_packet[21] & 0b01111111 ) ;
	//Low bytes
	temp_data[8]  = ( receive_packet[23] & 0b01111111 ) | ( ( receive_packet[37] & 0b00000001 ) << 7 ) ;
	temp_data[9]  = ( receive_packet[25] & 0b01111111 ) | ( ( receive_packet[37] & 0b00000010 ) << 6 ) ;
	temp_data[10] = ( receive_packet[27] & 0b01111111 ) | ( ( receive_packet[37] & 0b00000100 ) << 5 ) ;
	temp_data[11] = ( receive_packet[29] & 0b01111111 ) | ( ( receive_packet[37] & 0b00001000 ) << 4 ) ;
	temp_data[12] = ( receive_packet[31] & 0b01111111 ) | ( ( receive_packet[37] & 0b00010000 ) << 3 ) ;
	temp_data[13] = ( receive_packet[33] & 0b01111111 ) | ( ( receive_packet[37] & 0b00100000 ) << 2 ) ;
	temp_data[14] = ( receive_packet[35] & 0b01111111 ) | ( ( receive_packet[37] & 0b01000000 ) << 1 ) ;
	temp_data[15] = ( receive_packet[39] & 0b01111111 ) ;
	
	//generating check_sum
	uint16_t check_sum_testH = 0 ;
	uint16_t check_sum_testL = 0;
	for (int i = 0 ; i < 7 ; i++)
	{
		check_sum_testH +=  temp_data[i] ;
	}
	
	for (int i = 8 ; i < 15 ; i++ )
	{
		check_sum_testL +=  temp_data[i] ;
	}
	
	//saving checked data
	if( ( (check_sum_testH & 0x7F) == temp_data[7] ) && ( (check_sum_testL & 0x7F) == temp_data[15] ))
	{
		for (int i = 0 ; i < 16 ; i++)
		{
			received_data[i] = temp_data[i] ;
		}
	}
}