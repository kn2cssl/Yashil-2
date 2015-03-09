
/*
 * CFile1.c
 *
 * Created: 1/6/2014 3:37:12 PM
 *  Author: KNToosi
 */ 


#include "stdio.h"
#include "string.h"
#include <math.h>
#include "MPU.c"
//#include "lcd.h"
#include <avr/eeprom.h>
#include "util/delay.h"
#include "initialize.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"

////////////////imu//////////////
#define GyroThreshold          10
#define PeriodTime             0.012f

//................................................DCM.............................................................



#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

// MPU6000 sensibility  (theorical 0.0152 => 1/65.6LSB/deg/s at 500deg/s) (theorical 0.0305 => 1/32.8LSB/deg/s at 1000deg/s) ( 0.0609 => 1/16.4LSB/deg/s at 2000deg/s)
#define Gyro_Gain_X 0.06098
#define Gyro_Gain_Y 0.06098
#define Gyro_Gain_Z 0.06098

#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second


/////////////////Filter////////////////
uint16_t EEMEM z_a_d,y_a_d,x_a_d,z_g_d,y_g_d,x_g_d;

uint16_t EEMEM GRAVITY; // This equivalent to 1G in the raw data coming from the accelerometer 
///////////////Sensor variables/////////////////

int gyroX=0,gyroY=0,gyroZ=0,accelX=0,accelY=0,accelZ=0,a=0;
int AN[8]; //array that store the 6 ADC filtered data
int  AN_OFFSET[8]; //Array that stores the Offset of the gyros

float G_Dt=PeriodTime;    // Integration time (DCM algorithm)
float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros rutn rate in a vector
int SENSOR_SIGN[] = {1,-1,-1,-1,1,1,1,1,1}; //int SENSOR_SIGN[] = {1,-1,-1,-1,1,1,-1,1,-1};
	
 TWI_Master_t twiMaster;    /*!< TWI master module. */
 TWI_Slave_t twiSlave;      /*!< TWI slave module. */

//*********************functions ************************
union   ch2int
{
	int real;
	char byte[2];
};

float read_raw_data(int select)
{
	if (SENSOR_SIGN[select]<0)
	{
		return (AN_OFFSET[select]-AN[select]);
	}
	else
	{
		return (AN[select]-AN_OFFSET[select]);
	}
}

//************************************   i2c_function for mpu   **************************************

void i2c_writeReg(char reg_address, char data)
{
	char tx_buf[2];
	tx_buf[0] = reg_address;
	tx_buf[1] = data;
	TWI_MasterWriteRead(&twiMaster,0b11010000>>1,&tx_buf[0],2,0);
	while (twiMaster.status != TWIM_STATUS_READY) {
		/* Wait until transaction is complete. */
	}
	
	
}

unsigned char i2c_readReg(char reg_address)
{
	char data;
	char tx_buf[2];
	tx_buf[0] = reg_address;
	//TWI_MasterWriteRead(&twiMaster,0b11010010>>1,&tx_buf[0],1,1);  /////D2
	
	TWI_MasterWriteRead(&twiMaster,0b11010000>>1,&tx_buf[0],1,1); ///// D0
	while (twiMaster.status != TWIM_STATUS_READY) {
		/* Wait until transaction is complete. */
		
	}
	//if(twiMaster.status = TWIM_STATUS_READY)
	//{
		data=twiMaster.readData[0];
		return data;
	//}
}

void mpu6050_init(void)
{
	//TWBR = ((16000000L / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
	i2c_writeReg(MPUREG_PWR_MGMT_1,0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
	_delay_ms(5);
	i2c_writeReg(MPUREG_SMPLRT_DIV,0x00);             //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	_delay_ms(5);
	i2c_writeReg(MPUREG_PWR_MGMT_1,0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
	_delay_ms(5);
	i2c_writeReg(MPUREG_GYRO_CONFIG,BITS_FS_2000DPS);  // Gyro scale 2000º/s
	_delay_ms(5);
	// enable I2C bypass for AUX I2C
	#if defined(MAG)
	i2c_writeReg(MPUREG_USER_CTRL,0x00);             //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=0 (I2C bypass mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
	_delay_ms(5);
	i2c_writeReg(MPUREG_INT_PIN_CFG,0x02);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
	#endif
	_delay_ms(5);
	i2c_writeReg(MPUREG_CONFIG, BITS_DLPF_CFG_20HZ);
	_delay_ms(5);
	i2c_writeReg(MPUREG_ACCEL_CONFIG,0x08);// Accel scale 4g (4096LSB/g)
	_delay_ms(5);
}








//************************************************* MPU **************************************************
//


int  read_mpu(void)
{
	union ch2int x_g,y_g,z_g,x_a,y_a,z_a;
	//int z_test;
	gyroX=0;
	gyroY=0;
	gyroZ=0;
	
	accelX=0;
	accelY=0;
	accelZ=0;
	//a=i2c_readReg(117);    /////when you read this register,it returns 104 if gyro pin is connected to micro and working properly  
	
	//x_a.byte[1]=i2c_readReg(59);
	//x_a.byte[0]=i2c_readReg(60);
	//
	//y_a.byte[1]=i2c_readReg(61);
	//y_a.byte[0]=i2c_readReg(62);
	//
	//z_a.byte[1]=i2c_readReg(63);
	//z_a.byte[0]=i2c_readReg(64);
	//
	//x_g.byte[1]=i2c_readReg(67);
	//x_g.byte[0]=i2c_readReg(68);
	//
	//y_g.byte[1]=i2c_readReg(69);
	//y_g.byte[0]=i2c_readReg(70);
	
	z_g.byte[1]=i2c_readReg(71);
	z_g.byte[0]=i2c_readReg(72);
	
	
	
	//gyroX=x_g.real;
	//gyroY=y_g.real;
	
	gyroZ=z_g.real;
	gyroZ+=38;
	if (abs(gyroZ)<50)
	{
		gyroZ=0;
	}
	gyroZ = gyroZ*0.06098;
	
	
	//accelX=x_a.real;
	//accelY=y_a.real;
	//accelZ=z_a.real;
	 
	//gyroX = (abs(gyroX)> GyroThreshold) ? gyroX  : 0;
	//gyroY = (abs(gyroY)> GyroThreshold) ? gyroY  : 0;
	//gyroZ = (abs(gyroZ)> GyroThreshold) ? gyroZ  : 0;
	
	
	//eeprom_write_word(&x_g_d, -60);
	//eeprom_write_word(&y_g_d, 16);
	//eeprom_write_word(&z_g_d,  2);
	//eeprom_write_word(&x_a_d, 743);
	//eeprom_write_word(&y_a_d, -350);
	//eeprom_write_word(&z_a_d, 0);
	//
	//AN_OFFSET[0]=eeprom_read_word(&x_g_d);
	//AN_OFFSET[1]=eeprom_read_word(&y_g_d);
	//AN_OFFSET[2]=eeprom_read_word(&z_g_d);
	//
	//AN_OFFSET[3]=eeprom_read_word(&x_a_d);
	//AN_OFFSET[4]=eeprom_read_word(&y_a_d);
	//AN_OFFSET[5]=eeprom_read_word(&z_a_d);
	//////////matrix update ////////////////////
	//Gyro_Vector[0]=Gyro_Scaled_X(read_raw_data(0)); //gyro x roll
	//Gyro_Vector[1]=Gyro_Scaled_Y(read_raw_data(1)); //gyro y pitch
	//Gyro_Vector[2]=Gyro_Scaled_Z(read_raw_data(2)); //gyro Z yaw
	
	//Accel_Vector[0]=read_raw_data(3);               // acc x
	//Accel_Vector[1]=read_raw_data(4);               // acc y
	//Accel_Vector[2]=read_raw_data(5);               // acc z
	
//	char buff32[32];
// 	 //sprintf(buff32,"%d,%d,%d,%d,%d,%d\r ",x_a.real,y_a.real,z_a.real,x_g.real,y_g.real,z_g.real);
// 	 sprintf(buff32,"%d,%d,%d,%d,%d,%d\r ",gyroX,gyroY,gyroZ,accelX,accelY,accelZ);
// 	// sprintf(buff32,"%d,%d,%d,%d,%d,%d\r ",Accel_Vector[0],Accel_Vector[1],Accel_Vector[2],Gyro_Vector[0],Gyro_Vector[1],Gyro_Vector[2]);
// 	 for(int i=0;i<strlen(buff32);i++)
// 	 {
// 		 usart_putchar(&USARTE0,buff32[i]);
// 		 
// 	 }
	//
	//LCDGotoXY(0,0);
	//LCDStringRam("offset");
	//LCDGotoXY(0,1);
	//sprintf(buff32,"%2d,%2d,%2d",gyroX,gyroY,gyroZ);
	//LCDStringRam(buff32);
	
	return gyroZ;
	
}



//...............sensor......................\\
  
 
  //void mpu_ofsetinit(int type)
//{
	//LED_Red_PORT.OUTTGL=LED_Red_PIN_bm;
	////first variable....
	//int i=0,j=0,k=0,x=0,y=0,z=0;
	//int count=0;
	//long int x_min=10000,x_max=-10000,y_max=-10000,y_min=10000,z_max=-10000,z_min=10000,x_off=0,y_off=0,z_off=0;
	//long int i_min=10000,i_max=-10000,j_max=-10000,j_min=10000,k_max=-10000,k_min=10000,i_off=0,j_off=0,k_off=0;
//
//
	//_delay_ms(50);
	//
	//if(type==1)
	//{
		//while(x_off==0 || y_off==0 || z_off==0)
		//{
			//x=0;
			//y=0;
			//z=0;
			//x=i2c_readReg(67);
			//x=(x<<8);
			//x|=i2c_readReg(68);
			//
			//y=i2c_readReg(69);
			//y=(y<<8);
			//y|=i2c_readReg(70);
			//
			//z=i2c_readReg(71);
			//z=(z<<8);
			//z|=i2c_readReg(72);
			//
			//if(x_off==0)
			//{
				//x_min=x;
				//x_max=x;
			//}
//
		//if(y_off==0)
		//{
			//y_min=y;
			//y_max=y;
		//}
//
		//if(z_off==0)
		//{
			//z_min=z;
			//z_max=z;
		//}
//
		//for(count=0;count<100;count++)
		//{
			//x=i2c_readReg(67);
			//x=(x<<8);
			//x|=i2c_readReg(68);
	//
			//y=i2c_readReg(69);
			//y=(y<<8);
			//y|=i2c_readReg(70);
	//
			//z=i2c_readReg(71);
			//z=(z<<8);
			//z|=i2c_readReg(72);
	//
			//x_min=(x_min>x)?(x):(x_min);
			//x_max=(x_max<x)?(x):(x_max);
	//
			//y_min=(y_min>y)?(y):(y_min);
			//y_max=(y_max<y)?(y):(y_max);
	//
			//z_min=(z_min>z)?(z):(z_min);
			//z_max=(z_max<z)?(z):(z_max);
//
		//}
//
//
		//if( (x_max-x_min)<8 && x_off==0 )
		//{
			//eeprom_write_word(&x_g_d,(x_max+x_min)/2);
			//_delay_ms(20);
			//x_off=1;
		//}
		////printf("%d,%d,%d\r",y_max-y_min,y_max-y_min,z_max-z_min);
		//if( (y_max-y_min)<8 && y_off==0 )
		//{
			//eeprom_write_word(&y_g_d,(y_max+y_min)/2);
			//_delay_ms(20);
			//y_off=1;
		//}
//
		//if( (z_max-z_min)<8 && z_off==0 )
		//{
			//eeprom_write_word(&z_g_d,(z_max+z_min)/2);
			//_delay_ms(20);
			//z_off=1;
		//}
//
		//}
			//}
	//
	//if(type==2)
	//{
		//while(i_off==0 || j_off==0 || k_off==0)
		//{
			//i=0;
			//j=0;
			//k=0;
			//i=i2c_readReg(59);
			//i=(i<<8);
			//i|=i2c_readReg(60);
			//
			//j=i2c_readReg(61);
			//j=(j<<8);
			//j|=i2c_readReg(62);
			//
			//k=i2c_readReg(63);
			//k=(k<<8);
			//k|=i2c_readReg(64);
			//
			//if(i_off==0)
			//{
				//i_min=i;
				//i_max=i;
			//}
			//
			//if(j_off==0)
			//{
				//j_min=j;
				//j_max=j;
			//}
			//
			//if(k_off==0)
			//{
				//k_min=k;
				//k_max=k;
			//}
			//
			//for(count=0;count<80;count++)
			//{
				////acce........
				//i=i2c_readReg(59);
				//i=(i<<8);
				//i|=i2c_readReg(60);
				//
				//j=i2c_readReg(61);
				//j=(j<<8);
				//j|=i2c_readReg(62);
				//
				//k=i2c_readReg(63);
				//k=(k<<8);
				//k|=i2c_readReg(64);
//
				//i_min=(i_min>i)?(i):(i_min);
				//i_max=(i_max<i)?(i):(i_max);
				//
				//j_min=(j_min>y)?(j):(j_min);
				//j_max=(j_max<y)?(j):(j_max);
				//
				//k_min=(k_min>k)?(k):(k_min);
				//k_max=(k_max<k)?(k):(k_max);
			//}
//
			////printf("%d,%d,%d\r",i_max-i_min,j_max-j_min,k_max-k_min);
//
			//if( (i_max-i_min)<40 && i_off==0 )
			//{
				//eeprom_write_word(&x_a_d,(i_max+i_min)/2);
				//_delay_ms(20);
				//i_off=1;
			//}
			//
			//if( (j_max-j_min)<40 && j_off==0 )
			//{
				//
				//eeprom_write_word(&y_a_d,(j_max+j_min)/2);
				//_delay_ms(20);
				//j_off=1;
			//}
			//
			//if( (k_max-k_min)<40 && k_off==0 )
			//{
			    //eeprom_write_word(&z_a_d,(k_max+k_min)/2);
				//_delay_ms(20);
				//k_off=1;
			//}
		//}
		//
		//
	//}
	////int a,b,c,d,e,f;
	////a=eeprom_read_word(&x_a_d);
	////b=eeprom_read_word(&y_a_d);
	////c=eeprom_read_word(&z_a_d);
	////d=eeprom_read_word(&x_g_d);
	////e=eeprom_read_word(&y_g_d);
	////f=eeprom_read_word(&z_g_d);
		////char buff32[32];
		////// sprintf(buff32,"%d,%d,%d\r ",x,y,z);
		////// sprintf(buff32,"%d,%d,%d,%d,%d,%d\r ",(i_max+i_min)/2,(j_max+j_min)/2,(k_max+k_min)/2,(x_max+x_min)/2,(y_max+y_min)/2,(z_max+z_min)/2);
		//////sprintf(buff32,"%d,%d,%d,%d,%d,%d\r ",gyroX,gyroY,gyroZ,accelX,accelY,accelZ);
		////sprintf(buff32,"%d,%d,%d,%d,%d,%d\r ",x_g_d,y_g_d,z_g_d,y_a_d,y_a_d,y_a_d);
		//////sprintf(buff32,"%d,%d,%d,%d,%d,%d\r ",a,b,c,d,e,f);
		////for(int i=0;i<strlen(buff32);i++)
		////{
			////usart_putchar(&USARTE0,buff32[i]);
			////
			////
		////}
		//
	//z_a_d=0;
//}
// Returns an analog value with the offset corrected (calibrated value)

void Read_Sensor(void)
{
	
	read_mpu();// Read MPU6050 sensor values
	
	AN[0] = gyroX;
	AN[1] = gyroY;
	AN[2] = gyroZ;
	AN[3] = accelX;
	AN[4] = accelY;
	AN[5] = accelZ;
	//char buff32[32];
	////sprintf(buff32,"%d,%d,%d,%d,%d,%d\r ",x_a.real,y_a.real,z_a.real,x_g.real,y_g.real,z_g.real);
	 //sprintf(buff32,"%d,%d,%d,%d,%d,%d\r ",gyroX,gyroY,gyroZ,accelX,accelY,accelZ);
	////sprintf(buff32,"%d,%d,%d,%d,%d,%d\r ",AN[0],AN[1],AN[2],AN[3],AN[4],AN[5]);
	//for(int i=0;i<strlen(buff32);i++)
	//{
		//usart_putchar(&USARTE0,buff32[i]);
		//
	//}
	//
	//LCDGotoXY(0,0);
	//sprintf(buff2,"%2x ",AN[5]);
	//LCDStringRam(buff2);
	//LCDGotoXY(0,1);
	//LCDStringRam("accelZ");
	
}
//

//int gravity_set(void)
//{
	//long int x=0,y=0,z=0,i=0;
	//for(i=0;i<10;i++)
	//{
		//read_mpu();
		//x+=accelX-x_a_d;
		//y+=accelY-y_a_d;
		//z+=accelZ-z_a_d;
	//}
	//x/=10;
	//y/=10;
	//z/=10;
	//return (int)sqrt((float)(x)*(float)(x)+(float)(y)*(float)(y)+(float)(z)*(float)(z));
	//
	//
//}
//
//void Normalize(void)
//{
	//float error=0;
	//float temporary[3][3];
	//float renorm=0;
	//
	//error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19
//
	//Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
	//Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
	//
	//Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
	//Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
	//
	//Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
	//
	//renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
	//Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
	//
	//renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
	//Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
	//
	//renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
	//Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
	//
//}
//
//void Matrix_update(void)
//{
	//int x=0,y=0;
	//Gyro_Vector[0]=Gyro_Scaled_X(read_raw_data(0)); //gyro x roll
	//Gyro_Vector[1]=Gyro_Scaled_Y(read_raw_data(1)); //gyro y pitch
	//Gyro_Vector[2]=Gyro_Scaled_Z(read_raw_data(2)); //gyro Z yaw
	//
	//Accel_Vector[0]=read_raw_data(3);               // acc x
	//Accel_Vector[1]=read_raw_data(4);               // acc y
	//Accel_Vector[2]=read_raw_data(5);               // acc z
	//
	//Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding Integrator term
	//Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding proportional term
//
	////Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement
	//
	//Update_Matrix[0][0]=0;
	//Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
	//Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
	//Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
	//Update_Matrix[1][1]=0;
	//Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
	//Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
	//Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
	//Update_Matrix[2][2]=0;
	//
	//Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c
//
	//for(x=0; x<3; x++) //Matrix Addition (update)
	//{
		//for(y=0; y<3; y++)
		//{
			//DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
		//}
	//}
      //
//}
//
////void calibrate(void)
////{
	////
////
	////mpu_ofsetinit('g');
	////_delay_ms(1000);
////
	//////printf("Gyro Offset");
////
////
	////mpu_ofsetinit('a');
	////_delay_ms(1000);
////
	//////printf("Gyro Offset");
////
////}
//
			//
		//
//void Drift_correction(void)
//{
  //float mag_heading_x;
  //float mag_heading_y;
  //float errorCourse;
  ////Compensation the Roll, Pitch and Yaw drift. 
  //static float Scaled_Omega_P[3];
  //static float Scaled_Omega_I[3];
  //float Accel_magnitude;
  //float Accel_weight;
  //
  //
  ////*****Roll and Pitch***************
//
  //// Calculate the magnitude of the accelerometer vector
  //Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  //
  //Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  ////g_print=Accel_magnitude;
  //// Dynamic weighting of accelerometer info (reliability filter)
  //// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  //Accel_weight = constrain(fabs(1-Accel_magnitude),.015,.04,.002);  //  
  ////g_print=Accel_weight;
 //// g_print=fabs(1-Accel_magnitude);
  ////g2_print=1-1.1*fabs(1 - Accel_magnitude);
  //Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  //Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  //
  //Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  //Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  //
  ////*****YAW***************
  //// We make the gyro YAW drift correction based on compass magnetic heading
 //
  //mag_heading_x = Heading_X;
  //mag_heading_y = Heading_Y;
  //errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  //Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  //Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  //Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  //
  //Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  //Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
   //LED_Green_PORT.OUTTGL=LED_Green_PIN_bm;
//
//}
//
//
//void Euler_angles(void)
//{
	////LED_Red_PORT.OUTTGL=LED_Red_PIN_bm;
	//pitch = -asinf(DCM_Matrix[2][0]);
	////roll = atan2f(DCM_Matrix[2][1],DCM_Matrix[2][2]);
	//yaw = atan2f(DCM_Matrix[1][0],DCM_Matrix[0][0]);
		////
		////char buff32[32];
		////sprintf(buff32,"%d,%d,%d\r ",pitch,roll,yaw);
		////
		////for(int i=0;i<strlen(buff32);i++)
		////{
			////usart_putchar(&USARTE0,buff32[i]);
			////
		////}
//}