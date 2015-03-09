/*
 * IncFile1.h
 *
 * Created: 1/6/2014 3:37:50 PM
 *  Author: KNToosi
 */ 


#include "stdio.h"
#include "math.h"
#include "MPU.c"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"

TWI_Master_t twiMaster;    /*!< TWI master module. */
TWI_Slave_t twiSlave;      /*!< TWI slave module. */


extern void Read_Sensor();
extern int  read_mpu(void);
extern void mpu6050_init();
extern void Matrix_update(void);
extern void Normalize(void);
extern void Drift_correction();
extern void Euler_angles();
extern void mpu_ofsetinit();
extern unsigned char i2c_readReg(char reg_address);
extern void i2c_writeReg(char reg_address, char data);
extern float AN[8];

#ifndef INCFILE1_H_
#define INCFILE1_H_





#endif /* INCFILE1_H_ */