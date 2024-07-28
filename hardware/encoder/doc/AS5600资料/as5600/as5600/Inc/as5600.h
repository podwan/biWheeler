/*
   本代码由淘宝店智御电子提供
	 AS5600硬件购买链接：https://item.taobao.com/item.htm?spm=a1z38n.10677092.0.0.2c351debwMQV2w&id=584601996739
*/
/*******************************************************************************
* 文件名称：as5600.h
*
* 摘    要：1.采用软件模拟I2C通信协议
*           2.初始化as5600的相关设置,
*
* 当前版本：
* 作    者：智御电子	
* 日    期：2018/12/31
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#ifndef __MD_AS5600_H
#define __MD_AS5600_H

#include "stm32f0xx_hal.h"  //这个需要根据不同的芯片类型进行变换
#include "main.h"

#define SDA1_IN()  IIC_SDA_GPIO_Port->MODER &= ~(0x3<<(10*2))
#define SDA1_OUT() IIC_SDA_GPIO_Port->MODER &= ~(0x3<<(10*2));IIC_SDA_GPIO_Port->MODER |= (0x1<<(10*2))

#define Sim_I2C1_SCL_LOW          (IIC_SCL_GPIO_Port->ODR &= ~(IIC_SCL_Pin))
#define Sim_I2C1_SCL_HIG          (IIC_SCL_GPIO_Port->ODR |=  (IIC_SCL_Pin))
#define Sim_I2C1_SDA_LOW          (IIC_SDA_GPIO_Port->ODR &= ~(IIC_SDA_Pin))
#define Sim_I2C1_SDA_HIG          (IIC_SDA_GPIO_Port->ODR |=  (IIC_SDA_Pin))

#define Sim_I2C1_SDA_STATE        (IIC_SDA_GPIO_Port->IDR &= (IIC_SDA_Pin))

#define Sim_I2C1_DELAY 		Sim_I2C1_Delay(100000)
#define Sim_I2C1_NOP		  Sim_I2C1_Delay(400)  //25 

#define Sim_I2C1_READY			0x00
#define Sim_I2C1_BUS_BUSY		0x01
#define Sim_I2C1_BUS_ERROR	0x02

#define Sim_I2C1_NACK	      0x00
#define Sim_I2C1_ACK				0x01

#define I2C1_Direction_Transmitter 		0x00
#define I2C1_Direction_Receiver   	  0x01

#define AS5600_ADDRESS_MAG           0x36

typedef enum
{
/* set i2c address */ 
  _ams5600_Address = 0x36,
  _zmco = 0x00,
  _zpos_hi = 0x01,
  _zpos_lo = 0x02,
  _mpos_hi = 0x03,
  _mpos_lo = 0x04,
  _mang_hi = 0x05,
  _mang_lo = 0x06,
  _conf_hi = 0x07,    
  _conf_lo = 0x08,
  _raw_ang_hi = 0x0c,
  _raw_ang_lo = 0x0d,
  _ang_hi = 0x0e,
  _ang_lo = 0x0f,
  _stat = 0x0b,
  _agc = 0x1a,
  _mag_hi = 0x1b,
  _mag_lo = 0x1c,
  _burn = 0xff
}AMS5600Registers_t;


void PrintMenu(void);
void Programe_Run(void);
#endif 
		
