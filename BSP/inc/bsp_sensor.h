/**
  *******************************************************************************************************
  * File Name: bsp_sensor.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-1
  * Brief: ���ļ�Ϊ���ش��������ݲɼ��ṩ����,����оƬΪKEA128-LFQP80
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-1
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	
	
# ifndef __BSP_SENSOR_H
# define __BSP_SENSOR_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "derivative.h"


# define SENSOR_1				ADC_Channel_C0
# define SENSOR_2				ADC_Channel_C1
# define SENSOR_3				ADC_Channel_C2
# define SENSOR_4				ADC_Channel_C3
# define SENSOR_5				ADC_Channel_F6
# define SENSOR_6				ADC_Channel_F7

# define SENSOR_FIFO_SIZE		10

/* ������IDö�ٱ���   */
typedef enum
{
	SENSOR_H_L = 0x0,
	SENSOR_V_L,
	SENSOR_H_R,
	SENSOR_V_R,
	SENSOR_COUNT,
}SENSOR_ID_EnumType;

/*  ���������ݽṹ��  */
typedef struct 
{
	uint8_t Read;
	uint8_t Write;
	uint16_t Average;
	uint16_t CalibrationMax;
	uint16_t CalibrationMin;
	
	float NormalizedValue;
	uint16_t FIFO[SENSOR_FIFO_SIZE];
}Sensor_TypeDef;


void bsp_sensor_Config(void);
void bsp_sensor_DataProcess(void);
void bsp_sensor_Calibration(void);


# endif

/********************************************  END OF FILE  *******************************************/

