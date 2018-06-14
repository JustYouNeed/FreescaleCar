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


# define SENSOR_FIFO_SIZE		4

/* ������IDö�ٱ���   */
typedef enum
{
	S_F_H_L = 0x0,
	S_F_H_R,
	S_B_H_L,
	S_B_H_R,
	S_V_L,
	S_V_R,
	S_M,
	SENSOR_COUNT,
}SENSOR_ID_EnumType;

/*  ���������ݽṹ��  */
typedef struct 
{
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

