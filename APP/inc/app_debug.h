/**
  *******************************************************************************************************
  * File Name: app_debug.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-16
  * Brief: 本文件声明了有关使用串口上位机调用下位机参数的函数、变量、宏定义
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-2-16
	*     Mod: 建立文件
  *
  *******************************************************************************************************
  */	
	# ifndef __APP_DEBUG_H
	# define __APP_DEBUG_H
/*
	*******************************************************************************************************
	*                              INCLUDE FILES
	*******************************************************************************************************
*/
# include "bsp.h"

//PID参数在上传及接收时使用的变换因子
# define ANO_PID_TRAN_FAC_P 100
# define ANO_PID_TRAN_FAC_I 100
# define ANO_PID_TRAN_FAC_D	10


//默认PID参数，使用在线调参时可以保证程序初始化时有PID参数
# define DEFAULT_PIT_KP	0
# define DEFAULT_PIT_KI	0
# define DEFAULT_PIT_KD	0

# define DEFAULT_ROL_KP 28
# define DEFAULT_ROL_KI 34
# define DEFAULT_ROL_KD 45

# define DEFAULT_YAW_KP	0
# define DEFAULT_YAW_KI	0
# define DEFAULT_YAW_KD	0


# define app_debug_Handler	UART0_IRQHandler


# define ANO_DATA_PRECESS_ON	1   //	选择是否自动处理数据

																										//              1111 1111 1111 1111
# define BYTE1(num)	(uint8_t)(num & 0XFF)           //取低8位                      1111
# define BYTE2(num) (uint8_t)((num >> 8) & 0XFF)    //高8位                   1111
# define BYTE3(num)	(uint8_t)((num >> 16) & 0XFF)		//高16位             1111
# define BYTE4(num)	(uint8_t)((num >> 24) & 0XFF)   //高32位        1111 

//参数调整功能码，上位机发送
//01 ACC校准
//02 GYRO校准
//03 ACC与GYRO校准
//04 MAG校准
//05 BARO校准
# define ADJ_COMMAND	0x01	

//上位机请求返回PID参数功能码
# define REQUEST_PID  0x02

//上位机调整传感器参数功能码
//int16	int16	int16 int16 int16 int16 int16 int16 int16 int16
//THR		YAW		ROL		PIT		AUX1	AUX2	AUX3	AUX4	AUX5	AUX6
# define ADJ_SENSER	0x03

//上位机调整PID参数1功能码
//数据格式如下
//int16	int16	int16 int16 int16 int16 int16 int16 int16 
//ROL_P	ROL_I	ROL_D	PIT_P	PIT_I	PIT_D	YAW_P	YAW_I	YAW_D
# define ADJ_PID1	0x10

//上位机调整PID参数2功能码
//数据格式如下
//int16	int16	int16 int16 int16 int16 int16  int16  int16 
//ALT_P	ALT_I	ALT_D	POS_P	POS_I	POS_D	PID1_P PID1_I PID3_D 
# define ADJ_PID2	0x11


//上位机调整PID参数3功能码
//数据格式如下
//int16		int16		int16	
//PID2_P	PID2_I	PID2_D		
# define ADJ_PID3	0x12

# define ADJ_PID4 0x13
# define ADJ_PID5 0x14
# define ADJ_PID6 0x15

//上位机调整静态偏差功能码
//数据格式如下
//int16 						int16
//OFFSET_ROL*1000		OFFSET_PIT*1000
# define ADJ_OFFSET	0x16

//上位机请求进入bootmode功能码
# define BOOTMODE	0xf0

//接收自定义浮点型数据，一共4个
# define REC_FLOAT	0XF1

//接收16位整形
# define REC_INT16	0XF2

//接收32位整形，有符号
# define REC_INT32	0XF3

//接收字符串数据
# define REC_STRING	0XF4

# define REC_COM		0XF5
	
	
	
void app_debug_Config(void);
void app_debug_PIDUpload(void);
void app_debug_PIDDownload(void);
void app_debug_SensorDataReport(void);
void app_debug_DataProcess(void);
	
void app_debug_StorePara(void);
void app_debug_ReadPara(void);
void app_debug_ShowPara(void);
# endif

/********************************************  END OF FILE  *******************************************/
	

