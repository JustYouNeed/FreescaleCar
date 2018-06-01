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
# ifndef __APP_ANO_H
# define __APP_ANO_H

# define	PROTO_VERSION		6.0

//PID参数在上传及接收时使用的变换因子
# define ANO_PID_TRAN_FAC_P 10.0f
# define ANO_PID_TRAN_FAC_I 10.0f
# define ANO_PID_TRAN_FAC_D	1.0f

/*  帧头  */
# define FRAME_HEADER1	0XAA
# define FRAME_HEADER2	0XAF
# define FREME_LEN			32

																										//              1111 1111 1111 1111
# define BYTE1(num)	(uint8_t)(num & 0XFF)           //取低8位                      1111
# define BYTE2(num) (uint8_t)((num >> 8) & 0XFF)    //高8位                   1111
# define BYTE3(num)	(uint8_t)((num >> 16) & 0XFF)		//高16位             1111
# define BYTE4(num)	(uint8_t)((num >> 24) & 0XFF)   //高32位        1111 

# define MERGE4(byte4, byte3, byte2, byte1, type)		((type)(int32_t)((byte4 << 24) | (byte3 << 16) | (byte2 << 8) | (byte1)))
# define MERGE3(byte3, byte2, byte1, type)					((type)(int32_t)((byte3 << 16) | (byte2 << 8) | (byte1)))
# define MERGE2(byte2, byte1, type)	  							((type)(int32_t)((byte2 << 8) | (byte1)))
	
	
	/*  功能码  */
typedef enum
{
	VERSION = 0x00,
	STATUS = 0x01,
//	SENSOR = 0x02,
	ACK = 0x02,
	RCDATA = 0x03,
	GPSDATA = 0x04,
	POWER = 0x05,
	MOTOR = 0x06,
	SENSOR2 = 0x07,
	STRING = 0xa0,
	COMMAND = 0xe0,
	PID = 0x10,
}FunCode_EnumTypeDef;

/*  命令功能码  */
typedef enum
{
	ACK_PID = 0x01,
	ACK_MODE = 0x02,
	ACK_G_OFFSET = 0x0c,
	ACK_DEST = 0x21,
	ACK_RESET_PID = 0xa1,
	ACK_RESET_ALL = 0xa2,
}ACK_EnumTypeDef;

/*  数据帧结构体  */
typedef struct
{
	uint8_t Device;
	uint8_t SrcAddr;
	uint8_t DestAddr;
	uint8_t FunCode;
	uint8_t DataLength;
	uint8_t Data[FREME_LEN];
	uint8_t CheckSum;
}ANO_TypeDef;

void app_ano_Thread(void);
void app_ano_ReceiveData(uint8_t data);
	
void app_ano_CarDataReport(void);
	
void debug_StorePara(void);
void debug_ReadPara(void);
void debug_ShowPara(void);
# endif

/********************************************  END OF FILE  *******************************************/
	

