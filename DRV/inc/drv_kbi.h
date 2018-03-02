/**
  *******************************************************************************************************
  * File Name: drv_kbi.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-2-28
  * Brief: 本文件声明了有关操作KBI外设的底层驱动函数、变量以及宏定义
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*     Date: 2018-2-28
	*     Mod: 建立文件
  *
  *******************************************************************************************************
  */	
	
# ifndef __DRV_KBI_H
# define __DRV_KBI_H

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "derivative.h"

typedef void (* _cbKBICallBack)(void);

# define KBI_Channel_A0			((uint32_t)0x00000001)
# define KBI_Channel_A1			((uint32_t)0x00000002)
# define KBI_Channel_A2			((uint32_t)0x00000004)
# define KBI_Channel_A3			((uint32_t)0x00000008)
# define KBI_Channel_A4			((uint32_t)0x00000010)
# define KBI_Channel_A5			((uint32_t)0x00000020)
# define KBI_Channel_A6			((uint32_t)0x00000040)
# define KBI_Channel_A7			((uint32_t)0x00000080)

# define KBI_Channel_B0			((uint32_t)0x00000100)
# define KBI_Channel_B1			((uint32_t)0x00000200)
# define KBI_Channel_B2			((uint32_t)0x00000400)
# define KBI_Channel_B3			((uint32_t)0x00000800)
# define KBI_Channel_B4			((uint32_t)0x00001000)
# define KBI_Channel_B5			((uint32_t)0x00002000)
# define KBI_Channel_B6			((uint32_t)0x00004000)
# define KBI_Channel_B7			((uint32_t)0x00008000)

# define KBI_Channel_C0			((uint32_t)0x00010000)
# define KBI_Channel_C1			((uint32_t)0x00020000)
# define KBI_Channel_C2			((uint32_t)0x00040000)
# define KBI_Channel_C3			((uint32_t)0x00080000)
# define KBI_Channel_C4			((uint32_t)0x00100000)
# define KBI_Channel_C5			((uint32_t)0x00200000)
# define KBI_Channel_C6			((uint32_t)0x00400000)
# define KBI_Channel_C7			((uint32_t)0x00800000)

# define KBI_Channel_D0			((uint32_t)0x01000000)
# define KBI_Channel_D1			((uint32_t)0x02000000)
# define KBI_Channel_D2			((uint32_t)0x04000000)
# define KBI_Channel_D3			((uint32_t)0x08000000)
# define KBI_Channel_D4			((uint32_t)0x10000000)
# define KBI_Channel_D5			((uint32_t)0x20000000)
# define KBI_Channel_D6			((uint32_t)0x40000000)
# define KBI_Channel_D7			((uint32_t)0x80000000)

# define KBI_Channel_E0			((uint32_t)0xF0000001)
# define KBI_Channel_E1			((uint32_t)0xF0000002)
# define KBI_Channel_E2			((uint32_t)0xF0000004)
# define KBI_Channel_E3			((uint32_t)0xF0000008)
# define KBI_Channel_E4			((uint32_t)0xF0000010)
# define KBI_Channel_E5			((uint32_t)0xF0000020)
# define KBI_Channel_E6			((uint32_t)0xF0000040)
# define KBI_Channel_E7			((uint32_t)0xF0000080)

# define KBI_Channel_F0			((uint32_t)0xF0000100)
# define KBI_Channel_F1			((uint32_t)0xF0000200)
# define KBI_Channel_F2			((uint32_t)0xF0000400)
# define KBI_Channel_F3			((uint32_t)0xF0000800)
# define KBI_Channel_F4			((uint32_t)0xF0001000)
# define KBI_Channel_F5			((uint32_t)0xF0002000)
# define KBI_Channel_F6			((uint32_t)0xF0004000)
# define KBI_Channel_F7			((uint32_t)0xF0008000)

# define KBI_Channel_H0			((uint32_t)0xF0010000)
# define KBI_Channel_H1			((uint32_t)0xF0020000)
# define KBI_Channel_H2			((uint32_t)0xF0040000)
# define KBI_Channel_H3			((uint32_t)0xF0080000)
# define KBI_Channel_H4			((uint32_t)0xF0100000)
# define KBI_Channel_H5			((uint32_t)0xF0200000)
# define KBI_Channel_H6			((uint32_t)0xF0400000)
# define KBI_Channel_H7			((uint32_t)0xF0800000)


///*  KBI通道枚举变量类型  */
//typedef enum
//{
//	KBI_Channel_A0 = 0,
//	KBI_Channel_A1,
//	KBI_Channel_A2,
//	KBI_Channel_A3,
//	KBI_Channel_A4,
//	KBI_Channel_A5,
//	KBI_Channel_A6,
//	KBI_Channel_A7,
//	
//	KBI_Channel_B0,
//	KBI_Channel_B1,
//	KBI_Channel_B2,
//	KBI_Channel_B3,
//	KBI_Channel_B4,
//	KBI_Channel_B5,
//	KBI_Channel_B6,
//	KBI_Channel_B7,
//	
//	KBI_Channel_C0,
//	KBI_Channel_C1,
//	KBI_Channel_C2,
//	KBI_Channel_C3,
//	KBI_Channel_C4,
//	KBI_Channel_C5,
//	KBI_Channel_C6,
//	KBI_Channel_C7,
//	
//	KBI_Channel_D0,
//	KBI_Channel_D1,
//	KBI_Channel_D2,
//	KBI_Channel_D3,
//	KBI_Channel_D4,
//	KBI_Channel_D5,
//	KBI_Channel_D6,
//	KBI_Channel_D7,
//	
//	KBI_Channel_E0,
//	KBI_Channel_E1,
//	KBI_Channel_E2,
//	KBI_Channel_E3,
//	KBI_Channel_E4,
//	KBI_Channel_E5,
//	KBI_Channel_E6,
//	KBI_Channel_E7,
//	
//	KBI_Channel_F0,
//	KBI_Channel_F1,
//	KBI_Channel_F2,
//	KBI_Channel_F3,
//	KBI_Channel_F4,
//	KBI_Channel_F5,
//	KBI_Channel_F6,
//	KBI_Channel_F7,
//	
//	KBI_Channel_G0,
//	KBI_Channel_G1,
//	KBI_Channel_G2,
//	KBI_Channel_G3,
//	KBI_Channel_G4,
//	KBI_Channel_G5,
//	KBI_Channel_G6,
//	KBI_Channel_G7,
//	
//	KBI_Channel_H0,
//	KBI_Channel_H1,
//	KBI_Channel_H2,
//	KBI_Channel_H3,
//	KBI_Channel_H4,
//	KBI_Channel_H5,
//	KBI_Channel_H6,
//	KBI_Channel_H7,
//}KBI_ChannelType;


/*  KBI触发方式枚举变量  */
typedef enum
{
	KBI_TrigFalling = 0,
	KBI_TrigRising,
	KBI_TringFallingLow,
	KBI_TrigRisingHigh,
}KBI_TrigModeType;

/*  KBI初始化结构体  */
typedef struct
{
	uint32_t KBI_Channel;							/*  KBI通道  */
	uint8_t KBI_TrigMode;							/*  触发模式  */
	uint8_t KBI_PuPdCmd;							/*  是否开启上拉  */
	FunctionalState	KBI_EdgeOnlyCmd;	/*  是否只检测边沿  */
}KBI_InitTypeDef;


/*  用于管理KBI的结构体  */
typedef struct
{
	uint32_t KBI_Channel;
	uint8_t KBI_Used;
	_cbKBICallBack	_cb;
}KBI_ManageDef;



void drv_kbi_Init(KBI_InitTypeDef *KBI_InitStruct);
void drv_kbi_SetCallback(uint32_t KBI_Channel, _cbKBICallBack _cb);
# endif

/********************************************  END OF FILE  *******************************************/

