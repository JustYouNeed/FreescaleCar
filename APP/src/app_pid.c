/**
  *******************************************************************************************************
  * File Name: app_pid.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: ���ļ��ṩ��PID�㷨����,�Լ�PID�����Ķ�ȡ���洢����
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	
	
/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "app_pid.h"
# include "FreescaleCar.h"


/*
*********************************************************************************************************
*                        pid_ReadPara                  
*
* Description: ��Flash�ж�ȡ�洢��PID����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void pid_ReadPara(void)
{
	Car.PID.DirectionKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);
	Car.PID.DirectionKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.DirectionKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.SpeedKp = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);
	Car.PID.SpeedKi = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.SpeedKd = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
}

/*
*********************************************************************************************************
*                      pid_StorePara                    
*
* Description: ��PID�����洢��Flash��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void pid_StorePara(void)
{	
	drv_flash_EraseSector(PID_PARA_FLASH_ADDR);	/*  �Ȳ���һ��,��Ȼ�޷�д��  */
	drv_flash_WriteSector(PID_PARA_FLASH_ADDR, (const uint8_t*)&Car.PID, 48, 0);
}

/********************************************  END OF FILE  *******************************************/

