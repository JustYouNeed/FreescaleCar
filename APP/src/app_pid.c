/**
  *******************************************************************************************************
  * File Name: app_pid.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: 本文件提供了PID算法函数,以及PID参数的读取、存储功能
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 建立文件
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
* Description: 从Flash中读取存储的PID参数
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
	Car.PID.Kp_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 0, float);
	Car.PID.Ki_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 4, float);
	Car.PID.Kd_Straight = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 8, float);
	
	Car.PID.Kp_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 12, float);
	Car.PID.Ki_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 16, float);
	Car.PID.Kd_Curved = drv_flash_ReadSector(PID_PARA_FLASH_ADDR, 20, float);
}

/*
*********************************************************************************************************
*                      pid_StorePara                    
*
* Description: 将PID参数存储到Flash中
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
	drv_flash_EraseSector(PID_PARA_FLASH_ADDR);	/*  先擦除一遍,不然无法写入  */
	drv_flash_WriteSector(PID_PARA_FLASH_ADDR, (const uint8_t*)&Car.PID, 48, 0);
}

/********************************************  END OF FILE  *******************************************/

