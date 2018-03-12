/**
  *******************************************************************************************************
  * File Name: 
  * Author: 
  * Version: 
  * Date: 
  * Brief: 
  *******************************************************************************************************
  * History
  *
  *
  *******************************************************************************************************
  */	
	
# ifndef __APP_PID_H
# define __APP_PID_H

/*
	*******************************************************************************************************
	*                              INCLUDE FILES
	*******************************************************************************************************
*/
# include "bsp.h"
	
typedef struct
{
	float Kp_Straight;
	float Ki_Straight;
	float Kd_Straight;
	
	float Kp_Curved;
	float Ki_Curved;
	float Kd_Curved;
	
	float Error;
	int16_t Sum;
}PID_TypeDef;

extern PID_TypeDef PID;

void pid_ParaInit(void);
void pid_ReadPara(void);
void pid_StorePara(void);

# endif
	
	
/********************************************  END OF FILE  *******************************************/
	

