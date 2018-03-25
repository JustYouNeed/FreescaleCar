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

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "app_debug.h"
# include "bsp_led.h"
# include "FreescaleCar.h"


extern Uart_Str uart_info;
extern uint8_t TimerTaskRunMutexSignal;

/*  用于暂存从上位机接收到的数据  */
uint8_t RecBuff[512];
/*  接收到的上位机命令  */
uint8_t RecCommand;

/*
*********************************************************************************************************
*                      debug_Response                    
*
* Description: 响应上位机的命令
*             
* Arguments  : 1> funcode: 要响应的功能码
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void debug_Response(uint8_t funcode)
{
	uint8_t SendBuff[8] = {0};
	uint8_t i;
	
	SendBuff[0] = 0XAA;//帧起始
	SendBuff[1] = 0XAA;
	SendBuff[2] = 0XEF;
	SendBuff[3] = 7;
	SendBuff[4] = funcode;
	SendBuff[5] = RecBuff[RecBuff[3] + 0X04]&0xff;
	
	for(i = 0;i<7;i++) SendBuff[6] += SendBuff[i];
	
	bsp_uart_SendDataToBuff(COM0, SendBuff, 7);
}

/*
*********************************************************************************************************
*                      debug_DataProcess                    
*
* Description: 处理上位机下发的参数、命令
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 该函数可以定时调用,或者接收完一帧指令后自动调用
*********************************************************************************************************
*/
void debug_DataProcess(void)
{	
	if(!RecCommand) return;	/*  没有接收到指令  */
	
	switch(RecCommand)
	{
		case REQUEST_PID: debug_PIDParaReport(); break; /*  上传PID参数  */
		case ADJ_PID1:debug_PIDDownload();debug_Response(ADJ_PID1); break; /*  接收第PID参数1  */
		case ADJ_PID2:debug_Response(ADJ_PID2); break; /*  其他PID参数不接收  */
		case ADJ_PID3:debug_Response(ADJ_PID3); break; 
		case ADJ_PID4:debug_Response(ADJ_PID4); break; 
		case ADJ_PID5:debug_Response(ADJ_PID5); break; 
		case ADJ_PID6:debug_Response(ADJ_PID6); break; 
		case BOOTMODE:break;               /*  进入IAP下载模式命令  */
	}
	RecCommand = 0;     /*  处理完数据后接收标志复位  */
}

/*
*********************************************************************************************************
*                             debug_Handler             
*
* Description: 串口调试中断函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void debug_Handler(void)
{
	uint8_t RecvData;  //字节接收暂存
	uint8_t i = 0;  //
	uint8_t checkSum = 0; //
	static uint8_t uCnt = 0;
	
	(void)UART0_S1;
	
	if(UART0->S1 & UART_S1_RDRF_MASK)  /*  接收数据寄存器满  */
	{
		RecvData = UART0->D;		/*  读取数据并送入接收缓存区  */
		switch(uCnt)
		{
			case 0X00:
			{
					if(0XAA == RecvData) RecBuff[uCnt++] = RecvData;  //帧起始判断
					else uCnt = 0X00;
			}break;
			case 0X01:
			{
				if(0XAF == RecvData)	RecBuff[uCnt++] = RecvData;//帧起始判断
				else uCnt = 0X00;
			}break;
			case 0X02:RecBuff[uCnt++] = RecvData; break;  //功能码
			case 0X03:RecBuff[uCnt++] = RecvData; break; //数据长度，除去功能码以及起始帧与检验和
			default:if(uCnt < (RecBuff[3] + 0X05)) RecBuff[uCnt++] = RecvData;break;//接收数据
			//加0x05是因为 ano_info[len+4]中放着检验和，当接收完检验和时，uCnt = len+0x05,条件不成立
		}
		
		if(uCnt == (RecBuff[3] + 0X05))  //已经接收完整个数据帧
		{
			uCnt = 0;
			for(i = 0;i < RecBuff[3] + 0x04; i++) //计算检验和不包括最后一个的检验和，
			{
				checkSum += RecBuff[i]; //计算检验和
			}
			
			if((checkSum&0xff) != RecBuff[RecBuff[3] + 0X04]) 
			{
				RecCommand = 0; //接收错误
			}
			else 
			{
				RecCommand = RecBuff[2];		//数据检验无误，保存功能码
		#if ANO_DATA_PRECESS_ON==1  //选择是否接收完一帧数据后自动处理
				debug_DataProcess();
		#endif
			}
		}
	}
	
	/*  处理发送数据部分  */
	bsp_uart_IRQHandler(&uart_info);
}


/*
*********************************************************************************************************
*                     debug_DataUpload                     
*
* Description: 调试功能数据上传
*             
* Arguments  : 1> buff: 数据缓存区
*              2> funcode: 功能码
*							 3> len: 数据长度
*
* Reutrn     : 1> 0: 函数执行成功
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t debug_DataUpload(uint8_t *buff, uint8_t funCode, uint8_t len)
{
	uint8_t SendBuff[32] = {0};
	uint8_t i;
	if(len>27)return 0;//数据长度超过限制
	if(funCode>0xff) return 1;//功能码错误
		
	SendBuff[0] = 0XAA; //帧起始
	SendBuff[1] = 0XAA; //帧起始
	SendBuff[2] = funCode;  //功能码
	SendBuff[3] = len;   //数据长度，除去起始码和功能码以及长度
	
	/*  将数据复制到发送区并计算校验和  */
	for(i = 0; i < len; i++) 
	{
		SendBuff[i + 4] = buff[i];
		SendBuff[len + 4] += SendBuff[i];
	}
	/*  计算剩余的校验  */
	for(i = len; i< len + 4; i++) SendBuff[len + 4] += SendBuff[i];
	
	bsp_uart_SendDataToBuff(COM0, SendBuff, len + 5);
	
	return 0;
}


/*
*********************************************************************************************************
*                         debug_PIDParaReport                 
*
* Description: 填写PID参数并调用debug_DataUpload发送
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void debug_PIDParaReport(void)
{
	uint8_t Buff[18] = {0};
	short temp;
	
	temp = (short)(Car.PID.Kp_Straight * ANO_PID_TRAN_FAC_P);
	Buff[0] = BYTE2(temp);
	Buff[1] = BYTE1(temp);
	
	temp = (short)(Car.PID.Ki_Straight * ANO_PID_TRAN_FAC_I);
	Buff[2] = BYTE2(temp);
	Buff[3] = BYTE1(temp);
	
	temp = (short)(Car.PID.Kd_Straight * ANO_PID_TRAN_FAC_D);
	Buff[4] = BYTE2(temp);
	Buff[5] = BYTE1(temp);
	
	temp = (short)(Car.PID.Kp_Curved * ANO_PID_TRAN_FAC_P);
	Buff[6] = BYTE2(temp);
	Buff[7] = BYTE1(temp);
	
	temp = (short)(Car.PID.Ki_Curved * ANO_PID_TRAN_FAC_I);
	Buff[8] = BYTE2(temp);
	Buff[9] = BYTE1(temp);
	
	temp = (short)(Car.PID.Kd_Curved * ANO_PID_TRAN_FAC_D);
	Buff[10] = BYTE2(temp);
	Buff[11] = BYTE1(temp);

//	Buff[12] = BYTE2(Car.BaseSpeed);
//	Buff[13] = BYTE1(Car.BaseSpeed);
	
	temp = (short)(Car.PID.Velocity_Kp * ANO_PID_TRAN_FAC_P);
	Buff[12] = BYTE2(temp);
	Buff[13] = BYTE1(temp);
	
	temp = (short)(Car.PID.Velocity_Ki * ANO_PID_TRAN_FAC_I);
	Buff[14] = BYTE2(temp);
	Buff[15] = BYTE1(temp);
	
	temp = (short)(Car.PID.Velocity_Kd * ANO_PID_TRAN_FAC_D);
	Buff[16] = BYTE2(temp);
	Buff[17] = BYTE1(temp);
//	
//	temp = 0;
//	Buff[14] = BYTE2(temp);
//	Buff[15] = BYTE1(temp);
//	
//	temp = 0;
//	Buff[16] = BYTE2(temp);
//	Buff[17] = BYTE1(temp);
//	
	debug_DataUpload(Buff,0x10,18);
}

/*
*********************************************************************************************************
*                          debug_PIDDownload                
*
* Description: 接收上位机下发的PID参数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void debug_PIDDownload(void)
{
	//取前后两个8位的数据合并成一个16位的数据，并强制转换成一个float型的数据
	//转换完成后除以相应的传输因子
	uint8_t cnt = 4;
	
	Car.PID.Kp_Straight = MERGE(RecBuff[cnt], RecBuff[cnt+1], float) / ANO_PID_TRAN_FAC_P;
	cnt += 2;
	Car.PID.Ki_Straight = MERGE(RecBuff[cnt], RecBuff[cnt + 1], float) / ANO_PID_TRAN_FAC_I;
	cnt += 2;
	Car.PID.Kd_Straight = MERGE(RecBuff[cnt], RecBuff[cnt + 1], float) / ANO_PID_TRAN_FAC_D;
	cnt += 2;
	
	Car.PID.Kp_Curved = MERGE(RecBuff[cnt], RecBuff[cnt + 1], float) / ANO_PID_TRAN_FAC_P;
	cnt += 2;
	Car.PID.Ki_Curved = MERGE(RecBuff[cnt], RecBuff[cnt + 1], float) / ANO_PID_TRAN_FAC_I;
	cnt += 2;
	Car.PID.Kd_Curved = MERGE(RecBuff[cnt], RecBuff[cnt + 1], float) / ANO_PID_TRAN_FAC_D;
	cnt += 2;
	
	Car.PID.Velocity_Kp = MERGE(RecBuff[cnt], RecBuff[cnt + 1], float)/10;
	cnt += 2;
	Car.PID.Velocity_Ki = MERGE(RecBuff[cnt], RecBuff[cnt + 1], float) / 10;
	cnt += 2;
	Car.PID.Velocity_Kd = MERGE(RecBuff[cnt], RecBuff[cnt + 1], float)/10;
	cnt += 2;

	pid_StorePara();	/*  将PID参数保存到Flash中  */
}



/*
*********************************************************************************************************
*                      debug_SensorDataReport                    
*
* Description: 上传传感器数据
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void debug_SensorDataReport(void)
{
	uint8_t Buff[18] = {0};
	uint8_t cnt = 0;
	
	Buff[cnt++] = BYTE2(Car.Sensor[SENSOR_H_L].Average);
	Buff[cnt++] = BYTE1(Car.Sensor[SENSOR_H_L].Average);
	
	Buff[cnt++] = BYTE2(Car.Sensor[SENSOR_V_L].Average);
	Buff[cnt++] = BYTE1(Car.Sensor[SENSOR_V_L].Average);
	
	Buff[cnt++] = BYTE2(Car.Sensor[SENSOR_H_R].Average);
	Buff[cnt++] = BYTE1(Car.Sensor[SENSOR_H_R].Average);
	
	Buff[cnt++] = BYTE2(Car.Sensor[SENSOR_V_R].Average);
	Buff[cnt++] = BYTE1(Car.Sensor[SENSOR_V_R].Average);
	
	
	Buff[cnt++] = BYTE2((int16_t)(Car.HorizontalAE * 100));
	Buff[cnt++] = BYTE1((int16_t)(Car.HorizontalAE * 100));

	Buff[cnt++] = BYTE2((int16_t)(Car.VecticalAE * 100));
	Buff[cnt++] = BYTE1((int16_t)(Car.VecticalAE * 100));
	

	debug_DataUpload(Buff,0xf2,cnt);
}


/*
*********************************************************************************************************
*                                          
*
* Description: 
*             
* Arguments  : 
*
* Reutrn     : 
*
* Note(s)    : 
*********************************************************************************************************
*/
int16_t LPWM,RPWM;
void debug_MotorDataReport(void)
{
	uint8_t SendBuff[32] = {0};
	uint8_t cnt = 0;
	
	
	SendBuff[cnt++] = BYTE2(Car.Motor.LeftPwm);
	SendBuff[cnt++] = BYTE1(Car.Motor.LeftPwm);
	
	SendBuff[cnt++] = BYTE2(Car.Motor.RightPwm);
	SendBuff[cnt++] = BYTE1(Car.Motor.RightPwm);
	
	SendBuff[cnt++] = BYTE4(Car.Motor.LeftEncoder);
	SendBuff[cnt++] = BYTE3(Car.Motor.LeftEncoder);
	SendBuff[cnt++] = BYTE2(Car.Motor.LeftEncoder);
	SendBuff[cnt++] = BYTE1(Car.Motor.LeftEncoder);
	
	SendBuff[cnt++] = BYTE4(Car.Motor.RightEncoder);
	SendBuff[cnt++] = BYTE3(Car.Motor.RightEncoder);
	SendBuff[cnt++] = BYTE2(Car.Motor.RightEncoder);
	SendBuff[cnt++] = BYTE1(Car.Motor.RightEncoder);
	
	SendBuff[cnt++] = BYTE4((int)(Car.Motor.LeftSpeed*10));
	SendBuff[cnt++] = BYTE3((int)(Car.Motor.LeftSpeed*10));
	SendBuff[cnt++] = BYTE2((int)(Car.Motor.LeftSpeed*10));
	SendBuff[cnt++] = BYTE1((int)(Car.Motor.LeftSpeed*10));
	
	SendBuff[cnt++] = BYTE4((int)(Car.Motor.RightSpeed*10));
	SendBuff[cnt++] = BYTE3((int)(Car.Motor.RightSpeed*10));
	SendBuff[cnt++] = BYTE2((int)(Car.Motor.RightSpeed*10));
	SendBuff[cnt++] = BYTE1((int)(Car.Motor.RightSpeed*10));
	
	debug_DataUpload(SendBuff, 0xf1, cnt);
}
/*
*********************************************************************************************************
*                           debug_CarDataReport               
*
* Description: 车子调试数据上传函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 该函数应该周期性调用,以便及时收到车子的数据
*********************************************************************************************************
*/
void debug_CarDataReport(void)
{
	if(TimerTaskRunMutexSignal == 1) return ;
	TimerTaskRunMutexSignal = 1;
	
	/*  先处理收到的数据  */
	debug_DataProcess();
	
	/*  上传传感器数据  */
	debug_SensorDataReport();
	
	/*  上传电机数据  */
	debug_MotorDataReport();
	
	TimerTaskRunMutexSignal = 0;
}



/*
*********************************************************************************************************
*                              debug_ShowPara            
*
* Description: 将相关参数通过OLED屏幕显示出来
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void debug_ShowPara(void)
{
	if(TimerTaskRunMutexSignal == 1) return ;
	TimerTaskRunMutexSignal = 1;
	
//	bsp_oled_Clear();
//	bsp_oled_ShowInteger(0,0,(int)(Car.PID.Kp_Straight ), 16);
//	bsp_oled_ShowInteger(42,0,(int)(Car.PID.Ki_Straight ), 16);
//	bsp_oled_ShowInteger(84,0,(int)(Car.PID.Kd_Straight ), 16);
//	
//	bsp_oled_ShowString(0, 0, "L:");
//	bsp_oled_ShowInteger(24, 0, Car.Motor.LeftEncoder, 16);
//	bsp_oled_ShowInteger(66, 0, Car.Motor.LeftSpeed, 16);
//	
//	bsp_oled_ShowString(0, 2, "R:");
//	bsp_oled_ShowInteger(24, 2, Car.Motor.RightEncoder, 16);
//	bsp_oled_ShowInteger(66, 2, Car.Motor.RightSpeed, 16);
//	bsp_oled_ShowString(0, 2, "C0:");
//	bsp_oled_ShowInteger(24,2,Car.Sensor[SENSOR_H_L].CalibrationMax, 16);
//	
//	bsp_oled_ShowString(66, 2, "C3:");
//	bsp_oled_ShowInteger(90,2,Car.Sensor[SENSOR_V_R].CalibrationMax, 16);
//	
//	bsp_oled_ShowString(0, 4, "C0:");
//	bsp_oled_ShowInteger(24,4,Car.Sensor[SENSOR_H_L].Average, 16);
//	
//	bsp_oled_ShowString(66, 4, "C3:");
//	bsp_oled_ShowInteger(90,4,Car.Sensor[SENSOR_V_R].Average, 16);
//	
//	bsp_oled_ShowString(0, 6, "H:");
//	bsp_oled_ShowInteger(16,6,(int32_t)(Car.HorizontalAE * 10000), 16);
//	
//	bsp_oled_ShowString(80 - 16, 6, "V:");
//	bsp_oled_ShowInteger(80,6,(int32_t)(Car.VecticalAE), 16);

	TimerTaskRunMutexSignal = 0;
}

/********************************************  END OF FILE  *******************************************/

