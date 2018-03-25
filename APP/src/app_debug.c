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

/*  �����ݴ����λ�����յ�������  */
uint8_t RecBuff[512];
/*  ���յ�����λ������  */
uint8_t RecCommand;

/*
*********************************************************************************************************
*                      debug_Response                    
*
* Description: ��Ӧ��λ��������
*             
* Arguments  : 1> funcode: Ҫ��Ӧ�Ĺ�����
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
	
	SendBuff[0] = 0XAA;//֡��ʼ
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
* Description: ������λ���·��Ĳ���������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �ú������Զ�ʱ����,���߽�����һָ֡����Զ�����
*********************************************************************************************************
*/
void debug_DataProcess(void)
{	
	if(!RecCommand) return;	/*  û�н��յ�ָ��  */
	
	switch(RecCommand)
	{
		case REQUEST_PID: debug_PIDParaReport(); break; /*  �ϴ�PID����  */
		case ADJ_PID1:debug_PIDDownload();debug_Response(ADJ_PID1); break; /*  ���յ�PID����1  */
		case ADJ_PID2:debug_Response(ADJ_PID2); break; /*  ����PID����������  */
		case ADJ_PID3:debug_Response(ADJ_PID3); break; 
		case ADJ_PID4:debug_Response(ADJ_PID4); break; 
		case ADJ_PID5:debug_Response(ADJ_PID5); break; 
		case ADJ_PID6:debug_Response(ADJ_PID6); break; 
		case BOOTMODE:break;               /*  ����IAP����ģʽ����  */
	}
	RecCommand = 0;     /*  ���������ݺ���ձ�־��λ  */
}

/*
*********************************************************************************************************
*                             debug_Handler             
*
* Description: ���ڵ����жϺ���
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
	uint8_t RecvData;  //�ֽڽ����ݴ�
	uint8_t i = 0;  //
	uint8_t checkSum = 0; //
	static uint8_t uCnt = 0;
	
	(void)UART0_S1;
	
	if(UART0->S1 & UART_S1_RDRF_MASK)  /*  �������ݼĴ�����  */
	{
		RecvData = UART0->D;		/*  ��ȡ���ݲ�������ջ�����  */
		switch(uCnt)
		{
			case 0X00:
			{
					if(0XAA == RecvData) RecBuff[uCnt++] = RecvData;  //֡��ʼ�ж�
					else uCnt = 0X00;
			}break;
			case 0X01:
			{
				if(0XAF == RecvData)	RecBuff[uCnt++] = RecvData;//֡��ʼ�ж�
				else uCnt = 0X00;
			}break;
			case 0X02:RecBuff[uCnt++] = RecvData; break;  //������
			case 0X03:RecBuff[uCnt++] = RecvData; break; //���ݳ��ȣ���ȥ�������Լ���ʼ֡������
			default:if(uCnt < (RecBuff[3] + 0X05)) RecBuff[uCnt++] = RecvData;break;//��������
			//��0x05����Ϊ ano_info[len+4]�з��ż���ͣ�������������ʱ��uCnt = len+0x05,����������
		}
		
		if(uCnt == (RecBuff[3] + 0X05))  //�Ѿ���������������֡
		{
			uCnt = 0;
			for(i = 0;i < RecBuff[3] + 0x04; i++) //�������Ͳ��������һ���ļ���ͣ�
			{
				checkSum += RecBuff[i]; //��������
			}
			
			if((checkSum&0xff) != RecBuff[RecBuff[3] + 0X04]) 
			{
				RecCommand = 0; //���մ���
			}
			else 
			{
				RecCommand = RecBuff[2];		//���ݼ������󣬱��湦����
		#if ANO_DATA_PRECESS_ON==1  //ѡ���Ƿ������һ֡���ݺ��Զ�����
				debug_DataProcess();
		#endif
			}
		}
	}
	
	/*  ���������ݲ���  */
	bsp_uart_IRQHandler(&uart_info);
}


/*
*********************************************************************************************************
*                     debug_DataUpload                     
*
* Description: ���Թ��������ϴ�
*             
* Arguments  : 1> buff: ���ݻ�����
*              2> funcode: ������
*							 3> len: ���ݳ���
*
* Reutrn     : 1> 0: ����ִ�гɹ�
*
* Note(s)    : None.
*********************************************************************************************************
*/
uint8_t debug_DataUpload(uint8_t *buff, uint8_t funCode, uint8_t len)
{
	uint8_t SendBuff[32] = {0};
	uint8_t i;
	if(len>27)return 0;//���ݳ��ȳ�������
	if(funCode>0xff) return 1;//���������
		
	SendBuff[0] = 0XAA; //֡��ʼ
	SendBuff[1] = 0XAA; //֡��ʼ
	SendBuff[2] = funCode;  //������
	SendBuff[3] = len;   //���ݳ��ȣ���ȥ��ʼ��͹������Լ�����
	
	/*  �����ݸ��Ƶ�������������У���  */
	for(i = 0; i < len; i++) 
	{
		SendBuff[i + 4] = buff[i];
		SendBuff[len + 4] += SendBuff[i];
	}
	/*  ����ʣ���У��  */
	for(i = len; i< len + 4; i++) SendBuff[len + 4] += SendBuff[i];
	
	bsp_uart_SendDataToBuff(COM0, SendBuff, len + 5);
	
	return 0;
}


/*
*********************************************************************************************************
*                         debug_PIDParaReport                 
*
* Description: ��дPID����������debug_DataUpload����
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
* Description: ������λ���·���PID����
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
	//ȡǰ������8λ�����ݺϲ���һ��16λ�����ݣ���ǿ��ת����һ��float�͵�����
	//ת����ɺ������Ӧ�Ĵ�������
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

	pid_StorePara();	/*  ��PID�������浽Flash��  */
}



/*
*********************************************************************************************************
*                      debug_SensorDataReport                    
*
* Description: �ϴ�����������
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
* Description: ���ӵ��������ϴ�����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : �ú���Ӧ�������Ե���,�Ա㼰ʱ�յ����ӵ�����
*********************************************************************************************************
*/
void debug_CarDataReport(void)
{
	if(TimerTaskRunMutexSignal == 1) return ;
	TimerTaskRunMutexSignal = 1;
	
	/*  �ȴ����յ�������  */
	debug_DataProcess();
	
	/*  �ϴ�����������  */
	debug_SensorDataReport();
	
	/*  �ϴ��������  */
	debug_MotorDataReport();
	
	TimerTaskRunMutexSignal = 0;
}



/*
*********************************************************************************************************
*                              debug_ShowPara            
*
* Description: ����ز���ͨ��OLED��Ļ��ʾ����
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

