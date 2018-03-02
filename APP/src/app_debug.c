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

/*  �����ݴ����λ�����յ�������  */
uint8_t RecBuff[512];
/*  ���յ�����λ������  */
uint8_t RecCommand;


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
void app_debug_SendChar(uint8_t byte)
{
	drv_uart_SendData(UART0, byte);
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
void app_debug_Response(uint8_t funcode)
{
	uint8_t SendBuff[8];
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
void app_debug_DataProcess()
{	
	if(!RecCommand) return;	/*  û�н��յ�ָ��  */
	switch(RecCommand)
	{
		case ADJ_COMMAND:break;
		case REQUEST_PID: //PID������������
		{
			app_debug_PIDUpload();  //�ϴ���һ��PID����
		}break;
		case ADJ_SENSER:break;  //��������������
		case ADJ_PID1:app_debug_PIDDownload();	bsp_led_Toggle(1);app_debug_Response(ADJ_PID1); break; //���յ�һ��PID��������Ӧ
		case ADJ_PID2:app_debug_Response(ADJ_PID2); break; //
		case ADJ_PID3:app_debug_Response(ADJ_PID3); break; //
		case ADJ_PID4:app_debug_Response(ADJ_PID4); break; //
		case ADJ_PID5:app_debug_Response(ADJ_PID5); break; //
		case ADJ_PID6:app_debug_Response(ADJ_PID6); break; //
		case ADJ_OFFSET:break;                                //������ƫ����
		case BOOTMODE:break;                                  //����IAP����ģʽ����
	}
	RecCommand = 0;     //���������ݺ���ձ�־��λ
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
void app_debug_Handler(void)
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
					if(0XAA == RecvData)	
					{
						RecBuff[uCnt++] = RecvData;  //֡��ʼ�ж�
					}
					else uCnt = 0X00;
			}break;
			case 0X01:
			{
				if(0XAF == RecvData)	
				{
					RecBuff[uCnt++] = RecvData;//֡��ʼ�ж�
					
				}
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
//		#if ANO_DATA_PRECESS_ON==1  //ѡ���Ƿ������һ֡���ݺ��Զ�����
//				app_debug_DataProcess();
//		#endif
			}
		}
	}
	bsp_uart_IRQHandler(&uart_info);
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
uint8_t app_debug_DataUpload(uint8_t *buff, uint8_t funCode, uint8_t len)
{
	uint8_t SendBuff[32] = {0};
	uint8_t i;
	if(len>27)return 0;//���ݳ��ȳ�������
	if(funCode>0xff) return 1;//���������
		
	SendBuff[0] = 0XAA; //֡��ʼ
	SendBuff[1] = 0XAA; //֡��ʼ
	SendBuff[2] = funCode;  //������
	SendBuff[3] = len;   //���ݳ��ȣ���ȥ��ʼ��͹������Լ�����
	
	//��Ҫ���͵����ݸ��Ƶ�������
	for(i = 0; i < len; i++) SendBuff[i + 4] = buff[i];
	//����У���
	for(i = 0; i< len + 4; i++) SendBuff[len + 4] += SendBuff[i];
	
	bsp_uart_SendDataToBuff(COM0, SendBuff, len + 5);
	
	return 2;
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
uint8_t app_debug_DataSend(uint8_t *buff,uint8_t funCode, uint8_t len)
{
	uint8_t SendBuff[32] = {0};
	uint8_t i;
	
//	for(i = 0; i < 32; i++) SendBuff[i] = 0X00;
	if(len>27)return 0;//���ݳ��ȳ�������
	if(funCode>0xff) return 1;//���������
		
	SendBuff[0] = 0XAA; //֡��ʼ
	SendBuff[1] = 0XAF; //֡��ʼ
	SendBuff[2] = funCode;  //������
	SendBuff[3] = len;   //���ݳ��ȣ���ȥ��ʼ��͹������Լ�����
	
	//��Ҫ���͵����ݸ��Ƶ�������
	for(i = 0; i < len; i++) SendBuff[i + 4] = buff[i];
	//����У���
	for(i = 0; i< len + 4; i++) SendBuff[len + 4] += SendBuff[i];
	//ѭ����������
//	for(i = 0; i< len + 5; i++) 
//		app_debug_SendChar(SendBuff[i]);
	bsp_uart_SendDataToBuff(COM0, SendBuff, len + 5);
	return 2;
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
void app_debug_PIDUpload(void)
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
	
//	temp = 0;
//	Buff[12] = BYTE2(temp);
//	Buff[13] = BYTE1(temp);
//	
//	temp = 0;
//	Buff[14] = BYTE2(temp);
//	Buff[15] = BYTE1(temp);
//	
//	temp = 0;
//	Buff[16] = BYTE2(temp);
//	Buff[17] = BYTE1(temp);
//	
	app_debug_DataUpload(Buff,0x10,18);
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
void app_debug_PIDDownload(void)
{
	//ȡǰ������8λ�����ݺϲ���һ��16λ�����ݣ���ǿ��ת����һ��float�͵�����
	//ת����ɺ������Ӧ�Ĵ�������
	Car.PID.Kp_Straight = (float)((int16_t)((RecBuff[4] << 8) | (RecBuff[5]))) / ANO_PID_TRAN_FAC_P;
	Car.PID.Ki_Straight = (float)((int16_t)(RecBuff[6] << 8) | (RecBuff[7])) / ANO_PID_TRAN_FAC_I;
	Car.PID.Kd_Straight = (float)((int16_t)(RecBuff[8] << 8) | (RecBuff[9])) / ANO_PID_TRAN_FAC_D;
	
	Car.PID.Kp_Curved = (float)((int16_t)((RecBuff[10] << 8) | (RecBuff[11]))) / ANO_PID_TRAN_FAC_P;
	Car.PID.Ki_Curved = (float)((int16_t)(RecBuff[12] << 8) | (RecBuff[13])) / ANO_PID_TRAN_FAC_I;
	Car.PID.Kd_Curved = (float)((int16_t)(RecBuff[14] << 8) | (RecBuff[15])) / ANO_PID_TRAN_FAC_D;
	
	pid_StorePara();
//	ano_info.yaw_p = (float)((int16_t)((ano_info.RecBuff[16]<<8)|(ano_info.RecBuff[17])))/ANO_PID_TRAN_FAC_P;
//	ano_info.yaw_i = (float)((int16_t)(ano_info.RecBuff[18]<<8)|(ano_info.RecBuff[19]))/ANO_PID_TRAN_FAC_I;
//	ano_info.yaw_d = (float)((int16_t)(ano_info.RecBuff[20]<<8)|(ano_info.RecBuff[21]))/ANO_PID_TRAN_FAC_D;
}


/*
*********************************************************************************************************
*                            app_debug_SensorDataReport              
*
* Description: ����λ����������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/

void app_debug_SensorDataReport(void)
{
	uint8_t Buff[18];
	uint8_t i;
	for(i = 0;i< 18; i++) Buff[i] = 0x00;
	
	Buff[0] = (uint8_t)((Car.Sensor[SENSOR_ID_1].Average >> 8) & 0xff);
	Buff[1] = (uint8_t)(Car.Sensor[SENSOR_ID_1].Average & 0xff);
	
	Buff[2] = (uint8_t)((Car.Sensor[SENSOR_ID_2].Average >> 8) & 0xff);
	Buff[3] = (uint8_t)(Car.Sensor[SENSOR_ID_2].Average & 0xff);
	
	Buff[4] = (uint8_t)((Car.Sensor[SENSOR_ID_3].Average >> 8) & 0xff);
	Buff[5] = (uint8_t)(Car.Sensor[SENSOR_ID_3].Average & 0xff);
	
	Buff[6] = (uint8_t)((Car.Sensor[SENSOR_ID_4].Average >> 8) & 0xff);
	Buff[7] = (uint8_t)(Car.Sensor[SENSOR_ID_4].Average & 0xff);
	
	Buff[8] = (uint8_t)((Car.Motor.LeftEncoder >> 8) & 0xff);
	Buff[9] = (uint8_t)(Car.Motor.LeftEncoder & 0xff);
	
	Buff[10] = (uint8_t)((Car.Motor.RightEncoder >> 8) & 0xff);
	Buff[11] = (uint8_t)(Car.Motor.RightEncoder & 0xff);
	
//	Car.HorizontalAE = (Car.HorizontalAE < 0) ? (-Car.HorizontalAE) : Car.HorizontalAE;
	Buff[12] = (uint8_t)((Car.HorizontalAE) >> 8) & 0xff;
	Buff[13] = (uint8_t)(Car.HorizontalAE) & 0xff;
	
//	Car.VecticalAE = (Car.VecticalAE < 0) ? (-Car.VecticalAE) : Car.VecticalAE;
	Buff[14] = (uint8_t)(Car.VecticalAE >> 8) & 0xff;
	Buff[15] = (uint8_t)(Car.VecticalAE & 0xff);
//	
//	Buff[16] = (20 >> 8) & 0xff;
//	Buff[17] = 20 & 0xff;
	
	app_debug_DataUpload(Buff,0x02,18);
}
	

/*
*********************************************************************************************************
*                          app_debug_StorePara                
*
* Description: �������洢�������ϵĴ洢оƬ��
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void app_debug_StorePara(void)
{
	
}


/*
*********************************************************************************************************
*                                    app_debug_ReadPara      
*
* Description: �������ϵĴ洢оƬ�ж�ȡ������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void app_debug_ReadPara(void)
{
}

/*
*********************************************************************************************************
*                              app_debug_ShowPara            
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

extern uint8_t TimerTaskRunMutexSignal;
void app_debug_ShowPara(void)
{
	if(TimerTaskRunMutexSignal == 1) return ;
	TimerTaskRunMutexSignal = 1;
	
	bsp_oled_Clear();
	bsp_oled_ShowInteger(0,0,(int)(Car.PID.Kp_Straight * 100), 16);
	bsp_oled_ShowInteger(42,0,(int)(Car.PID.Ki_Straight * 100), 16);
	bsp_oled_ShowInteger(84,0,(int)(Car.PID.Kd_Straight * 100), 16);
	
//	bsp_oled_ShowInteger(0,2,LeftEncoderCounter, 16);
//	bsp_oled_ShowInteger(60,2,RightEncoderCounter, 16);
//	bsp_oled_ShowInteger(0,4,LeftSpeed, 16);
//	bsp_oled_ShowInteger(60,4,RightSpeed, 16);
	
//	drv_adc_GetMultiADCResult(ADC_Value);
	bsp_oled_ShowString(0, 2, "C0:");
	bsp_oled_ShowInteger(24,2,Car.Sensor[SENSOR_ID_1].CalibrationMax, 16);
	
	bsp_oled_ShowString(66, 2, "C3:");
	bsp_oled_ShowInteger(90,2,Car.Sensor[SENSOR_ID_4].CalibrationMax, 16);
	
	bsp_oled_ShowString(0, 4, "C0:");
	bsp_oled_ShowInteger(24,4,Car.Sensor[SENSOR_ID_1].Average, 16);
	
	bsp_oled_ShowString(66, 4, "C3:");
	bsp_oled_ShowInteger(90,4,Car.Sensor[SENSOR_ID_4].Average, 16);
	
	bsp_oled_ShowString(0, 6, "H:");
	bsp_oled_ShowInteger(16,6,(int32_t)(Car.HorizontalAE * 1000), 16);
	
		bsp_oled_ShowString(80 - 16, 6, "V:");
	bsp_oled_ShowInteger(80,6,(int32_t)(Car.VecticalAE * 1000), 16);
//	bsp_oled_ShowString(0, 6, "F6:");
//	bsp_oled_ShowInteger(24,6,ADC_Value[4], 16);
//	
//	bsp_oled_ShowString(66, 6, "F7:");
//	bsp_oled_ShowInteger(90,6,ADC_Value[5], 16);
	
	app_debug_SensorDataReport();
	TimerTaskRunMutexSignal = 0;
}

/********************************************  END OF FILE  *******************************************/

