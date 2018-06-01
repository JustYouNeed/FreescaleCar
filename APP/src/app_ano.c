/**
  *******************************************************************************************************
  * File Name: app_ano.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: �������ģ��,�ṩ����λ��ͨ�ŵĹ���
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: �����ļ�,��Ӧ������λ��V5.0
	*		
	*
  *
  *******************************************************************************************************
  */	

/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "app.h"
# include "bsp.h"
# include "FreescaleCar.h"

/*
  *******************************************************************************************************
  *                              LOCAL VARIABLE
  *******************************************************************************************************
*/
static ANO_TypeDef ANO_RX;						/*  ���ڽ��յĽṹ��  */
static ANO_TypeDef ANO_TX;						/*  ���ڷ���  */
static uint8_t g_ucDataRecCnt = 0;		/*  ����Э���������ʱ����,����֡  */
static uint8_t g_ucDatacnt = 0;				/*  ����Э��������ݲ��ּ���  */

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
static void app_ano_Response(uint8_t funcode)
{
	uint8_t SendBuff[8] = {0};
	uint8_t i;
	
	SendBuff[0] = 0XAA;//֡��ʼ
	SendBuff[1] = 0XAA;
	SendBuff[2] = 0XEF;
	SendBuff[3] = 7;
	SendBuff[4] = funcode;
	SendBuff[5] = ANO_RX.Data[ANO_RX.DataLength + 0X04]&0xff;
	
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
//	if(!RecCommand) return;	/*  û�н��յ�ָ��  */
//	
//	switch(RecCommand)
//	{
//		case REQUEST_PID: debug_PIDParaReport(); break; /*  �ϴ�PID����  */
//		case ADJ_PID1:debug_PIDDownload();debug_Response(ADJ_PID1); break; /*  ���յ�PID����1  */
//		case ADJ_PID2:debug_Response(ADJ_PID2); break; /*  ����PID����������  */
//		case ADJ_PID3:debug_Response(ADJ_PID3); break; 
//		case ADJ_PID4:debug_Response(ADJ_PID4); break; 
//		case ADJ_PID5:debug_Response(ADJ_PID5); break; 
//		case ADJ_PID6:debug_Response(ADJ_PID6); break; 
//		case BOOTMODE:break;               /*  ����IAP����ģʽ����  */
//	}
}



/*
*********************************************************************************************************
*                     app_ano_DataUpload                     
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
uint8_t app_ano_DataUpload(uint8_t *buff, uint8_t funCode, uint8_t len)
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
* Description: ��дPID����������app_ano_DataUpload����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void app_ano_PIDReport(void)
{
	uint8_t Buff[18] = {0};
	short temp = 0;
	
	temp = (short)(Car.DirFuzzy.KPMax * ANO_PID_TRAN_FAC_P);
	Buff[0] = BYTE2(temp);
	Buff[1] = BYTE1(temp);
	
	temp = (short)(Car.DirFuzzy.KIMax * ANO_PID_TRAN_FAC_I);
	Buff[2] = BYTE2(temp);
	Buff[3] = BYTE1(temp);
	
	temp = (short)(Car.DirFuzzy.KDMax * ANO_PID_TRAN_FAC_D);
	Buff[4] = BYTE2(temp);
	Buff[5] = BYTE1(temp);
	
	
	temp = (short)(Car.LVelPID.Kp * 100);
	Buff[6] = BYTE2(temp);
	Buff[7] = BYTE1(temp);
	
	temp = (short)(Car.LVelPID.Ki * 100);
	Buff[8] = BYTE2(temp);
	Buff[9] = BYTE1(temp);
	
	temp = (short)(Car.LVelPID.Kd * 100);
	Buff[10] = BYTE2(temp);
	Buff[11] = BYTE1(temp);
	
	app_ano_DataUpload(Buff,PID,18);
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
void app_ano_PIDDownload(void)
{
	//ȡǰ������8λ�����ݺϲ���һ��16λ�����ݣ���ǿ��ת����һ��float�͵�����
	//ת����ɺ������Ӧ�Ĵ�������
	uint8_t cnt = 0;
	
	Car.DirFuzzy.KPMax = MERGE2(ANO_RX.Data[cnt], ANO_RX.Data[cnt + 1], float) / ANO_PID_TRAN_FAC_P;
	cnt += 2;
	Car.DirFuzzy.KIMax = MERGE2(ANO_RX.Data[cnt], ANO_RX.Data[cnt + 1], float) / ANO_PID_TRAN_FAC_I;
	cnt += 2;
	Car.DirFuzzy.KDMax = MERGE2(ANO_RX.Data[cnt], ANO_RX.Data[cnt + 1], float) / ANO_PID_TRAN_FAC_D;
	cnt += 2;
	
//	Car.PID.Kp_Curved = MERGE2(ANO_RX.Data[cnt], ANO_RX.Data[cnt + 1], float) / ANO_PID_TRAN_FAC_P;
//	cnt += 2;
//	Car.PID.Ki_Curved = MERGE2(ANO_RX.Data[cnt], ANO_RX.Data[cnt + 1], float) / ANO_PID_TRAN_FAC_I;
//	cnt += 2;
//	Car.PID.Kd_Curved = MERGE2(ANO_RX.Data[cnt], ANO_RX.Data[cnt + 1], float) / ANO_PID_TRAN_FAC_D;
//	cnt += 2;
	
	Car.LVelPID.Kp = MERGE2(ANO_RX.Data[cnt], ANO_RX.Data[cnt + 1], float)/100;
	cnt += 2;
	Car.LVelPID.Ki = MERGE2(ANO_RX.Data[cnt], ANO_RX.Data[cnt + 1], float) / 100;
	cnt += 2;
	Car.LVelPID.Kd = MERGE2(ANO_RX.Data[cnt], ANO_RX.Data[cnt + 1], float)/100;
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
	uint8_t Buff[32] = {0};
	uint8_t cnt = 0;
	
	Buff[cnt++] = BYTE2(Car.Sensor[SENSOR_H_L].Average);
	Buff[cnt++] = BYTE1(Car.Sensor[SENSOR_H_L].Average);
	
	Buff[cnt++] = BYTE2(Car.Sensor[SENSOR_V_L].Average);
	Buff[cnt++] = BYTE1(Car.Sensor[SENSOR_V_L].Average);
	
	Buff[cnt++] = BYTE2(Car.Sensor[SENSOR_V_R].Average);
	Buff[cnt++] = BYTE1(Car.Sensor[SENSOR_V_R].Average);
	
	Buff[cnt++] = BYTE2(Car.Sensor[SENSOR_H_R].Average);
	Buff[cnt++] = BYTE1(Car.Sensor[SENSOR_H_R].Average);
		
	
	Buff[cnt++] = BYTE2((int16_t)(Car.HorizontalAE ));
	Buff[cnt++] = BYTE1((int16_t)(Car.HorizontalAE ));

	Buff[cnt++] = BYTE2((int16_t)(Car.VecticalAE));
	Buff[cnt++] = BYTE1((int16_t)(Car.VecticalAE));
	
	
	Buff[cnt++] = BYTE2((int16_t)(Car.AE));
	Buff[cnt++] = BYTE1((int16_t)(Car.AE));
	
	Buff[cnt++] = BYTE2(Car.Sensor[SENSOR_M].Average);
	Buff[cnt++] = BYTE1(Car.Sensor[SENSOR_M].Average);
	
	app_ano_DataUpload(Buff,0xf2,cnt);
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
	
	SendBuff[cnt++] = BYTE2((int16_t)(Car.CarSpeed));
	SendBuff[cnt++] = BYTE1((int16_t)(Car.CarSpeed));
	
	SendBuff[cnt++] = BYTE2((int16_t)(Car.Motor.LeftSpeed));
	SendBuff[cnt++] = BYTE1((int16_t)(Car.Motor.LeftSpeed));
	
	SendBuff[cnt++] = BYTE2((int16_t)(Car.Motor.RightSpeed));
	SendBuff[cnt++] = BYTE1((int16_t)(Car.Motor.RightSpeed));

	
	app_ano_DataUpload(SendBuff, 0xf1, cnt);
}

void debug_MPUDataReport(void)
{
	uint8_t SendBuff[32] = {0};
	uint8_t cnt = 0;
	
	SendBuff[cnt ++] = BYTE2(Car.MPU.Accx);
	SendBuff[cnt ++] = BYTE1(Car.MPU.Accx);
	
	SendBuff[cnt ++] = BYTE2(Car.MPU.Accy);
	SendBuff[cnt ++] = BYTE1(Car.MPU.Accy);
	
	SendBuff[cnt ++] = BYTE2(Car.MPU.Accz);
	SendBuff[cnt ++] = BYTE1(Car.MPU.Accz);
	
	SendBuff[cnt ++] = BYTE2(Car.MPU.Gyrox);
	SendBuff[cnt ++] = BYTE1(Car.MPU.Gyrox);
	
	SendBuff[cnt ++] = BYTE2(Car.MPU.Gyroy);
	SendBuff[cnt ++] = BYTE1(Car.MPU.Gyroy);
	
	SendBuff[cnt ++] = BYTE2(Car.MPU.Gyroz);
	SendBuff[cnt ++] = BYTE1(Car.MPU.Gyroz);
	
	
	app_ano_DataUpload(SendBuff, 0x02, 18);
	
	cnt = 0;
	SendBuff[cnt++] = BYTE2((int16_t)(Car.MPU.Roll * 100));
	SendBuff[cnt++] = BYTE1((int16_t)(Car.MPU.Roll * 100));
	
	SendBuff[cnt++] = BYTE2((int16_t)((Car.MPU.Pitch) * 100));
	SendBuff[cnt++] = BYTE1((int16_t)((Car.MPU.Pitch) * 100));
	
	SendBuff[cnt++] = BYTE2((int16_t)(Car.MPU.Yaw * 10));
	SendBuff[cnt++] = BYTE1((int16_t)(Car.MPU.Yaw * 10));

	app_ano_DataUpload(SendBuff,0x01,12);
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
void app_ano_CarDataReport(void)
{	
	/*  �ȴ����յ�������  */
	app_ano_Thread();
	
	/*  �ϴ�����������  */
	debug_SensorDataReport();
	
	/*  �ϴ��������  */
	debug_MotorDataReport();
	
	debug_MPUDataReport();
	
}

/*
*********************************************************************************************************
*                               app_ano_CheckSum           
*
* Description: У�����λ�����յ��������Ƿ���ȷ
*             
* Arguments  : None.
*
* Reutrn     : 1> 0:У��ʧ��
*							 2> 1:�ɹ�
*
* Note(s)    : None.
*********************************************************************************************************
*/
static uint8_t app_ano_CheckSum(void)
{
	uint8_t cnt = 0;
	
	ANO_RX.CheckSum = 0;
	ANO_RX.CheckSum += FRAME_HEADER1;
	ANO_RX.CheckSum += FRAME_HEADER2;
	ANO_RX.CheckSum += ANO_RX.FunCode;
	ANO_RX.CheckSum += ANO_RX.DataLength;
	
	for(cnt = 0; cnt < ANO_RX.DataLength; cnt++)
		ANO_RX.CheckSum += ANO_RX.Data[cnt];
	
	return ANO_RX.CheckSum == ANO_RX.Data[ANO_RX.DataLength];
}

/*
*********************************************************************************************************
*                            app_ano_HandlerAck              
*
* Description: ����ACK����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
static void app_ano_HandlerAck(void)
{
	uint8_t cmd = ANO_RX.Data[0];

	switch(cmd)
	{
		case ACK_PID:	app_ano_PIDReport();break;		/*  ����PID  */
		case ACK_MODE: break;
		case ACK_G_OFFSET: break;
		case ACK_DEST: break;
		case ACK_RESET_PID: break;
		case ACK_RESET_ALL:break;
		default: break;
	}
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
void app_ano_Thread(void)
{
	if(!app_ano_CheckSum()) return;		/*  У������򷵻�  */
	
	if(!ANO_RX.FunCode) return;		/*  û��������Ҫ�����򷵻�  */

	switch(ANO_RX.FunCode)
	{
		case ACK:	app_ano_HandlerAck();break;		
		case VERSION:break;
		case STATUS:break;
		case RCDATA:break;
		case GPSDATA:break;
		case POWER:break;
		case MOTOR:break;
		case SENSOR2:break;
		case STRING:break;
		case COMMAND:break;
		default:break;
	}
	
	ANO_RX.FunCode = 0;
	g_ucDataRecCnt = 0;
	g_ucDatacnt = 0;
}


/*
*********************************************************************************************************
*                             app_ano_ReceiveData             
*
* Description: ͨ��Э��������ݲ�Ԥ����
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void app_ano_ReceiveData(uint8_t data)
{
	switch(g_ucDataRecCnt)
	{
		case 0X00:		/*  ����֡ͷ  */
		{
			if(FRAME_HEADER1 == data) g_ucDataRecCnt++;
			else g_ucDataRecCnt = 0;
		}break;
		case 0X01:			/*  ����֡ͷ  */
		{
			if(FRAME_HEADER2 == data) g_ucDataRecCnt++;
			else g_ucDataRecCnt = 0;
		}break;
		case 0X02:ANO_RX.FunCode = data; g_ucDataRecCnt++; break;		/*  ������  */
		case 0X03:ANO_RX.DataLength = data; g_ucDataRecCnt++;break;		/*  ���ݳ���  */
		
		/*  ����������,��5����Ϊ������һ���ֽڵ����ݲ�����������  */
		default:if(g_ucDataRecCnt++ < ANO_RX.DataLength + 5) ANO_RX.Data[g_ucDatacnt++] = data;break;	
	}
}


/********************************************  END OF FILE  *******************************************/

