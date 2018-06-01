/**
  *******************************************************************************************************
  * File Name: app_ano.c
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-3-2
  * Brief: 程序调试模块,提供与上位机通信的功能
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 建立文件,对应匿名上位机V5.0
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
static ANO_TypeDef ANO_RX;						/*  用于接收的结构体  */
static ANO_TypeDef ANO_TX;						/*  用于发送  */
static uint8_t g_ucDataRecCnt = 0;		/*  用于协议接收数据时计数,整个帧  */
static uint8_t g_ucDatacnt = 0;				/*  用于协议接收数据部分计数  */

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
static void app_ano_Response(uint8_t funcode)
{
	uint8_t SendBuff[8] = {0};
	uint8_t i;
	
	SendBuff[0] = 0XAA;//帧起始
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
//	if(!RecCommand) return;	/*  没有接收到指令  */
//	
//	switch(RecCommand)
//	{
//		case REQUEST_PID: debug_PIDParaReport(); break; /*  上传PID参数  */
//		case ADJ_PID1:debug_PIDDownload();debug_Response(ADJ_PID1); break; /*  接收第PID参数1  */
//		case ADJ_PID2:debug_Response(ADJ_PID2); break; /*  其他PID参数不接收  */
//		case ADJ_PID3:debug_Response(ADJ_PID3); break; 
//		case ADJ_PID4:debug_Response(ADJ_PID4); break; 
//		case ADJ_PID5:debug_Response(ADJ_PID5); break; 
//		case ADJ_PID6:debug_Response(ADJ_PID6); break; 
//		case BOOTMODE:break;               /*  进入IAP下载模式命令  */
//	}
}



/*
*********************************************************************************************************
*                     app_ano_DataUpload                     
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
uint8_t app_ano_DataUpload(uint8_t *buff, uint8_t funCode, uint8_t len)
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
* Description: 填写PID参数并调用app_ano_DataUpload发送
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
* Description: 接收上位机下发的PID参数
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
	//取前后两个8位的数据合并成一个16位的数据，并强制转换成一个float型的数据
	//转换完成后除以相应的传输因子
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
* Description: 车子调试数据上传函数
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : 该函数应该周期性调用,以便及时收到车子的数据
*********************************************************************************************************
*/
void app_ano_CarDataReport(void)
{	
	/*  先处理收到的数据  */
	app_ano_Thread();
	
	/*  上传传感器数据  */
	debug_SensorDataReport();
	
	/*  上传电机数据  */
	debug_MotorDataReport();
	
	debug_MPUDataReport();
	
}

/*
*********************************************************************************************************
*                               app_ano_CheckSum           
*
* Description: 校验从上位机接收到的数据是否正确
*             
* Arguments  : None.
*
* Reutrn     : 1> 0:校验失败
*							 2> 1:成功
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
* Description: 处理ACK命令
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
		case ACK_PID:	app_ano_PIDReport();break;		/*  请求PID  */
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
	if(!app_ano_CheckSum()) return;		/*  校验错误则返回  */
	
	if(!ANO_RX.FunCode) return;		/*  没有命令需要处理则返回  */

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
* Description: 通信协议接收数据并预解析
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
		case 0X00:		/*  接收帧头  */
		{
			if(FRAME_HEADER1 == data) g_ucDataRecCnt++;
			else g_ucDataRecCnt = 0;
		}break;
		case 0X01:			/*  接收帧头  */
		{
			if(FRAME_HEADER2 == data) g_ucDataRecCnt++;
			else g_ucDataRecCnt = 0;
		}break;
		case 0X02:ANO_RX.FunCode = data; g_ucDataRecCnt++; break;		/*  功能字  */
		case 0X03:ANO_RX.DataLength = data; g_ucDataRecCnt++;break;		/*  数据长度  */
		
		/*  接收数据区,加5是因为有另外一个字节的数据不属于数据区  */
		default:if(g_ucDataRecCnt++ < ANO_RX.DataLength + 5) ANO_RX.Data[g_ucDatacnt++] = data;break;	
	}
}


/********************************************  END OF FILE  *******************************************/

