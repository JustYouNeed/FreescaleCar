/**
  *******************************************************************************************************
  * File Name: drv_rtc.h
  * Author: Vector
  * Version: V1.0.0
  * Date: 2018-6-19
  * Brief: ���ļ��ṩ��KEA128оƬ��RTC����Ĵ�����װ
  *******************************************************************************************************
  * History
  *		1.Author: Vector
	*			Date: 2018-6-29
	*			Mod: �����ļ�
  *
  *******************************************************************************************************
  */	
# ifndef __DRV_RTC_H
# define __DRV_RTC_H

/*  RCTʱ��Դ  */
# define RTC_ClockSource_ExClock				(uint8_t)(0x00)
# define RTC_ClockSource_LPOClock				(uint8_t)(0x01)
# define RTC_ClockSource_ICSClock				(uint8_t)(0x02)
# define RTC_ClockSource_BusClock				(uint8_t)(0x03)


/*  ʱ��Ԥ��Ƶ  */
# define RTC_Prescaler_None					(uint8_t)(0x00)
# define RTC_Prescaler_Div1					(uint8_t)(0x01)
# define RTC_Prescaler_Div2					(uint8_t)(0x02)
# define RTC_Prescaler_Div4					(uint8_t)(0x03)
# define RTC_Prescaler_Div8					(uint8_t)(0x04)
# define RTC_Prescaler_Div16				(uint8_t)(0x05)
# define RTC_Prescaler_Div32				(uint8_t)(0x06)
# define RTC_Prescaler_Div64				(uint8_t)(0x07)


# define RTC_IT_RTIF				(uint32_t)(0x00000080)


void drv_rtc_ITConfig(uint32_t RTC_IT, FunctionalState NewState);


# endif

/********************************************  END OF FILE  *******************************************/

