/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		KEA128_adc
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/


#ifndef _KEA128_adc_h
#define _KEA128_adc_h

#include "misc.h"
#include "common.h"
#include "SKEAZ1284.h"
#include "KEA128_port_cfg.h"

typedef enum
{
    ADC0_SE0=0,  //A0
    ADC0_SE1,    //A1
    ADC0_SE2,    //A6
    ADC0_SE3,    //A7
    ADC0_SE4,    //B0
    ADC0_SE5,    //B1
    ADC0_SE6,    //B2
    ADC0_SE7,    //B3
    ADC0_SE8,    //C0
    ADC0_SE9,    //C1
    ADC0_SE10,   //C2
    ADC0_SE11,   //C3
    ADC0_SE12,   //F4
    ADC0_SE13,   //F5
    ADC0_SE14,   //F6
    ADC0_SE15    //F7
} ADCn_Ch;



//����λ��
typedef enum ADC_nbit
{
    ADC_8bit   = 0x00,
    ADC_10bit  = 0x01,
    ADC_12bit  = 0x02
} ADC_nbit;


void adc_init(ADCn_Ch adcn_ch);
uint16 adc_once(ADCn_Ch adcn_ch, ADC_nbit bit);
void adc_stop(void);




#endif
