/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		misc
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/



#ifndef _misc_h
#define _misc_h

#include "common.h"

void write_vtor (int);                                              //�����ж�������ĵ�ַ
                                                                    
                                                                    
#define enable_irq(irq)                 NVIC_EnableIRQ(irq)         //ʹ��IRQ
#define disable_irq(irq)                NVIC_DisableIRQ(irq)        //��ֹIRQ
#define set_irq_priority(irq,pri0)      NVIC_SetPriority(irq,pri0)  //�������ȼ������ȼ���Χ0-3
                                                                    
                                                                    
#define EnableInterrupts                __enable_irq()              //ʹ��ȫ���ж�
#define DisableInterrupts               __disable_irq()             //��ֹȫ���ж�




#endif
