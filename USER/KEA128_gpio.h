/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		KEA128_gpio
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/


#ifndef _KEA128_gpio_h
#define _KEA128_gpio_h



//#include "misc.h"
//#include "common.h"
#include "SKEAZ1284.h"
//#include "KEA128_port_cfg.h"


//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO            8λ�˿ڵ�ַ����
//  @param      x               xȡֵ��Χ 0����Ӧϵͳ���ϵ�A0-A7�˸����� 1����Ӧϵͳ���ϵ�B0-B7�˸����� 
//                                        2����Ӧϵͳ���ϵ�C0-C7�˸����� 3����Ӧϵͳ���ϵ�D0-D7�˸�����
//  @since      v2.0
//  Sample usage:               A_PDOR8(2) = 0XFF;   //C0-C7�˸���������ߵ�ƽ
//-------------------------------------------------------------------------------------------------------------------
#define A_PDOR8(x)    *(uint8 *)(FGPIOA_BASE+0x00+x) //����A�˿ڵ�8λ�����ַ
#define A_PSOR8(x)    *(uint8 *)(FGPIOA_BASE+0x04+x) //����A�˿ڵ�8λ��λ��ַ
#define A_PCOR8(x)    *(uint8 *)(FGPIOA_BASE+0x08+x) //����A�˿ڵ�8λ�����ַ
#define A_PTOR8(x)    *(uint8 *)(FGPIOA_BASE+0x0C+x) //����A�˿ڵ�8λȡ����ַ
#define A_PDIR8(x)    *(uint8 *)(FGPIOA_BASE+0x10+x) //����A�˿ڵ�8λ�����ַ
#define A_PDDR8(x)    *(uint8 *)(FGPIOA_BASE+0x14+x) //����A�˿ڵ�8λ�����ַ
#define A_PIDR8(x)    *(uint8 *)(FGPIOA_BASE+0x18+x) //����A�˿ڵ�8λ���õ�ַ                                                     
//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO            8λ�˿ڵ�ַ����
//  @param      x               xȡֵ��Χ 0����Ӧϵͳ���ϵ�E0-E7�˸����� 1����Ӧϵͳ���ϵ�F0-F7�˸����� 
//                                        2����Ӧϵͳ���ϵ�G0-G7�˸����� 3����Ӧϵͳ���ϵ�H0-H7�˸�����
//  @since      v2.0
//  Sample usage:               B_PDOR8(3) = 0XFF;   //H0-H7�˸���������ߵ�ƽ
//-------------------------------------------------------------------------------------------------------------------                                          
#define B_PDOR8(x)    *(uint8 *)(FGPIOB_BASE+0x00+x) //����B�˿ڵ�8λ�����ַ
#define B_PSOR8(x)    *(uint8 *)(FGPIOB_BASE+0x04+x) //����B�˿ڵ�8λ��λ��ַ
#define B_PCOR8(x)    *(uint8 *)(FGPIOB_BASE+0x08+x) //����B�˿ڵ�8λ�����ַ
#define B_PTOR8(x)    *(uint8 *)(FGPIOB_BASE+0x0C+x) //����B�˿ڵ�8λȡ����ַ
#define B_PDIR8(x)    *(uint8 *)(FGPIOB_BASE+0x10+x) //����B�˿ڵ�8λ�����ַ
#define B_PDDR8(x)    *(uint8 *)(FGPIOB_BASE+0x14+x) //����B�˿ڵ�8λ�����ַ
#define B_PIDR8(x)    *(uint8 *)(FGPIOB_BASE+0x18+x) //����B�˿ڵ�8λ���õ�ַ                              
//-------------------------------------------------------------------------------------------------------------------
//  @brief      GPIO            8λ�˿ڵ�ַ����
//  @param      x               xȡֵ��Χ 0����Ӧϵͳ���ϵ�I0-I7�˸�����
//  @since      v2.0
//  Sample usage:               B_PDOR8(0) = 0XFF;   //I0-I7�˸���������ߵ�ƽ
//-------------------------------------------------------------------------------------------------------------------                              
#define C_PDOR8(x)    *(uint8 *)(FGPIOC_BASE+0x00+x) //����C�˿ڵ�8λ�����ַ
#define C_PSOR8(x)    *(uint8 *)(FGPIOC_BASE+0x04+x) //����C�˿ڵ�8λ��λ��ַ
#define C_PCOR8(x)    *(uint8 *)(FGPIOC_BASE+0x08+x) //����C�˿ڵ�8λ�����ַ
#define C_PTOR8(x)    *(uint8 *)(FGPIOC_BASE+0x0C+x) //����C�˿ڵ�8λȡ����ַ
#define C_PDIR8(x)    *(uint8 *)(FGPIOC_BASE+0x10+x) //����C�˿ڵ�8λ�����ַ
#define C_PDDR8(x)    *(uint8 *)(FGPIOC_BASE+0x14+x) //����C�˿ڵ�8λ�����ַ
#define C_PIDR8(x)    *(uint8 *)(FGPIOC_BASE+0x18+x) //����C�˿ڵ�8λ���õ�ַ



void    gpio_init(PTX_n ptx_n, GPIO_MOD ddr, uint8 dat);//��ʼ��gpio
void    gpio_ddr(PTX_n ptx_n, GPIO_MOD ddr);            //�������ŷ���
uint8   gpio_get(PTX_n ptx_n);                          //��ȡ����״̬
void    gpio_set(PTX_n ptx_n, uint8 dat);               //��������״̬
void    gpio_turn(PTX_n ptx_n);                         //��ת����״̬


#endif
