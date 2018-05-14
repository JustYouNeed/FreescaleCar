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

#include "KEA128_gpio.h"


static FGPIO_Type * const GPIOX[] = FGPIO_BASES;


//-------------------------------------------------------------------------------------------------------------------
//  @brief      gpio ��ʼ��
//  @param      ptx_n           ѡ�������   
//  @param      ddr             ���ŷ���    GPI������ GPO�����
//  @param      dat             ����������ݣ��������Ϊ�����
//  @return     void
//  @since      v2.0
//  Sample usage:               gpio_init(A5,GPO,0);   //��ʼ��A5Ϊgpioģʽ������Ϊ���ģʽ����ʼ������͵�ƽ
//-------------------------------------------------------------------------------------------------------------------
void gpio_init(PTX_n ptx_n, GPIO_MOD ddr, uint8 dat)
{
    vuint8 ptx,ptn;
    uint32 temp;
    
    ptx = PTX(ptx_n);//��¼ģ���    //A,B......
    ptn = PTn(ptx_n);//��¼���ź�    //0,1,2,3......
    
    if(dat) dat = 1; //����dat��ֵ;
    
    
    if(GPI == ddr)
    {
        temp = GPIOX[ptx]->PIDR;
        temp &= ~((uint32)1<<ptn);
        GPIOX[ptx]->PIDR = temp;    //ȡ���������
        
        temp = GPIOX[ptx]->PDDR;    
        temp &= ~((uint32)1<<ptn);
        GPIOX[ptx]->PDDR = temp;    //����Ϊ����ģʽ
    }
    else
    {
        temp = GPIOX[ptx]->PIDR;
        temp |= ((uint32)1<<ptn);
        GPIOX[ptx]->PIDR = temp;    //�������
        
        temp = GPIOX[ptx]->PDDR;    
        temp |= ((uint32)1<<ptn);
        GPIOX[ptx]->PDDR = temp;    //����Ϊ���ģʽ
        
        temp = GPIOX[ptx]->PDOR;    
        temp &= ~((uint32)1<<ptn);
        temp |=  (uint32)dat<<ptn;
        GPIOX[ptx]->PDOR = temp;    //���ö˿�״̬
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������ŷ���
//  @param      ptx_n           ѡ�������   
//  @param      ddr             ���ŷ���    GPI������ GPO�����
//  @return     void
//  @since      v2.0
//  Sample usage:               gpio_ddr(A5,GPO);   //����A5Ϊ���ģʽ
//-------------------------------------------------------------------------------------------------------------------
void gpio_ddr(PTX_n ptx_n, GPIO_MOD ddr)
{
    vuint8 ptx,ptn;
    uint32 temp;
    
    ptx = PTX(ptx_n);//��¼ģ���    //A,B......
    ptn = PTn(ptx_n);//��¼���ź�    //0,1,2,3......
    
    if(GPI == ddr)
    {
        temp = GPIOX[ptx]->PIDR;
        temp &= ~((uint32)1<<ptn);
        GPIOX[ptx]->PIDR = temp;    //ȡ���������
        
        temp = GPIOX[ptx]->PDDR;    
        temp &= ~((uint32)1<<ptn);
        GPIOX[ptx]->PDDR = temp;    //����Ϊ����ģʽ
    }
    else
    {
        temp = GPIOX[ptx]->PIDR;
        temp |= ((uint32)1<<ptn);
        GPIOX[ptx]->PIDR = temp;    //�������
        
        temp = GPIOX[ptx]->PDDR;    
        temp |= ((uint32)1<<ptn);
        GPIOX[ptx]->PDDR = temp;    //����Ϊ���ģʽ
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ����״̬
//  @param      ptx_n           ѡ�������   
//  @return     �ܽŵ�״̬      1Ϊ�ߵ�ƽ��0Ϊ�͵�ƽ
//  @since      v2.0
//  Sample usage:               uint8 status = gpio_get(A5);   //��ȡA5Ϊ״̬
//-------------------------------------------------------------------------------------------------------------------
uint8 gpio_get(PTX_n ptx_n)
{
    vuint8 ptx,ptn;
    uint32 temp;
    
    ptx = PTX(ptx_n);//��¼ģ���    //A,B......
    ptn = PTn(ptx_n);//��¼���ź�    //0,1,2,3......
    
    //���ö˿�״̬
    temp = GPIOX[ptx]->PDIR;
    return ( (temp >> ptn) & 0x1 );
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��������״̬
//  @param      ptx_n           ѡ�������   
//  @param      dat             1Ϊ�ߵ�ƽ��0Ϊ�͵�ƽ
//  @return     void
//  @since      v2.0
//  Sample usage:               gpio_set(A5,0);   //����A5Ϊ�͵�ƽ
//-------------------------------------------------------------------------------------------------------------------
void gpio_set(PTX_n ptx_n, uint8 dat)
{
    vuint8 ptx,ptn;
    uint32 temp;
    
    ptx = PTX(ptx_n);//��¼ģ���    //A,B......
    ptn = PTn(ptx_n);//��¼���ź�    //0,1,2,3......
    
    //���ö˿�״̬
    //ʹ����������λ�Ĵ�����������ֶ��������ɱ����ж���������ͬʱ���������
    temp = ((uint32)1<<ptn);
    if(dat) GPIOX[ptx]->PSOR =  temp;
    else    GPIOX[ptx]->PCOR =  temp;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ת����״̬
//  @param      ptx_n           ѡ�������   
//  @return     void
//  @since      v2.0
//  Sample usage:               gpio_turn(A5);   //��תA5
//-------------------------------------------------------------------------------------------------------------------
void gpio_turn(PTX_n ptx_n)
{
    vuint8 ptx,ptn;
    uint32 temp;
    
    ptx = PTX(ptx_n);//��¼ģ���    //A,B......
    ptn = PTn(ptx_n);//��¼���ź�    //0,1,2,3......
    
    //���ö˿�״̬
    temp = ((uint32)1<<ptn);
    GPIOX[ptx]->PTOR = temp;
}

