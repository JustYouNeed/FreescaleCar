/**
  *******************************************************************************************************
  * File Name: bsp_encoder.c
  * Author: Vector
  * Version: V2.1.1
  * Date: 2018-2-28
  * Brief: ���ļ��ṩ���йر������Ļ�����������,���ʼ������ȡ������ֵ��,�����������жϷ�ʽ����
  *******************************************************************************************************
  * History	
  *		1.Author: Vector
	*			Date: 2018-2-28
	*     Mod: �����ļ�
	*
	*		2.Author: Vector
	*			Date: 2018-3-2
	*			Mod: 1.��ԭ�������ı������������ϵ�Car�ṹ����,���ڹ���
	*
	*		3.Author: Vector
	*			Date: 2018-3-16
	*			Mod: 1.��������������ʽ��KBI�жϸ�ΪFTM������
	*					 2.ɾ������������жϻص�����
	*				   3.�޸ĵ���ٶȼ��㷽ʽ,�ɼ���������ֵ���Ϊ��ȡ�����������㷽ʽ
	*
	*		4.Author: Vector
	*			Date: 2018-5-4
	*			Mod: ȥ���ٶȼ��㹦��,ֻ��ȡ��������ֵ
  *
  *******************************************************************************************************
  */	


/*
  *******************************************************************************************************
  *                              INCLUDE FILES
  *******************************************************************************************************
*/
# include "bsp_encoder.h"
# include "bsp_timer.h"
# include "FreescaleCar.h"





void ftm_count_mux(FTMn ftmn)
{
    
    switch(ftmn)
    {
        case ftm0:
        {
            SIM->SCGC |= SIM_SCGC_FTM0_MASK;                //����FTM����ʱ��
            SIM->PINSEL &= ~SIM_PINSEL_FTM0CLKPS_MASK;      //����ⲿʱ������ѡ��


						SIM->PINSEL |= SIM_PINSEL_FTM0CLKPS(1);     //ѡ���ⲿʱ���������� E0

            
        }break;
        
        case ftm1:
        {
            SIM->SCGC |= SIM_SCGC_FTM1_MASK;                //����FTM����ʱ��
            SIM->PINSEL &= ~SIM_PINSEL_FTM1CLKPS_MASK;      //����ⲿʱ������ѡ��

						SIM->PINSEL |= SIM_PINSEL_FTM1CLKPS(2);     //ѡ���ⲿʱ���������� E7
        }break;
        
        
    }
}
static FTM_Type * const FTMX[] = FTM_BASES;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ⲿ������ʼ��  ��ȡ����ֵ�����ڱ��������٣��޷�ʶ����ֻ�ܼ���������ʹ�ô���������ı�����
//  @param      ftmn            ѡ��ģ��
//  @return     void
//  @since      v2.0
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void ftm_count_init(FTMn ftmn)
{
    ftm_count_mux(ftmn);                                    //���Ÿ��� �������� ������Ӧ����ʱ��
    
    FTMX[ftmn]->SC |= FTM_SC_PS(0);	                        //��Ƶϵ��	
    FTMX[ftmn]->SC |= FTM_SC_CLKS(3);                       //ѡ���ⲿʱ����ΪFTM����ʱ��
                
    FTMX[ftmn]->CNT = 0;                                    //���س�ʼ��ֵ
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ����ֵ      ���ڱ��������٣��޷�ʶ����ֻ�ܼ���������ʹ�ô���������ı�����
//  @param      ftmn            ѡ��ģ��
//  @return     uint16          ���ؼ���ֵ
//  @since      v2.0
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
uint16_t ftm_count_get(FTMn ftmn)
{
    return FTMX[ftmn]->CNT ;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������ֵ      ���ڱ��������٣��޷�ʶ����ֻ�ܼ���������ʹ�ô���������ı�����
//  @param      ftmn            ѡ��ģ��
//  @since      v2.0
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void ftm_count_clean(FTMn ftmn)
{
    FTMX[ftmn]->CNT = 0;
}


/*
*********************************************************************************************************
*                              bsp_encoder_SpeedCalc            
*
* Description: �ɱ�������ֵ�������ٶ�
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_encoder_ReadCounter(void)
{	
	/*  ��ȡFTM������ֵ  */
//	Car.Motor.RightEncoder += (uint32_t)FTM0->CNT;
//	Car.Motor.LeftEncoder += (uint32_t)FTM1->CNT;
	
	/*  ��ռ�����  */
	FTM0->CNT = 0;
	FTM1->CNT = 0;
}


/*
*********************************************************************************************************
*                            bsp_encoder_Config              
*
* Description: ��ʼ��������
*             
* Arguments  : None.
*
* Reutrn     : None.
*
* Note(s)    : None.
*********************************************************************************************************
*/
void bsp_encoder_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

//	/*  ��ʼ����߱�����,����FTM0,����E0  */
//	SIM->SCGC |= SIM_SCGC_FTM0_MASK;
//	SIM->PINSEL &= ~SIM_PINSEL_FTM0CLKPS_MASK;
//	SIM->PINSEL |= SIM_PINSEL_FTM0CLKPS(1);
//	FTM0->SC &= ~(3);
//	FTM0->SC |= (3 << 3);
//	FTM0->CNT = 0;
//	
//	/*  ��ʼ����߱�����,����FTM1,����E7  */
//	SIM->SCGC |= SIM_SCGC_FTM1_MASK;
//	SIM->PINSEL &= ~SIM_PINSEL_FTM1CLKPS_MASK;
//	SIM->PINSEL |= SIM_PINSEL_FTM1CLKPS(2);
//	FTM1->SC |= FTM_SC_PS(0);
//	FTM1->SC |= FTM_SC_CLKS(3);  
//	FTM1->CNT = 0;
	
	
	/*  ��ʼ����������������  */
	GPIO_InitStruct.GPIO_HDrv = DISABLE;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Pin = LEFTENCONDER_DIR_PIN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	drv_gpio_Init(&GPIO_InitStruct);
	
	/*  �ұ߱�������������  */
	GPIO_InitStruct.GPIO_Pin = RIGHTENCONDER_DIR_PIN;
	drv_gpio_Init(&GPIO_InitStruct);
//	

	ftm_count_init(ftm0);
	ftm_count_init(ftm1);
	
	drv_gpio_PullCmd(LEFTENCONDER_DIR_PIN, ENABLE);
	drv_gpio_PullCmd(RIGHTENCONDER_DIR_PIN, ENABLE);
}


/********************************************  END OF FILE  *******************************************/


