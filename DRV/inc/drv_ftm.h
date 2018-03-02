# ifndef __DRV_FTM_H
# define __DRV_FTM_H

# include "derivative.h"

# define FTM_OCChannel_0				(uint8_t)0x01
# define FTM_OCChannel_1				(uint8_t)0x02
# define FTM_OCChannel_2				(uint8_t)0x04
# define FTM_OCChannel_3				(uint8_t)0x08
# define FTM_OCChannel_4				(uint8_t)0x10
# define FTM_OCChannel_5				(uint8_t)0x20
# define FTM_OCChannel_6				(uint8_t)0x40
# define FTM_OCChannel_7				(uint8_t)0x80

/*  FTM通道定义  */
/*  规则: bit0-3为寄存器位偏移,bit4-7为寄存器清零量,bit8-11为寄存器标志位值,bit12-14为FTM编号,bit15为寄存器编号  */
/*  FTM2  */
# define PWM_Channel0_C0				((uint16_t)0xa030)
# define PWM_Channel0_H0				((uint16_t)0xa430)
# define PWM_Channel0_F0				((uint16_t)0xa830)
# define PWM_Channel1_C1				((uint16_t)0xa072)
# define PWM_Channel1_H1				((uint16_t)0xa472)
# define PWM_Channel1_F1				((uint16_t)0xa872)
# define PWM_Channel2_C2				((uint16_t)0xa0b4)
# define PWM_Channel2_D0				((uint16_t)0xa4b4)
# define PWM_Channel2_G4				((uint16_t)0xa8b4)
# define PWM_Channel3_C3				((uint16_t)0xa0f6)
# define PWM_Channel3_D1				((uint16_t)0xa4f6)
# define PWM_Channel3_G5				((uint16_t)0xa8f6)
# define PWM_Channel4_B4				((uint16_t)0xa118)
# define PWM_Channel4_G6				((uint16_t)0xa518)
# define PWM_Channel5_B5				((uint16_t)0xa159)
# define PWM_Channel5_G7				((uint16_t)0xa559)

/*  FTM0  */
# define PWM_Channel0_A0				((uint16_t)0x0018)
# define PWM_Channel0_B2				((uint16_t)0x0418)
# define PWM_Channel1_A1				((uint16_t)0x0059)
# define PWM_Channel1_B3				((uint16_t)0x0459)

/*  FTM1  */
# define PWM_Channel0_C4				((uint16_t)0x101a)
# define PWM_Channel0_H2				((uint16_t)0x141a)
# define PWM_Channel1_C5				((uint16_t)0x105b)
# define PWM_Channel1_E7				((uint16_t)0x145b)



typedef struct
{
	uint8_t FTM_Prescaler;		/*  时钟预分频系数  */
	uint16_t FTM_CounterMode;	/*  计数模式  */
	uint32_t FTM_Period;			/*  定时器周期  */
	uint8_t FTM_ClockSource;	/*  时钟源  */
	uint8_t  FTM_RepetitionCounter;
	uint8_t  FTM_IRQCmd;
}FTM_BaseInitTypeDef;

typedef struct
{
	uint8_t  FTM_OCChannel;
	uint16_t FTM_OCMode;
	uint16_t FTM_OutputState;
	uint16_t FTM_Pulse;
	uint16_t FTM_OCPolarity;
	uint16_t FTM_OCIdleState;
}FTM_OCInitTypeDef;

typedef struct
{
	uint16_t PWM_Channel;
	uint8_t PWM_IdleState;
	uint8_t PWM_OutputState;
	uint8_t	PWM_Frequency;
	uint16_t	PWM_Pulse;
}PWM_InitTypeDef;


typedef enum
{
	FTM_Prescaler_1 = 0,
	FTM_Prescaler_2,
	FTM_Prescaler_4,
	FTM_Prescaler_8,
	FTM_Prescaler_16,
	FTM_Prescaler_32,
	FTM_Prescaler_64,
	FTM_Prescaler_128,
}FTM_PrescalerTypeDef;


void drv_ftm_BaseInit(FTM_Type *FTMx, FTM_BaseInitTypeDef *FTM_BaseInitStruct);
void drv_ftm_OCInit(FTM_Type *FTMx, FTM_OCInitTypeDef *FTM_OCInitStruct);
void drv_ftm_PWMInit(PWM_InitTypeDef *PWM_InitStruct);
void drv_ftm_PWMSetDuty(uint16_t FTM_Channel, uint16_t duty);
void drv_ftm_EncoderInit(uint16_t Encoder_Channel);
void drv_ftm_ReadEncoder(uint16_t Encoder_Channel);

# endif

