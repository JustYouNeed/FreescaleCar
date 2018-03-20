# ifndef __BSP_ENCODER_H
# define __BSP_ENCODER_H

# include "derivative.h"



# define LEFTENCONDER_CHANNEL		KBI_Channel_B3
# define LEFTENCONDER_DIR_PIN		GPIO_Pin_B2

# define RIGHTENCONDER_CHANNEL		KBI_Channel_C5
# define RIGHTENCONDER_DIR_PIN		GPIO_Pin_E6

# define READ_DIR(pin)		drv_gpio_ReadPin(pin)

void bsp_encoder_Config(void);

void bsp_encoder_ReadCounter(void);

# endif
