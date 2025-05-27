/*
 * DACTest.h
 *
 *  Created on: Jul 26, 2024
 *      Author: Natanael.matias
 */

#ifndef INC_DACSIM_H_
#define INC_DACSIM_H_

#include "main.h"
#include "stm32g4xx_hal_iwdg.h"

/*****************************************************************************/

#define NUMDAC			6
#define TAMANHO_ARRAY 	3586
#define CHANNEL_1		DAC_CHANNEL_1
#define CHANNEL_2		DAC_CHANNEL_2
#define CHANNEL_3		DAC_CHANNEL_1
#define CHANNEL_4		DAC_CHANNEL_1
#define CHANNEL_5		DAC_CHANNEL_2
#define CHANNEL_6		DAC_CHANNEL_1
#define CHANNEL_7		DAC_CHANNEL_2
#define ALIGN12R		DAC_ALIGN_12B_R

#define WDT_PRESCALER	1
#define WDT_RELOAD		999

#define __PWM_PULSE()	(TIM8->CR1 |= TIM_CR1_CEN)
#define __PRINT_RESET()		do{													\
								if(__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))		\
									LOG_Msg("Low Power.\r\n",12);				\
								else if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))	\
									LOG_Msg("Independent Watchdog.\r\n",23);	\
								else if(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))	\
									LOG_Msg("Reset pin.\r\n",12);				\
								else if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))	\
									LOG_Msg("Software.\r\n",11);				\
								else											\
									LOG_Msg("Unknown.\r\n",10);					\
							} while(0)

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp3;
extern OPAMP_HandleTypeDef hopamp4;
extern DAC_HandleTypeDef hdac1;
extern DAC_HandleTypeDef hdac2;
extern DAC_HandleTypeDef hdac3;
extern DAC_HandleTypeDef hdac4;
extern TIM_HandleTypeDef htim8;


typedef struct{
	DAC_HandleTypeDef 	*pDAC;
	uint32_t 			channel;
	const uint32_t		*pDATA;
}ArrayDac_TypeDef;


void FWC_Start(const ArrayDac_TypeDef *pD);
void SysInit(void);
void LoopMain(void);
void WatchdogInit(void);
void LEDsInit(void);
void WatchdogReset(void);
void LOG_Msg(char *pMsg, uint8_t size);
void TIM3_Callback(void);
void EXTI_ButtonIRQHandler(void);
void PrintStatus(void);

#endif /* INC_DACSIM_H_ */
