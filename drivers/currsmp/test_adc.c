/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stm32h7xx_ll_adc.h>
#include <stm32h7xx_ll_gpio.h>
#include <stm32h7xx_ll_bus.h>
#include <stm32h7xx_ll_rcc.h>

/* USER CODE BEGIN 0 */

ErrorStatus LL_GPIO_Init(GPIO_TypeDef *GPIOx, LL_GPIO_InitTypeDef *GPIO_InitStruct)
{
  uint32_t pinpos, currentpin;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
  assert_param(IS_LL_GPIO_PIN(GPIO_InitStruct->Pin));
  assert_param(IS_LL_GPIO_MODE(GPIO_InitStruct->Mode));
  assert_param(IS_LL_GPIO_PULL(GPIO_InitStruct->Pull));

  /* ------------------------- Configure the port pins ---------------- */
  /* Initialize  pinpos on first pin set */
  pinpos = POSITION_VAL(GPIO_InitStruct->Pin);

  /* Configure the port pins */
  while (((GPIO_InitStruct->Pin) >> pinpos) != 0x00000000U)
  {
    /* Get current io position */
    currentpin = (GPIO_InitStruct->Pin) & (0x00000001UL << pinpos);

    if (currentpin != 0x00000000U)
    {

      if ((GPIO_InitStruct->Mode == LL_GPIO_MODE_OUTPUT) || (GPIO_InitStruct->Mode == LL_GPIO_MODE_ALTERNATE))
      {
        /* Check Speed mode parameters */
        assert_param(IS_LL_GPIO_SPEED(GPIO_InitStruct->Speed));

        /* Speed mode configuration */
        LL_GPIO_SetPinSpeed(GPIOx, currentpin, GPIO_InitStruct->Speed);

        /* Check Output mode parameters */
        assert_param(IS_LL_GPIO_OUTPUT_TYPE(GPIO_InitStruct->OutputType));

        /* Output mode configuration*/
        LL_GPIO_SetPinOutputType(GPIOx, GPIO_InitStruct->Pin, GPIO_InitStruct->OutputType);

      }

      /* Pull-up Pull down resistor configuration*/
      LL_GPIO_SetPinPull(GPIOx, currentpin, GPIO_InitStruct->Pull);

      if (GPIO_InitStruct->Mode == LL_GPIO_MODE_ALTERNATE)
      {
        /* Check Alternate parameter */
        assert_param(IS_LL_GPIO_ALTERNATE(GPIO_InitStruct->Alternate));

        /* Alternate function configuration */
        if (currentpin < LL_GPIO_PIN_8)
        {
          LL_GPIO_SetAFPin_0_7(GPIOx, currentpin, GPIO_InitStruct->Alternate);
        }
        else
        {
          LL_GPIO_SetAFPin_8_15(GPIOx, currentpin, GPIO_InitStruct->Alternate);
        }
      }

      /* Pin Mode configuration */
      LL_GPIO_SetPinMode(GPIOx, currentpin, GPIO_InitStruct->Mode);
    }
    pinpos++;
  }

  return (SUCCESS);
}

/* USER CODE END 0 */

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */


  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLL2P);

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0   ------> ADC1_INP16
  PA1   ------> ADC1_INP17
  PA2   ------> ADC1_INP14
  PA6   ------> ADC1_INP3
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 interrupt Init */

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLL2P);

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  /**ADC2 GPIO Configuration
  PA3   ------> ADC2_INP15
  PC5   ------> ADC2_INP8
  PB0   ------> ADC2_INP9
  PB1   ------> ADC2_INP5
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ADC2 interrupt Init */
  NVIC_SetPriority(ADC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(ADC_IRQn);

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
  ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_3RANKS;
  ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
  LL_ADC_INJ_Init(ADC2, &ADC_INJ_InitStruct);
  LL_ADC_INJ_SetQueueMode(ADC2, LL_ADC_INJ_QUEUE_DISABLE);
  LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC2);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC2);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  __IO uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_15);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_15, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_15, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetChannelPreselection(ADC2, LL_ADC_CHANNEL_15);

  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_8);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);

  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_9);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_9, LL_ADC_SINGLE_ENDED);

  /** Configure Injected Channel
  */
  LL_ADC_INJ_SetSequencerRanks(ADC2, LL_ADC_INJ_RANK_3, LL_ADC_CHANNEL_5);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/* USER CODE BEGIN 1 */
#include "stdio.h"
/* USER CODE BEGIN 1 */
void adc1_start(void)
{
  LL_ADC_Disable(ADC1);
  LL_ADC_StartCalibration(ADC1,LL_ADC_CALIB_OFFSET,LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC1));
  LL_ADC_Disable(ADC2);
  LL_ADC_StartCalibration(ADC2,LL_ADC_CALIB_OFFSET,LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC2));

  LL_ADC_Enable(ADC1);
  LL_ADC_Enable(ADC2);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0);
  while (LL_ADC_IsActiveFlag_ADRDY(ADC2) == 0);

  LL_ADC_EnableIT_JEOS(ADC1);
  LL_ADC_DisableIT_JEOS(ADC2);

  LL_ADC_INJ_StartConversion(ADC1);
  printf("adc init finish\r\n");


  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

uint16_t adc_inj_data[6];

void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  LL_GPIO_TogglePin(GPIOE,LL_GPIO_PIN_1);

  if (LL_ADC_IsActiveFlag_JEOS(ADC1)) 
  {    
    LL_ADC_ClearFlag_JEOS(ADC1);
    adc_inj_data[0] = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    adc_inj_data[1] = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
    adc_inj_data[2] = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_3);

  }
  if (LL_ADC_IsActiveFlag_JEOS(ADC2)) {
    LL_ADC_ClearFlag_JEOS(ADC2);

    adc_inj_data[3] = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_1);
    adc_inj_data[4] = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_2);
    adc_inj_data[5] = LL_ADC_INJ_ReadConversionData12(ADC2, LL_ADC_INJ_RANK_3);
  }
  /* USER CODE END ADC_IRQn 0 */

  /* USER CODE BEGIN ADC_IRQn 1 */
  
  /* USER CODE END ADC_IRQn 1 */
}