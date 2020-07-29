/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t comm_=0;
uint8_t bit1=0;
uint8_t bit2=0;
uint32_t LS = TIM_CHANNEL_1 ;
uint32_t HU = TIM_CHANNEL_2 ;
uint32_t HV = TIM_CHANNEL_3 ;
uint32_t HW = TIM_CHANNEL_4 ;
uint32_t LU = GPIO_PIN_1 ;
uint32_t LV = GPIO_PIN_2 ;
uint32_t LW = GPIO_PIN_3 ;

/*uint32_t LU = TIM_CHANNEL_2 ;
uint32_t LV = TIM_CHANNEL_3 ;
uint32_t LW = TIM_CHANNEL_4 ;*/
volatile   uint32_t tick;
volatile   uint32_t prev_tick;
volatile   uint32_t interval;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
    htim1.Instance->CCR1 = (0);
	  htim1.Instance->CCR2 = (0);
	  htim1.Instance->CCR3 = (0);
    htim1.Instance->CCR4 = (0);
	  prev_tick = __HAL_TIM_GET_COUNTER(&htim3);
	
	    while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4))
		{ 
		tick = __HAL_TIM_GET_COUNTER(&htim3);	
		//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		interval = tick-prev_tick;
		if(interval>16)
		{
			while(1)
			{
			htim1.Instance->CCR1 = (0);
	    htim1.Instance->CCR2 = (0);
	    htim1.Instance->CCR3 = (0);
      htim1.Instance->CCR4 = (0);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);	
			}
		}
		}
	  /*for(int h=0;h<1000;h++)
	  {}*/
		
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	
	

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
	//Stop all PWM
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	HAL_TIM_PWM_Stop(&htim1,HU);
	HAL_TIM_PWM_Stop(&htim1,HV);
	HAL_TIM_PWM_Stop(&htim1,HW);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
	//Get hall reading
	comm_= ((GPIOB->IDR)&0xF000)>>13;
	
	//comm Swap P0 P1
	/* bit1 = (comm_&1);  
   bit2 = (comm_>>2)&1;  
   bit1= (bit1 ^ bit2); 
   bit1=(bit1)|(bit1<<2);	 
   comm_ = bit1 ^ comm_;  */
  
	 //commutatuion switch
	switch(comm_)
	{
		case 1:
		{
		HAL_TIM_PWM_Start(&htim1,HV);
		//HAL_TIM_PWM_Start(&htim2,LW);
		HAL_GPIO_WritePin(GPIOA,LW,GPIO_PIN_SET);
		break;
		}
		case 3:
		 {
		HAL_TIM_PWM_Start(&htim1,HU);
		//HAL_TIM_PWM_Start(&htim2,LW);
	  HAL_GPIO_WritePin(GPIOA,LW,GPIO_PIN_SET);
		break;
		}
	  case 2:
		 {
		HAL_TIM_PWM_Start(&htim1,HU);
		//HAL_TIM_PWM_Start(&htim2,LV);
		HAL_GPIO_WritePin(GPIOA,LV,GPIO_PIN_SET);
		break;
		}
	  case 6:
		 {
		HAL_TIM_PWM_Start(&htim1,HW);
		//HAL_TIM_PWM_Start(&htim2,LV);
		HAL_GPIO_WritePin(GPIOA,LV,GPIO_PIN_SET);
		break;
		}
	  case 4:
		 {
		HAL_TIM_PWM_Start(&htim1,HW);
		//HAL_TIM_PWM_Start(&htim2,LU);
		HAL_GPIO_WritePin(GPIOA,LU,GPIO_PIN_SET);
		break;
		}
		case 5:
		 {
		HAL_TIM_PWM_Start(&htim1,HV);
		//HAL_TIM_PWM_Start(&htim2,LU);
		HAL_GPIO_WritePin(GPIOA,LU,GPIO_PIN_SET);
		break;
		}
	  default:
		{
	  HAL_TIM_PWM_Stop(&htim1,HU);
	  HAL_TIM_PWM_Stop(&htim1,HV);
	  HAL_TIM_PWM_Stop(&htim1,HW);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
	  /*HAL_TIM_PWM_Stop(&htim2,LU);
	  HAL_TIM_PWM_Stop(&htim2,LV);
	  HAL_TIM_PWM_Stop(&htim2,LW);	*/
		}	
	}	

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
