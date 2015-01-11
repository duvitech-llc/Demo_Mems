/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32F0xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h> // strlen
#include <stdio.h>  // sprintf
#include <math.h>   // trunc

/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
char dataOut[256];

volatile AxesRaw_TypeDef ACC_Value;
volatile AxesRaw_TypeDef GYR_Value;
volatile AxesRaw_TypeDef MAG_Value;
volatile float PRESSURE_Value;
volatile float HUMIDITY_Value;
volatile float TEMPERATURE_Value;
volatile float TEMPERATURE2_Value;

/* Private functions ---------------------------------------------------------*/
static void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec);
static float celsius2fahrenheit(float c);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* This sample code shows how to use GPIO HAL API to toggle LED2 IOs
    in an infinite loop. */

  /* STM32F0xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 48 MHz */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  printf("\n\r\n\rDuvitech 2015\n\r");
  printf("George Vigelette\n\r");
  printf("gvigelet@duvitech.com\n\r\n\r");

  printf("initializing temperature sensor...\n\r");
  if(BSP_HUM_TEMP_Init() != HUM_TEMP_OK ||
		  BSP_HUM_TEMP_CheckID() != HUM_TEMP_OK)
  {
	  printf("problem with temperature sensor\n\r");
  }

  printf("initializing pressure sensor...\n\r");
  if(BSP_PRESSURE_Init() != PRESSURE_OK ||
		  BSP_PRESSURE_CheckID() != PRESSURE_OK)
  {
	  printf("problem with pressure sensor\n\r");
  }

  printf("initializing magnetometer sensor...\n\r");
  if(BSP_MAGNETO_Init() != MAGNETO_OK ||
		  BSP_MAGNETO_Check_M_ID() != MAGNETO_OK)
  {
	  printf("problem with magnetometer sensor\n\r");
  }

  printf("initializing imu sensor...\n\r");
    if(BSP_IMU_6AXES_Init() != IMU_6AXES_OK ||
    		BSP_IMU_6AXES_Check_XG_ID() != IMU_6AXES_OK)
    {
  	  printf("problem with imu sensor\n\r");
    }

  printf("Running...\n\r");
  while (1)
  {
    HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    /* Insert delay 100 ms */
    HAL_Delay(100);
  }
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
	    int32_t d1, d2, d3, d4, d5, d6, d7, d8;
	    int32_t data[3];
	    int32_t gdata[6];

	    if(BSP_HUM_TEMP_isInitialized()) {
	        BSP_HUM_TEMP_GetHumidity((float *)&HUMIDITY_Value);
	        BSP_HUM_TEMP_GetTemperature((float *)&TEMPERATURE_Value);
	        floatToInt(HUMIDITY_Value, &d1, &d2, 2);
	        float tempF = celsius2fahrenheit(TEMPERATURE_Value);
	        floatToInt(tempF, &d3, &d4, 2);
            sprintf(dataOut, "HUM: %d.%02d rH     TEMP: %d.%02d (f)\n\r", (int)d1, (int)d2, (int)d3, (int)d4);
	        printf(dataOut);
	    }

	    if(BSP_PRESSURE_isInitialized())
	    {
	        BSP_PRESSURE_GetPressure((float *)&PRESSURE_Value);
	        BSP_PRESSURE_GetTemperature((float *)&TEMPERATURE2_Value);
	        floatToInt(PRESSURE_Value, &d5, &d6, 2);
	        float tempF2 = celsius2fahrenheit(TEMPERATURE2_Value);
	        floatToInt(tempF2, &d7, &d8, 2);
            sprintf(dataOut, "PRESS: %d.%02d hPa     TEMP: %d.%02d (f)\n\r", (int)d5, (int)d6, (int)d7, (int)d8);
	        printf(dataOut);
	    }

	    if(BSP_MAGNETO_isInitialized())
	    {
	        BSP_MAGNETO_M_GetAxesRaw((AxesRaw_TypeDef *)&MAG_Value);
            data[0] = MAG_Value.AXIS_X;
            data[1] = MAG_Value.AXIS_Y;
            data[2] = MAG_Value.AXIS_Z;

            sprintf(dataOut, "MAG_X: %d, MAG_Y: %d, MAG_Z: %d\n\r", (int)data[0], (int)data[1], (int)data[2]);
	        printf(dataOut);
	    }


	    if(BSP_IMU_6AXES_isInitialized()) {
	        BSP_IMU_6AXES_X_GetAxesRaw((AxesRaw_TypeDef *)&ACC_Value);
	        BSP_IMU_6AXES_G_GetAxesRaw((AxesRaw_TypeDef *)&GYR_Value);
            gdata[0] = ACC_Value.AXIS_X;
            gdata[1] = ACC_Value.AXIS_Y;
            gdata[2] = ACC_Value.AXIS_Z;
            gdata[3] = GYR_Value.AXIS_X;
            gdata[4] = GYR_Value.AXIS_Y;
            gdata[5] = GYR_Value.AXIS_Z;

            sprintf(dataOut, "ACC_X: %d, ACC_Y: %d, ACC_Z: %d\n\r", (int)gdata[0], (int)gdata[1], (int)gdata[2]);
	        printf(dataOut);
            sprintf(dataOut, "GYR_X: %d, GYR_Y: %d, GYR_Z: %d\n\r", (int)gdata[3], (int)gdata[4], (int)gdata[5]);
	        printf(dataOut);

	    }
  }
}

/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_int the pointer to the integer part as output
 * @param  out_dec the pointer to the decimal part as output
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
static void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec)
{
    *out_int = (int32_t)in;
    in = in - (float)(*out_int);
    *out_dec = (int32_t)trunc(in*pow(10,dec_prec));
}

static float celsius2fahrenheit(float c)
{
  return 32 + (c * (180.0 / 100.0));
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __SYSCFG_CLK_ENABLE();

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Enable and set EXTI line 4_15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
