/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @date    13/05/2015 09:14:38
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f4xx_it.h"
#include "xnucleoihm02a1_interface.h"
#include "example_usart.h"
#include "L6470.h"

// variables from main
extern int motor_speed_x;
extern int motor_speed_y;

extern int dir_x;
extern int dir_y;
extern int read_lx, read_hx, read_ly, read_hy;
/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @addtogroup STM32F4XX_IT
  * @{
  */

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
  * @addtogroup STM32F4XX_IT_Exported_Functions
  * @{
  */

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI Line1 interrupt.
*/
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/**
* @brief This function handles EXTI Line0 interrupt.
*/
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}


//define new handler here WITH CODE IN IT

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
  USART_ITCharManager(&huart2);
}

/**
* @brief This function handles EXTI Line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}


//our new interrupt function - Used to set motor speed to zero (x or y) when limit switch is pressed

void EXTI9_5_IRQHandler(void) {

  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET) // For x motor
  {

    dir_x = 0; // setting the direction to reverse when limit switch hit

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);

    for (int i = 0; i < 10000; i++) {}
  }


  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET) // For x motor
  {

    dir_x = 1; // setting the direction to reverse when limit switch hit

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);

    for (int i = 0; i < 10000; i++) {}
  }


  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET) // For y motor
  {

    dir_y = 1; // setting the direction to reverse when limit switch hit

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);

    for (int i = 0; i < 10000; i++) {}
  }


  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET) // For y motor
  {

    dir_y = 0; // setting the direction to reverse when limit switch hit

    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);

    for (int i = 0; i < 10000; i++) {}
  }



  // else // for y motor
  // {

  //   if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) != RESET) {
  //     dir_y = 0;
  //   }
  //   else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) != RESET) {
  //     dir_y = 1;
  //   }

  //   __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
  //   __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);

  //   for (int i = 0; i < 10000; i++) {}
  // }

}

/**
  * @}
  */ /* End of STM32F4XX_IT_Exported_Functions */

/**
  * @}
  */ /* End of STM32F4XX_IT */

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
