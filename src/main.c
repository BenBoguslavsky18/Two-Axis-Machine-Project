/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 09/10/2014 11:13:03
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
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

  #include <stdio.h>
  #include "example.h"
  #include "example_usart.h"
  #include "L6470.h"
  #include "stm32f4xx_hal_adc.h"
  #include <stdbool.h>
  
  /**
    * @defgroup   MotionControl
    * @{
    */
  
  /**
    * @addtogroup BSP
    * @{
    */
  
  /**
    * @}
    */ /* End of BSP */
  
  /**
    * @addtogroup MicrosteppingMotor_Example
    * @{
    */
  
  /**
    * @defgroup   ExampleTypes
    * @{
    */
  
  //#define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
  #define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example
  #if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
    #error "Please select an option only!"
  #elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
    #error "Please select an option!"
  #endif
  #if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
    #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
  #endif
  
  /**
    * @}
    */ /* End of ExampleTypes */
  
  /**
    * @brief The FW main module
    */
  
   // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   // GLOBAL VARIABLE INIT
  
  volatile int switch_pin_value;

  volatile int sig_gen_pin_value;
  
  // starting limit switch logic
   volatile int on_switch_x;
   volatile int on_switch_y;
   volatile int motor_speed_x;
   volatile int motor_speed_y;
  
   // direction veriables
   volatile int read_lx, read_hx, read_ly, read_hy;

   volatile int dir_x =1;
   volatile int dir_y =1;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
   void interrupt_function();
  
  
   // function for reading ADC value from separate channels (GPT)
   uint8_t Read_ADC_Channel(uint32_t channel)
   {
       ADC_ChannelConfTypeDef sConfig;
       sConfig.Channel = channel;
       sConfig.Rank = 1;
       HAL_ADC_ConfigChannel(&hadc1, &sConfig);
   
       HAL_ADC_Start(&hadc1);
       HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
       uint8_t adc_value = HAL_ADC_GetValue(&hadc1);
       HAL_ADC_Stop(&hadc1);
   
       return adc_value;
   }
  
  
  
  int main(void)
  {
    /* NUCLEO board initialization */
    NUCLEO_Board_Init();
    
    /* X-NUCLEO-IHM02A1 initialization */
    BSP_Init();
  
   // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Configure 4 pins on interrupt line 9-5 for motor control
    GPIO_InitTypeDef GPIO_InitStruct_Input_Interrupt_LX; // for low X limit  
  
    GPIO_InitStruct_Input_Interrupt_LX.Pin = GPIO_PIN_9; //set the pin
    GPIO_InitStruct_Input_Interrupt_LX.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct_Input_Interrupt_LX.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct_Input_Interrupt_LX.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_Input_Interrupt_LX);
    
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  
  
    GPIO_InitTypeDef GPIO_InitStruct_Input_Interrupt_HX; // for high X limit    
  
    GPIO_InitStruct_Input_Interrupt_HX.Pin = GPIO_PIN_7; //set the pin
    GPIO_InitStruct_Input_Interrupt_HX.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct_Input_Interrupt_HX.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct_Input_Interrupt_HX.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_Input_Interrupt_HX);
  
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    
  
    GPIO_InitTypeDef GPIO_InitStruct_Input_Interrupt_LY; // for low Y limit  
  
    GPIO_InitStruct_Input_Interrupt_LY.Pin = GPIO_PIN_6; //set the pin
    GPIO_InitStruct_Input_Interrupt_LY.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct_Input_Interrupt_LY.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct_Input_Interrupt_LY.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_Input_Interrupt_LY);
  
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  
  
    GPIO_InitTypeDef GPIO_InitStruct_Input_Interrupt_HY;  // for high Y limit  
  
    GPIO_InitStruct_Input_Interrupt_HY.Pin = GPIO_PIN_8; //set the pin
    GPIO_InitStruct_Input_Interrupt_HY.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct_Input_Interrupt_HY.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct_Input_Interrupt_HY.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_Input_Interrupt_HY);
  
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    //Configuring ADC, potentiometer pin
    GPIO_InitTypeDef GPIO_InitStruct_Pot;  
  
    GPIO_InitStruct_Pot.Pin = GPIO_PIN_0; //set the pin
    GPIO_InitStruct_Pot.Mode = GPIO_MODE_ANALOG; 
    GPIO_InitStruct_Pot.Pull = GPIO_NOPULL;
    GPIO_InitStruct_Pot.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_Pot);
  
    volatile uint8_t adc1; //int to store the ADC value
  
    //Configuring 2nd ADC, potentiometer pin
    GPIO_InitTypeDef GPIO_InitStruct_Pot2;  

    GPIO_InitStruct_Pot2.Pin = GPIO_PIN_1; //set the pin
    GPIO_InitStruct_Pot2.Mode = GPIO_MODE_ANALOG; 
    GPIO_InitStruct_Pot2.Pull = GPIO_NOPULL;
    GPIO_InitStruct_Pot2.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_Pot2);
  
    volatile uint8_t adc2; //int to store the ADC value

    MX_ADC1_Init();  //initialize ADC
    char buffer[50]; //buffer to print the current ADC value to console 

  
    // ==================== OTHER SETUP -- DO NOT TOUCH ==================== //
    #ifdef NUCLEO_USE_USART
      /* Transmit the initial message to the PC via UART */
      USART_TxWelcomeMessage();
        USART_Transmit(&huart2, " X-CUBE-SPN2 v1.0.0\n\r");
    #endif
      
    #if defined (MICROSTEPPING_MOTOR_EXAMPLE)
      /* Perform a batch commands for X-NUCLEO-IHM02A1 */
      MicrosteppingMotor_Example_01();
      
      /* Infinite loop */
      while (1);
    #elif defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)
      /* Fill the L6470_DaisyChainMnemonic structure */
      Fill_L6470_DaisyChainMnemonic();
      
      /*Initialize the motor parameters */
      Motor_Param_Reg_Init();
    // ==================== END OTHER SETUP -- DO NOT TOUCH ==================== //
   
      
    // Logic for starting machine, yes or no on limit switch
    read_lx = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
    read_hx = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);

    read_ly = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
    read_hy = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);

    // checking direction x
    if (read_hx == 1) {
      dir_x = 1;
    }
    else {
      dir_x = 0;
    }

    // checking direction y
    if (read_ly == 1) {
      dir_y = 0;
    }
    else {
      dir_y = 1;
    }


  
  /* Infinite loop */
  while (1)
  {
    // running both motors
    L6470_Run(0, dir_x, motor_speed_x); 
    L6470_Run(1, dir_y, motor_speed_y);
  
  // reading the adc values
    adc1 = Read_ADC_Channel(ADC_CHANNEL_8);
    adc2 = Read_ADC_Channel(ADC_CHANNEL_1);
  
    // printing the directions and motor speeds
    sprintf(buffer, "%u --- %u -------------- %d --- %d\r\n", dir_x, dir_y, motor_speed_x, motor_speed_y);
    USART_Transmit(&huart2, (uint8_t*)buffer);

    // getting the speed based on the adc
    if (adc1 > 50) {
      motor_speed_x = (adc1/5)* 200;
    }
    else {
      motor_speed_x = 0;
    }

    if (adc2 > 50) {
      motor_speed_y = (adc2/5) * 500;
    }
    else {
      motor_speed_y = 0;
    }
  
  
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Previous lab work, can be ignored
  
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // LAB 1 - READING OUR PIN (Limit Switch) AND TURNING ON LED and FOR LAB 2 with TIGHT POLLING
  
    // switch_pin_value = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, switch_pin_value);
  
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // LAB 2 - INTERRUPTS
  
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }
    
  #endif
  }
  
  
  
  #ifdef USE_FULL_ASSERT
  
  /**
     * @brief Reports the name of the source file and the source line number
     * where the assert_param error has occurred.
     * @param file: pointer to the source file name
     * @param line: assert_param error line source number
     * @retval None
     */
  void assert_failed(uint8_t* file, uint32_t line)
  {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
  
  }
  
  #endif
  
  /**
    * @}
    */ /* End of MicrosteppingMotor_Example */
  
  /**
    * @}
    */ /* End of MotionControl */
  
  /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
  