/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define beep_Pin GPIO_PIN_0
#define beep_GPIO_Port GPIOA
#define head_mostleft_Pin GPIO_PIN_1
#define head_mostleft_GPIO_Port GPIOA
#define head_left_Pin GPIO_PIN_2
#define head_left_GPIO_Port GPIOA
#define head_right_Pin GPIO_PIN_6
#define head_right_GPIO_Port GPIOA
#define head_mostright_Pin GPIO_PIN_7
#define head_mostright_GPIO_Port GPIOA
#define oled_d0_Pin GPIO_PIN_7
#define oled_d0_GPIO_Port GPIOE
#define oled_d1_Pin GPIO_PIN_8
#define oled_d1_GPIO_Port GPIOE
#define oled_res_Pin GPIO_PIN_9
#define oled_res_GPIO_Port GPIOE
#define oled_ds_Pin GPIO_PIN_10
#define oled_ds_GPIO_Port GPIOE
#define oled_cs_Pin GPIO_PIN_11
#define oled_cs_GPIO_Port GPIOE
#define IR_Pin GPIO_PIN_12
#define IR_GPIO_Port GPIOB
#define head2_Pin GPIO_PIN_13
#define head2_GPIO_Port GPIOB
#define head1_Pin GPIO_PIN_14
#define head1_GPIO_Port GPIOB
#define right_sensor_Pin GPIO_PIN_15
#define right_sensor_GPIO_Port GPIOB
#define collect_Pin GPIO_PIN_8
#define collect_GPIO_Port GPIOD
#define DI5_Pin GPIO_PIN_9
#define DI5_GPIO_Port GPIOD
#define key_save_Pin GPIO_PIN_6
#define key_save_GPIO_Port GPIOD
#define key_run_Pin GPIO_PIN_7
#define key_run_GPIO_Port GPIOD
#define key_sub_Pin GPIO_PIN_4
#define key_sub_GPIO_Port GPIOB
#define key_add_Pin GPIO_PIN_5
#define key_add_GPIO_Port GPIOB
#define key_page_up_Pin GPIO_PIN_6
#define key_page_up_GPIO_Port GPIOB
#define key_up_Pin GPIO_PIN_7
#define key_up_GPIO_Port GPIOB
#define key_down_Pin GPIO_PIN_8
#define key_down_GPIO_Port GPIOB
#define key_page_down_Pin GPIO_PIN_9
#define key_page_down_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
