/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define HID_INBUF_SIZE 64
#define HID_OUTBUF_SIZE 64

#define PO2_Pin GPIO_PIN_2
#define PO2_GPIO_Port GPIOE
#define PO3_Pin GPIO_PIN_3
#define PO3_GPIO_Port GPIOE
#define PO4_Pin GPIO_PIN_4
#define PO4_GPIO_Port GPIOE
#define PO5_Pin GPIO_PIN_5
#define PO5_GPIO_Port GPIOE
#define PO6_Pin GPIO_PIN_6
#define PO6_GPIO_Port GPIOE
#define INP_SEL0_Pin GPIO_PIN_4
#define INP_SEL0_GPIO_Port GPIOC
#define INP_SEL1_Pin GPIO_PIN_5
#define INP_SEL1_GPIO_Port GPIOC
#define OUT_SEL0_Pin GPIO_PIN_0
#define OUT_SEL0_GPIO_Port GPIOB
#define OUT_SEL1_Pin GPIO_PIN_1
#define OUT_SEL1_GPIO_Port GPIOB
#define OUT_SEL2_Pin GPIO_PIN_2
#define OUT_SEL2_GPIO_Port GPIOB
#define PO7_Pin GPIO_PIN_7
#define PO7_GPIO_Port GPIOE
#define PO8_Pin GPIO_PIN_8
#define PO8_GPIO_Port GPIOE
#define PO9_Pin GPIO_PIN_9
#define PO9_GPIO_Port GPIOE
#define PO10_Pin GPIO_PIN_10
#define PO10_GPIO_Port GPIOE
#define PO11_Pin GPIO_PIN_11
#define PO11_GPIO_Port GPIOE
#define PO12_Pin GPIO_PIN_12
#define PO12_GPIO_Port GPIOE
#define PO13_Pin GPIO_PIN_13
#define PO13_GPIO_Port GPIOE
#define PO14_Pin GPIO_PIN_14
#define PO14_GPIO_Port GPIOE
#define PO15_Pin GPIO_PIN_15
#define PO15_GPIO_Port GPIOE
#define PI8_Pin GPIO_PIN_8
#define PI8_GPIO_Port GPIOD
#define PI9_Pin GPIO_PIN_9
#define PI9_GPIO_Port GPIOD
#define PI10_Pin GPIO_PIN_10
#define PI10_GPIO_Port GPIOD
#define PI11_Pin GPIO_PIN_11
#define PI11_GPIO_Port GPIOD
#define PI12_Pin GPIO_PIN_12
#define PI12_GPIO_Port GPIOD
#define PI13_Pin GPIO_PIN_13
#define PI13_GPIO_Port GPIOD
#define PI14_Pin GPIO_PIN_14
#define PI14_GPIO_Port GPIOD
#define PI15_Pin GPIO_PIN_15
#define PI15_GPIO_Port GPIOD
#define INP_SEL2_Pin GPIO_PIN_6
#define INP_SEL2_GPIO_Port GPIOC
#define PI0_Pin GPIO_PIN_0
#define PI0_GPIO_Port GPIOD
#define PI1_Pin GPIO_PIN_1
#define PI1_GPIO_Port GPIOD
#define PI2_Pin GPIO_PIN_2
#define PI2_GPIO_Port GPIOD
#define PI3_Pin GPIO_PIN_3
#define PI3_GPIO_Port GPIOD
#define PI4_Pin GPIO_PIN_4
#define PI4_GPIO_Port GPIOD
#define PI5_Pin GPIO_PIN_5
#define PI5_GPIO_Port GPIOD
#define PI6_Pin GPIO_PIN_6
#define PI6_GPIO_Port GPIOD
#define PI7_Pin GPIO_PIN_7
#define PI7_GPIO_Port GPIOD
#define SPI1_SCB_Pin GPIO_PIN_4
#define SPI1_SCB_GPIO_Port GPIOB
#define PO0_Pin GPIO_PIN_0
#define PO0_GPIO_Port GPIOE
#define PO1_Pin GPIO_PIN_1
#define PO1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define ADC_BUF_SIZE 10
#define INPPin_MAX_BOARD 4
#define INPPin_PIN_PER_CHIP 8
#define INPPin_CHIP_PER_BOARD 2
#define INPPin_PIN_PER_BOARD INPPin_PIN_PER_CHIP*INPPin_CHIP_PER_BOARD

#define OUTPin_MAX_BOARD 4
#define OUTPin_CHIP_PER_BOARD 2
#define OUTPin_MAX_CHIPS OUTPin_MAX_BOARD*OUTPin_CHIP_PER_BOARD

#define USB_TX_Buffer_id (USB_TX_Buffer[0])
#define USB_TX_Buffer_data (&USB_TX_Buffer[1])
#define USB_TX_Buffer_byte(i) (USB_TX_Buffer[1+i])
#define USB_TX_Buffer_short(i) (*((uint16_t*)&USB_TX_Buffer[1+i*2]))
#define USB_TX_Buffer_word(i) (*((uint32_t*)&USB_TX_Buffer[1+i*4]))

#ifdef INP_SEL0_Pin
#define INP_SEL0 INP_SEL0_GPIO_Port,INP_SEL0_Pin
#endif
#ifdef INP_SEL1_Pin
#define INP_SEL1 INP_SEL1_GPIO_Port,INP_SEL1_Pin
#endif
#ifdef INP_SEL2_Pin
#define INP_SEL2 INP_SEL2_GPIO_Port,INP_SEL2_Pin
#endif

#ifdef OUT_SEL0_Pin
#define OUT_SEL0 OUT_SEL0_GPIO_Port,OUT_SEL0_Pin
#endif
#ifdef OUT_SEL1_Pin
#define OUT_SEL1 OUT_SEL1_GPIO_Port,OUT_SEL1_Pin
#endif
#ifdef OUT_SEL2_Pin
#define OUT_SEL2 OUT_SEL2_GPIO_Port,OUT_SEL2_Pin
#endif
#ifdef SPI1_SCB_Pin
#define SPI1_SCB SPI1_SCB_GPIO_Port,SPI1_SCB_Pin
#endif



typedef union {
  uint16_t data;
  struct {
    uint16_t out:8;
    uint16_t diag:8;
  };
  struct {
    uint16_t out1:1;
    uint16_t out2:1;
    uint16_t out3:1;
    uint16_t out4:1;
    uint16_t out5:1;
    uint16_t out6:1;
    uint16_t out7:1;
    uint16_t out8:1;
    uint16_t diag1:1;
    uint16_t diag2:1;
    uint16_t diag3:1;
    uint16_t diag4:1;
    uint16_t diag5:1;
    uint16_t diag6:1;
    uint16_t diag7:1;
    uint16_t diag8:1;
  };
} OutCtl_t;

typedef union {
  uint16_t data;
  struct {
    uint16_t termwarn:1;
    uint16_t out1:1;
    uint16_t out2:1;
    uint16_t out3:1;
    uint16_t out4:1;
    uint16_t out5:1;
    uint16_t out6:1;
    uint16_t out7:1;
    uint16_t out8:1;
    uint16_t in5:1;
    uint16_t in6:1;
    uint16_t in7:1;
    uint16_t in8:1;
    uint16_t none:2;
    uint16_t power:1;
  };
} OutStat_t;

extern uint8_t USB_RX_Buffer[HID_INBUF_SIZE];
extern uint8_t InpPinDataReady, OutPinDataReady;
extern uint8_t OutPinData[OUTPin_MAX_CHIPS];
  
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
