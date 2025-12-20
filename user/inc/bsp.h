#ifndef __BSP_H__
#define __BSP_H__

#include "stm32f10x.h"

// --- Pin Definitions (Refactored for Safety) ---

// PIR Sensor (Remapped to PC1)
#define PIR_PORT        GPIOC
#define PIR_PIN         GPIO_Pin_1
#define PIR_RCC         RCC_APB2Periph_GPIOC

// Touch Sensor (Remapped to PC2)
#define TOUCH_PORT      GPIOC
#define TOUCH_PIN       GPIO_Pin_2
#define TOUCH_RCC       RCC_APB2Periph_GPIOC
#define TOUCH_EXTI_Line EXTI_Line2
#define TOUCH_PortSource GPIO_PortSourceGPIOC
#define TOUCH_PinSource  GPIO_PinSource2
#define TOUCH_IRQn       EXTI2_IRQn

// Light Sensor (Restored to PB0/ADC12_IN8 for Unmoved HW)
#define LIGHT_PORT      GPIOB
#define LIGHT_PIN       GPIO_Pin_0
#define LIGHT_RCC       RCC_APB2Periph_GPIOB
#define LIGHT_ADC_CH    ADC_Channel_8

// Servo Motor (Remapped to PB6/TIM4_CH1)
#define SERVO_PORT      GPIOB
#define SERVO_PIN       GPIO_Pin_6
#define SERVO_RCC_GPIO  RCC_APB2Periph_GPIOB
#define SERVO_RCC_TIM   RCC_APB1Periph_TIM4

// LED (Remapped to PD3, PD4)
#define LED_PORT        GPIOD
#define LED_RCC         RCC_APB2Periph_GPIOD
#define LED1_PIN        GPIO_Pin_3 // Green (Occupied)
#define LED2_PIN        GPIO_Pin_4 // Red (Suspicious/Vacant)

// --- Function Prototypes ---
void BSP_Init(void);
void GPIO_Configuration(void);
void ADC_Configuration(void);
void TIM_Configuration(void); // For Servo
void NVIC_Configuration(void);
void EXTI_Configuration(void);
void USART_Configuration(void); // For Bluetooth

// Hardware Control Functions
void Servo_SetAngle(uint8_t angle);
void LED_Set(uint8_t led, uint8_t state); // 1: Green, 2: Red
uint16_t LightSensor_Read(void);
uint8_t PIR_Read(void);
void Bluetooth_Send(uint8_t data);

#endif // __BSP_H__

