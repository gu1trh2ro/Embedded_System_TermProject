#ifndef __BSP_H__
#define __BSP_H__

#include "stm32f10x.h"

// --- Pin Definitions (Refactored for Safety) ---

// PIR Sensor (Changed from PA5 to PE2)
#define PIR_PORT        GPIOE
#define PIR_PIN         GPIO_Pin_2
#define PIR_RCC         RCC_APB2Periph_GPIOE

// Touch Sensor (Changed from PA1 to PE3)
#define TOUCH_PORT      GPIOE
#define TOUCH_PIN       GPIO_Pin_3
#define TOUCH_RCC       RCC_APB2Periph_GPIOE
#define TOUCH_EXTI_Line EXTI_Line3
#define TOUCH_PortSource GPIO_PortSourceGPIOE
#define TOUCH_PinSource  GPIO_PinSource3
#define TOUCH_IRQn       EXTI3_IRQn

// Light Sensor (Changed from PB0 to PC0/ADC12_IN10)
// Using onboard Potentiometer for testing
#define LIGHT_PORT      GPIOC
#define LIGHT_PIN       GPIO_Pin_0
#define LIGHT_RCC       RCC_APB2Periph_GPIOC
#define LIGHT_ADC_CH    ADC_Channel_10

// Servo Motor (Changed from PA6 to PE9/TIM1_CH1)
#define SERVO_PORT      GPIOE
#define SERVO_PIN       GPIO_Pin_9
#define SERVO_RCC_GPIO  RCC_APB2Periph_GPIOE
#define SERVO_RCC_TIM   RCC_APB2Periph_TIM1

// LED & LCD (Existing)
#define LED_PORT        GPIOC
#define LED1_PIN        GPIO_Pin_8 // Green (Occupied)
#define LED2_PIN        GPIO_Pin_9 // Red (Suspicious/Vacant)

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
