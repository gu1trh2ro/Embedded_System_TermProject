#include "bsp.h"
#include <stdio.h>
#include "bluetooth.h"
#include "stm32f10x_usart.h"

void GPIO_Configuration(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable Clocks (Updated for new pins)
    // PIR/Touch/Light -> GPIOC, Servo -> GPIOB, LED -> GPIOD
    RCC_APB2PeriphClockCmd(PIR_RCC | TOUCH_RCC | LIGHT_RCC | SERVO_RCC_GPIO | LED_RCC | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // PIR Sensor (PC1) - Input Pull-down
    GPIO_InitStructure.GPIO_Pin = PIR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(PIR_PORT, &GPIO_InitStructure);

    // Touch Sensor (PC2) - Input Pull-down
    GPIO_InitStructure.GPIO_Pin = TOUCH_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(TOUCH_PORT, &GPIO_InitStructure);

    // Light Sensor (PC0/ADC) - Analog Input
    GPIO_InitStructure.GPIO_Pin = LIGHT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(LIGHT_PORT, &GPIO_InitStructure);

    // Servo Motor (PB6/TIM4_CH1) - Alt Function Push Pull
    GPIO_InitStructure.GPIO_Pin = SERVO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_PORT, &GPIO_InitStructure);

    // LEDs (PD3, PD4) - Output Push Pull
    GPIO_InitStructure.GPIO_Pin = LED1_PIN | LED2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);

    // --- USART1 (PC) & USART2 (Bluetooth) GPIO Configuration ---
    
    /* USART1 TX (PA9) -> Alternate Function Push-Pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 RX (PA10) -> Input Pull-Up */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 TX (PA2) -> Alternate Function Push-Pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 RX (PA3) -> Input Pull-Up */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void ADC_Configuration(void) {
    ADC_InitTypeDef ADC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);
    
    // Calibration
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

void TIM_Configuration(void) {
    // Servo Motor using TIM4 CH1 (PB6)
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    // TIM4 is on APB1
    RCC_APB1PeriphClockCmd(SERVO_RCC_TIM, ENABLE);

    // PWM Frequency = 50Hz (20ms Period) for Servo
    // SystemCoreClock is usually 72MHz. 
    // TIM4 (APB1) -> PCLK1 * 2 if APB1 prescaler != 1. Assuming 72MHz.
    // Prescaler = 71 -> 1MHz Timer Clock (1us tick)
    // Period = 20000 -> 20ms
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // PWM Channel 1 Configuration
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1500; // 1.5ms (Center/90deg)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    // TIM4 does not need TIM_CtrlPWMOutputs (Only TIM1/TIM8 do)
    TIM_Cmd(TIM4, ENABLE);
}

void EXTI_Configuration(void) {
    EXTI_InitTypeDef EXTI_InitStructure;
    
    // Connect EXTI Line to Touch Pin
    GPIO_EXTILineConfig(TOUCH_PortSource, TOUCH_PinSource);

    EXTI_InitStructure.EXTI_Line = TOUCH_EXTI_Line;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // Touch triggers on rise
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

void NVIC_Configuration(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable Touch Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = TOUCH_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USART_Configuration(void) {
    // Bluetooth UART (USART2) initialization is handled in USART2_Init()
    // PC UART (USART1) initialization is handled in USART1_Init()
}

void USART1_Init(void) {
    USART_InitTypeDef USART1_InitStructure;

    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1_5;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART1_InitStructure);

    /* RX Interrupt Enable */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
}

void USART2_Init(void) {
    USART_InitTypeDef USART2_InitStructure;

    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1_5;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART2_InitStructure);

    /* RX Interrupt Enable */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* Enable USART2 */
    USART_Cmd(USART2, ENABLE);
}

void BSP_Init(void) {
    GPIO_Configuration();
    ADC_Configuration();
    TIM_Configuration();
    EXTI_Configuration();
    LCD_Init();
    LCD_Init();
    // Bluetooth_Init(); // Removed old init
    USART1_Init();       // PC Communications
    USART2_Init();       // Bluetooth Communications
    NVIC_Configuration();
}

// --- Hardware Abstraction ---

void Servo_SetAngle(uint8_t angle) {
    uint16_t pulse = 500; // Default 0
    if (angle == 90) pulse = 1500;
    else if (angle == 0) pulse = 500;
    
    TIM_SetCompare1(TIM4, pulse);
}

void LED_Set(uint8_t led, uint8_t state) {
    uint16_t pin = (led == 1) ? LED1_PIN : LED2_PIN;
    // Assuming Active High for now as per common LED wiring on these pins
    if (state) GPIO_SetBits(LED_PORT, pin);
    else GPIO_ResetBits(LED_PORT, pin);
}

uint16_t LightSensor_Read(void) {
    ADC_RegularChannelConfig(ADC1, LIGHT_ADC_CH, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    return ADC_GetConversionValue(ADC1);
}

uint8_t PIR_Read(void) {
    return GPIO_ReadInputDataBit(PIR_PORT, PIR_PIN);
}

void Bluetooth_Send(uint8_t data) {
// Stub
}

void Bluetooth_SendString(char *str) {
    char *p = str;
    volatile uint32_t timeout;
    
    // Send to Bluetooth (USART2) Only
    while (*p) {
        timeout = 0;
        while ((USART2->SR & USART_SR_TXE) == 0) {
            if (++timeout > 10000) break; // Timeout guard
        }
        if (timeout <= 10000) USART_SendData(USART2, *p);
        
        p++;
    }
    
    // Newline/Space for readability
    // Note: Removed \r\n to prevent phantom echo to PC. Using Space separator instead.
    timeout = 0;
    while ((USART2->SR & USART_SR_TXE) == 0) { if (++timeout > 10000) break; }
    if (timeout <= 10000) USART_SendData(USART2, ' ');
}
