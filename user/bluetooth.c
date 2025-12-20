#include "bluetooth.h"
#include "misc.h"

/*
 * Bluetooth Initialization extracted from main (1).c
 * Configures USART2 for Bluetooth communication.
 * PIN: PA2 (TX), PA3 (RX)
 * Baud: 9600
 */
void Bluetooth_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 1. RCC Configuration */
    /* Give Clocks to GPIOA (USART2 pins) and USART2 */
    /* Note: AFIO might be needed if remapping, but default PA2/PA3 doesn't typically need explicit AFIO clock for default mapping, 
       but reference code enabled it. We will stick to reference code pattern if possible, 
       but redundant clock enables are fine. */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* 2. GPIO Configuration */
    /* USART2 TX (PA2) -> Alternate Function Push-Pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 RX (PA3) -> Input Pull-Up */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 3. USART2 Configuration */
    /* Values from reference: 9600, 8b, 1.5 StopBits, No Parity */
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);

    /* 4. NVIC Configuration */
    /* Enable RX Interrupt */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* Configure NVIC for USART2 */
    /* Reference used Preemption 3, Sub 1. We should check if this conflicts, 
       but for now we copy as requested. */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 5. Enable USART2 */
    USART_Cmd(USART2, ENABLE);
}
