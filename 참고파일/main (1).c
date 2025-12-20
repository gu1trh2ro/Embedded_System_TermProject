#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

/* ========================
 *  Function Prototypes
 * ======================== */
void RCC_Configure(void);
void GPIO_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void NVIC_Configure(void);

/* ===========================================================
 *  RCC_Configure()
 *  - 주변 장치(USART1, USART2, GPIOA, AFIO)의 클럭 활성화
 * =========================================================== */
void RCC_Configure(void)
{
    /* GPIOA (USART1/2 TX, RX 핀용) 클럭 Enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* USART1 클럭 Enable (APB2) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* USART2 클럭 Enable (APB1) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* AFIO (Alternate Function IO) 클럭 Enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

/* ===========================================================
 *  GPIO_Configure()
 *  - USART1, USART2의 TX/RX 핀 모드 설정
 * =========================================================== */
void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* USART1 TX (PA9) → Alternate Function Push-Pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1 RX (PA10) → Input Pull-Up */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 TX (PA2) → Alternate Function Push-Pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 RX (PA3) → Input Pull-Up */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* ===========================================================
 *  USART1_Init()
 *  - PC 통신용 USART1 설정 및 RX 인터럽트 활성화
 * =========================================================== */
void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

    USART1_InitStructure.USART_BaudRate = 9600;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1_5;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART1_InitStructure);

    /* RX 인터럽트 활성화 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* USART1 모듈 Enable */
    USART_Cmd(USART1, ENABLE);
}

/* ===========================================================
 *  USART2_Init()
 *  - 블루투스 모듈 통신용 USART2 설정 및 RX 인터럽트 활성화
 * =========================================================== */
void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1_5;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART2_InitStructure);

    /* RX 인터럽트 활성화 */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    /* USART2 모듈 Enable */
    USART_Cmd(USART2, ENABLE);
}

/* ===========================================================
 *  NVIC_Configure()
 *  - USART1, USART2 인터럽트 우선순위 및 NVIC 설정
 * =========================================================== */
void NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* NVIC 우선순위 그룹 설정 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* USART1 인터럽트 설정 */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART2 인터럽트 설정 */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ===========================================================
 *  USART1_IRQHandler()
 *  - PC(USART1)에서 수신된 데이터를 블루투스(USART2)로 전송
 * =========================================================== */
void USART1_IRQHandler(void)
{
    uint16_t word;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        /* 수신된 데이터 읽기 */
        word = USART_ReceiveData(USART1);

        /* 전송 버퍼가 비워질 때까지 대기 */
        while ((USART2->SR & USART_SR_TXE) == 0);

        /* 블루투스로 전송 */
        USART_SendData(USART2, word);

        /* RX 인터럽트 플래그 클리어 */
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

/* ===========================================================
 *  USART2_IRQHandler()
 *  - 블루투스(USART2)에서 수신된 데이터를 PC(USART1)로 전송
 * =========================================================== */
void USART2_IRQHandler(void)
{
    uint16_t word;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        /* 수신된 데이터 읽기 */
        word = USART_ReceiveData(USART2);

        /* 전송 버퍼가 비워질 때까지 대기 */
        while ((USART1->SR & USART_SR_TXE) == 0);

        /* PC로 전송 */
        USART_SendData(USART1, word);

        /* RX 인터럽트 플래그 클리어 */
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

/* ===========================================================
 *  main()
 *  - 시스템 초기화 및 USART1/USART2 양방향 통신 준비
 * =========================================================== */
int main(void)
{
    SystemInit();        // 시스템 클럭 기본 설정
    RCC_Configure();     // 주변 장치 클럭 설정
    GPIO_Configure();    // GPIO 핀 모드 설정
    USART1_Init();       // PC 통신용 USART1 설정
    USART2_Init();       // 블루투스 통신용 USART2 설정
    NVIC_Configure();    // 인터럽트 설정

    while (1) {
        // 메인 루프는 비워둠
        // 인터럽트를 통해 실시간 통신 수행
    }
    return 0;
}
