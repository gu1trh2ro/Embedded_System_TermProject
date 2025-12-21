/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <includes.h>

// =======================================================
// [RTOS 통합] µC/OS-III 관련 외부 선언 (main.c에 정의되어 있음)
// =======================================================
// #include "os.h" // includes.h에 포함됨
extern OS_Q   PirDataQ;    // 터치 센서 이벤트를 Status_Task로 전달 (MSG_TOUCH_RESET_CODE)
extern OS_Q   BluetoothRxQ; // 수신된 블루투스 데이터를 Bluetooth_Task로 전달
extern const uint8_t MSG_TOUCH_RESET_CODE; // main.c의 상수를 extern 선언
extern const uint8_t MSG_TOUCH_RELEASE_CODE;

// Global Event Flags
extern volatile uint8_t g_touch_event;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/* Cortex-M3 Processor Exceptions Handlers                             */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  */
void PendSV_Handler(void)
{
    // µC/OS-III의 컨텍스트 스위칭 핸들러
    OS_CPU_PendSVHandler(); 
}

/**
  * @brief  This function handles SysTick Handler.
  */
void SysTick_Handler(void)
{
    // µC/OS-III의 시스템 틱 핸들러
    OS_CPU_SysTickHandler(); 
}

/******************************************************************************/
/* STM32F10x Peripherals Interrupt Handlers                          */
/******************************************************************************/

/**
  * @brief  This function handles EXTI Line 0 interrupt request (Touch Sensor: PC.0).
  * (Touch Press -> Set Event Flag)
  */
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        // Set Global Touch Event Flag
        g_touch_event = 1;
        
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}


/**
  * @brief  This function handles USART2 global interrupt request (Bluetooth RX).
  * (원격 제어 명령 수신용)
  */
/**
  * @brief  This function handles USART1 global interrupt request (PC Communication).
  * (PC에서 수신된 데이터를 블루투스로 전송 - Echo)
  */
void USART1_IRQHandler(void)
{
    uint16_t word;
    volatile uint32_t timeout = 0;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        /* 수신된 데이터 읽기 */
        word = USART_ReceiveData(USART1);       

        /* 전송 버퍼가 비워질 때까지 대기 (Timeout 적용) */
        // ISR 내에서 무한 대기하면 시스템이 멈춥니다. 보호 코드 추가.
        while ((USART2->SR & USART_SR_TXE) == 0) {
            if (++timeout > 5000) break; // Timeout guard
        }
        
        /* Timeout 안났을 때만 전송 */
        if (timeout <= 5000) {
            USART_SendData(USART2, word);
        }

        /* RX 인터럽트 플래그 클리어 */
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

/**
  * @brief  This function handles USART2 global interrupt request (Bluetooth RX).
  * (원격 제어 명령 수신용 + PC Echo)
  */
void USART2_IRQHandler(void)
{
    OS_ERR err;
    uint8_t rx_data;
    volatile uint32_t timeout = 0;
    
    // 수신 버퍼가 비어있지 않음 확인 (데이터 수신)
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        // 1. 데이터 읽기
        rx_data = (uint8_t)USART_ReceiveData(USART2);

        // [Reference Logic] PC로 에코 전송 (Timeout 적용)
        while ((USART1->SR & USART_SR_TXE) == 0) {
            if (++timeout > 5000) break; 
        }
        
        if (timeout <= 5000) {
            USART_SendData(USART1, rx_data);
        }

        // 2. 수신된 데이터를 Bluetooth_Task의 큐에 전송 (비동기 처리)
         OSQPost(&BluetoothRxQ, &rx_data, sizeof(rx_data), OS_OPT_POST_FIFO, &err);
        
        // 3. 인터럽트 플래그 클리어
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}


/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/