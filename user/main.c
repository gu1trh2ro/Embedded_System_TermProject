#include "core_cm3.h"
#include "lcd.h"
#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_iwdg.h" // [안정성 추가] IWDG 헤더 추가
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "touch.h"
#include <stdint.h>
// =======================================================
// [RTOS 통합] µC/OS-III 핵심 헤더 파일을 포함합니다.
#include "os.h" // 실제 경로에 맞게 수정 필요
// =======================================================

// -----------------------------------------------------------
// RTOS 상수 및 전역 변수 선언
// -----------------------------------------------------------
#define TASK_STK_SIZE 512
#define N_TASKS 7
#define PIR_SAMPLE_COUNT 20 // 50ms * 20 = 1초 안정화 가정

// 태스크 우선순위 (제안서 기반)
#define APP_TASK_START_PRIO 3
#define PIR_TASK_PRIO 4
#define LIGHT_TASK_PRIO 4
#define STATUS_TASK_PRIO 5
#define DISPLAY_TASK_PRIO 6
#define BLUETOOTH_TASK_PRIO 6
#define SERVO_TASK_PRIO 7

// 태스크 스택 선언
CPU_STK AppTaskStartStk[TASK_STK_SIZE];
CPU_STK PIRTaskStk[TASK_STK_SIZE];
CPU_STK LightSensorTaskStk[TASK_STK_SIZE];
CPU_STK StatusTaskStk[TASK_STK_SIZE];
CPU_STK DisplayTaskStk[TASK_STK_SIZE];
CPU_STK BluetoothTaskStk[TASK_STK_SIZE];
CPU_STK ServoTaskStk[TASK_STK_SIZE];

// RTOS 오브젝트 선언 (프로젝트 제안서 기반)
OS_Q PirDataQ;     // PIR_Task -> Status_Task
OS_Q StateQ;       // Status_Task -> Display/Bluetooth
OS_Q ServoQ;       // Status_Task -> Servo_Task
OS_Q BluetoothRxQ; // USART2_IRQHandler -> Bluetooth_Task (수신 명령용)

OS_SEM LightSensorSem; // Status_Task -> Light_Sensor_Task 호출용 세마포어

// 시스템 상태 및 데이터
volatile uint32_t ADC_Value[1];
// 상태 코드 정의 (예시)
typedef enum { STATUS_OCCUPIED, STATUS_VACANT, STATUS_SUSPICIOUS } SeatStatus;
SeatStatus current_status = STATUS_VACANT;

// -----------------------------------------------------------
// 하드웨어 상수 및 함수 프로토타입 (main.c 내부에 정의됨)
// -----------------------------------------------------------
#define SERVO_TIM_PORT GPIOA
#define SERVO_TIM_PIN GPIO_Pin_6
#define SERVO_TIM TIM3
#define SERVO_TIM_RCC RCC_APB1Periph_TIM3
#define PWM_PERIOD 1000
#define PWM_PRESCALER (720 - 1)
#define MIN_PULSE_CCR 50 // 깃발 내림
#define MID_PULSE_CCR 75 // 깃발 올림, 능동 알림

void RCC_Configure(void);
void GPIO_Configure(void);
void ADC_Configure(void);
void DMA_Configure(void);
void BT_USART_Init(void);
void BT_NVIC_Configure(void);
void BT_SendData(uint8_t data);
void Servo_Init(void);
void Servo_SetAngle(uint8_t angle);
void Touch_EXTI_Init(void);
void IWDG_Configure(void); // [안정성 추가] IWDG 설정 함수

// -----------------------------------------------------------
// 모든 RTOS 태스크 함수 정의 (main.c 내부에 통합)
// -----------------------------------------------------------

/**
 * @brief 초기화, 태스크 생성, 큐 생성 후 자체 종료 (Prio 3)
 */
void AppTaskStart(void *p_arg) {
  OS_ERR err;
  (void)p_arg;

  // 1. RTOS 오브젝트 생성
  OSQCreate(&PirDataQ, "PIR Data Q", 10, &err);
  OSQCreate(&StateQ, "State Q", 10, &err);
  OSQCreate(&ServoQ, "Servo Q", 5, &err);
  OSQCreate(&BluetoothRxQ, "BT RX Q", 5, &err);
  OSSemCreate(&LightSensorSem, "Light Sem", 0, &err);

  // 2. 모든 애플리케이션 태스크 생성
  OSTaskCreate((OS_TCB *)&PIRTaskStk, "PIR Task", PIR_Task, 0, PIR_TASK_PRIO,
               PIRTaskStk, TASK_STK_SIZE / 10, TASK_STK_SIZE, 0, 0, 0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
  OSTaskCreate((OS_TCB *)&LightSensorTaskStk, "Light Task", Light_Sensor_Task,
               0, LIGHT_TASK_PRIO, LightSensorTaskStk, TASK_STK_SIZE / 10,
               TASK_STK_SIZE, 0, 0, 0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
  OSTaskCreate((OS_TCB *)&StatusTaskStk, "Status Task", Status_Task, 0,
               STATUS_TASK_PRIO, StatusTaskStk, TASK_STK_SIZE / 10,
               TASK_STK_SIZE, 0, 0, 0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
  OSTaskCreate((OS_TCB *)&DisplayTaskStk, "Display Task", Display_Task, 0,
               DISPLAY_TASK_PRIO, DisplayTaskStk, TASK_STK_SIZE / 10,
               TASK_STK_SIZE, 0, 0, 0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
  OSTaskCreate((OS_TCB *)&BluetoothTaskStk, "BT Task", Bluetooth_Task, 0,
               BLUETOOTH_TASK_PRIO, BluetoothTaskStk, TASK_STK_SIZE / 10,
               TASK_STK_SIZE, 0, 0, 0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);
  OSTaskCreate((OS_TCB *)&ServoTaskStk, "Servo Task", Servo_Task, 0,
               SERVO_TASK_PRIO, ServoTaskStk, TASK_STK_SIZE / 10, TASK_STK_SIZE,
               0, 0, 0, OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);

  // 3. AppTaskStart는 할 일을 완료했으므로 삭제
  OSTaskDel(0, &err);
}

/**
 * @brief PIR 센서 감지 및 데이터 큐 전송 (Prio 4)
 */
void PIR_Task(void *p_arg) {
  OS_ERR err;
  uint8_t i = 0;
  uint8_t motion_detected = 0;
  uint8_t pir_data; // 0: 움직임 없음, 1: 움직임 감지

  (void)p_arg;

  while (DEF_TRUE) {
    // 50ms 주기 샘플링
    OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STK_CHK, &err);

    // PIR 센서 핀 읽기 (PA5)
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == SET) {
      motion_detected = 1; // 움직임 감지
    }

    i++;
    if (i >= PIR_SAMPLE_COUNT) { // 1초(20회)마다 판단 후 전송
      pir_data = motion_detected;
      OSQPost(&PirDataQ, &pir_data, sizeof(pir_data), OS_OPT_POST_FIFO, &err);

      motion_detected = 0;
      i = 0;
    }
  }
}

/**
 * @brief 조도 센서 값 읽기 (Status_Task의 요청이 있을 때만 동작) (Prio 4)
 */
void Light_Sensor_Task(void *p_arg) {
  OS_ERR err;
  (void)p_arg;

  while (DEF_TRUE) {
    // Status_Task의 요청(Sem Post)을 기다림
    OSSemPend(&LightSensorSem, 0, OS_OPT_PEND_BLOCKING, 0, &err);

    // ADC 변환 시작 및 DMA 완료 대기 (DMA는 Continuous/Circular 모드 가정)
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    // ADC_Value[0]에 값이 채워졌다고 가정하고 Status_Task가 접근하도록 함.
    // 실제 구현에서는 DMA Transfer Complete Interrupt를 통해 동기화하는 것이
    // 이상적입니다.
  }
}

/**
 * @brief 시스템의 상태 결정 및 전파 (Prio 5) - **핵심 로직**
 */
void Status_Task(void *p_arg) {
  OS_ERR err;
  OS_MSG_SIZE msg_size;
  uint32_t *p_msg;
  uint8_t pir_status;
  uint32_t idle_counter = 0; // 1초 단위
  uint32_t suspicious_counter = 0;

  (void)p_arg;

  while (DEF_TRUE) {
    // [안정성 추가] 워치독 카운터 갱신 (시스템이 멈추지 않았음을 하드웨어에
    // 알림) 이 태스크는 최소 1초에 한 번은 실행되므로 여기서 리로드하는 것이
    // 적절함.
    IWDG_ReloadCounter();

    // 1. PIR 또는 인터럽트 이벤트 대기 (가장 높은 우선순위)
    p_msg = (uint32_t *)OSQPend(&PirDataQ, 1000, OS_OPT_PEND_BLOCKING,
                                &msg_size, 0, &err); // 1초(1000ms)마다 확인

    if (err == OS_ERR_NONE) {
      // 1.1. PIR 데이터 수신
      pir_status = *(uint8_t *)p_msg;
      if (pir_status == 1) {
        // 움직임 감지: Occupied 상태로 리셋
        if (current_status != STATUS_OCCUPIED) {
          current_status = STATUS_OCCUPIED;
          OSQPost(&StateQ, &current_status, sizeof(current_status),
                  OS_OPT_POST_FIFO, &err);
        }
        idle_counter = 0;
        suspicious_counter = 0;
      } else {
        // 움직임 없음
        if (current_status == STATUS_OCCUPIED) {
          idle_counter++;
        }
      }
    } else if (err == OS_ERR_TIMEOUT) {
      // 1초 타임아웃 발생 (PIR 이벤트가 없었을 때)
      if (current_status == STATUS_OCCUPIED) {
        idle_counter++;
      }
    }

    // 2. 자리 비움(Vacant) 로직 및 센서 의존성 판단 (제안서 10분, 15분 기준)
    if (idle_counter >= 600) { // 10분 (600초) 이상 무감지
      if (current_status == STATUS_OCCUPIED) {

        // 2.1. Light_Sensor_Task 깨우기
        OSSemPost(&LightSensorSem, OS_OPT_POST_FIFO, &err);
        OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STK_CHK,
                      &err); // 조도 센서값 읽기 대기

        // 2.2. 조도 값 판단 (3700은 임시 기준 값)
        if (ADC_Value[0] > 3700) { // 밝음: 짐만 있음 (Suspicious)
          current_status = STATUS_SUSPICIOUS;
          suspicious_counter = 0;
        } else { // 어두움: 완전 비움 (Vacant)
          current_status = STATUS_VACANT;
        }
        OSQPost(&StateQ, &current_status, sizeof(current_status),
                OS_OPT_POST_FIFO, &err);
      }
    }

    // 3. 능동 관리 로직 (Suspicious 상태 추가 5분)
    if (current_status == STATUS_SUSPICIOUS) {
      suspicious_counter++;
      if (suspicious_counter >= 300) { // 추가 5분 (300초) 지속 (총 15분)
        // 깃발 동작 명령 전송
        OSQPost(&ServoQ, 0, 0, OS_OPT_POST_FIFO, &err);
        suspicious_counter = 0; // 초기화
      }
    }
  }
}

/**
 * @brief LED 및 LCD 출력 갱신 (Prio 6)
 */
void Display_Task(void *p_arg) {
  OS_ERR err;
  OS_MSG_SIZE msg_size;
  SeatStatus *p_status;

  (void)p_arg;

  while (DEF_TRUE) {
    // StateQ에서 새로운 상태가 올 때까지 대기
    p_status = (SeatStatus *)OSQPend(&StateQ, 0, OS_OPT_PEND_BLOCKING,
                                     &msg_size, 0, &err);

    if (err == OS_ERR_NONE) {
      switch (*p_status) {
      case STATUS_OCCUPIED:
        LCD_Clear(GREEN);
        LCD_ShowString(40, 40, "사용 중", BLACK, GREEN);
        GPIO_SetBits(GPIOC, GPIO_Pin_9);   // Green ON
        GPIO_ResetBits(GPIOC, GPIO_Pin_8); // Red OFF
        break;
      case STATUS_VACANT:
        LCD_Clear(WHITE);
        LCD_ShowString(40, 40, "자리 비움", BLACK, WHITE);
        GPIO_SetBits(GPIOC, GPIO_Pin_8);   // Red ON
        GPIO_ResetBits(GPIOC, GPIO_Pin_9); // Green OFF
        break;
      case STATUS_SUSPICIOUS:
        LCD_Clear(YELLOW);
        LCD_ShowString(40, 40, "짐만 있음", BLACK, YELLOW);
        GPIO_SetBits(GPIOC, GPIO_Pin_8);   // Red ON
        GPIO_ResetBits(GPIOC, GPIO_Pin_9); // Green OFF
        break;
      }
    }
  }
}

/**
 * @brief 상태 코드를 블루투스로 원격 전송 및 원격 명령 수신 (Prio 6)
 */
void Bluetooth_Task(void *p_arg) {
  OS_ERR err;
  OS_MSG_SIZE msg_size;
  SeatStatus *p_status;
  uint8_t *p_rx_data;

  (void)p_arg;

  // 블루투스 모듈이 초기화될 시간을 대기 (예시)
  OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STK_CHK, &err);

  while (DEF_TRUE) {
    // 1. 상태 변화 전송 (StateQ 확인)
    p_status = (SeatStatus *)OSQPend(&StateQ, 10, OS_OPT_PEND_NON_BLOCKING,
                                     &msg_size, 0, &err);
    if (err == OS_ERR_NONE) {
      // 상태 코드를 블루투스로 전송 (예: 0x01=Occupied, 0x02=Vacant,
      // 0x03=Suspicious)
      BT_SendData((uint8_t)(*p_status + 1));
    }

    // 2. 원격 명령 수신 (BluetoothRxQ 확인)
    p_rx_data = (uint8_t *)OSQPend(&BluetoothRxQ, 0, OS_OPT_PEND_NON_BLOCKING,
                                   &msg_size, 0, &err);
    if (err == OS_ERR_NONE) {
      // 수신된 명령 처리 로직
    }

    OSTimeDlyHMSM(0, 0, 0, 10, OS_OPT_TIME_HMSM_STK_CHK, &err); // 약간의 딜레이
  }
}

/**
 * @brief 서보 모터 동작 (Prio 7)
 */
void Servo_Task(void *p_arg) {
  OS_ERR err;
  (void)p_arg;

  while (DEF_TRUE) {
    // ServoQ에서 능동 알림 명령을 기다림
    OSQPend(&ServoQ, 0, OS_OPT_PEND_BLOCKING, 0, 0, &err);

    // 능동 알림 동작: 0도 -> 90도 -> 0도
    Servo_SetAngle(90); // 깃발 올리기
    OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STK_CHK, &err);
    Servo_SetAngle(0); // 깃발 내리기
  }
}

// -----------------------------------------------------------
// 하드웨어 드라이버 함수 구현
// -----------------------------------------------------------

/* ===========================================================
 * 1. 시스템 주변 장치 클럭 설정 (RCC)
 * =========================================================== */
void RCC_Configure(void) {
  // GPIO 클럭 활성화 (GPIOA, GPIOB, GPIOC, AFIO)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                             RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO,
                         ENABLE);

  // ADC 및 DMA 클럭 활성화 (조도 센서)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // USART2 클럭 활성화 (블루투스)
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  // TIM3 클럭 활성화 (서보 모터)
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  // Watchdog 클럭은 LSI(Low Speed Internal)를 사용하므로 별도 APB 클럭 불필요
}

/* ===========================================================
 * 2. 시스템 GPIO 핀 설정 (GPIO)
 * =========================================================== */
void GPIO_Configure(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // 2.1. 조도 센서 입력 (PB.0) - Analog Input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // 2.2. 블루투스 USART2 TX/RX (PA2/PA3) 핀 설정
  // TX (PA2) → Alternate Function Push-Pull
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // RX (PA3) → Input Pull-Up
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 2.3. PIR 센서 입력 (PA5) - Input Floating
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 2.4. LED 출력 (PC8/PC9) - Push-Pull Output
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // 2.5. 터치 센서 입력 (PA1) - Input Pull-Down (EXTI용)
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 2.6. 서보 모터 PWM 출력 (PA6) - Alternate Function Push-Pull
  GPIO_InitStructure.GPIO_Pin = SERVO_TIM_PIN; // PA6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SERVO_TIM_PORT, &GPIO_InitStructure);
}

/* ===========================================================
 * 3. ADC 설정 (조도 센서)
 * =========================================================== */
void ADC_Configure() {
  ADC_InitTypeDef ADC_InitStructure;

  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;

  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1,
                           ADC_SampleTime_55Cycles5); // PB.0 -> ADC1_CH8

  ADC_DMACmd(ADC1, ENABLE);

  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  while (ADC_GetResetCalibrationStatus(ADC1)) {
  }
  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1)) {
  }
}

/* ===========================================================
 * 4. DMA 설정 (조도 센서)
 * =========================================================== */
void DMA_Configure(void) {
  DMA_InitTypeDef DMA_Instructure;
  DMA_DeInit(DMA1_Channel1);
  DMA_Instructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_Instructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Value[0];

  DMA_Instructure.DMA_Mode = DMA_Mode_Circular;
  DMA_Instructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_Instructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Instructure.DMA_BufferSize = 1;
  DMA_Instructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_Instructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_Instructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_Instructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_Instructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_Init(DMA1_Channel1, &DMA_Instructure);
  DMA_Cmd(DMA1_Channel1, ENABLE);
}

/* ===========================================================
 * 5. 블루투스 USART 초기화 (BT_USART_Init)
 * =========================================================== */
void BT_USART_Init(void) {
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1_5;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);

  USART_ITConfig(USART2, USART_IT_RXNE,
                 ENABLE); // 원격 제어 수신 인터럽트 활성화

  USART_Cmd(USART2, ENABLE);
}

/* ===========================================================
 * 6. 블루투스 NVIC 설정 (BT_NVIC_Configure)
 * =========================================================== */
void BT_NVIC_Configure(void) {
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  // USART2 인터럽트 설정
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/* ===========================================================
 * 7. 블루투스 데이터 전송 함수 (Bluetooth_Task에서 사용)
 * [안정성 추가] 무한 대기 방지 타임아웃 적용
 * =========================================================== */
void BT_SendData(uint8_t data) {
  // [안정성] 타임아웃 카운터 추가 (약 일정 시간 대기 후 포기)
  uint32_t timeout_counter = 0;
  uint32_t timeout_max = 100000; // 시스템 클럭에 따라 조절 필요

  // TXE 플래그가 설정될 때까지 대기하거나 타임아웃 발생 시 중단
  while ((USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) &&
         (timeout_counter < timeout_max)) {
    timeout_counter++;
  }

  // 타임아웃이 발생하지 않았을 경우에만 전송
  if (timeout_counter < timeout_max) {
    USART_SendData(USART2, data);
  }
}

/* ===========================================================
 * 8. 서보 모터 초기화 (Servo_Init)
 * =========================================================== */
void Servo_Init(void) {
  TIM_TimeBaseInitTypeDef TIM_BaseStruct;
  TIM_OCInitTypeDef TIM_OCStruct;

  // Timer 기본 설정 (50Hz PWM)
  TIM_BaseStruct.TIM_Prescaler = PWM_PRESCALER;
  TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_BaseStruct.TIM_Period = PWM_PERIOD - 1;
  TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(SERVO_TIM, &TIM_BaseStruct);

  // PWM 출력 설정
  TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCStruct.TIM_Pulse = MIN_PULSE_CCR; // 초기 0도 설정

  TIM_OC1Init(SERVO_TIM, &TIM_OCStruct);
  TIM_OC1PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);

  TIM_Cmd(SERVO_TIM, ENABLE);
}

/* ===========================================================
 * 9. 서보 모터 각도 설정 (Servo_Task에서 사용)
 * =========================================================== */
void Servo_SetAngle(uint8_t angle) {
  uint16_t ccr_value;

  // 제안서 요구사항: 90도(알림) 또는 0도(대기)
  if (angle >= 90) {
    ccr_value = MID_PULSE_CCR; // 90도
  } else {
    ccr_value = MIN_PULSE_CCR; // 0도
  }

  TIM_SetCompare1(SERVO_TIM, ccr_value);
}

/* ===========================================================
 * 10. 터치 센서 EXTI 인터럽트 초기화
 * =========================================================== */
void Touch_EXTI_Init(void) {
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // Connect EXTI Line 1 to PA1
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

  // EXTI Line 1 Configuration
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // NVIC Configuration
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  // RTOS Task보다 높은 우선순위 (ISR 역할을 위해)
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/* ===========================================================
 * 11. [안정성 추가] IWDG(독립 워치독) 초기화 설정
 * =========================================================== */
void IWDG_Configure(void) {
  // 1. IWDG 쓰기 접근 허용 (KR 레지스터에 0x5555 기록)
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  // 2. IWDG 프리스케일러 설정 (LSI 클럭은 약 40kHz)
  // 40kHz / 64 = 625Hz
  IWDG_SetPrescaler(IWDG_Prescaler_64);

  // 3. IWDG 리로드 값 설정
  // 3000 / 625Hz = 4.8초 (약 4~5초 동안 갱신 없으면 시스템 리셋)
  IWDG_SetReload(3000);

  // 4. IWDG 카운터 초기화 (갱신)
  IWDG_ReloadCounter();

  // 5. IWDG 활성화
  IWDG_Enable();
}

// ===========================================================
// 12. 메인 함수 (RTOS 부팅)
// ===========================================================
int main() {

  OS_ERR err;

  SystemInit(); // 클럭 시스템 초기화

  // 1. 하드웨어 주변 장치 초기화 (RTOS 시작 전 수행)
  RCC_Configure();
  GPIO_Configure();

  // [센서/구동부 초기화]
  ADC_Configure(); // 조도 센서
  DMA_Configure();
  BT_USART_Init();   // 블루투스
  Servo_Init();      // 서보 모터
  Touch_EXTI_Init(); // 터치 센서

  // [인터럽트 설정]
  BT_NVIC_Configure(); // USART2 인터럽트 설정
  // Touch_EXTI_Init()에서 EXTI1 인터럽트 설정 처리됨

  // [출력 장치 초기화]
  LCD_Init();
  // Touch_Configuration(); // LCD 터치 드라이버는 사용하지 않으므로 비활성화
  LCD_Clear(WHITE);

  // [안정성 추가] RTOS 진입 직전 Watchdog 활성화
  IWDG_Configure();

  // 2. RTOS 초기화 및 AppTaskStart 태스크 생성
  OSInit(&err); // RTOS 초기화

  // AppTaskStart 생성 (모든 태스크와 큐를 생성하는 초기화 태스크)
  OSTaskCreate((OS_TCB *)&AppTaskStartStk, "App Start Task", AppTaskStart, 0,
               APP_TASK_START_PRIO, AppTaskStartStk, TASK_STK_SIZE / 10,
               TASK_STK_SIZE, 0, 0, 0,
               OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR, &err);

  // 3. RTOS 스케줄러 시작 (이후 제어권은 RTOS로 넘어감)
  OSStart(&err);

  // OSStart()가 실패하거나 오류 발생 시 무한 루프
  // [안정성] 에러 발생 시 LED 점멸 (시스템 멈춤 시각화)
  while (DEF_TRUE) {
    GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9); // LED ON
    // 단순 딜레이 루프
    for (int i = 0; i < 1000000; i++)
      ;
    GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9); // LED OFF
    for (int i = 0; i < 1000000; i++)
      ;
  }
}
