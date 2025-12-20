#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"

volatile uint16_t g_LightValue = 0;   // 조도 센서 값 (ADC 인터럽트에서 갱신)

// ─── RCC 설정: ADC1, GPIOB 클럭 ─────────────────
void RCC_Configure(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // PB0 (ADC 채널 8)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);   // ADC1
}

// ─── GPIO 설정: PB0 = ADC 입력 ───────────────────
void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;    // PB0 -> ADC 채널 8
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // 아날로그 입력
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// ─── NVIC 설정: ADC 인터럽트 ─────────────────────
void NVIC_Configure(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// ─── ADC 설정: 단발 변환 + 인터럽트 ───────────────
void ADC_Configure(void)
{
    ADC_InitTypeDef ADC_InitStructure;

    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode       = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;              // 터치할 때마다 1번만 변환
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // PB0 -> ADC_Channel_8
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_55Cycles5);

    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);  // 변환 완료 인터럽트
    ADC_Cmd(ADC1, ENABLE);

    // 캘리브레이션
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1)) ;
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1)) ;
}

// ─── ADC 인터럽트 핸들러 ─────────────────────────
void ADC1_2_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
    {
        g_LightValue = ADC_GetConversionValue(ADC1); // 전역변수에 저장
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
    }
}

int main(void)
{
    uint16_t rawTouchX = 0;
    uint16_t rawTouchY = 0;
    uint16_t touchX    = 0;
    uint16_t touchY    = 0;

    SystemInit();

    // 1) 조도센서용 클럭/핀/ADC/인터럽트 설정
    RCC_Configure();
    GPIO_Configure();
    ADC_Configure();
    NVIC_Configure();

    // 2) LCD 초기화
    LCD_Init();

    // 3) 터치 설정 + 보정 (캘리브레이션)
    Touch_Configuration();
    Touch_Adjust();   // 여기서 화면에 십자(+) 나오고, 몇 번 터치해야 끝남

    // 4) 보정이 끝난 뒤에 화면 초기화 + 팀명/라벨 출력
    LCD_Clear(WHITE);
    LCD_ShowString(40, 40, (u8*)"TUE_Team01", BLACK, WHITE);
    LCD_ShowString(40, 60, (u8*)"X:",         BLACK, WHITE);
    LCD_ShowString(40, 80, (u8*)"Y:",         BLACK, WHITE);
    LCD_ShowString(40,100, (u8*)"ADC:",       BLACK, WHITE);

    while (1)
{
    // 1) 터치될 때까지 대기
    Touch_GetXY(&rawTouchX, &rawTouchY, 1);
    Convert_Pos(rawTouchX, rawTouchY, &touchX, &touchY);

    // 2) 조도 센서 한 번 변환 시작
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    // 3) 좌표/조도값 숫자 갱신
    LCD_ShowNum(70, 60, touchX,       4, BLACK, WHITE); // X
    LCD_ShowNum(70, 80, touchY,       4, BLACK, WHITE); // Y
    LCD_ShowNum(90,100, g_LightValue, 4, BLACK, WHITE); // ADC

    // 4) 터치 위치에 작은 원 그리기 (범위 체크 없이 그냥)
    LCD_DrawCircle(touchX, touchY, 4);
}

}
