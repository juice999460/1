#include "stm32f10x.h"
#include "adc.h"
#include "usart.h"
#include "pwm.h"
#include "gpio.h"
#include "delay.h"
#include <stdio.h>

// 根据学号末位7配置参数
#define ADC_SAMPLE_RATE     50      // 50Hz采样率
#define UART_BAUDRATE      9600    // 波特率9600
#define DEFAULT_THRESHOLD  35.0f   // 默认阈值35°C
#define BREATH_FREQ        2       // 呼吸灯频率2Hz

float temperature = 0.0f;
float threshold = DEFAULT_THRESHOLD;
uint8_t threshold_index = 1; // 阈值索引(0:25, 1:30, 2:35)

// 温度阈值数组
const float thresholds[] = {25.0f, 30.0f, 35.0f};

void System_Init(void);
void LED_Startup_Sequence(void);

int main(void) {
    System_Init();
    LED_Startup_Sequence();
    
    while(1) {
        // 1. 采集温度
        temperature = Get_Temperature();
        
        // 2. 更新LED状态
        if(temperature > threshold) {
            PWM_Set_Freq(BREATH_FREQ);
            GPIO_ResetBits(GPIOA, GPIO_Pin_0); // 绿灯灭
        } else {
            PWM_Stop();
            GPIO_SetBits(GPIOA, GPIO_Pin_0);   // 绿灯亮
        }
        
        // 3. 串口发送温度数据
        printf("Temp: %.1f°C\r\n", temperature);
        
        // 4. 延时控制采样率
        Delay_ms(1000/ADC_SAMPLE_RATE);
    }
}

void System_Init(void) {
    Delay_Init();
    ADC1_Init();
    USART1_Init(UART_BAUDRATE);
    PWM_Init(BREATH_FREQ);
    GPIO_Init();
    
    // 配置按键中断(PA8)
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void LED_Startup_Sequence(void) {
    for(int i=0; i<2; i++) {
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
        Delay_ms(200);
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        Delay_ms(200);
    }
    GPIO_SetBits(GPIOA, GPIO_Pin_0);
}
#include "stm32f10x_it.h"
#include "stm32f10x_exti.h"
#include "usart.h"

extern float threshold;
extern uint8_t threshold_index;

void EXTI9_5_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line8) != RESET) {
        // 循环切换阈值
        threshold_index = (threshold_index + 1) % 3;
        threshold = thresholds[threshold_index];
        
        // 串口发送新阈值
        printf("New Threshold: %.1f°C\r\n", threshold);
        
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
}
#include "adc.h"
#include "stm32f10x_adc.h"
#include "delay.h"

void ADC1_Init(void) {
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
    
    // 校准ADC
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}

float Get_Temperature(void) {
    uint16_t adc_value;
    float voltage, temp;
    
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    adc_value = ADC_GetConversionValue(ADC1);
    
    voltage = (float)adc_value * 3.3f / 4095.0f; // 12位ADC, 3.3V参考电压
    temp = voltage * 100.0f; // LM35: 10mV/°C
    
    return temp;
}
#include "pwm.h"
#include "stm32f10x_tim.h"

void PWM_Init(uint16_t freq) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 配置PA1为PWM输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 定时器基础配置
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // ARR值
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; // 72MHz/72 = 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // PWM模式配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 500; // 初始占空比50%
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    
    PWM_Set_Freq(freq);
}

void PWM_Set_Freq(uint16_t freq) {
    // 呼吸灯频率设置
    uint16_t arr = (uint16_t)(1000 / freq);
    TIM_SetAutoreload(TIM2, arr - 1);
    TIM_SetCompare2(TIM2, arr / 2); // 50%占空比
}

void PWM_Stop(void) {
    TIM_SetCompare2(TIM2, 0); // 占空比设为0
}
#include "usart.h"
#include <stdio.h>

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
    USART_SendData(USART1, (uint8_t)ch);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    return ch;
}

void USART1_Init(uint32_t baudrate) {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    
    // 配置PA9(TX)为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置PA10(RX)为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);
    
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void) {
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART1);
        
        // 检查是否收到学号末两位(16进制)
        if(data == 0x07) { // 学号末位7对应0x07
            printf("Current Threshold: %.1f°C\r\n", threshold);
        } else {
            printf("invalid instruction.\r\n");
        }
    }
}
#include "gpio.h"

void GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // 配置PA0(绿灯)为推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置PA8(按键)为上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
#include "delay.h"
#include "stm32f10x.h"

static uint8_t fac_us = 0;
static uint16_t fac_ms = 0;

void Delay_Init(void) {
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    fac_us = SystemCoreClock / 8000000;
    fac_ms = (uint16_t)fac_us * 1000;
}

void Delay_us(uint32_t nus) {
    uint32_t temp;
    SysTick->LOAD = nus * fac_us;
    SysTick->VAL = 0x00;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    do {
        temp = SysTick->CTRL;
    } while((temp & 0x01) && !(temp & (1 << 16)));
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL = 0x00;
}

void Delay_ms(uint16_t nms) {
    uint32_t temp;
    SysTick->LOAD = (uint32_t)nms * fac_ms;
    SysTick->VAL = 0x00;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    do {
        temp = SysTick->CTRL;
    } while((temp & 0x01) && !(temp & (1 << 16)));
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->VAL = 0x00;
}
