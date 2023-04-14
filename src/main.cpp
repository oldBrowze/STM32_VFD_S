#include <stm32f4xx.h>

#include "clock_configuration.hpp"
#include "stm32f4_usart.hpp"
#include "generator.hpp"

[[deprecated("Перенести в SystemInit()")]] __attribute__((constructor(101))) void rcc_enable()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN |
                    RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_GPIOEEN |
                    RCC_AHB1ENR_DMA2EN;

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN |
                    RCC_APB2ENR_SYSCFGEN |
                    RCC_APB2ENR_TIM1EN |
                    RCC_APB2ENR_SPI1EN |
                    RCC_APB2ENR_USART1EN;

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
}

void TIM_enable()
{
    TIM4->DIER |= TIM_DIER_UIE; //прерывание
    TIM4->ARR = 10000 - 1;
    TIM4->PSC = 8400 - 1;
    //TIM4->PSC = 500 - 1;

    TIM4->EGR |= TIM_EGR_UG;
    TIM4->CR1 |= TIM_CR1_CEN;

    NVIC_SetPriority(TIM4_IRQn, Driver::IRQ_Priority::TIM4_display_update);
    NVIC_EnableIRQ(TIM4_IRQn);
}

int main()
{
    using VFD = Driver::VFDController; 
    __enable_irq();
    Clock::enable_clock();
    TIM_enable();

    VFD::configuration();

    while (true);
    return 0;
}

extern "C" void TIM4_IRQHandler()
{
    using VFD = Driver::VFDController;

    if (TIM4->SR & TIM_SR_UIF_Msk)
        TIM4->SR &= ~TIM_SR_UIF_Msk;
    
    //char buffer[100];

    ADC1->CR2 |= ADC_CR2_JSWSTART;
    //snprintf(buffer, sizeof buffer, "CH1: %d\nCH2: %d\nCH3: %d\n", VFD::DMA_buffer[0], VFD::DMA_buffer[1], VFD::DMA_buffer[2]);
    //VFD::update_frequency();
}
