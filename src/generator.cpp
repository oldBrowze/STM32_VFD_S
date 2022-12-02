#include "generator.hpp"

namespace Driver
{
    void VFDController::configuration()
    {
        driver_signal_configuration();
        pwm_configuration();
        encoder_configuration();
        //adc_configuration();
        button_configuration();
    }

    void VFDController::pwm_configuration()
    {
        NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
        NVIC_SetPriority(TIM1_UP_TIM10_IRQn, IRQ_Priority::TIM1_UP_TIM10_pwm_generation);

        /* конфигурация каналов ШИМ */
        /* инверсные каналы - AF01 (PB13 - CH1N / PB14 - CH2N / PB15 - CH3N) */
        GPIOB->MODER |= (0b10 << GPIO_MODER_MODER13_Pos) | (0b10 << GPIO_MODER_MODER14_Pos) | (0b10 << GPIO_MODER_MODER15_Pos);
        GPIOB->AFR[1] |= (0b1 << GPIO_AFRH_AFSEL13_Pos) | (0b1 << GPIO_AFRH_AFSEL14_Pos) | (0b1 << GPIO_AFRH_AFSEL15_Pos);

        /* прямые каналы - AF01 (PA8 - CH1 / PA9 - CH2 / PA10 - CH3) */
        GPIOA->MODER |= (0b10 << GPIO_MODER_MODER8_Pos) | (0b10 << GPIO_MODER_MODER9_Pos) | (0b10 << GPIO_MODER_MODER10_Pos);
        GPIOA->AFR[1] |= (0b1 << GPIO_AFRH_AFSEL8_Pos) | (0b1 << GPIO_AFRH_AFSEL9_Pos) | (0b1 << GPIO_AFRH_AFSEL10_Pos);

        // PWM Mode 1 на канала OC1, OC2, OC3
        _TIM_PWM_BASE->CCMR1 |= (0b110 << TIM_CCMR1_OC1M_Pos) | (0b110 << TIM_CCMR1_OC2M_Pos);
        _TIM_PWM_BASE->CCMR2 |= (0b110 << TIM_CCMR2_OC3M_Pos);

        _TIM_PWM_BASE->CCER |= (0b1 << TIM_CCER_CC1E_Pos) |
                               (0b1 << TIM_CCER_CC2E_Pos) |
                               (0b1 << TIM_CCER_CC3E_Pos) |
                               (0b1 << TIM_CCER_CC1NE_Pos) |
                               (0b1 << TIM_CCER_CC2NE_Pos) |
                               (0b1 << TIM_CCER_CC3NE_Pos);

        _TIM_PWM_BASE->BDTR = TIM_BDTR_MOE;
        _TIM_PWM_BASE->DIER = TIM_DIER_UIE;
        //_TIM_PWM_BASE->EGR |= TIM_EGR_UG;

        _TIM_PWM_BASE->PSC = (cnt_to_psc(encoder_counter));
        _TIM_PWM_BASE->ARR = ARR_value;

        _TIM_PWM_BASE->CR1 |= TIM_CR1_CEN;
    }

    void VFDController::driver_signal_configuration()
    {
        /* конфигурация ITRIP (выход) - PA5*/
        GPIOA->MODER |= (0b01 << GPIO_MODER_MODE5_Pos);
        GPIOA->BSRR = GPIO_BSRR_BR5_Msk; //по умолчанию лог. 0

        /* конфигурация VFO (вход) PB1 PU + EXTI*/
        GPIOB->MODER |= (0b00 << GPIO_MODER_MODE1_Pos);
        GPIOB->PUPDR |= (0b01 << GPIO_PUPDR_PUPD1_Pos);

        NVIC_EnableIRQ(EXTI1_IRQn);
        NVIC_SetPriority(EXTI1_IRQn, IRQ_Priority::EXTI1_VFO_detected);

        EXTI->IMR |= EXTI_IMR_MR1_Msk;
        EXTI->FTSR |= EXTI_FTSR_TR1_Msk;

        SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PB;
    }

    void VFDController::encoder_configuration()
    {
        NVIC_EnableIRQ(EXTI0_IRQn);
        NVIC_EnableIRQ(EXTI2_IRQn);

        NVIC_SetPriority(EXTI0_IRQn, IRQ_Priority::EXTI0_encoder_rotate);
        NVIC_SetPriority(EXTI2_IRQn, IRQ_Priority::EXTI2_encoder_button);

        /* конфигурация ног для чтения энкодера(входы)
         *  PA0 - ENC_A | PA1 - ENC_B | PB2 - ENC_KEY
         */
        GPIOA->MODER |= (0b00 << GPIO_MODER_MODE0_Pos) | (0b00 << GPIO_MODER_MODE1_Pos);
        GPIOA->PUPDR |= (0b10 << GPIO_PUPDR_PUPD0_Pos) | (0b10 << GPIO_PUPDR_PUPD1_Pos);

        GPIOB->MODER |= (0b00 << GPIO_MODER_MODE2_Pos);
        GPIOB->PUPDR |= (0b10 << GPIO_PUPDR_PUPD2_Pos);

        EXTI->IMR |= EXTI_IMR_MR0_Msk | EXTI_IMR_MR2_Msk;

        EXTI->FTSR |= EXTI_FTSR_TR0_Msk | EXTI_FTSR_TR2_Msk;

        SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA | SYSCFG_EXTICR1_EXTI2_PB;
    }

    void VFDController::adc_configuration()
    {
        NVIC_EnableIRQ(DMA2_Stream0_IRQn);
        NVIC_SetPriority(DMA2_Stream0_IRQn, IRQ_Priority::DMA2_Stream0_transfer_from_adc);

        /* конфигурация портов ADC как analog mode
         *  PA2 - ADC_IN2
         *  PA3 - ADC_IN3
         *  PA4 - ADC_IN4
         */
        GPIOA->MODER |= (0b11 << GPIO_MODER_MODE2_Pos) |
                        (0b11 << GPIO_MODER_MODE3_Pos) |
                        (0b11 << GPIO_MODER_MODE4_Pos);

        // NVIC_EnableIRQ(ADC_IRQn);
        ADC1->CR2 |= ADC_CR2_ADON;
        /* Настройка ADC */
        ADC1->CR1 |= ADC_CR1_SCAN;
        ADC1->CR2 |= ADC_CR2_DDS | ADC_CR2_DMA | ADC_CR2_CONT | ADC_CR2_EOCS;

        // 28 циклов на измерение
        ADC1->SMPR2 |= (0b010 << ADC_SMPR2_SMP0_Pos) |
                       (0b010 << ADC_SMPR2_SMP1_Pos) |
                       (0b010 << ADC_SMPR2_SMP2_Pos);

        // 3 канала
        ADC1->SQR1 |= (0b0010 << ADC_SQR1_L_Pos);

        /*
            1 канал ADC_IN2
            2 канал ADC_IN3
            3 канал ADC_IN4
        */
        ADC1->SQR3 |= (0b0010 << ADC_SQR3_SQ1_Pos) |
                      (0b0011 << ADC_SQR3_SQ2_Pos) |
                      (0b0100 << ADC_SQR3_SQ3_Pos);

        /* настройка DMA ADC */

        //канал 0
        DMA2_Stream0->NDTR = 3;
        DMA2_Stream0->PAR = reinterpret_cast<std::uintptr_t>(&ADC1->DR);
        DMA2_Stream0->M0AR = reinterpret_cast<std::uintptr_t>(DMA_buffer.data());

        DMA2_Stream0->CR = (0x0 << DMA_SxCR_CHSEL_Pos) |
                           (0b11 << DMA_SxCR_PL_Pos) |
                           (0b01 << DMA_SxCR_MSIZE_Pos) |
                           (0b01 << DMA_SxCR_PSIZE_Pos) |
                           (DMA_SxCR_MINC) |
                           (DMA_SxCR_CIRC) |
                           (0b00 << DMA_SxCR_DIR_Pos) |
                           (DMA_SxCR_TCIE);

        ADC1->CR2 |= ADC_CR2_SWSTART;
        DMA2_Stream0->CR |= DMA_SxCR_EN;
    }

    void VFDController::button_configuration()
    {
        __enable_irq();
        NVIC_EnableIRQ(EXTI15_10_IRQn);
        NVIC_SetPriority(EXTI15_10_IRQn, IRQ_Priority::EXTI15_10_button_start_stop);

        NVIC_EnableIRQ(EXTI9_5_IRQn);
        NVIC_SetPriority(EXTI9_5_IRQn, IRQ_Priority::EXTI9_5_button_reverse);

        GPIOC->MODER |= (0b00 << GPIO_MODER_MODE15_Pos)/* | (0b00 << GPIO_MODER_MODE7_Pos) | (0b00 << GPIO_MODER_MODE8_Pos)*/;
        GPIOC->PUPDR |= (0b10 << GPIO_PUPDR_PUPD15_Pos)/* | (0b00 << GPIO_MODER_MODE7_Pos) | (0b00 << GPIO_MODER_MODE8_Pos)*/;

        GPIOB->MODER |= (0b00 << GPIO_MODER_MODE9_Pos) | (0b00 << GPIO_MODER_MODE8_Pos);
        GPIOB->PUPDR |= (0b10 << GPIO_PUPDR_PUPD9_Pos) | (0b00 << GPIO_PUPDR_PUPD8_Pos);

        EXTI->IMR |= EXTI_IMR_MR15_Msk | EXTI_IMR_MR9_Msk | EXTI_IMR_MR8_Msk;

        EXTI->FTSR |= EXTI_FTSR_TR15_Msk | EXTI_FTSR_TR9_Msk | EXTI_FTSR_TR8_Msk;
        //EXTI->RTSR |= EXTI_RTSR_TR9_Msk | EXTI_RTSR_TR7_Msk | EXTI_RTSR_TR8_Msk;
        //SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PB;
        SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PC;    
        SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI9_PB;
        SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PB;
    }

    void VFDController::set_frequency(const uint16_t &_frequency)
    {
        _TIM_PWM_BASE->PSC = cnt_to_psc(_frequency);
        //frequency = get_frequency();
    }

    uint8_t VFDController::get_frequency()
    {
        return 0;
    }


    void VFDController::update_frequency()
    {
        static uint8_t _digit = 0;

        switch(_digit)
        {
            case 0:
                tranceiver_SPI.transmit(digits_code[0], symbols_code[(encoder_counter % 10)]); 
                break;
            case 1: 
                tranceiver_SPI.transmit(digits_code[1], symbols_code[(encoder_counter / 10 % 10)]); 
                break;
            case 2: 
                tranceiver_SPI.transmit(digits_code[2], symbols_code[(encoder_counter / 100)]); 
                break;
            case 3: 
                tranceiver_SPI.transmit(digits_code[3], symbols_code[10]); 
                break;
        }

        if(++_digit > 3)
            _digit = 0;
    }

}

extern "C" void DMA2_Stream0_IRQHandler()
{
    // using VFD = Driver::VFDController;

    if (DMA2->LISR & DMA_LISR_TCIF0)
    {
        DMA2->LISR = ~DMA_LISR_TCIF0;

        //__disable_irq();

        // FIXME: не оптимизировано
        //передача буфера
        // VFD::tranceiver.transmit(std::string_view("ADC value is: "));
        // VFD::tranceiver.transmit(std::string_view(std::to_string(ADC1->DR)));
        // VFD::tranceiver.transmit(std::string_view("\n"));

        //__enable_irq();
    }
}

/**
 * @brief Обработчик поворота энкодера
 *
 */
extern "C" void EXTI0_IRQHandler()
{
    using VFD = Driver::VFDController;

    if (EXTI->PR & EXTI_PR_PR0_Msk)
    {
        EXTI->PR = EXTI_PR_PR0_Msk;
    }


    if (GPIOA->IDR & GPIO_IDR_ID1) //влево
    {
        if (--VFD::encoder_counter < 10)
            VFD::encoder_counter = 10;
        VFD::set_frequency(VFD::encoder_counter);
    }
    else //вправо
    {
        if (++VFD::encoder_counter > 100)
            VFD::encoder_counter = 100;
        VFD::set_frequency(VFD::encoder_counter);
    }
}

/**
 * @brief Обработчик поступления сигнала VFO (спадающий фронт, см. даташит)
 *
 */
extern "C" void EXTI1_IRQHandler()
{
    if (EXTI->PR & EXTI_PR_PR1_Msk)
    {
        EXTI->PR = EXTI_PR_PR1_Msk;

        //VFD::toggle_driver();
    }
}

/**
 * @brief Обработчик нажатия на энкодер
 *
 */
extern "C" void EXTI2_IRQHandler()
{
    using VFD = Driver::VFDController;

    if (EXTI->PR & EXTI_PR_PR2_Msk)
    {
        EXTI->PR = EXTI_PR_PR2_Msk;
        VFD::toggle_driver();
    }
}

/**
 * @brief Обработчик нажатия кнопки остановки
 * 
 */
extern "C" void EXTI15_10_IRQHandler()
{
    using VFD = Driver::VFDController;

    /* Кнопка остановки */
    if (EXTI->PR & EXTI_PR_PR15_Msk)
    {
        EXTI->PR = EXTI_PR_PR15_Msk;
        
        VFD::stop_driver();
    }
    //VFD::toggle_driver();
}

/**
 * @brief Обработчик нажатия кнопок пуска / реверса 
 * 
 */
extern "C" void EXTI9_5_IRQHandler()
{
    using VFD = Driver::VFDController;

    /* Кнопка запуска */
    if (EXTI->PR & EXTI_PR_PR9_Msk)
    {
        EXTI->PR = EXTI_PR_PR9_Msk;
        VFD::start_driver();
    }
    
    /* Кнопка реверса */
    if (EXTI->PR & EXTI_PR_PR8_Msk)
    {
        EXTI->PR = EXTI_PR_PR8_Msk;
        VFD::is_reverse = !VFD::is_reverse;
    }
    //VFD::toggle_driver();
}

/**
 * @brief Обработчик-формирователь синусоидальной ШИМ
 *
 */
extern "C" void TIM1_UP_TIM10_IRQHandler()
{
    using VFD = Driver::VFDController;

    if (VFD::_TIM_PWM_BASE->SR & TIM_SR_UIF)
    {
        VFD::_TIM_PWM_BASE->SR = 0;

        VFD::_TIM_PWM_BASE->CCR1 = VFD::_sine_table_phases[(!VFD::is_reverse) ? VFD::phase_U : VFD::phase_W] * VFD::koeff_voltage;
        VFD::_TIM_PWM_BASE->CCR2 = VFD::_sine_table_phases[VFD::phase_V] * VFD::koeff_voltage;
        VFD::_TIM_PWM_BASE->CCR3 = VFD::_sine_table_phases[(!VFD::is_reverse) ? VFD::phase_W : VFD::phase_U] * VFD::koeff_voltage;

        if (++VFD::phase_U == VFD::array_size)
            VFD::phase_U = 0;

        if (++VFD::phase_V == VFD::array_size)
            VFD::phase_V = 0;

        if (++VFD::phase_W == VFD::array_size)
            VFD::phase_W = 0;
    }
    // GPIOA->ODR ^= GPIO_ODR_OD5_Msk;
}