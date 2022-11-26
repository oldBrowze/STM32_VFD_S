#include "stm32f4_usart.hpp"

namespace Driver
{
    
    USART::USART(volatile USART_TypeDef* uart_base, const uint32_t& baudrate) : _base{uart_base}, _baudrate{baudrate}
    {
        uart_base->CR1 = USART_CR1_UE_Msk;
        uart_base->BRR = 84'000'000 / baudrate;
        uart_base->CR1 |= USART_CR1_TE_Msk;

        pin_configuration();
    }
    
    inline void USART::pin_configuration()
    {
        //TODO:на каждый uart свой пин сделать

        //PA9 - Tx
        /*
        GPIOA->MODER |= (0b10 << GPIO_MODER_MODER9_Pos);
        GPIOA->AFR[1] |= (0b111 << GPIO_AFRH_AFSEL9_Pos);
        */

        /* PB6 - Tx */

        GPIOB->MODER |= (0b10 << GPIO_MODER_MODER6_Pos);
        GPIOB->AFR[0] |= (0b111 << GPIO_AFRL_AFSEL6_Pos);
    }
    
    void USART::transmit(const uint8_t& message)
    {
        _base->DR = static_cast<uint8_t>(message);
        while((_base->SR & USART_SR_TXE) == false);
    }

    void USART::transmit(const std::string_view message)
    {
        for(const auto &i : message)
        {
            _base->DR = static_cast<uint8_t>(i);
            while((_base->SR & USART_SR_TXE) == false);
        }
    }

    template<uint8_t size> 
    void USART::transmit(const std::array<uint8_t, size>& message)
    {
        for(const auto &i : message)
        {
            _base->DR = static_cast<uint8_t>(i);
            while((_base->SR & USART_SR_TXE) == false);
        }
    }
}