#pragma once

#include <stm32f4xx.h>
#include <array>
#include <string>

namespace Driver
{

class USART
{
private:
    volatile USART_TypeDef* _base;
    const uint32_t _baudrate;

private:
    inline void pin_configuration();

public:
    //USART() = delete;
    //USART operator=(const USART&)   = delete;
    //USART(const USART_TypeDef*)     = delete;
    USART(volatile USART_TypeDef* uart_base, const uint32_t& baudrate);


public:
    void transmit(const uint8_t&);

    template<uint8_t size>
    void transmit(const std::array<uint8_t, size>&);

    void transmit(const std::string_view);
};

};