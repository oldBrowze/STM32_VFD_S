#pragma once

#include <stm32f4xx.h>

namespace Driver
{

    class SPI
    {
    private:
        volatile SPI_TypeDef *_SPI_BASE;

    public:
        SPI(volatile SPI_TypeDef *);

        void transmit(const uint8_t &);
        void transmit(const uint8_t &, const uint8_t &);
    };

};