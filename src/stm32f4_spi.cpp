#include "stm32f4_spi.hpp"

namespace Driver
{
    SPI::SPI(volatile SPI_TypeDef *_base) : _SPI_BASE{_base}
    {
        _SPI_BASE->CR1 |= (0b111 << SPI_CR1_BR_Pos) |
                          SPI_CR1_BIDIMODE | 
                          SPI_CR1_LSBFIRST |
                          //SPI_CR1_CPOL | SPI_CR1_CPHA |
                          SPI_CR1_BIDIOE |
                          SPI_CR1_MSTR | 
                          SPI_CR1_SSI |
                          SPI_CR1_SSM;

        _SPI_BASE->CR1 |= SPI_CR1_SPE;

        // PA12 - SPI1_CS
        GPIOA->MODER |= (0b01 << GPIO_MODER_MODE12_Pos);
        GPIOA->BSRR = GPIO_BSRR_BS12_Msk; //по умолчанию лог. 0
        // GPIOA->OSPEEDR |= (0b11 << GPIO_OSPEEDR_OSPEED15_Pos);
        // GPIOA->BSRR = GPIO_BSRR_BS15;

        // PB3 - SPI1_SCK
        GPIOB->MODER |= (0b10 << GPIO_MODER_MODER3_Pos);
        GPIOB->AFR[0] |= (0b101 << GPIO_AFRL_AFSEL3_Pos);
        GPIOB->OSPEEDR |= (0b11 << GPIO_OSPEEDR_OSPEED3_Pos);

        // PB4 - SPI1_MISO
        //    GPIOB->MODER |= (0b10 << GPIO_MODER_MODER4_Pos);
        //    GPIOB->AFR[0] |= (0x5 << GPIO_AFRL_AFSEL4_Pos);

        // PB5 - SPI1_MOSI
        GPIOB->MODER |= (0b10 << GPIO_MODER_MODER5_Pos);
        GPIOB->AFR[0] |= (0b101 << GPIO_AFRL_AFSEL5_Pos);
        GPIOB->OSPEEDR |= (0b11 << GPIO_OSPEEDR_OSPEED5_Pos);
    }

    void SPI::transmit(const uint8_t &_command)
    {

        while (!(_SPI_BASE->SR & SPI_SR_TXE));
	
        GPIOA->BSRR = GPIO_BSRR_BR12_Msk;

        _SPI_BASE->DR = _command;


        while (!(_SPI_BASE->SR & SPI_SR_TXE));
        while ((_SPI_BASE->SR & SPI_SR_BSY));

        GPIOA->BSRR = GPIO_BSRR_BS12_Msk;
    }
    void SPI::transmit(const uint8_t &_digit, const uint8_t &_symbol)
    {
        transmit(_digit);
        transmit(_symbol);
    }
}