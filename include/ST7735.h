#pragma once

#include <stm32f4xx.h>
#include <cctype> // std::tolower()

#include "stm32f4_spi.hpp"
#include "ST7735_fonts.hpp"


#ifdef _DELAY_ENABLE

volatile uint32_t __ticks = 0;
void _delay_ms(const uint32_t& ms)
{
    uint32_t current = __ticks;
    while((__ticks - current) <= ms);
}

void systick_enable()
{
    NVIC_EnableIRQ(SysTick_IRQn);
    SysTick_Config(84'000'000ul / 1000); // 1 ms
}

#endif

namespace Driver
{

class ST7735
{
public:
    enum color : uint16_t
    {
        YELLOW  = 0xFFE0,
        BLUE    = 0x001F,
        GREEN   = 0x07E0,
        RED     = 0xF800,
        BLACK   = 0x0,
        WHITE   = 0xFFFF
    };

    enum registers : uint8_t
    {
        MADCTL  = 0x36,
        RAMWR   = 0x2C,
        CASET   = 0x2A,
        RASET   = 0x2B
    };
    enum commands : uint8_t
    {
        MADCTL_MY = 0x80,
        MADCTL_MX = 0x40,
        MADCTL_MV = 0x20,
        MADCTL_ML = 0x10
    };
private:
    std::uint_fast8_t
        x_max,
        x_min,
        y_max,
        y_min;
    std::uint_fast16_t 
        background_color = color::BLACK;
private:
    Driver::SPI& transceiver;

    void write_command(const std::uint8_t &command) 
    {
        AO_reset();
        transceiver.transmit(command);
    }
    void write_data(const std::uint8_t &data) 
    {
        AO_set();
        transceiver.transmit(data);
    }

    void AO_set() noexcept          { GPIOA->BSRR = GPIO_BSRR_BS11_Msk; }
    void AO_reset() noexcept        { GPIOA->BSRR = GPIO_BSRR_BR11_Msk; }
    void RST_set() noexcept         { GPIOA->BSRR = GPIO_BSRR_BS15_Msk; }
    void RST_reset() noexcept       { GPIOA->BSRR = GPIO_BSRR_BR15_Msk; }
    void CS_set() noexcept          { GPIOA->BSRR = GPIO_BSRR_BR12_Msk; }
    void CS_reset() noexcept        { GPIOA->BSRR = GPIO_BSRR_BS12_Msk; }
public:
    explicit ST7735(Driver::SPI& driver, const uint_fast8_t& x_size = 128, const uint_fast8_t& y_size = 160);

    void data_bus_config() noexcept;

    void configuration() noexcept;
    void pin_configuration() noexcept;
    
    void set_x_size(const uint16_t& x_min, const uint16_t& x_max) noexcept;
    void set_y_size(const uint16_t& y_min, const uint16_t& y_max) noexcept;
    void draw_rect(const uint16_t& x_min, const uint16_t& x_max, 
                        const uint16_t& y_min, const uint16_t& y_max, 
                        const uint16_t& color) noexcept;
    
    void draw_char(const uint8_t& symbol, const uint16_t& x_pos, const uint16_t& y_pos,
        const std::uint_fast16_t& color, const std::uint_fast16_t& backcolor) noexcept;
    void draw_text(const uint8_t& symbol, const uint16_t& x_pos, const uint16_t& y_pos,
        const std::uint_fast16_t& color, const std::uint_fast16_t& backcolor) noexcept;
public:
    void set_size_default() noexcept
    {
        CS_set();

        set_x_size(x_min, x_max);
        set_y_size(y_min, y_max);
    
        CS_reset();
    }
};

};