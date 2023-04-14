#include "ST7735.h"

extern void _delay_ms(const uint32_t& ms);

using namespace Driver;

ST7735::ST7735(Driver::SPI& driver, const uint_fast8_t& x_size, const uint_fast8_t& y_size) : x_max{x_size}, y_max{y_size}, transceiver{driver}
{

}

void ST7735::configuration()
{
    pin_configuration();
    /* ST7735 init */
    //GPIOA->BSRR = GPIO_BSRR_BR12_Msk; //CS off
    CS_set();

    RST_reset();
    _delay_ms(10);
    RST_set();
    _delay_ms(10); 

    write_command(0x11); // sleep out
    _delay_ms(100);

    write_command(0x3A); // pixel format
    write_data(0b101); // 16bit

    write_command(registers::MADCTL);
    write_data(commands::MADCTL_MV | commands::MADCTL_MX);

    write_command(0x29); // display on 

    //GPIOA->BSRR = GPIO_BSRR_BS12_Msk; //CS on
    CS_reset();
}

void ST7735::set_x_size(const uint16_t& x_min, const uint16_t& x_max)
{
    write_command(registers::CASET); // CASET - ColAdr

    write_data((x_min & 0xFF00) >> 8); // x_min = 0
    write_data(x_min & 0x00FF); 
    write_data((x_max & 0xFF00) >> 8); // x_min = 128
    write_data(x_max & 0x00FF);
}

void ST7735::set_y_size(const uint16_t& y_min, const uint16_t& y_max)
{
    write_command(registers::RASET); // RASET - RowAdr

    write_data((y_min & 0xFF00) >> 8); // x_min = 0
    write_data(y_min & 0x00FF); 
    write_data((y_max & 0xFF00) >> 8); // x_min = 128
    write_data(y_max & 0x00FF);
}

void ST7735::draw_rect(const uint16_t& x_min, const uint16_t& x_max, 
                        const uint16_t& y_min, const uint16_t& y_max, 
                        const uint16_t& color)
{
    CS_set();

    uint32_t size = (x_max - x_min) * (y_max - y_min);

    set_x_size(x_min, x_max - 1);
    set_y_size(y_min, y_max - 1);

    write_command(registers::RAMWR); // RAMWR

    for(auto pixel = 0ul; pixel < size; pixel++)
    {
        write_data((color & 0xFF00) >> 8); //полубайт
        write_data(color & 0x00FF); 
    }

    CS_reset();
}

void ST7735::draw_char(const uint8_t& symbol, const uint16_t& x_pos, const uint16_t& y_pos, const std::uint_fast16_t& color, const std::uint_fast16_t& backcolor)
{
    CS_set();

    std::uint_fast8_t 
        symbol_width    = 16,
        symbol_height    = 22;


    set_x_size(x_pos, x_pos + symbol_width - 1);
    set_y_size(y_pos, y_pos + symbol_height - 1);

    write_command(registers::RAMWR); // RAMWR

    std::uint16_t _symbol = 0x0;
    

    for(auto depth = 0u; depth < symbol_height; depth++)
    {
        for(auto bit = 0u; bit < symbol_width; bit++)
        {
            _symbol = (FONTS::consolas_18pt[22*(symbol - 65) + depth] >> ((symbol_width) - bit)) & 0x1;
            write_data((_symbol != 0) ? ((color & 0xFF00) >> 8) : ((backcolor & 0xFF00) >> 8));  //полубайт старший
            write_data((_symbol != 0) ? (color & 0x00FF) : (backcolor & 0x00FF));                //младший полубайт
        }
    }

    CS_reset();
}

void ST7735::draw_text(const uint8_t& symbol, const uint16_t& x_pos, const uint16_t& y_pos, const std::uint_fast16_t& color, const std::uint_fast16_t& backcolor)
{

    //std::uint8_t x_offset = x_pos + 20;
    //std::uint8_t x_offset = y_pos + ;
}

void ST7735::pin_configuration()
{
    // PA15 - ST7735_RESET
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODE15_Pos);
    GPIOA->BSRR = GPIO_BSRR_BS15_Msk; //по умолчанию лог. 1

    // PA11 - ST7735_AO
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODE11_Pos);
    GPIOA->BSRR = GPIO_BSRR_BS11_Msk; //по умолчанию лог. 1

    // PA12 - ST7735_CS
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODE12_Pos);
    GPIOA->BSRR = GPIO_BSRR_BS12_Msk; //по умолчанию лог. 1
}