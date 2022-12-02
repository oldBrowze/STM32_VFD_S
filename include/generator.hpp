#pragma once

#include <stm32f4xx.h>
#include <stm32f4_usart.hpp>
#include <stm32f4_spi.hpp>
#include <array>

#include <settings.hpp>

extern "C" void TIM1_UP_TIM10_IRQHandler();
extern "C" void EXTI0_IRQHandler();
extern "C" void EXTI1_IRQHandler();
extern "C" void EXTI2_IRQHandler();
extern "C" void EXTI15_10_IRQHandler();
extern "C" void EXTI9_5_IRQHandler();
extern "C" void DMA2_Stream0_IRQHandler();

/* Конфигурационная карта портов модуля Generator
 *   Порт    |   Функция         | Состояние |   Описание
 *   PA8     |   TIM1_CH1        |   Выход   |   прямой канал фазы U
 *   PA9     |   TIM1_CH2        |   Выход   |   прямой канал фазы V
 *   PA10    |   TIM1_CH3        |   Выход   |   прямой канал фазы W
 *   PB13    |   TIM1_CH1N       |   Выход   |   инверсный канал фазы U
 *   PB14    |   TIM1_CH2N       |   Выход   |   инверсный канал фазы V
 *   PB15    |   TIM1_CH3N       |   Выход   |   инверсный канал фазы W
 *************************
 *   PA0     |   ENCODER_A       |   Вход    |   Канал А энкодера
 *   PA1     |   ENCODER_B       |   Вход    |   Канал B энкодера
 *   PB2     |   ENCODER_KEY     |   Вход    |   Кнопка энкодера
 *************************
 *   PA12    |   SPI1_CS         |   Выход   |   Выбор ведомого устройства (SH_CP 74HC595)
 *   PB3     |   SPI1_SCK        |   Выход   |   Тактовый сигнал ведомого (ST_CP 74HC595)
 *   PB5     |   SPI1_MOSI       |   Выход   |   Сигнал данных для ведомого (DS 74HC595)
 *************************
 *   PA5     |   ITRIP_SIGNAL    |   Выход   |   Сигнал с МК на драйвер о выключении ШИМ (см. даташит)
 *   PB1     |   VFO_Signal      |   Вход    |   Сигнал с драйвера на МК об ошибке (см. даташит)
 *************************
 *   PA2     |   ADC_IN2         |   Аналог  |   Ток фазы U
 *   PA3     |   ADC_IN3         |   Аналог  |   Ток фазы V
 *   PA4     |   ADC_IN4         |   Аналог  |   Ток фазы W
 */
namespace Driver
{
    /* system constants */

    //Больше - выше приоритет
    enum IRQ_Priority : uint8_t
    {
        EXTI2_encoder_button = 1,
        EXTI0_encoder_rotate = 1,
        EXTI1_VFO_detected = 13,
        DMA2_Stream0_transfer_from_adc = 14,
        EXTI15_10_button_start_stop = 10,
        EXTI9_5_button_reverse = 11,
        TIM1_UP_TIM10_pwm_generation = 15
    };

    class VFDController final
    {
    public:
        static inline Driver::USART tranceiver_USART{USART1, 115'200};
        static inline Driver::SPI tranceiver_SPI{SPI1};

    private:
        static inline TIM_TypeDef *_TIM_PWM_BASE = TIM1;
        static inline TIM_TypeDef *_TIM_ENC_BASE = TIM3;
        static inline GPIO_TypeDef *_ITRIP_PIN_BASE = GPIOA;
        static inline GPIO_TypeDef *_VFO_PIN_BASE = GPIOB;

        static constexpr auto array_size = __settings::_size;
        static constexpr auto dfreq = 100u;
        static constexpr auto ARR_value = 8400 - 1; //несущая - 10 кГЦ
        static constexpr auto PSC_value = 0;

        static constexpr auto DMA_buffer_size = 3;
        static inline uint8_t encoder_counter = 50;
        static constexpr auto
            phase_U_start_value = 0u,
            phase_V_start_value = array_size / 3,
            phase_W_start_value = array_size - array_size / 3;

        static inline auto
            phase_U = phase_U_start_value,
            phase_V = phase_V_start_value,
            phase_W = phase_W_start_value;

        static inline auto koeff_voltage = 1.0f; // U/f = const

        static inline bool is_reverse = false;
        //static inline uint8_t frequency{10};
    private:
        static constexpr std::array<uint16_t, array_size> _sine_table_phases = __settings::_array;
        static inline std::array<uint16_t, DMA_buffer_size> DMA_buffer;

        public:
        /* [A B C D E F G] DP=0 */
        static inline std::array<uint8_t, 13> symbols_code
        {
            0x7e << 1, //0
            0x06 << 1, //1
            0x6d << 1, //2
            0x79 << 1, //3
            0x33 << 1, //4
            0x5b << 1, //5
            0x5f << 1, //6
            0x70 << 1, //7
            0x7f << 1, //8
            0x7b << 1, //9
            0x47 << 1, //F(uppercase)
            0x4f << 1, //E(uppercase)
            0x05 << 1 //r(lowercase)
        };

        /*4 3 2 1*/
        static inline std::array<uint8_t, 4> digits_code
        {
            0x80, //1
            0x40, //2
            0x20, //3
            0x10 //4
        };
    public:
        VFDController() = delete;
        VFDController(const VFDController &) = delete;
        VFDController operator=(const VFDController &) = delete;

    public:
        static void configuration();
        static void driver_signal_configuration();
        static void pwm_configuration();
        static void encoder_configuration();
        static void adc_configuration();
        static void button_configuration();

        friend void ::TIM1_UP_TIM10_IRQHandler();
        friend void ::EXTI0_IRQHandler();
        friend void ::EXTI1_IRQHandler();
        friend void ::EXTI2_IRQHandler();
        friend void ::DMA2_Stream0_IRQHandler();
        friend void ::EXTI15_10_IRQHandler();
        friend void ::EXTI9_5_IRQHandler();
    public:
        /**
         * @brief Конвертер желаемой частоты(ЖЧ), снимаемой с потенциометра, в ARR таймера
         * @param counter
         * @return uint8_t
         */

        static inline constexpr uint16_t cnt_to_psc(const uint16_t counter)
        {

            if (counter > 100)
                return cnt_to_psc(100);
            if (counter < 5)
                return cnt_to_psc(5);
            return static_cast<float>(F_CPU) / ((ARR_value - 1) * array_size * counter);
        }

        static void set_frequency(const uint16_t &);
        static uint8_t get_frequency();
        static void update_frequency();


        /**
         * @brief Подает на ITRIP низкий уровень
         *
         */
        static inline void start_driver()
        {
            _ITRIP_PIN_BASE->BSRR = GPIO_BSRR_BR5_Msk;
        }

        /**
         * @brief Подает на ITRIP высокий уровень
         *
         */
        static inline void stop_driver()
        {
            _ITRIP_PIN_BASE->BSRR = GPIO_BSRR_BS5_Msk;
        }

        /**
         * @brief Изменяет состояние драйвера (ITRIP Pin)
         *
         */
        static inline void toggle_driver()
        {
            _ITRIP_PIN_BASE->ODR ^= GPIO_ODR_OD5_Msk;
        }

        /**
         * @brief Позволяет получить состояние драйвера (ITRIP Pin)
         *
         * @return true Если драйвер включен
         * @return false Если драйвер выключен
         */
        static inline bool get_driver_condition()
        {
            return !(_ITRIP_PIN_BASE->IDR & GPIO_IDR_ID5);
        }
    };

};