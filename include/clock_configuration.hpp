/**
 * @file stm32f4_rcc.hpp
 * @author Ivan Rogozhnikov (oldbrowze@gmail.com)
 * @brief Файл для инициализации тактирования контроллеров STM32F4xx
 * @version 0.1
 * @date 2022-08-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include "stm32f4xx.h"

#define HSE_VALUE 25000000UL
#define HSI_VALUE 16000000UL

namespace Clock
{
    //TODO:перенести в systeminit()
    constexpr uint32_t
        PLL_M = 25, //8
        PLL_N = 336, //336
        PLL_P = 4,//2
        PLL_Q = 7; //7
    /**
     * @brief 
     * Установка тактирования от PLL(HSE) с заданными коэффициентами
     * @param _pll_m Коэффициент M
     * @param _pll_n Коэффициент N
     * @param _pll_p Коэффициент P
     * @param _pll_q Коэффициент Q
     */


    void enable_clock(const uint32_t _pll_m = PLL_M,
                        const uint32_t _pll_n = PLL_N,
                        const uint32_t _pll_p = PLL_P,
                        const uint32_t _pll_q = PLL_Q)
    {

        /*
            настройки флешпамяти

            Латентность - 2
        */
        
        FLASH->ACR |= FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_2WS;


        /* Запускаем HSE */
        RCC->CR |= RCC_CR_HSEON;
        while(!(RCC->CR & RCC_CR_HSERDY));

        //RCC->PLLCFGR = 0;

        /* Определяем коэффициенты и устанавливаем источник PLL - HSE */
        RCC->PLLCFGR = (_pll_q << RCC_PLLCFGR_PLLQ_Pos) |
                        ((_pll_p == 2 ? 0b0 : (_pll_p == 4 ? 0b1 : 0b0)) << RCC_PLLCFGR_PLLP_Pos) | /* искл. случай */
                        RCC_PLLCFGR_PLLSRC_HSE |
                        (_pll_n << RCC_PLLCFGR_PLLN_Pos) |
                        (_pll_m << RCC_PLLCFGR_PLLM_Pos);

        /* Активируем PLL */
        RCC->CR |= RCC_CR_PLLON;
        while ((RCC->CR & RCC_CR_PLLRDY) == 0);

        RCC->CFGR |= RCC_CFGR_HPRE_DIV1 |   // делитель шины AHB 
                    RCC_CFGR_PPRE1_DIV2 |   // делитель шины APB1
                    RCC_CFGR_PPRE2_DIV1;    // делитель шины APB2

        /* Переключаемся на PLL */
        RCC->CFGR |= RCC_CFGR_SW_PLL;
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);


        SystemCoreClockUpdate();
    }
}
