#pragma once

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

extern volatile uint32_t ticks;

void SysTick_Init(void);
uint32_t millis(void);
void delay_ms(uint32_t t);