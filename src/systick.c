#include "systick.h"

volatile uint32_t ticks;

// Call this after setting up the clock
void SysTick_Init(void) {
    // Enable the SysTick interrupt every 1ms
    systick_set_frequency(1000, rcc_ahb_frequency);
	  systick_counter_enable();
	  systick_interrupt_enable();
}

uint32_t millis(void) {
    return ticks;
}

void delay_ms(uint32_t t) {
  uint32_t elapsed;
  uint32_t start = millis();
  do {
    elapsed = millis() - start;
  } while (elapsed < t) ;
}

void sys_tick_handler(void) {
    ticks++;
}