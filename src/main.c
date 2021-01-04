// STM32F4Discovery
#define HSE_VALUE ((uint32_t)8000000)
#include "main.h"
#include "systick.h"
#include "led.h"
#include "init.h"
#include "cdc_loop.h"

uint32_t c;
uint8_t *nextBuffer;

int main(void) {
    init();
    uint32_t current_buffer, start_time, c;
    c = 0;
    while(1) {
        start_time = millis();
        while(busyFlag);
        busyFlag = 1;
        current_buffer = DMA2_Stream2->CR | DMA_SxCR_CT;
        nextBuffer = current_buffer ? buffer2 : buffer1;
        while(busyFlag);
        busyFlag = 1;
        LED_fillBuffer(frame, nextBuffer);
        LED_waveEffect(frame);
        //LED_gradient(frame);
        //LED_Lines(frame, c);
        c++;
        c %= WIDTH;
        //while(millis() - start_time < 100);
    }
    return 0;
}