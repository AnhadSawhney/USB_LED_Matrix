#include "main.h"

void tim5_isr(void) {
    /* Clear the interrupt flag right away.
     * Due to pipelining, the register itself might not get updated for several
     * cycles. If we wait until the end of the ISR to clear the flag,
     * it can trigger again immediately */
    //TIM5->SR &= ~TIM_SR_UIF_Msk;
    timer_clear_flag(TIM5, TIM_SR_UIF);

    if(bit == 0) { // the first 64 bits of a new row have been latched. Set the row select to match.
        //GPIOB->ODR = (GPIOB->ODR & ~(row_mask)) | row;
        //gpio_clear(GPIOB, row_mask);
        //gpio_set(GPIOB, row);
        gpio_port_write(GPIOB, row);
        row++;
        row &= row_mask;
    }
    bit++;
    bit &= 0x7;
    //TIM5->CCR2 = 1280 - (BRIGHTNESS * (1 << bit)); // set the duty cycle of the NEXT ~OE pulse
    timer_set_oc_value(TIM5, TIM_OC2, 1280 - (BRIGHTNESS * (1 << bit)));
}

void dma2_stream2_isr(void) {
    //DMA2->LIFCR |= DMA_LIFCR_CTCIF2; // make sure the interrupt flag is clear
    dma_clear_interrupt_flags(DMA2, DMA_STREAM2, DMA_TCIF);
    frame_count++;
    busyFlag = 0; // main loop watches this flag to know when to fill up the next buffer
}