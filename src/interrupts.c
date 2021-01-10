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
        //gpio_port_write(GPIOB, row); // This works for panels that are direct row select. 

        // Certain panels are (ABC Shift + DE direct)
        // The SM5266RowAddressSetter (ABC Shifter + DE direct) sets bits ABC using
        // a 8 bit shifter and DE directly. The panel this works with has 8 SM5266
        // shifters (4 for the top 32 rows and 4 for the bottom 32 rows).
        // DE is used to select the active shifter
        // (rows 1-8/33-40, 9-16/41-48, 17-24/49-56, 25-32/57-64).
        // Rows are enabled by shifting in 8 bits (high bit first) with a high bit
        // enabling that row. This allows up to 8 rows per group to be active at the
        // same time (if they have the same content), but that isn't implemented here.
        // BK, DIN and DCK are the designations on the SM5266P datasheet.
        // BK = Enable Input, DIN = Serial In, DCK = Clock

        //D = PB3, E = PB4
        (row & 1<<3)? gpio_set(GPIOB, GPIO3) : gpio_clear(GPIOB, GPIO3);
        (row & 1<<4)? gpio_set(GPIOB, GPIO4) : gpio_clear(GPIOB, GPIO4);

        gpio_set(GPIOB, GPIO2); // Enable serial input for the shifter, BK = C = PB2
        for (int r = 7; r >= 0; r--) {
            (row % 8 == r) ? gpio_set(GPIOB, GPIO1) : gpio_clear(GPIOB, GPIO1); // DIN = B = PB1
            gpio_set(GPIOB, GPIO0); // DIN = A = PB0
            //gpio_set(GPIOB, GPIO0);  // Longer clock time; tested with Pi3
            gpio_clear(GPIOB, GPIO0);
        }
        gpio_clear(GPIOB, GPIO2); // Disable serial input to keep unwanted bits out of the shifters

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