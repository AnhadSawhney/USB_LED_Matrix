#include "main.h"
#include "led.h"
#include "systick.h"
#include "init.h"

static void initGPIO(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4);

    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5);
    
    // PB0-4 are A,B,C,D,E pins
    // PC0-PC5 are RGB pins
}

static void initDMA(void) {
    // Channel 7
    // Double Buffer, FIFO
    // Very high priority
    // Memory Increment
    // 1byte-1byte transfers
    // Memory to Peripheral

    rcc_periph_clock_enable(RCC_DMA2);

    nvic_enable_irq(NVIC_DMA2_STREAM2_IRQ);

    dma_stream_reset(DMA2, DMA_STREAM2);

    dma_channel_select(DMA2, DMA_STREAM2, DMA_SxCR_CHSEL_7);
    dma_enable_double_buffer_mode(DMA2, DMA_STREAM2);
	dma_set_priority(DMA2, DMA_STREAM2, DMA_SxCR_PL_VERY_HIGH);
    dma_set_memory_size(DMA2, DMA_STREAM2, DMA_SxCR_MSIZE_8BIT);
	dma_set_peripheral_size(DMA2, DMA_STREAM2, DMA_SxCR_PSIZE_8BIT);
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM2);
    dma_set_transfer_mode(DMA2, DMA_STREAM2, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);

    dma_set_number_of_data(DMA2, DMA_STREAM2, (uint16_t)(WIDTH*BITS_PER_CHANNEL*SCAN_RATE));

    dma_set_peripheral_address(DMA2, DMA_STREAM2, (uint32_t) &GPIOC_ODR);
    dma_set_memory_address(DMA2, DMA_STREAM2, (uint32_t) buffer1);
    dma_set_memory_address_1(DMA2, DMA_STREAM2, (uint32_t) buffer2);

    dma_enable_fifo_mode(DMA2, DMA_STREAM2);
    dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM2); //clears TCIF

    dma_enable_stream(DMA2, DMA_STREAM2);
}

static void initTimers(void) {
    // Timer 8, running 9Mhz 50% duty cycle
    // CLK on CH1, output to PC6
    //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  //already enabled in GPIO init

    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6); // PC6 in AF mode
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO6); // high speed
    gpio_set_af(GPIOC, GPIO_AF3, GPIO6); // AF3 for PC6 is TIM8_CH1

    rcc_periph_clock_enable(RCC_TIM8);

    timer_set_period(TIM8, 19); 
    timer_set_oc_mode(TIM8, TIM_OC1, TIM_OCM_PWM1); //PWM mode 1
    timer_enable_oc_output(TIM8, TIM_OC1); // OC1 enabled

    timer_enable_irq(TIM8, TIM_DIER_CC1DE); // DMA request on CH1 (falling edge of clock)
    timer_set_oc_value(TIM8, TIM_OC1, 10);
    timer_enable_break_main_output(TIM8); // Main output enabled.. if this isn't set, there is no output on the pins!
    timer_set_prescaler(TIM8, (PRESCALE+1)*2-1); // scale down for testing
    timer_generate_event(TIM8, TIM_EGR_UG); // trigger a UEV to update the preload, auto-reload, and capture-compare shadow registers
    timer_clear_flag(TIM8, 0x1FFF); //clear all flags
    timer_set_master_mode(TIM8, TIM_CR2_MMS_UPDATE); // Master mode - update

    // Timer 5
    // LAT on CH1, output to PA0
    // OE on CH2, output to PA1 

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0 | GPIO1); // PA0, PA1 in AF mode
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0 | GPIO1); // high speed
    gpio_set_af(GPIOA, GPIO_AF2, GPIO0 | GPIO1); // AF2 for PA0 is TIM5_CH1, AF2 for PA1 is TIM5_CH2

    rcc_periph_clock_enable(RCC_TIM5); 
    timer_slave_set_trigger(TIM5, TIM_SMCR_TS_ITR3); // select ITR3 as the trigger input (for TIM5, this is TIM8) 
    timer_slave_set_mode(TIM5, TIM_SMCR_SMS_TM); // slave mode set to trigger mode
    timer_set_period(TIM5,  1279); //TODO: GENERALIZE

    timer_set_oc_mode(TIM5, TIM_OC1, TIM_OCM_PWM1); //PWM mode 1
    timer_set_oc_value(TIM5, TIM_OC1, 12);
    timer_enable_oc_output(TIM5, TIM_OC1); // OC1 enabled

    timer_set_oc_mode(TIM5, TIM_OC2, TIM_OCM_PWM1); //PWM mode 1
    timer_set_oc_value(TIM5, TIM_OC2, 1280); // start with OC2 (OE) held at 1 (off) //TODO: GENERALIZE
    timer_enable_oc_preload(TIM5, TIM_OC2); // preload enabled for CCR2
    timer_enable_oc_output(TIM5, TIM_OC2); // OC2 enabled

    timer_set_prescaler(TIM5, PRESCALE); // scale down for testing

    timer_generate_event(TIM5, TIM_EGR_UG); // trigger a UEV to update the preload, auto-reload, and capture-compare shadow registers
    
    timer_clear_flag(TIM5, 0x1FFF); //clear all flags

    timer_enable_irq(TIM5, TIM_DIER_UIE); // enable TIM5 interrupt on UEV

    timer_set_counter(TIM5, 15);

    nvic_enable_irq(NVIC_TIM5_IRQ);

    timer_enable_counter(TIM8);
    
    timer_set_oc_value(TIM5, TIM_OC2, 1280 - BRIGHTNESS); // put the first value in CCR2 preload it will be loaded at the first UEV //TODO: GENERALIZE
}

void init(void) {
    // enable the data and instruction cache and prefetch
    //FLASH->ACR |= FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN;
    //flash_dcache_enable();
    //flash_icache_enable();

    // This will set the clock to 168MHz
    // APB1 = 4, APB2 = 2
    //STM32F4DISCOVERY: 42Mhz and 84Mhz
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    flash_prefetch_enable();

    SysTick_Init();

    // Set up any input/output pins
    initGPIO();
    initDMA();

    //DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM5_STOP;
    //MMIO32(DBGMCU_BASE + 0x08) |= 0x8;
    //DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM8_STOP; 
    //MMIO32(DBGMCU_BASE + 0x0C) |= 0x2;

    initTimers();
    initUSB();
    busyFlag = 1;
}