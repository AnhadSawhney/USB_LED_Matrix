#include "main.h"
#include "led.h"
#include "systick.h"
#include "init.h"

/*void initClock(void) {
    // 1. Enable clock to PWR in APB1
    // 2. Set up the HSE and PLL
    // 3. Enable overdrive and wait for it to be enabled
    // 4. Adjust the flash latency
    // 5. Set AHB/APB1/APB2 prescalers
    // 6. Wait for PLL lock
    // 7. Select PLL as sysclock
    // 8. Call SystemCoreClockUpdate();

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // Enable the HSE in bypass mode (there is a 8MHz signal coming from the ST-LINK)
    RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    //Set the main PLL M, N, P, Q, R, and HSE as the input
    //NUCLEO_STM32F446: M = 4, N = 180, P = 2 = 180MHz SYSCLK, Q = 8, R = 4 (Q & R not used)
    //RCC->PLLCFGR = RCC_PLLCFGR_PLLM_2 | (180 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) | RCC_PLLCFGR_PLLQ_3 | RCC_PLLCFGR_PLLR_2 | RCC_PLLCFGR_PLLSRC_HSE;
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ; //zero out pllq
    RCC->PLLCFGR |= 7 << RCC_PLLCFGR_PLLQ_Pos; // put 7 in pllq
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP; // zero out pllp, pllp = 0 is div/2
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN; //zero out plln
    RCC->PLLCFGR |= 168 << RCC_PLLCFGR_PLLN_Pos; // PLL-N: x168
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM; //zero out pllm
    RCC->PLLCFGR |= 4 << RCC_PLLCFGR_PLLM_Pos; // PLL-M: /4
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE; // PLL source is HSE
    RCC->CR |= RCC_CR_PLLON; // Activate the PLL (Output: 168 MHz)

    
    // enable and switch on overdrive (enables 180Mhz clock on STM32F446)
    //PWR->CR |= PWR_CR_ODEN;
    //while(!(PWR->CSR & PWR_CSR_ODRDY));

    //PWR->CR |= PWR_CR_ODSWEN;
    //while(!(PWR->CSR & PWR_CSR_ODSWRDY));

    // Set the flash wait time
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
    if((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_5WS) {
        _error_handler();
    }

    // set APB prescalers
    // APB1 = 4, APB2 = 2
    //NUCLEO_STM32F446: 45MHz and 90Mhz
    //STM32F4DISCOVERY: 42Mhz and 84Mhz
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 // AHB divider: /1 (168 MHz)
        | RCC_CFGR_PPRE1_DIV4 // APB1 divider: /4 (42 MHz)
        | RCC_CFGR_PPRE2_DIV2; // APB2 divider: /2 (84 MHz)

    // wait for PLL lock
    while(!(RCC->CR & RCC_CR_PLLRDY));

    // Switch SYSCLK to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    SystemCoreClockUpdate();
    
    // set TIMPRE so the timers run at 180MHz
    // not available on STM32F407 so prescalers must be adjusted instead. Refresh rate is 2x slower
    //RCC->DCKCFGR |= RCC_DCKCFGR_TIMPRE;
}*/

void initGPIO(void) {
    /*RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;*/

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    //gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);
    //gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO5);
    
    /*GPIOA->MODER |= (0x1U << GPIO_MODER_MODE5_Pos);
    GPIOA->OTYPER |= (0x0U << GPIO_OTYPER_OT5_Pos);
    GPIOA->OSPEEDR |= (0x0U << GPIO_OSPEEDR_OSPEED5_Pos);
    GPIOA->PUPDR |= (0x0U << GPIO_PUPDR_PUPD5_Pos);*/

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4);

    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0 | GPIO1 | GPIO2 | GPIO3 | GPIO4 | GPIO5);
    
    // PB0-4 are A,B,C,D,E pins
    /*GPIOB->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4); 
    GPIOB->MODER |= GPIO_MODER_MODE0_0 |
                    GPIO_MODER_MODE1_0 |
                    GPIO_MODER_MODE2_0 |
                    GPIO_MODER_MODE3_0 |
                    GPIO_MODER_MODE4_0;

    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED0_0 | GPIO_OSPEEDR_OSPEED0_1 |
                      GPIO_OSPEEDR_OSPEED1_0 | GPIO_OSPEEDR_OSPEED1_1 |
                      GPIO_OSPEEDR_OSPEED2_0 | GPIO_OSPEEDR_OSPEED2_1 |
                      GPIO_OSPEEDR_OSPEED3_0 | GPIO_OSPEEDR_OSPEED3_1 |
                      GPIO_OSPEEDR_OSPEED4_0 | GPIO_OSPEEDR_OSPEED4_1; */

    
 
    // PC0-PC5 are RGB pins
    /*GPIOC->MODER |= GPIO_MODER_MODE0_0 |
                    GPIO_MODER_MODE1_0 |
                    GPIO_MODER_MODE2_0 |
                    GPIO_MODER_MODE3_0 |
                    GPIO_MODER_MODE4_0 |
                    GPIO_MODER_MODE5_0;

    GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED0_0 | GPIO_OSPEEDR_OSPEED0_1 |
                      GPIO_OSPEEDR_OSPEED1_0 | GPIO_OSPEEDR_OSPEED1_1 |
                      GPIO_OSPEEDR_OSPEED2_0 | GPIO_OSPEEDR_OSPEED2_1 |
                      GPIO_OSPEEDR_OSPEED3_0 | GPIO_OSPEEDR_OSPEED3_1 |
                      GPIO_OSPEEDR_OSPEED4_0 | GPIO_OSPEEDR_OSPEED4_1 |
                      GPIO_OSPEEDR_OSPEED5_0 | GPIO_OSPEEDR_OSPEED5_1; */
}

void initDMA(void) {
    // Channel 7
    // Double Buffer
    // Very high priority
    // Memory Inc
    // 1byte-1byte transfers
    // Memory to Peripheral

    /*RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    
    DMA2_Stream2->CR |= (7<<DMA_SxCR_CHSEL_Pos) | DMA_SxCR_DBM | DMA_SxCR_PL_0 | DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
    DMA2_Stream2->NDTR = 0x4000; // 64 col, 8 Bits per colour channel, 32 rows
    DMA2_Stream2->PAR = (uint32_t)&(GPIOC->ODR);
    DMA2_Stream2->M0AR = (uint32_t)buffer1;
    DMA2_Stream2->M1AR = (uint32_t)buffer2;
    DMA2_Stream2->FCR |= DMA_SxFCR_DMDIS; // turn on the FIFO
    DMA2_Stream2->CR |= DMA_SxCR_TCIE; // enable the transfer complete interrupt
    DMA2->LIFCR |= DMA_LIFCR_CTCIF2; // make sure the interrupt flag is clear

    NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    DMA2_Stream2->CR |= DMA_SxCR_EN; // enable the stream
    */
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

    dma_set_number_of_data(DMA2, DMA_STREAM2, 0x4000);

    dma_set_peripheral_address(DMA2, DMA_STREAM2, (uint32_t) &GPIOC_ODR);
    dma_set_memory_address(DMA2, DMA_STREAM2, (uint32_t) buffer1);
    dma_set_memory_address_1(DMA2, DMA_STREAM2, (uint32_t) buffer2);

    dma_enable_fifo_mode(DMA2, DMA_STREAM2);
    dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM2); //clears TCIF

    dma_enable_stream(DMA2, DMA_STREAM2);
}

void initTimers(void) {
    // Timer 8, running 9Mhz 50% duty cycle
    // CLK on CH1, output to PC6
    //RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  //already enabled in GPIO init

    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO6);
    gpio_set_af(GPIOC, GPIO_AF3, GPIO6);
    
    /*GPIOC->MODER |= GPIO_MODER_MODE6_1; // PC6 in AF mode
    GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED6_0 | GPIO_OSPEEDR_OSPEED6_1; // high speed
    GPIOC->AFR[0] |= (0x3U << GPIO_AFRL_AFSEL6_Pos); // AF3 for PC6 is TIM8_CH1*/

    //RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    rcc_periph_clock_enable(RCC_TIM8);
    
    //TIM8->ARR = 19;

    timer_set_period(TIM8, 19); 

    //TIM8->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; //PWM mode 1

    timer_set_oc_mode(TIM8, TIM_OC1, TIM_OCM_PWM1);
    
    //TIM8->DIER |= TIM_DIER_CC1DE; // DMA request on CH1 (falling edge of clock)

    timer_enable_irq(TIM8, TIM_DIER_CC1DE);

    //TIM8->CCR1 = 10;
    //TIM8->CCER |= TIM_CCER_CC1E; // OC1 enabled

    timer_set_oc_value(TIM8, TIM_OC1, 10);
    timer_enable_oc_output(TIM8, TIM_OC1);

    //TIM8->BDTR |= TIM_BDTR_MOE; // Main output enabled.. if this isn't set, there is no output on the pins!

    timer_enable_break_main_output(TIM8);

    timer_set_prescaler(TIM8, (PRESCALE+1)*2-1);
    //TIM8->PSC = (PRESCALE+1)*2-1; // scale down for testing
    //TIM8->EGR |= TIM_EGR_UG; // trigger a UEV to update the preload, auto-reload, and capture-compare shadow registers

    timer_generate_event(TIM8, TIM_EGR_UG);

    //TIM8->SR = 0;

    timer_clear_flag(TIM8, 0x1FFF); //clear all flags

    //TIM8->CR2 |= TIM_CR2_MMS_1; // Master mode - update
    
    timer_set_master_mode(TIM8, TIM_CR2_MMS_UPDATE);

    // Timer 5
    // LAT on CH1, output to PA0
    // OE on CH2, output to PA1 

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0 | GPIO1);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0 | GPIO1);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO0 | GPIO1);

    /*GPIOA->MODER |= GPIO_MODER_MODE0_1; // PA0 in AF mode
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0_0 | GPIO_OSPEEDR_OSPEED0_1; // high speed
    GPIOA->AFR[0] |= (0x2U << GPIO_AFRL_AFSEL0_Pos); // AF2 for PA0 is TIM5_CH1

    GPIOA->MODER |= GPIO_MODER_MODE1_1; // PA1 in AF mode
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED1_0 | GPIO_OSPEEDR_OSPEED1_1; // high speed
    GPIOA->AFR[0] |= (0x2U << GPIO_AFRL_AFSEL1_Pos); // AF2 for PA1 is TIM5_CH2*/

    //RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    rcc_periph_clock_enable(RCC_TIM5);
    
    //TIM5->SMCR |= TIM_SMCR_TS_1 | TIM_SMCR_TS_0; // select ITR3 as the trigger input (for TIM5, this is TIM8) 
    //TIM5->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1; // slave mode set to trigger mode
    //TIM5->ARR = 1279;

    timer_slave_set_trigger(TIM5, TIM_SMCR_TS_ITR3);
    timer_slave_set_mode(TIM5, TIM_SMCR_SMS_TM);
    timer_set_period(TIM5, 1279);

    //TIM5->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; //PWM mode 1
    //TIM5->CCR1 = 12;

    timer_set_oc_mode(TIM5, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_value(TIM5, TIM_OC1, 12);
    timer_enable_oc_output(TIM5, TIM_OC1);

    //TIM5->CCER |= TIM_CCER_CC1E; // OC1 enabled

    timer_set_oc_mode(TIM5, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_value(TIM5, TIM_OC2, 1280);
    timer_enable_oc_preload(TIM5, TIM_OC2);
    timer_enable_oc_output(TIM5, TIM_OC2);


    //TIM5->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; //PWM mode 1
    //TIM5->CCMR1 |= TIM_CCMR1_OC2PE; // preload enabled for CCR2
    //TIM5->CCR2 = 1280; // start with OC2 (OE) held at 1 (off)
    //TIM5->CCER |= TIM_CCER_CC2E; // OC2 enabled

    timer_set_prescaler(TIM5, PRESCALE);

    //TIM5->PSC = PRESCALE; // scale down for testing
    //TIM5->EGR |= TIM_EGR_UG; // trigger a UEV to update the preload, auto-reload, and capture-compare shadow registers

    timer_generate_event(TIM5, TIM_EGR_UG);
    //TIM5->SR = 0; // clear the status flags
    
    timer_clear_flag(TIM5, 0x1FFF); //clear all flags

    //TIM5->DIER |= TIM_DIER_UIE; // enable TIM5 interrupt on UEV

    timer_enable_irq(TIM5, TIM_DIER_UIE);

    //TIM5->CNT = 15;

    timer_set_counter(TIM5, 15);

    //NVIC_EnableIRQ(TIM5_IRQn);

    nvic_enable_irq(NVIC_TIM5_IRQ);

    // Start the timer
    //TIM8->CR1 |= TIM_CR1_CEN;

    timer_enable_counter(TIM8);

    //TIM5->CCR2 = 1280 - BRIGHTNESS; // put the first value in CCR2 preload it will be loaded at the first UEV

    timer_set_oc_value(TIM5, TIM_OC2, 1280 - BRIGHTNESS);
}

void init(void) {
    // enable the data and instruction cache and prefetch
    //FLASH->ACR |= FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN;
    //flash_dcache_enable();
    //flash_icache_enable();
    //flash_prefetch_enable();

    // This will set the clock to 168MHz
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    SysTick_Init();

    // Set up any input/output pins
    initGPIO();
    initDMA();

    //DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM5_STOP;
    //MMIO32(DBGMCU_BASE + 0x08) |= 0x8;
    //DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM8_STOP; 
    //MMIO32(DBGMCU_BASE + 0x0C) |= 0x2;

    initTimers();
    busyFlag = 1;
}