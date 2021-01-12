# USB_LED_Matrix
 Based on https://github.com/bikefrivolously/led_matrix

Quiangli 64x64 3mm pixel pitch HUB75E led matrix. 
Many 64x64 matricies have ABCDE directly encode row numbers. These ones control a shift register with ABC, and have direct selection with D and E.

## Pin Mapping
STM32F407VG Discovery Board -> 3.3v to 5v level shifters -> LED Matrix

STM32 | HUB75 LED Matrix | Purpose
------------ | ------------- | -------------
PC0 | RED 1 | Pixel data bus
PC1 | BLUE 1 | Pixel data bus
PC2 | GREEN 1 | Pixel data bus
PC3 | RED 2 | Pixel data bus
PC4 | BLUE 2 | Pixel data bus
PC5 | GREEN 2 | Pixel data bus
PB0 | A | Row select shift register data
PB1 | B | Row select shift register clock
PB2 | C | Row select shift register output enable
PB3 | D | Direct row select
PB4 | E | Direct row select
PC6 | CLK | Pixel strobe clock
PA0 | LAT | Latch pixel data
PA1 | OE | Output enable (duty cycle changes for PWM)

Optional / Built in to discovery board:
STM32 | Purpose
------------ | -------------
PD12 | Heartbeat LED
PD13 | Heartbeat LED
PD14 | Heartbeat LED
PD15 | Heartbeat LED
