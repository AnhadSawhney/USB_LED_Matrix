#include "main.h"
#include "led.h"
#include "cdcacm.h"

static void usbdev_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) {
	(void)ep;

	int len;
	uint8_t buf[64];
	static uint8_t color;
	static uint32_t index;
	len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
	for(uint8_t i = 0; i < len; i++) {
		if(color == 0) frame[index+i].R = buf[i];
		if(color == 1) frame[index+i].G = buf[i];
		if(color == 2) frame[index+i].B = buf[i];
	}
	index += len;
	if(index >= WIDTH*HEIGHT) {
		index = 0;
		color++;
		if(color > 2) {
			color = 0;
		}
	}
}

static void usbdev_set_config(usbd_device *usbd_dev, uint16_t wValue) {
	(void)wValue;

	//IN (to host) endpoints 0x81 to 0x87
	//OUT (to device) endpoints 0x01 to 0x0F

	//endpoint 0x00 reserved for control
	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, usbdev_data_rx_cb); //OUT
	//usbd_ep_setup(usbd_dev, 0x02, USB_ENDPOINT_ATTR_ISOCHRONOUS, 768, NULL); //OUT, 768 bytes = 4 rows, max 1023
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL); //IN 
}

void initUSB(void) { 
	nvic_enable_irq(NVIC_OTG_FS_IRQ);
    rcc_periph_clock_enable(RCC_OTGFS); //GPIOA must be inited before this 
    
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12); // enable USB pins
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12); // Alternate functions are USBDP and USBDM

	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, usbdev_set_config);
}