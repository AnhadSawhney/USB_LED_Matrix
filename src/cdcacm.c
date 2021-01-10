#include "main.h"
#include "led.h"
#include "cdcacm.h"

static void usbdev_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) {
	(void)ep;

	//char buf[WIDTH*HEIGHT*3];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, frame, WIDTH*HEIGHT*3);
}

static void usbdev_set_config(usbd_device *usbd_dev, uint16_t wValue) {
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, usbdev_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);
}

void initUSB(void) { 
    rcc_periph_clock_enable(RCC_OTGFS); //GPIOA must be inited before this 
    
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12); // enable USB pins
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12); // Alternate functions are USBDP and USBDM

	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, usbdev_set_config);
}