#pragma once

#include <bitmacros.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx.h"
#include <usb.h>
#include <usb_cdc.h>
#include <usb_hid.h>
#include <hid_usage_desktop.h>
#include <hid_usage_button.h>

void cdc_init_usbd(void);

usbd_device udev;