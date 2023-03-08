/*
 * usb_device_desc.h
 *
 *  Created on: Mar 7, 2023
 *      Author: anh
 */

#ifndef USB_DEVICE_DESC_H_
#define USB_DEVICE_DESC_H_

#include "periph_en.h"
#ifdef ENABLE_USB


#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx.h"
#include "usb/usb_driver_def.h"

extern usb_dev_descriptor_t fs_descriptor; // USBD_DescriptorsTypeDef


#ifdef __cplusplus
}
#endif

#endif /* ENABLE_USB */

#endif /* USB_DEVICE_DESC_H_ */
