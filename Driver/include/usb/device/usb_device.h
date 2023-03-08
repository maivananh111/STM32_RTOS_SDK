/*
 * usb_device.h
 *
 *  Created on: Mar 7, 2023
 *      Author: anh
 */

#ifndef USB_DEVICE_H_
#define USB_DEVICE_H_

#include "periph_en.h"
#ifdef ENABLE_USB


#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx.h"
#include "usb/usb_driver.h"
#include "usb/device/usb_device_desc.h"


return_t usb_dev_init(usb_dev_handler_t *dev, usb_dev_descriptor_t *desc, uint8_t id);

return_t usb_dev_register_class(usb_dev_handler_t *dev, usb_dev_class_t *devclass);

return_t usb_device_init(void); //MX_USB_DEVICE_Init






#ifdef __cplusplus
}
#endif

#endif /* ENABLE_USB */


#endif /* USB_DEVICE_H_ */
