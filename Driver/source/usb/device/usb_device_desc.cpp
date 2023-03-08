/*
 * usb_device_desc.cpp
 *
 *  Created on: Mar 7, 2023
 *      Author: anh
 */
#include "periph_en.h"
#ifdef ENABLE_USB

#include "usb/device/usb_device_desc.h"



uint8_t * USBD_FS_DeviceDescriptor(usb_dev_speed_t speed, uint16_t *length);
uint8_t * USBD_FS_LangIDStrDescriptor(usb_dev_speed_t speed, uint16_t *length);
uint8_t * USBD_FS_ManufacturerStrDescriptor(usb_dev_speed_t speed, uint16_t *length);
uint8_t * USBD_FS_ProductStrDescriptor(usb_dev_speed_t speed, uint16_t *length);
uint8_t * USBD_FS_SerialStrDescriptor(usb_dev_speed_t speed, uint16_t *length);
uint8_t * USBD_FS_ConfigStrDescriptor(usb_dev_speed_t speed, uint16_t *length);
uint8_t * USBD_FS_InterfaceStrDescriptor(usb_dev_speed_t speed, uint16_t *length);
#if (USBD_LPM_ENABLED == 1)
uint8_t * USBD_FS_USR_BOSDescriptor(usb_dev_speed_t speed, uint16_t *length);
#endif /* (USBD_LPM_ENABLED == 1) */


usb_dev_descriptor_t fs_descriptor ={ //USBD_DescriptorsTypeDef
  USBD_FS_DeviceDescriptor
, USBD_FS_LangIDStrDescriptor
, USBD_FS_ManufacturerStrDescriptor
, USBD_FS_ProductStrDescriptor
, USBD_FS_SerialStrDescriptor
, USBD_FS_ConfigStrDescriptor
, USBD_FS_InterfaceStrDescriptor
#if (USBD_LPM_ENABLED == 1)
, USBD_FS_USR_BOSDescriptor
#endif /* (USBD_LPM_ENABLED == 1) */
};

#endif

