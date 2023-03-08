/*
 * usb_device_cdc.h
 *
 *  Created on: Mar 7, 2023
 *      Author: anh
 */

#ifndef USB_DEVICE_CDC_H_
#define USB_DEVICE_CDC_H_

#include "periph_en.h"
#ifdef ENABLE_USB


#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx.h"
#include "usb/device/usb_device_cdc.h"
#include "usb/usb_driver_def.h"


#ifndef CDC_IN_EP
#define CDC_IN_EP                                   0x81U  /* EP1 for data IN */
#endif /* CDC_IN_EP */
#ifndef CDC_OUT_EP
#define CDC_OUT_EP                                  0x01U  /* EP1 for data OUT */
#endif /* CDC_OUT_EP */
#ifndef CDC_CMD_EP
#define CDC_CMD_EP                                  0x82U  /* EP2 for CDC commands */
#endif /* CDC_CMD_EP  */

#ifndef CDC_HS_BINTERVAL
#define CDC_HS_BINTERVAL                            0x10U
#endif /* CDC_HS_BINTERVAL */

#ifndef CDC_FS_BINTERVAL
#define CDC_FS_BINTERVAL                            0x10U
#endif /* CDC_FS_BINTERVAL */

/* CDC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define CDC_DATA_HS_MAX_PACKET_SIZE                 512U  /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE                 64U  /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE                         8U  /* Control Endpoint Packet size */

#define USB_CDC_CONFIG_DESC_SIZ                     67U
#define CDC_DATA_HS_IN_PACKET_SIZE                  CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_OUT_PACKET_SIZE                 CDC_DATA_HS_MAX_PACKET_SIZE

#define CDC_DATA_FS_IN_PACKET_SIZE                  CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE                 CDC_DATA_FS_MAX_PACKET_SIZE

#define CDC_REQ_MAX_DATA_SIZE                       0x7U
/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/
#define CDC_SEND_ENCAPSULATED_COMMAND               0x00U
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01U
#define CDC_SET_COMM_FEATURE                        0x02U
#define CDC_GET_COMM_FEATURE                        0x03U
#define CDC_CLEAR_COMM_FEATURE                      0x04U
#define CDC_SET_LINE_CODING                         0x20U
#define CDC_GET_LINE_CODING                         0x21U
#define CDC_SET_CONTROL_LINE_STATE                  0x22U
#define CDC_SEND_BREAK                              0x23U


typedef struct{
	uint32_t bitrate;
	uint8_t  format;
	uint8_t  paritytype;
	uint8_t  datatype;
} usb_dev_cdc_linecode_t;

typedef struct _usb_dev_cdc_interface_t{
	int8_t (* Init)(void);
	int8_t (* DeInit)(void);
	int8_t (* Control)(uint8_t cmd, uint8_t *pbuf, uint16_t length);
	int8_t (* Receive)(uint8_t *Buf, uint32_t *Len);
	int8_t (* TransmitCplt)(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
} usb_dev_cdc_interface_t;


typedef struct{
	uint32_t data[CDC_DATA_HS_MAX_PACKET_SIZE / 4U];      /* Force 32-bit alignment */
	uint8_t  CmdOpCode;
	uint8_t  CmdLength;
	uint8_t  *RxBuffer;
	uint8_t  *TxBuffer;
	uint32_t RxLength;
	uint32_t TxLength;

	__IO uint32_t TxState;
	__IO uint32_t RxState;
} usb_dev_cdc_handler_t;

extern usb_dev_class_t usb_device_cdc_class; // USBD_ClassTypeDef


uint8_t usb_dev_cdc_register_interface(usb_dev_handler_t *dev, usb_dev_cdc_interface_t *interface);

uint8_t usb_dev_cdc_set_Txbuffer(usb_dev_handler_t *dev, uint8_t *buff, uint32_t length);
uint8_t usb_dev_cdc_set_Rxbuffer(usb_dev_handler_t *dev, uint8_t *buff);

uint8_t usb_dev_cdc_transmit(usb_dev_handler_t *dev);
uint8_t usb_dev_cdc_receive(usb_dev_handler_t *dev);

#ifdef __cplusplus
}
#endif

#endif

#endif /* USB_DEVICE_CDC_H_ */
