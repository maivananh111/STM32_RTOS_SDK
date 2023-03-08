/*
 * usb_driver_def.h
 *
 *  Created on: Mar 7, 2023
 *      Author: anh
 */

#ifndef USB_DRIVER_DEF_H_
#define USB_DRIVER_DEF_H_

#include "periph_en.h"
#ifdef ENABLE_USB


#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx.h"


#ifndef USBD_MAX_NUM_INTERFACES
#define USBD_MAX_NUM_INTERFACES                         1U
#endif /* USBD_MAX_NUM_CONFIGURATION */

#ifndef USBD_MAX_NUM_CONFIGURATION
#define USBD_MAX_NUM_CONFIGURATION                      1U
#endif /* USBD_MAX_NUM_CONFIGURATION */

#ifndef USBD_MAX_SUPPORTED_CLASS
#define USBD_MAX_SUPPORTED_CLASS                       1U
#endif /* USBD_MAX_SUPPORTED_CLASS */

#ifndef USBD_MAX_CLASS_ENDPOINTS
#define USBD_MAX_CLASS_ENDPOINTS                       5U
#endif /* USBD_MAX_CLASS_ENDPOINTS */

#ifndef USBD_MAX_CLASS_INTERFACES
#define USBD_MAX_CLASS_INTERFACES                      5U
#endif /* USBD_MAX_CLASS_INTERFACES */

#ifndef USBD_LPM_ENABLED
#define USBD_LPM_ENABLED                                0U
#endif /* USBD_LPM_ENABLED */

#ifndef USBD_SELF_POWERED
#define USBD_SELF_POWERED                               1U
#endif /*USBD_SELF_POWERED */

#ifndef USBD_MAX_POWER
#define USBD_MAX_POWER                                  0x32U /* 100 mA */
#endif /* USBD_MAX_POWER */

#ifndef USBD_SUPPORT_USER_STRING_DESC
#define USBD_SUPPORT_USER_STRING_DESC                   0U
#endif /* USBD_SUPPORT_USER_STRING_DESC */

#ifndef USBD_CLASS_USER_STRING_DESC
#define USBD_CLASS_USER_STRING_DESC                     0U
#endif /* USBD_CLASS_USER_STRING_DESC */

#define  USB_LEN_DEV_QUALIFIER_DESC                     0x0AU
#define  USB_LEN_DEV_DESC                               0x12U
#define  USB_LEN_CFG_DESC                               0x09U
#define  USB_LEN_IF_DESC                                0x09U
#define  USB_LEN_EP_DESC                                0x07U
#define  USB_LEN_OTG_DESC                               0x03U
#define  USB_LEN_LANGID_STR_DESC                        0x04U
#define  USB_LEN_OTHER_SPEED_DESC_SIZ                   0x09U

#define  USBD_IDX_LANGID_STR                            0x00U
#define  USBD_IDX_MFC_STR                               0x01U
#define  USBD_IDX_PRODUCT_STR                           0x02U
#define  USBD_IDX_SERIAL_STR                            0x03U
#define  USBD_IDX_CONFIG_STR                            0x04U
#define  USBD_IDX_INTERFACE_STR                         0x05U

#define  USB_REQ_TYPE_STANDARD                          0x00U
#define  USB_REQ_TYPE_CLASS                             0x20U
#define  USB_REQ_TYPE_VENDOR                            0x40U
#define  USB_REQ_TYPE_MASK                              0x60U

#define  USB_REQ_RECIPIENT_DEVICE                       0x00U
#define  USB_REQ_RECIPIENT_INTERFACE                    0x01U
#define  USB_REQ_RECIPIENT_ENDPOINT                     0x02U
#define  USB_REQ_RECIPIENT_MASK                         0x03U

#define  USB_REQ_GET_STATUS                             0x00U
#define  USB_REQ_CLEAR_FEATURE                          0x01U
#define  USB_REQ_SET_FEATURE                            0x03U
#define  USB_REQ_SET_ADDRESS                            0x05U
#define  USB_REQ_GET_DESCRIPTOR                         0x06U
#define  USB_REQ_SET_DESCRIPTOR                         0x07U
#define  USB_REQ_GET_CONFIGURATION                      0x08U
#define  USB_REQ_SET_CONFIGURATION                      0x09U
#define  USB_REQ_GET_INTERFACE                          0x0AU
#define  USB_REQ_SET_INTERFACE                          0x0BU
#define  USB_REQ_SYNCH_FRAME                            0x0CU

#define  USB_DESC_TYPE_DEVICE                           0x01U
#define  USB_DESC_TYPE_CONFIGURATION                    0x02U
#define  USB_DESC_TYPE_STRING                           0x03U
#define  USB_DESC_TYPE_INTERFACE                        0x04U
#define  USB_DESC_TYPE_ENDPOINT                         0x05U
#define  USB_DESC_TYPE_DEVICE_QUALIFIER                 0x06U
#define  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION        0x07U
#define  USB_DESC_TYPE_IAD                              0x0BU
#define  USB_DESC_TYPE_BOS                              0x0FU

#define USB_CONFIG_REMOTE_WAKEUP                        0x02U
#define USB_CONFIG_SELF_POWERED                         0x01U

#define USB_FEATURE_EP_HALT                             0x00U
#define USB_FEATURE_REMOTE_WAKEUP                       0x01U
#define USB_FEATURE_TEST_MODE                           0x02U

#define USB_DEVICE_CAPABITY_TYPE                        0x10U

#define USB_CONF_DESC_SIZE                              0x09U
#define USB_IF_DESC_SIZE                                0x09U
#define USB_EP_DESC_SIZE                                0x07U
#define USB_IAD_DESC_SIZE                               0x08U

#define USB_HS_MAX_PACKET_SIZE                          512U
#define USB_FS_MAX_PACKET_SIZE                          64U
#define USB_MAX_EP0_SIZE                                64U

/*  Device Status */
#define USBD_STATE_DEFAULT                              0x01U
#define USBD_STATE_ADDRESSED                            0x02U
#define USBD_STATE_CONFIGURED                           0x03U
#define USBD_STATE_SUSPENDED                            0x04U


/*  EP0 State */
#define USBD_EP0_IDLE                                   0x00U
#define USBD_EP0_SETUP                                  0x01U
#define USBD_EP0_DATA_IN                                0x02U
#define USBD_EP0_DATA_OUT                               0x03U
#define USBD_EP0_STATUS_IN                              0x04U
#define USBD_EP0_STATUS_OUT                             0x05U
#define USBD_EP0_STALL                                  0x06U

#define USBD_EP_TYPE_CTRL                               0x00U
#define USBD_EP_TYPE_ISOC                               0x01U
#define USBD_EP_TYPE_BULK                               0x02U
#define USBD_EP_TYPE_INTR                               0x03U



typedef struct {
	uint8_t   bmRequest;
	uint8_t   bRequest;
	uint16_t  wValue;
	uint16_t  wIndex;
	uint16_t  wLength;
} usb_dev_setup_req_t;

typedef struct{
	uint8_t   bLength;
	uint8_t   bDescriptorType;
	uint16_t  wTotalLength;
	uint8_t   bNumInterfaces;
	uint8_t   bConfigurationValue;
	uint8_t   iConfiguration;
	uint8_t   bmAttributes;
	uint8_t   bMaxPower;
} __PACKED usb_dev_config_desc_t;

typedef struct{
	uint8_t   bLength;
	uint8_t   bDescriptorType;
	uint16_t  wTotalLength;
	uint8_t   bNumDeviceCaps;
} usb_dev_bos_desc_t;

typedef struct{
	uint8_t   bLength;
	uint8_t   bDescriptorType;
	uint8_t   bEndpointAddress;
	uint8_t   bmAttributes;
	uint16_t  wMaxPacketSize;
	uint8_t   bInterval;
} __PACKED usb_dev_ep_desc_t;

typedef  struct{
	uint8_t  bLength;
	uint8_t  bDescriptorType;
	uint8_t  bDescriptorSubType;
} usb_dev_desc_header_t;

struct usb_dev_handler_t;

typedef struct _Device_cb{
	uint8_t (*Init)(struct usb_dev_handler_t *dev, uint8_t cfgidx);
	uint8_t (*DeInit)(struct usb_dev_handler_t *dev, uint8_t cfgidx);
	/* Control Endpoints*/
	uint8_t (*Setup)(struct usb_dev_handler_t *dev, usb_dev_setup_req_t  *req);
	uint8_t (*EP0_TxSent)(struct usb_dev_handler_t *dev);
	uint8_t (*EP0_RxReady)(struct usb_dev_handler_t *dev);
	/* Class Specific Endpoints*/
	uint8_t (*DataIn)(struct usb_dev_handler_t *dev, uint8_t epnum);
	uint8_t (*DataOut)(struct usb_dev_handler_t *dev, uint8_t epnum);
	uint8_t (*SOF)(struct usb_dev_handler_t *dev);
	uint8_t (*IsoINIncomplete)(struct usb_dev_handler_t *dev, uint8_t epnum);
	uint8_t (*IsoOUTIncomplete)(struct usb_dev_handler_t *dev, uint8_t epnum);

	uint8_t  *(*GetHSConfigDescriptor)(uint16_t *length);
	uint8_t  *(*GetFSConfigDescriptor)(uint16_t *length);
	uint8_t  *(*GetOtherSpeedConfigDescriptor)(uint16_t *length);
	uint8_t  *(*GetDeviceQualifierDescriptor)(uint16_t *length);
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  uint8_t  *(*GetUsrStrDescriptor)(struct _USBD_HandleTypeDef *pdev, uint8_t index,  uint16_t *length);
#endif /* USBD_SUPPORT_USER_STRING_DESC  */

} usb_dev_class_t; // USBD_ClassTypeDef

typedef enum{
	USBD_SPEED_HIGH  = 0U,
	USBD_SPEED_FULL  = 1U,
	USBD_SPEED_LOW   = 2U,
} usb_dev_speed_t;

typedef enum{
	USBD_OK = 0U,
	USBD_BUSY,
	USBD_EMEM,
	USBD_FAIL,
} usb_dev_status_t;

typedef struct{
	uint8_t *(*GetDeviceDescriptor)(usb_dev_speed_t speed, uint16_t *length);
	uint8_t *(*GetLangIDStrDescriptor)(usb_dev_speed_t speed, uint16_t *length);
	uint8_t *(*GetManufacturerStrDescriptor)(usb_dev_speed_t speed, uint16_t *length);
	uint8_t *(*GetProductStrDescriptor)(usb_dev_speed_t speed, uint16_t *length);
	uint8_t *(*GetSerialStrDescriptor)(usb_dev_speed_t speed, uint16_t *length);
	uint8_t *(*GetConfigurationStrDescriptor)(usb_dev_speed_t speed, uint16_t *length);
	uint8_t *(*GetInterfaceStrDescriptor)(usb_dev_speed_t speed, uint16_t *length);
#if (USBD_CLASS_USER_STRING_DESC == 1)
  uint8_t *(*GetUserStrDescriptor)(usb_dev_speed_t speed, uint8_t idx, uint16_t *length);
#endif /* USBD_CLASS_USER_STRING_DESC */
#if ((USBD_LPM_ENABLED == 1U) || (USBD_CLASS_BOS_ENABLED == 1))
  uint8_t *(*GetBOSDescriptor)(usb_dev_speed_t speed, uint16_t *length);
#endif /* (USBD_LPM_ENABLED == 1U) || (USBD_CLASS_BOS_ENABLED == 1) */
} usb_dev_descriptor_t; // USBD_DescriptorsTypeDef

typedef struct{
	uint32_t status;
	uint32_t total_length;
	uint32_t rem_length;
	uint32_t maxpacket;
	uint16_t is_used;
	uint16_t bInterval;
} usb_dev_ep_t; // USBD_EndpointTypeDef


typedef struct usb_dev_handler_t{
	uint8_t                 id;
	uint32_t                dev_config;
	uint32_t                dev_default_config;
	uint32_t                dev_config_status;
	usb_dev_speed_t         dev_speed;
	usb_dev_ep_t            ep_in[16];
	usb_dev_ep_t            ep_out[16];
	__IO uint32_t           ep0_state;
	uint32_t                ep0_data_len;
	__IO uint8_t            dev_state;
	__IO uint8_t            dev_old_state;
	uint8_t                 dev_address;
	uint8_t                 dev_connection_status;
	uint8_t                 dev_test_mode;
	uint32_t                dev_remote_wakeup;
	uint8_t                 ConfIdx;

	usb_dev_setup_req_t     request;
	usb_dev_descriptor_t    *pDesc;
	usb_dev_class_t         *pClass[USBD_MAX_SUPPORTED_CLASS];
	void                    *pClassData;
	void                    *pClassDataCmsit[USBD_MAX_SUPPORTED_CLASS];
	void                    *pUserData[USBD_MAX_SUPPORTED_CLASS];
	void                    *pData;
	void                    *pBosDesc;
	void                    *pConfDesc;
	uint32_t                classId;
	uint32_t                NumClasses;
} usb_dev_handler_t; // USBD_HandleTypeDef

typedef enum{
	OUT   = 0x00,
	IN    = 0x80,
} usb_dev_ep_dir_t;

typedef enum{
	USBD_CDC_NETWORK_CONNECTION = 0x00,
	USBD_CDC_RESPONSE_AVAILABLE = 0x01,
	USBD_CDC_CONNECTION_SPEED_CHANGE = 0x2A
} usb_dev_cdc_notify_code_t;

__STATIC_INLINE uint16_t SWAPBYTE(uint8_t *addr){
	uint16_t _SwapVal, _Byte1, _Byte2;
	uint8_t *_pbuff = addr;

	_Byte1 = *(uint8_t *)_pbuff;
	_pbuff++;
	_Byte2 = *(uint8_t *)_pbuff;

	_SwapVal = (_Byte2 << 8) | _Byte1;

	return _SwapVal;
}

#ifndef LOBYTE
#define LOBYTE(x)  ((uint8_t)((x) & 0x00FFU))
#endif /* LOBYTE */

#ifndef HIBYTE
#define HIBYTE(x)  ((uint8_t)(((x) & 0xFF00U) >> 8U))
#endif /* HIBYTE */

#ifndef MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#endif /* MIN */

#ifndef MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#endif /* MAX */

#if  defined ( __GNUC__ )
#ifndef __weak
#define __weak   __attribute__((weak))
#endif /* __weak */
#ifndef __packed
#define __packed __attribute__((__packed__))
#endif /* __packed */
#endif /* __GNUC__ */

#if defined ( __GNUC__ ) && !defined (__CC_ARM) /* GNU Compiler */
#ifndef __ALIGN_END
#define __ALIGN_END    __attribute__ ((aligned (4U)))
#endif /* __ALIGN_END */
#ifndef __ALIGN_BEGIN
#define __ALIGN_BEGIN
#endif /* __ALIGN_BEGIN */
#endif /* ( __GNUC__ ) && !defined (__CC_ARM) */




#ifdef __cplusplus
}
#endif

#endif /* ENABLE_USB */

#endif /* USB_DRIVER_DEF_H_ */
