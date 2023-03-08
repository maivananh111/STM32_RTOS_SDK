/*
 * usb_device_cdc.cpp
 *
 *  Created on: Mar 7, 2023
 *      Author: anh
 */
#include "periph_en.h"
#ifdef ENABLE_USB


#include "usb/device/usb_device_cdc.h"
#include "usb/usb_driver_def.h"
#include "usb/usb_driver.h"
#include "cstddef"
#include "string.h"

uint8_t usb_dev_cdc_init(usb_dev_handler_t *dev, uint8_t cfgidx);
uint8_t usb_dev_cdc_deinit(usb_dev_handler_t *dev, uint8_t cfgidx);
uint8_t usb_dev_cdc_setup(usb_dev_handler_t *dev, usb_dev_setup_req_t *req);
uint8_t usb_dev_cdc_datain(usb_dev_handler_t *dev, uint8_t epnum);
uint8_t usb_dev_cdc_dataout(usb_dev_handler_t *dev, uint8_t epnum);
uint8_t usb_dev_cdc_ep0_rxready(usb_dev_handler_t *dev);
#ifndef USE_USBD_COMPOSITE
uint8_t *usb_dev_cdc_get_fsconfigdesc(uint16_t *length);
uint8_t *usb_dev_cdc_get_hsconfigdesc(uint16_t *length);
uint8_t *usb_dev_cdc_get_otherspeed_configdesc(uint16_t *length);
uint8_t *usb_dev_cdc_get_devqualifierdesc(uint16_t *length);
#endif

usb_dev_status_t usb_dev_get_status(status_t status);
void *usb_dev_static_static_malloc(uint32_t size);
void usb_dev_static_free(void *p);

#ifndef USE_USBD_COMPOSITE
__ALIGN_BEGIN static uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END ={
	USB_LEN_DEV_QUALIFIER_DESC,
	USB_DESC_TYPE_DEVICE_QUALIFIER,
	0x00,
	0x02,
	0x00,
	0x00,
	0x00,
	0x40,
	0x01,
	0x00,
};

__ALIGN_BEGIN static uint8_t USBD_CDC_CfgDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END ={
	/* Configuration Descriptor */
	0x09,                                       /* bLength: Configuration Descriptor size */
	USB_DESC_TYPE_CONFIGURATION,                /* bDescriptorType: Configuration */
	USB_CDC_CONFIG_DESC_SIZ,                    /* wTotalLength */
	0x00,
	0x02,                                       /* bNumInterfaces: 2 interfaces */
	0x01,                                       /* bConfigurationValue: Configuration value */
	0x00,                                       /* iConfiguration: Index of string descriptor
												 describing the configuration */
	#if (USBD_SELF_POWERED == 1U)
	0xC0,                                       /* bmAttributes: Bus Powered according to user configuration */
	#else
	0x80,                                       /* bmAttributes: Bus Powered according to user configuration */
	#endif /* USBD_SELF_POWERED */
	USBD_MAX_POWER,                             /* MaxPower (mA) */

	/* Interface Descriptor */
	0x09,                                       /* bLength: Interface Descriptor size */
	USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: Interface */
	/* Interface descriptor type */
	0x00,                                       /* bInterfaceNumber: Number of Interface */
	0x00,                                       /* bAlternateSetting: Alternate setting */
	0x01,                                       /* bNumEndpoints: One endpoint used */
	0x02,                                       /* bInterfaceClass: Communication Interface Class */
	0x02,                                       /* bInterfaceSubClass: Abstract Control Model */
	0x01,                                       /* bInterfaceProtocol: Common AT commands */
	0x00,                                       /* iInterface */

	/* Header Functional Descriptor */
	0x05,                                       /* bLength: Endpoint Descriptor size */
	0x24,                                       /* bDescriptorType: CS_INTERFACE */
	0x00,                                       /* bDescriptorSubtype: Header Func Desc */
	0x10,                                       /* bcdCDC: spec release number */
	0x01,

	/* Call Management Functional Descriptor */
	0x05,                                       /* bFunctionLength */
	0x24,                                       /* bDescriptorType: CS_INTERFACE */
	0x01,                                       /* bDescriptorSubtype: Call Management Func Desc */
	0x00,                                       /* bmCapabilities: D0+D1 */
	0x01,                                       /* bDataInterface */

	/* ACM Functional Descriptor */
	0x04,                                       /* bFunctionLength */
	0x24,                                       /* bDescriptorType: CS_INTERFACE */
	0x02,                                       /* bDescriptorSubtype: Abstract Control Management desc */
	0x02,                                       /* bmCapabilities */

	/* Union Functional Descriptor */
	0x05,                                       /* bFunctionLength */
	0x24,                                       /* bDescriptorType: CS_INTERFACE */
	0x06,                                       /* bDescriptorSubtype: Union func desc */
	0x00,                                       /* bMasterInterface: Communication class interface */
	0x01,                                       /* bSlaveInterface0: Data Class Interface */

	/* Endpoint 2 Descriptor */
	0x07,                                       /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
	CDC_CMD_EP,                                 /* bEndpointAddress */
	0x03,                                       /* bmAttributes: Interrupt */
	LOBYTE(CDC_CMD_PACKET_SIZE),                /* wMaxPacketSize */
	HIBYTE(CDC_CMD_PACKET_SIZE),
	CDC_FS_BINTERVAL,                           /* bInterval */

	/* Data class interface descriptor */
	0x09,                                       /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_INTERFACE,                    /* bDescriptorType: */
	0x01,                                       /* bInterfaceNumber: Number of Interface */
	0x00,                                       /* bAlternateSetting: Alternate setting */
	0x02,                                       /* bNumEndpoints: Two endpoints used */
	0x0A,                                       /* bInterfaceClass: CDC */
	0x00,                                       /* bInterfaceSubClass */
	0x00,                                       /* bInterfaceProtocol */
	0x00,                                       /* iInterface */

	/* Endpoint OUT Descriptor */
	0x07,                                       /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
	CDC_OUT_EP,                                 /* bEndpointAddress */
	0x02,                                       /* bmAttributes: Bulk */
	LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize */
	HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
	0x00,                                       /* bInterval */

	/* Endpoint IN Descriptor */
	0x07,                                       /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT,                     /* bDescriptorType: Endpoint */
	CDC_IN_EP,                                  /* bEndpointAddress */
	0x02,                                       /* bmAttributes: Bulk */
	LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),        /* wMaxPacketSize */
	HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
	0x00                                        /* bInterval */
};
#endif /* USE_USBD_COMPOSITE  */

static uint8_t CDCInEpAdd = CDC_IN_EP;
static uint8_t CDCOutEpAdd = CDC_OUT_EP;
static uint8_t CDCCmdEpAdd = CDC_CMD_EP;



usb_dev_class_t usb_device_cdc_class = { // USBD_ClassTypeDef
	usb_dev_cdc_init,
	usb_dev_cdc_deinit,
	usb_dev_cdc_setup,
	NULL,                 /* EP0_TxSent */
	usb_dev_cdc_ep0_rxready,
	usb_dev_cdc_datain,
	usb_dev_cdc_dataout,
	NULL,
	NULL,
	NULL,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
    usb_dev_cdc_get_hsconfigdesc,
	usb_dev_cdc_get_fsconfigdesc,
	usb_dev_cdc_get_otherspeed_configdesc,
	usb_dev_cdc_get_devqualifierdesc,
#endif
};

void *usb_dev_static_malloc(uint32_t size){
	static uint32_t mem[(sizeof(usb_dev_cdc_handler_t)/4)+1];
	return mem;
}

void usb_dev_static_free(void *p){

}

usb_dev_status_t usb_dev_get_status(status_t status){
	usb_dev_status_t usb_status = USBD_OK;

	switch (status){
		case OKE :
		  usb_status = USBD_OK;
		break;
		case ERR :
		  usb_status = USBD_FAIL;
		break;
		case BUSY :
		  usb_status = USBD_BUSY;
		break;
		case TIMEOUT :
		  usb_status = USBD_FAIL;
		break;
		default :
		  usb_status = USBD_FAIL;
		break;
	}
	return usb_status;
}

uint8_t usb_dev_cdc_init(usb_dev_handler_t *dev, uint8_t cfgidx){
	(void)(cfgidx);
	usb_dev_cdc_handler_t *cdc;

	cdc = (usb_dev_cdc_handler_t *)usb_dev_static_malloc(sizeof(usb_dev_cdc_handler_t));

	if (cdc == NULL){
		dev->pClassDataCmsit[dev->classId] = NULL;
		return (uint8_t)USBD_EMEM;
	}

	memset(cdc, 0, sizeof(usb_dev_cdc_handler_t));

	dev->pClassDataCmsit[dev->classId] = (void *)cdc;
	dev->pClassData = dev->pClassDataCmsit[dev->classId];

#ifdef USE_USBD_COMPOSITE
	/* Get the Endpoints addresses allocated for this class instance */
	CDCInEpAdd  = USBD_CoreGetEPAdd(dev, USBD_EP_IN, USBD_EP_TYPE_BULK);
	CDCOutEpAdd = USBD_CoreGetEPAdd(dev, USBD_EP_OUT, USBD_EP_TYPE_BULK);
	CDCCmdEpAdd = USBD_CoreGetEPAdd(dev, USBD_EP_IN, USBD_EP_TYPE_INTR);
#endif /* USE_USBD_COMPOSITE */

	if (dev->dev_speed == USBD_SPEED_HIGH){
		(void)usb_driver_ep_open(dev, CDCInEpAdd, USBD_EP_TYPE_BULK, CDC_DATA_HS_IN_PACKET_SIZE);

		dev->ep_in[CDCInEpAdd & 0xFU].is_used = 1U;

		(void)USBD_LL_OpenEP(dev, CDCOutEpAdd, USBD_EP_TYPE_BULK, CDC_DATA_HS_OUT_PACKET_SIZE);

		dev->ep_out[CDCOutEpAdd & 0xFU].is_used = 1U;
		dev->ep_in[CDCCmdEpAdd & 0xFU].bInterval = CDC_HS_BINTERVAL;
	}
	else{
		(void)USBD_LL_OpenEP(dev, CDCInEpAdd, USBD_EP_TYPE_BULK, CDC_DATA_FS_IN_PACKET_SIZE);

		dev->ep_in[CDCInEpAdd & 0xFU].is_used = 1U;

		(void)USBD_LL_OpenEP(dev, CDCOutEpAdd, USBD_EP_TYPE_BULK, CDC_DATA_FS_OUT_PACKET_SIZE);

		dev->ep_out[CDCOutEpAdd & 0xFU].is_used = 1U;
		dev->ep_in[CDCCmdEpAdd & 0xFU].bInterval = CDC_FS_BINTERVAL;
	}

	(void)USBD_LL_OpenEP(dev, CDCCmdEpAdd, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
	dev->ep_in[CDCCmdEpAdd & 0xFU].is_used = 1U;

	cdc->RxBuffer = NULL;

	((usb_dev_cdc_interface_t *)dev->pUserData[dev->classId])->Init();

	cdc->TxState = 0U;
	cdc->RxState = 0U;

	if (cdc->RxBuffer == NULL)
		return (uint8_t)USBD_EMEM;

	if (dev->dev_speed == USBD_SPEED_HIGH)
		(void)USBD_LL_PrepareReceive(dev, CDCOutEpAdd, cdc->RxBuffer, CDC_DATA_HS_OUT_PACKET_SIZE);
	else
		(void)USBD_LL_PrepareReceive(dev, CDCOutEpAdd, cdc->RxBuffer, CDC_DATA_FS_OUT_PACKET_SIZE);

	return (uint8_t)USBD_OK;
}

uint8_t usb_dev_cdc_deinit(usb_dev_handler_t *dev, uint8_t cfgidx){
	(void)(cfgidx);

#ifdef USE_USBD_COMPOSITE
	/* Get the Endpoints addresses allocated for this CDC class instance */
	CDCInEpAdd  = USBD_CoreGetEPAdd(dev, USBD_EP_IN, USBD_EP_TYPE_BULK);
	CDCOutEpAdd = USBD_CoreGetEPAdd(dev, USBD_EP_OUT, USBD_EP_TYPE_BULK);
	CDCCmdEpAdd = USBD_CoreGetEPAdd(dev, USBD_EP_IN, USBD_EP_TYPE_INTR);
#endif /* USE_USBD_COMPOSITE */

	(void)USBD_LL_CloseEP(dev, CDCInEpAdd);
	dev->ep_in[CDCInEpAdd & 0xFU].is_used = 0U;

	(void)USBD_LL_CloseEP(dev, CDCOutEpAdd);
	dev->ep_out[CDCOutEpAdd & 0xFU].is_used = 0U;

	(void)USBD_LL_CloseEP(dev, CDCCmdEpAdd);
	dev->ep_in[CDCCmdEpAdd & 0xFU].is_used = 0U;
	dev->ep_in[CDCCmdEpAdd & 0xFU].bInterval = 0U;

	if (dev->pClassDataCmsit[dev->classId] != NULL){
		((usb_dev_cdc_interface_t *)dev->pUserData[dev->classId])->DeInit();
		(void)USBD_free(dev->pClassDataCmsit[dev->classId]);
		dev->pClassDataCmsit[dev->classId] = NULL;
		dev->pClassData = NULL;
	}

	return (uint8_t)USBD_OK;
}

uint8_t usb_dev_cdc_setup(usb_dev_handler_t *dev, usb_dev_setup_req_t *req){
	usb_dev_cdc_handler_t *cdc = (usb_dev_cdc_handler_t *)dev->pClassDataCmsit[dev->classId];
	uint16_t len;
	uint8_t ifalt = 0U;
	uint16_t status_info = 0U;
	usb_dev_status_t ret = USBD_OK;

	if (cdc == NULL)
		return (uint8_t)USBD_FAIL;

	switch (req->bmRequest & USB_REQ_TYPE_MASK){
		case USB_REQ_TYPE_CLASS:{
			if (req->wLength != 0U){
				if ((req->bmRequest & 0x80U) != 0U){
					((usb_dev_cdc_interface_t *)dev->pUserData[dev->classId])->Control(req->bRequest, (uint8_t *)cdc->data, req->wLength);

					len = MIN(CDC_REQ_MAX_DATA_SIZE, req->wLength);
					(void)USBD_CtlSendData(dev, (uint8_t *)cdc->data, len);
				}
				else{
					cdc->CmdOpCode = req->bRequest;
					cdc->CmdLength = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);

					(void)USBD_CtlPrepareRx(dev, (uint8_t *)cdc->data, cdc->CmdLength);
				}
			}
			else{
				((usb_dev_cdc_interface_t *)dev->pUserData[dev->classId])->Control(req->bRequest, (uint8_t *)req, 0U);
			}
		}
		break;

		case USB_REQ_TYPE_STANDARD:{
			switch (req->bRequest){
				case USB_REQ_GET_STATUS:
					if (dev->dev_state == USBD_STATE_CONFIGURED){
						(void)USBD_CtlSendData(dev, (uint8_t *)&status_info, 2U);
					}
					else{
						USBD_CtlError(dev, req);
						ret = USBD_FAIL;
					}
				break;

				case USB_REQ_GET_INTERFACE:
					if (dev->dev_state == USBD_STATE_CONFIGURED){
						(void)USBD_CtlSendData(dev, &ifalt, 1U);
					}
					else{
						USBD_CtlError(dev, req);
						ret = USBD_FAIL;
					}
				break;

				case USB_REQ_SET_INTERFACE:
					if (dev->dev_state != USBD_STATE_CONFIGURED){
						USBD_CtlError(dev, req);
						ret = USBD_FAIL;
					}
				break;

				case USB_REQ_CLEAR_FEATURE:
				break;

				default:
					USBD_CtlError(dev, req);
					ret = USBD_FAIL;
				break;
		    }
		}
		break;

		default:
			USBD_CtlError(dev, req);
			ret = USBD_FAIL;
		break;
	}

	return (uint8_t)ret;
}

uint8_t usb_dev_cdc_datain(usb_dev_handler_t *dev, uint8_t epnum){
	usb_dev_cdc_handler_t *cdc;
	usb_driver_t *drv = (usb_driver_t *)dev->pData;

	if (dev->pClassDataCmsit[dev->classId] == NULL)
		return (uint8_t)USBD_FAIL;

	cdc = (usb_dev_cdc_handler_t *)dev->pClassDataCmsit[dev->classId];

	if ((dev->ep_in[epnum & 0xFU].total_length > 0U) &&
						((dev->ep_in[epnum & 0xFU].total_length % drv->in_ep[epnum & 0xFU].maxpacket) == 0U)){
		dev->ep_in[epnum & 0xFU].total_length = 0U;
		(void)USBD_LL_Transmit(dev, epnum, NULL, 0U);
	}
	else{
		cdc->TxState = 0U;

		if (((usb_dev_cdc_interface_t *)dev->pUserData[dev->classId])->TransmitCplt != NULL){
			((usb_dev_cdc_interface_t *)dev->pUserData[dev->classId])->TransmitCplt(cdc->TxBuffer, &cdc->TxLength, epnum);
		}
	}

	return (uint8_t)USBD_OK;
}

uint8_t usb_dev_cdc_dataout(usb_dev_handler_t *dev, uint8_t epnum){
	usb_dev_cdc_handler_t *cdc = (usb_dev_cdc_handler_t *)dev->pClassDataCmsit[dev->classId];

	if (dev->pClassDataCmsit[dev->classId] == NULL)
		return (uint8_t)USBD_FAIL;

	cdc->RxLength = USBD_LL_GetRxDataSize(dev, epnum);

	((usb_dev_cdc_interface_t *)dev->pUserData[dev->classId])->Receive(cdc->RxBuffer, &cdc->RxLength);

	return (uint8_t)USBD_OK;
}

uint8_t usb_dev_cdc_ep0_rxready(usb_dev_handler_t *dev){
	usb_dev_cdc_handler_t *cdc = (usb_dev_cdc_handler_t *)dev->pClassDataCmsit[dev->classId];

	if (cdc == NULL)
		return (uint8_t)USBD_FAIL;

	if ((dev->pUserData[dev->classId] != NULL) && (cdc->CmdOpCode != 0xFFU)){
		((usb_dev_cdc_interface_t *)dev->pUserData[dev->classId])->Control(cdc->CmdOpCode, (uint8_t *)cdc->data, (uint16_t)cdc->CmdLength);
		cdc->CmdOpCode = 0xFFU;
	}

	return (uint8_t)USBD_OK;
}
#ifndef USE_USBD_COMPOSITE
uint8_t *usb_dev_cdc_get_fsconfigdesc(uint16_t *length){
	usb_dev_ep_desc_t *pEpCmdDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_CMD_EP);
	usb_dev_ep_desc_t *pEpOutDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_OUT_EP);
	usb_dev_ep_desc_t *pEpInDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_IN_EP);

	if (pEpCmdDesc != NULL)
		pEpCmdDesc->bInterval = CDC_FS_BINTERVAL;

	if (pEpOutDesc != NULL)
		pEpOutDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;

	if (pEpInDesc != NULL)
		pEpInDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;

	*length = (uint16_t)sizeof(USBD_CDC_CfgDesc);
	return USBD_CDC_CfgDesc;
}

uint8_t *usb_dev_cdc_get_hsconfigdesc(uint16_t *length){
	usb_dev_ep_desc_t *pEpCmdDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_CMD_EP);
	usb_dev_ep_desc_t *pEpOutDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_OUT_EP);
	usb_dev_ep_desc_t *pEpInDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_IN_EP);

	if (pEpCmdDesc != NULL)
		pEpCmdDesc->bInterval = CDC_HS_BINTERVAL;

	if (pEpOutDesc != NULL)
		pEpOutDesc->wMaxPacketSize = CDC_DATA_HS_MAX_PACKET_SIZE;

	if (pEpInDesc != NULL)
		pEpInDesc->wMaxPacketSize = CDC_DATA_HS_MAX_PACKET_SIZE;

	*length = (uint16_t)sizeof(USBD_CDC_CfgDesc);
	return USBD_CDC_CfgDesc;
}

uint8_t *usb_dev_cdc_get_otherspeed_configdesc(uint16_t *length){
	usb_dev_ep_desc_t *pEpCmdDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_CMD_EP);
	usb_dev_ep_desc_t *pEpOutDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_OUT_EP);
	usb_dev_ep_desc_t *pEpInDesc = USBD_GetEpDesc(USBD_CDC_CfgDesc, CDC_IN_EP);

	if (pEpCmdDesc != NULL)
		pEpCmdDesc->bInterval = CDC_FS_BINTERVAL;

	if (pEpOutDesc != NULL)
		pEpOutDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;

	if (pEpInDesc != NULL)
		pEpInDesc->wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;

	*length = (uint16_t)sizeof(USBD_CDC_CfgDesc);
	return USBD_CDC_CfgDesc;
}

uint8_t *usb_dev_cdc_get_devqualifierdesc(uint16_t *length){
	*length = (uint16_t)sizeof(USBD_CDC_DeviceQualifierDesc);

	return USBD_CDC_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE  */

uint8_t usb_dev_cdc_register_interface(usb_dev_handler_t *dev, usb_dev_cdc_interface_t *interface){
	if (interface == NULL)
		return (uint8_t)USBD_FAIL;

	dev->pUserData[dev->classId] = interface;

	return (uint8_t)USBD_OK;
}

uint8_t usb_dev_cdc_set_Txbuffer(usb_dev_handler_t *dev, uint8_t *buff, uint32_t length){
	usb_dev_cdc_handler_t *cdc = (usb_dev_cdc_handler_t *)dev->pClassDataCmsit[dev->classId];

	if (cdc == NULL)
		return (uint8_t)USBD_FAIL;

	cdc->TxBuffer = buff;
	cdc->TxLength = length;

	return (uint8_t)USBD_OK;
}

uint8_t usb_dev_cdc_set_Rxbuffer(usb_dev_handler_t *dev, uint8_t *buff){
	usb_dev_cdc_handler_t *cdc = (usb_dev_cdc_handler_t *)dev->pClassDataCmsit[dev->classId];

	if (cdc == NULL)
		return (uint8_t)USBD_FAIL;

	cdc->RxBuffer = buff;

	return (uint8_t)USBD_OK;
}

uint8_t usb_dev_cdc_transmit(usb_dev_handler_t *dev){
	usb_dev_cdc_handler_t *cdc = (usb_dev_cdc_handler_t *)dev->pClassDataCmsit[dev->classId];
	usb_dev_status_t ret = USBD_BUSY;

#ifdef USE_USBD_COMPOSITE
	/* Get the Endpoints addresses allocated for this class instance */
	CDCInEpAdd  = USBD_CoreGetEPAdd(dev, USBD_EP_IN, USBD_EP_TYPE_BULK);
#endif /* USE_USBD_COMPOSITE */
	if (dev->pClassDataCmsit[dev->classId] == NULL)
		return (uint8_t)USBD_FAIL;

	if (cdc->TxState == 0U){
		cdc->TxState = 1U;

		dev->ep_in[CDCInEpAdd & 0xFU].total_length = cdc->TxLength;

		(void)USBD_LL_Transmit(dev, CDCInEpAdd, cdc->TxBuffer, cdc->TxLength);

		ret = USBD_OK;
	}

	return (uint8_t)ret;
}

uint8_t usb_dev_cdc_receive(usb_dev_handler_t *dev){
	usb_dev_cdc_handler_t *cdc = (usb_dev_cdc_handler_t *)dev->pClassDataCmsit[dev->classId];

#ifdef USE_USBD_COMPOSITE
	/* Get the Endpoints addresses allocated for this class instance */
	CDCOutEpAdd = USBD_CoreGetEPAdd(dev, USBD_EP_OUT, USBD_EP_TYPE_BULK);
#endif /* USE_USBD_COMPOSITE */

	if (dev->pClassDataCmsit[dev->classId] == NULL)
		return (uint8_t)USBD_FAIL;

	if (dev->dev_speed == USBD_SPEED_HIGH){
		(void)USBD_LL_PrepareReceive(dev, CDCOutEpAdd, cdc->RxBuffer, CDC_DATA_HS_OUT_PACKET_SIZE);
	}
	else{
		(void)USBD_LL_PrepareReceive(dev, CDCOutEpAdd, cdc->RxBuffer, CDC_DATA_FS_OUT_PACKET_SIZE);
	}

	return (uint8_t)USBD_OK;
}

#endif /* ENABLE_USB */
