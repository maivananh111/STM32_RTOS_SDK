/*
 * usb_device.cpp
 *
 *  Created on: Mar 7, 2023
 *      Author: anh
 */
#include "periph_en.h"
#ifdef ENABLE_USB

#include "usb/device/usb_device.h"
#include "usb/device/usb_device_desc.h"
#include "usb/device/usb_device_cdc.h"

usb_dev_handler_t usb_dev_fs; // USBD_HandleTypeDef

return_t usb_dev_init(usb_dev_handler_t *dev, usb_dev_descriptor_t *desc, uint8_t id){
	return_t ret;

	dev->pClass[0] = NULL;
	dev->pUserData[0] = NULL;
	dev->pConfDesc = NULL;
	dev->pDesc = desc;
	dev->dev_state = USBD_STATE_DEFAULT;
	dev->id = id;

	return usb_ll_init(dev);
}

return_t usb_dev_register_class(usb_dev_handler_t *dev, usb_dev_class_t *devclass){
	uint16_t len = 0U;

	dev->pClass[0] = devclass;

#ifdef USE_USB_HS
	if (dev->pClass[dev->classId]->GetHSConfigDescriptor != NULL){
		dev->pConfDesc = (void *)dev->pClass[dev->classId]->GetHSConfigDescriptor(&len);
	}
#else /* Default USE_USB_FS */
	if (dev->pClass[dev->classId]->GetFSConfigDescriptor != NULL)
	{
		dev->pConfDesc = (void *)dev->pClass[dev->classId]->GetFSConfigDescriptor(&len);
	}
#endif /* USE_USB_FS */

	dev->NumClasses ++;

	return {OKE, 0};
}

return_t usb_device_init(void){ //MX_USB_DEVICE_Init
	return_t ret;

	ret = usb_dev_init(&usb_dev_fs, &fs_descriptor, DEVICE_FS);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	ret = usb_dev_register_class(&usb_dev_fs, &usb_device_cdc_class);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}







	return ret;
}

#endif

