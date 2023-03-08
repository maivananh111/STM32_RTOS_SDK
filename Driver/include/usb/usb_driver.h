/*
 * usb_driver.h
 *
 *  Created on: Mar 6, 2023
 *      Author: anh
 */

#ifndef USB_DRIVER_H_
#define USB_DRIVER_H_

#include "periph_en.h"
#ifdef ENABLE_USB

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx.h"
#include "status.h"
#include "usb_ll.h"
#include "usb_driver_def.h"


#define DEVICE_FS 		0
#define DEVICE_HS 		1


return_t usb_driver_hardware_init(uint32_t interruptpriority);
return_t usb_ll_init(usb_dev_handler_t *dev);
return_t usb_driver_init(usb_driver_t *drv);

return_t usb_driver_set_tx_fifo(usb_driver_t *drv, uint8_t fifo, uint16_t size);
return_t usb_driver_set_rx_fifo(usb_driver_t *drv, uint16_t size);


return_t usb_driver_start(usb_driver_t *drv);
return_t usb_driver_stop(usb_driver_t *drv);

return_t usb_driver_dev_connect(usb_driver_t *drv);
return_t usb_driver_dev_disconnect(usb_driver_t *drv);

return_t usb_driver_set_address(usb_driver_t *drv, uint8_t address);

return_t usb_driver_ep_open(usb_driver_t *drv, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);
return_t usb_driver_ep_close(usb_driver_t *drv, uint8_t ep_addr);

return_t usb_driver_ep_receive(usb_driver_t *drv, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
return_t usb_driver_ep_transmit(usb_driver_t *drv, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);

return_t usb_driver_ep_setstall(usb_driver_t *drv, uint8_t ep_addr);
return_t usb_driver_ep_clearstall(usb_driver_t *drv, uint8_t ep_addr);

return_t usb_driver_ep_flush(usb_driver_t *drv, uint8_t ep_addr);
return_t usb_driver_ep_abort(usb_driver_t *drv, uint8_t ep_addr);

uint32_t usb_driver_ep_getRxcount(usb_driver_t *drv, uint8_t ep_addr);

return_t usb_driver_activate_remotewakeup(usb_driver_t *drv);
return_t usb_driver_deactivate_remotewakeup(usb_driver_t *drv);

return_t usb_driver_set_testmode(usb_driver_t *drv, uint8_t testmode);

usb_state_t usb_driver_getstate(usb_driver_t *drv);


#ifdef __cplusplus
 }
#endif

#endif /* ENABLE_USB */

#endif /* USB_DRIVER_H_ */
