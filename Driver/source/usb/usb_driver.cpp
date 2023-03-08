/*
 * usb_driver.cpp
 *
 *  Created on: Mar 6, 2023
 *      Author: anh
 */

#include "periph_en.h"
#ifdef ENABLE_USB

#include "usb/usb_driver.h"
#include "gpio.h"


usb_driver_t usb_driver_fs;



return_t usb_driver_hardware_init(uint32_t interruptpriority){
	return_t ret;

	gpio_port_clock_enable(GPIOA);
	gpio_set_alternatefunction(GPIOA, 11, AF10_USB);
	gpio_set_alternatefunction(GPIOA, 12, AF10_USB);
	gpio_set_alternatefunction_type(GPIOA, 11, GPIO_OUTPUT_PUSHPULL);
	gpio_set_alternatefunction_type(GPIOA, 11, GPIO_OUTPUT_PUSHPULL);

	RCC -> AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	__NVIC_ClearPendingIRQ(OTG_FS_IRQn);
	__NVIC_SetPriority(OTG_FS_IRQn, interruptpriority);
	__NVIC_EnableIRQ(OTG_FS_IRQn);
	__NVIC_ClearPendingIRQ(OTG_FS_IRQn);

	return ret;
}


return_t usb_driver_init(usb_driver_t *drv){
	return_t ret;
	uint8_t i = 0;

	ret = usb_driver_hardware_init(drv->interruptpriority);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	if ((drv->usb->CID & (0x1U << 8)) == 0U) drv->conf.dma_enable = 0U;

	drv->usb->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT; // Disable USB OTG FS interrupt.

	ret = usb_core_init(drv);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	ret = usb_set_curent_mode(drv->usb, drv->conf.mode);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	for (i = 0U; i < drv->conf.dev_endpoints; i++){
		drv->in_ep[i].is_in = 1U;
		drv->in_ep[i].num = i;
		drv->in_ep[i].tx_fifo_num = i;
		drv->in_ep[i].type = EP_TYPE_CTRL;
		drv->in_ep[i].maxpacket = 0U;
		drv->in_ep[i].xfer_buff = 0U;
		drv->in_ep[i].xfer_len = 0U;
	}

	for (i = 0U; i < drv->conf.dev_endpoints; i++){
		drv->out_ep[i].is_in = 0U;
		drv->out_ep[i].num = i;
		drv->out_ep[i].type = EP_TYPE_CTRL;
		drv->out_ep[i].maxpacket = 0U;
		drv->out_ep[i].xfer_buff = 0U;
		drv->out_ep[i].xfer_len = 0U;
	}

	ret = usb_ll_dev_init(drv);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	drv->address = 0U;

#if defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)
  /* Activate LPM */
  if(drv->conf.lpm_enable == 1U){
	  usb_activate_lpm(drv);
  }
#endif /* defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx) */

  	ret = usb_ll_dev_disconnect(drv->usb);

	return ret;
}


return_t usb_driver_set_tx_fifo(usb_driver_t *drv, uint8_t fifo, uint16_t size){
	uint8_t i;
	uint32_t Tx_Offset;

	Tx_Offset = drv->usb->GRXFSIZ;

	if (fifo == 0U)
		drv->usb->DIEPTXF0_HNPTXFSIZ = ((uint32_t)size << 16) | Tx_Offset;

	else{
		Tx_Offset += (drv->usb->DIEPTXF0_HNPTXFSIZ) >> 16;
		for (i = 0U; i < (fifo - 1U); i++)
		  Tx_Offset += (drv->usb->DIEPTXF[i] >> 16);

		drv->usb->DIEPTXF[fifo - 1U] = ((uint32_t)size << 16) | Tx_Offset;
	}

	return {OKE, 0};
}


return_t usb_driver_set_rx_fifo(usb_driver_t *drv, uint16_t size){
	drv->usb->GRXFSIZ = size;
	return {OKE, 0};
}


return_t usb_ll_init(usb_dev_handler_t *dev){
	return_t ret;
	if (dev->id == DEVICE_FS) {
		usb_driver_fs.pData = dev;
		dev->pData = &usb_driver_fs;

		usb_driver_fs.usb = USB_OTG_FS;
		usb_driver_fs.conf.dev_endpoints = 4;
		usb_driver_fs.conf.speed = USB_OTG_SPEED_FULL;
		usb_driver_fs.conf.dma_enable = DISABLE;
		usb_driver_fs.conf.phy_interface = USB_OTG_EMBEDDED_PHY;
		usb_driver_fs.conf.sof_enable = DISABLE;
		usb_driver_fs.conf.low_power_enable = DISABLE;
		usb_driver_fs.conf.lpm_enable = DISABLE;
		usb_driver_fs.conf.vbus_sensing_enable = DISABLE;
		usb_driver_fs.conf.use_dedicated_ep1 = DISABLE;

		ret = usb_driver_init(&usb_driver_fs);
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}

		usb_driver_set_rx_fifo(&usb_driver_fs,    0x80);
		usb_driver_set_tx_fifo(&usb_driver_fs, 0, 0x40);
		usb_driver_set_tx_fifo(&usb_driver_fs, 1, 0x80);
	}

	return ret;
}


return_t usb_driver_start(usb_driver_t *drv){
	usb_t *usb = drv->usb;

	if ((drv->conf.battery_charging_enable == 1U) && (drv->conf.phy_interface != USB_OTG_ULPI_PHY))
		usb->GCCFG |= USB_OTG_GCCFG_PWRDWN;

	usb->GAHBCFG |= USB_OTG_GAHBCFG_GINT; // ENABLE Driver

	(void)usb_ll_dev_connect(usb);

	return {OKE, 0};
}


return_t usb_driver_stop(usb_driver_t *drv){
	usb_t *usb = drv->usb;

	usb->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
	usb_ll_dev_disconnect(usb);

	(void)usb_flush_tx_fifo(usb, 0x10U);

	if ((drv->conf.battery_charging_enable == 1U) && (drv->conf.phy_interface != USB_OTG_ULPI_PHY))
		usb->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);

	return {OKE, 0};
}

return_t usb_driver_dev_connect(usb_driver_t *drv){
	usb_t *usb = drv->usb;

	if((drv->conf.battery_charging_enable == 1U) && (drv->conf.phy_interface != USB_OTG_ULPI_PHY))
		usb->GCCFG |= USB_OTG_GCCFG_PWRDWN;

	(void)usb_ll_dev_connect(usb);

	return {OKE, 0};
}
return_t usb_driver_dev_disconnect(usb_driver_t *drv){
	usb_t *usb = drv->usb;

	(void)usb_ll_dev_disconnect(usb);

	if((drv->conf.battery_charging_enable == 1U) && (drv->conf.phy_interface != USB_OTG_ULPI_PHY))
		usb->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);

	return {OKE, 0};
}

return_t usb_driver_set_address(usb_driver_t *drv, uint8_t address){
	drv->address = address;
	(void)usb_ll_set_devaddress(drv->usb, address);

	return {OKE, 0};
}

return_t usb_driver_ep_open(usb_driver_t *drv, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type){
	usb_ep_t *ep;

	if ((ep_addr & 0x80U) == 0x80U){
		ep = &drv->in_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 1U;
	}
	else
	{
		ep = &drv->out_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 0U;
	}

	ep->num = ep_addr & EP_ADDR_MSK;
	ep->maxpacket = ep_mps;
	ep->type = ep_type;

	if (ep->is_in != 0U)
		ep->tx_fifo_num = ep->num;

	if (ep_type == EP_TYPE_BULK)
		ep->data_pid_start = 0U;

	(void)usb_ll_activate_ep(drv->usb, ep);

	return {OKE, 0};
}
return_t usb_driver_ep_close(usb_driver_t *drv, uint8_t ep_addr){
	usb_ep_t *ep;

	if ((ep_addr & 0x80U) == 0x80U){
		ep = &drv->in_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 1U;
	}
	else{
		ep = &drv->out_ep[ep_addr & EP_ADDR_MSK];
		ep->is_in = 0U;
	}
	ep->num   = ep_addr & EP_ADDR_MSK;

	(void)usb_ll_deactivate_ep(drv->usb, ep);

	return {OKE, 0};
}

return_t usb_driver_ep_receive(usb_driver_t *drv, uint8_t ep_addr, uint8_t *pBuf, uint32_t len){
	usb_ep_t *ep;

	ep = &drv->out_ep[ep_addr & EP_ADDR_MSK];
	ep->xfer_buff = pBuf;
	ep->xfer_len = len;
	ep->xfer_count = 0U;
	ep->is_in = 0U;
	ep->num = ep_addr & EP_ADDR_MSK;

	if (drv->conf.dma_enable == 1U)
		ep->dma_addr = (uint32_t)pBuf;

	if ((ep_addr & EP_ADDR_MSK) == 0U)
		(void)usb_ep0_startxfer(drv->usb, ep, (uint8_t)drv->conf.dma_enable);
	else
		(void)usb_ep_startxfer(drv->usb, ep, (uint8_t)drv->conf.dma_enable);

	return {OKE, 0};;
}

uint32_t usb_driver_ep_getRxcount(usb_driver_t *drv, uint8_t ep_addr){
	return drv->out_ep[ep_addr & EP_ADDR_MSK].xfer_count;
}

return_t usb_driver_ep_transmit(usb_driver_t *drv, uint8_t ep_addr, uint8_t *pBuf, uint32_t len){
	usb_ep_t *ep;

	ep = &drv->in_ep[ep_addr & EP_ADDR_MSK];
	ep->xfer_buff = pBuf;
	ep->xfer_len = len;
	ep->xfer_count = 0U;
	ep->is_in = 1U;
	ep->num = ep_addr & EP_ADDR_MSK;

	if (drv->conf.dma_enable == 1U)
		ep->dma_addr = (uint32_t)pBuf;

	if ((ep_addr & EP_ADDR_MSK) == 0U)
		(void)usb_ep0_startxfer(drv->usb, ep, (uint8_t)drv->conf.dma_enable);
	else
		(void)usb_ep_startxfer(drv->usb, ep, (uint8_t)drv->conf.dma_enable);

	return {OKE, 0};
}

return_t usb_driver_ep_setstall(usb_driver_t *drv, uint8_t ep_addr){
	usb_ep_t *ep;

	  if (((uint32_t)ep_addr & EP_ADDR_MSK) > drv->conf.dev_endpoints)
	    return {ERR, __LINE__};

	  if ((0x80U & ep_addr) == 0x80U){
	    ep = &drv->in_ep[ep_addr & EP_ADDR_MSK];
	    ep->is_in = 1U;
	  }
	  else{
	    ep = &drv->out_ep[ep_addr];
	    ep->is_in = 0U;
	  }

	  ep->is_stall = 1U;
	  ep->num = ep_addr & EP_ADDR_MSK;

	  (void)usb_ll_ep_set_stall(drv->usb, ep);

	  if ((ep_addr & EP_ADDR_MSK) == 0U)
	    (void)usb_ll_ep0_outstart(drv->usb, (uint8_t)drv->conf.dma_enable, (uint8_t *)drv->setup);


	  return {OKE, 0};
}
return_t usb_driver_ep_clearstall(usb_driver_t *drv, uint8_t ep_addr);

return_t usb_driver_ep_flush(usb_driver_t *drv, uint8_t ep_addr);
return_t usb_driver_ep_abort(usb_driver_t *drv, uint8_t ep_addr);

return_t usb_driver_activate_remotewakeup(usb_driver_t *drv);
return_t usb_driver_deactivate_remotewakeup(usb_driver_t *drv);

return_t usb_driver_set_testmode(usb_driver_t *drv, uint8_t testmode);

usb_state_t usb_driver_getstate(usb_driver_t *drv);








#endif /* ENABLE_USB */
