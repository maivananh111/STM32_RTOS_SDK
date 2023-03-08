/*
 * usb_ll.cpp
 *
 *  Created on: Mar 6, 2023
 *      Author: anh
 */

#include "periph_en.h"
#ifdef ENABLE_USB
#include "usb/usb_ll.h"



void usb_write(usb_t *usb, uint8_t *data, uint8_t ep_num, uint16_t len, uint8_t dma);

return_t usb_core_reset(usb_t *usb){
	return_t ret;

	ret = wait_flag_in_register_timeout(&(usb->GRSTCTL), USB_OTG_GRSTCTL_AHBIDL, FLAG_SET, 50);
	if(is_timeout(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	usb->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

	ret = wait_flag_in_register_timeout(&(usb->GRSTCTL), USB_OTG_GRSTCTL_CSRST, FLAG_RESET, 50);
	if(is_timeout(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

  return ret;
}

return_t usb_set_curent_mode(usb_t *usb, usb_otg_mode_t mode){
	return_t ret;

	usb->GUSBCFG &=~ (USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);

	if (mode == USB_HOST_MODE){
		usb->GUSBCFG |= USB_OTG_GUSBCFG_FHMOD;
		ret = wait_flag_in_register_timeout(&(usb->GINTSTS), USB_OTG_GINTSTS_CMOD, FLAG_SET, 50);
		if(is_timeout(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	else if (mode == USB_DEVICE_MODE){
		usb->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
		ret = wait_flag_in_register_timeout(&(usb->GINTSTS), USB_OTG_GINTSTS_CMOD, FLAG_RESET, 50);
		if(is_timeout(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	else{
		set_return(&ret, UNSUPPORTED, __LINE__);
		return ret;
	}

	return ret;
}

return_t usb_core_init(usb_driver_t *drv){
	return_t ret;
	usb_t *usb = drv -> usb;
	if (drv->conf.phy_interface == USB_OTG_ULPI_PHY){
		usb->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);
		usb->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);
		usb->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);

		if (drv->conf.use_external_vbus == 1U)
			usb->GUSBCFG |= USB_OTG_GUSBCFG_ULPIEVBUSD;

		ret = usb_core_reset(usb);
	}
	else {
		usb->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

		ret = usb_core_reset(usb);

		if (drv->conf.battery_charging_enable == 0U)
			usb->GCCFG |= USB_OTG_GCCFG_PWRDWN;
		else
			usb->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);
	}

	if (drv->conf.dma_enable == 1U){
		usb->GAHBCFG |= USB_OTG_GAHBCFG_HBSTLEN_2;
		usb->GAHBCFG |= USB_OTG_GAHBCFG_DMAEN;
	}

	return ret;
}

return_t usb_flush_tx_fifo(usb_t *usb, uint32_t tx_fifo_size){
	return_t ret;

	ret = wait_flag_in_register_timeout(&(usb->GRSTCTL), USB_OTG_GRSTCTL_AHBIDL, FLAG_SET, 50);
	if(is_timeout(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	usb->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (tx_fifo_size << 6));

	ret = wait_flag_in_register_timeout(&(usb->GRSTCTL), USB_OTG_GRSTCTL_TXFFLSH, FLAG_RESET, 50);
	if(is_timeout(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	return ret;
}

return_t usb_flush_rx_fifo(usb_t *usb){
	return_t ret;

	ret = wait_flag_in_register_timeout(&(usb->GRSTCTL), USB_OTG_GRSTCTL_AHBIDL, FLAG_SET, 50);
	if(is_timeout(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	usb->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;

	ret = wait_flag_in_register_timeout(&(usb->GRSTCTL), USB_OTG_GRSTCTL_RXFFLSH, FLAG_RESET, 50);
	if(is_timeout(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	return ret;
}

void usb_write(usb_t *usb, uint8_t *data, uint8_t ep_num, uint16_t len, uint8_t dma){
	uint32_t USB_BASE = (uint32_t)usb;
	uint8_t *pdata = data;
	uint32_t count32b;
	uint32_t i;

	if (dma == 0U){
		count32b = ((uint32_t)len + 3U) / 4U;
		for (i = 0U; i < count32b; i++){
			USB_DFIFO((uint32_t)ep_num) = __UNALIGNED_UINT32_READ(pdata);
			pdata++;
			pdata++;
			pdata++;
			pdata++;
		}
	}
}

return_t usb_ll_dev_connect(usb_t *usb){
	uint32_t USB_BASE = (uint32_t)(usb);

	USB_PCGCCTL &=~ (USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);

	USB_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;

	return {OKE, 0};
}

return_t usb_ll_dev_disconnect(usb_t *usb){
	uint32_t USB_BASE = (uint32_t)(usb);

	USB_PCGCCTL &=~ (USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);

	USB_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;

	return {OKE, 0};
}

#if defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)
return_t usb_activate_lpm(usb_driver_t *drv){
	  drv->lpm_active = 1U;
	  drv->lpm_state = USB_LPM_L0;
	  drv->usb->GINTMSK |= USB_OTG_GINTMSK_LPMINTM;
	  drv->usb->GLPMCFG |= (USB_OTG_GLPMCFG_LPMEN | USB_OTG_GLPMCFG_LPMACK | USB_OTG_GLPMCFG_ENBESL);
}


#endif /* defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx) */


return_t usb_ll_set_devaddress(usb_t *usb, uint8_t address){
	uint32_t USB_BASE = (uint32_t)(usb);

	USB_DEVICE->DCFG &= ~(USB_OTG_DCFG_DAD);
	USB_DEVICE->DCFG |= ((uint32_t)address << 4) & USB_OTG_DCFG_DAD;



	return {OKE, 0};
}

return_t usb_ll_activate_ep(usb_t *usb, usb_ep_t *ep){
	uint32_t USB_BASE = (uint32_t)(usb);
	uint32_t epnum = (uint32_t)ep->num;

	if (ep->is_in == 1U){
		USB_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK));

		if ((USB_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0U){
			USB_INEP(epnum)->DIEPCTL |= (ep->maxpacket & USB_OTG_DIEPCTL_MPSIZ) |
									   ((uint32_t)ep->type << 18) | (epnum << 22) |
									   USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
									   USB_OTG_DIEPCTL_USBAEP;
		}
	}
	else{
		USB_DEVICE->DAINTMSK |= USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16);

		if (((USB_OUTEP(epnum)->DOEPCTL) & USB_OTG_DOEPCTL_USBAEP) == 0U){
			USB_OUTEP(epnum)->DOEPCTL |= (ep->maxpacket & USB_OTG_DOEPCTL_MPSIZ) |
										((uint32_t)ep->type << 18) |
										USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
										USB_OTG_DOEPCTL_USBAEP;
		}
	}
	return {OKE, 0};
}

return_t usb_ll_deactivate_ep(usb_t *usb, usb_ep_t *ep){
	uint32_t USB_BASE = (uint32_t)(usb);
	uint32_t epnum = (uint32_t)ep->num;

	if (ep->is_in == 1U){
		if ((USB_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA){
			USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
			USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
		}

		USB_DEVICE->DEACHMSK &= ~(USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK)));
		USB_DEVICE->DAINTMSK &= ~(USB_OTG_DAINTMSK_IEPM & (uint32_t)(1UL << (ep->num & EP_ADDR_MSK)));
		USB_INEP(epnum)->DIEPCTL &= ~(USB_OTG_DIEPCTL_USBAEP |
								   USB_OTG_DIEPCTL_MPSIZ |
								   USB_OTG_DIEPCTL_TXFNUM |
								   USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
								   USB_OTG_DIEPCTL_EPTYP);
	}
	else{
		if ((USB_OUTEP(epnum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA){
			USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
			USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS;
		}

		USB_DEVICE->DEACHMSK &= ~(USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16));
		USB_DEVICE->DAINTMSK &= ~(USB_OTG_DAINTMSK_OEPM & ((uint32_t)(1UL << (ep->num & EP_ADDR_MSK)) << 16));
		USB_OUTEP(epnum)->DOEPCTL &= ~(USB_OTG_DOEPCTL_USBAEP |
									USB_OTG_DOEPCTL_MPSIZ |
									USB_OTG_DOEPCTL_SD0PID_SEVNFRM |
									USB_OTG_DOEPCTL_EPTYP);
	}

	return {OKE, 0};
}

return_t usb_ep0_startxfer(usb_t *usb, usb_ep_t *ep, uint8_t dma){
	uint32_t USB_BASE = (uint32_t)(usb);
	uint32_t epnum = (uint32_t)ep->num;

	if (ep->is_in == 1U){
		if (ep->xfer_len == 0U){
			USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
			USB_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19));
			USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
		}
		else{
			USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
			USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);

			if (ep->xfer_len > ep->maxpacket)
				ep->xfer_len = ep->maxpacket;

			USB_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19));
			USB_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & ep->xfer_len);
		}

		if (dma == 1U){
			if ((uint32_t)ep->dma_addr != 0U)
				USB_INEP(epnum)->DIEPDMA = (uint32_t)(ep->dma_addr);

			USB_INEP(epnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
		}
		else{
			USB_INEP(epnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

			if (ep->xfer_len > 0U)
				USB_DEVICE->DIEPEMPMSK |= 1UL << (ep->num & EP_ADDR_MSK);
		}
	}
	else{
		USB_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
		USB_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

		if (ep->xfer_len > 0U)
			ep->xfer_len = ep->maxpacket;

		ep->xfer_size = ep->maxpacket;

		USB_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
		USB_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & ep->xfer_size);

		if (dma == 1U){
			if ((uint32_t)ep->xfer_buff != 0U)
			USB_OUTEP(epnum)->DOEPDMA = (uint32_t)(ep->xfer_buff);
		}

		USB_OUTEP(epnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}

	return {OKE, 0};
}


return_t usb_ep_startxfer(usb_t *usb, usb_ep_t *ep, uint8_t dma){
	uint32_t USB_BASE = (uint32_t)usb;
	uint32_t epnum = (uint32_t)ep->num;
	uint16_t pktcnt;

	if (ep->is_in == 1U){
		if (ep->xfer_len == 0U){
			USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
			USB_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (1U << 19));
			USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
		}
		else{
			USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
			USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
			USB_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_PKTCNT & (((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket) << 19));
			USB_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_XFRSIZ & ep->xfer_len);

			if (ep->type == EP_TYPE_ISOC){
				USB_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_MULCNT);
				USB_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_MULCNT & (1U << 29));
			}
		}

		if (dma == 1U){
			if ((uint32_t)ep->dma_addr != 0U)
				USB_INEP(epnum)->DIEPDMA = (uint32_t)(ep->dma_addr);

			if (ep->type == EP_TYPE_ISOC){
				if ((USB_DEVICE->DSTS & (1U << 8)) == 0U)
					USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SODDFRM;
				else
					USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
			}

			USB_INEP(epnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
		}
		else{
			USB_INEP(epnum)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

			if (ep->type != EP_TYPE_ISOC){
				if (ep->xfer_len > 0U)
					USB_DEVICE->DIEPEMPMSK |= 1UL << (ep->num & EP_ADDR_MSK);
			}
			else{
				if ((USB_DEVICE->DSTS & (1U << 8)) == 0U)
					USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SODDFRM;
				else
					USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;

				usb_write(usb, ep->xfer_buff, ep->num, (uint16_t)ep->xfer_len, dma);
			}
		}
	}
	else {
		USB_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_XFRSIZ);
		USB_OUTEP(epnum)->DOEPTSIZ &= ~(USB_OTG_DOEPTSIZ_PKTCNT);

		if (ep->xfer_len == 0U){
			USB_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_XFRSIZ & ep->maxpacket);
			USB_OUTEP(epnum)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
		}
		else{
			pktcnt = (uint16_t)((ep->xfer_len + ep->maxpacket - 1U) / ep->maxpacket);
			ep->xfer_size = ep->maxpacket * pktcnt;

			USB_OUTEP(epnum)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_PKTCNT & ((uint32_t)pktcnt << 19);
			USB_OUTEP(epnum)->DOEPTSIZ |= USB_OTG_DOEPTSIZ_XFRSIZ & ep->xfer_size;
		}

		if (dma == 1U){
			if ((uint32_t)ep->xfer_buff != 0U)
				USB_OUTEP(epnum)->DOEPDMA = (uint32_t)(ep->xfer_buff);
		}

		if (ep->type == EP_TYPE_ISOC){
			if ((USB_DEVICE->DSTS & (1U << 8)) == 0U)
				USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SODDFRM;
			else
				USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
		}

		USB_OUTEP(epnum)->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
	}

	return {OKE, 0};
}


return_t usb_ll_ep_set_stall(usb_t *usb, usb_ep_t *ep){
	uint32_t USB_BASE = (uint32_t)usb;
	uint32_t epnum = (uint32_t)ep->num;

	if (ep->is_in == 1U){
		if (((USB_INEP(epnum)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0U) && (epnum != 0U))
			USB_INEP(epnum)->DIEPCTL &= ~(USB_OTG_DIEPCTL_EPDIS);

		USB_INEP(epnum)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
	}
	else{
		if (((USB_OUTEP(epnum)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == 0U) && (epnum != 0U))
			USB_OUTEP(epnum)->DOEPCTL &= ~(USB_OTG_DOEPCTL_EPDIS);

		USB_OUTEP(epnum)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
	}

	return {OKE, 0};
}

return_t usb_ll_ep0_outstart(usb_t *usb, uint8_t dma, uint8_t *setup){
	uint32_t USB_BASE = (uint32_t)usb;
	uint32_t gSNPSiD = *(__IO uint32_t *)(&usb->CID + 0x1U);

	if (gSNPSiD > USB_OTG_CORE_ID_300A){
		if ((USB_OUTEP(0U)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)
			return {OKE, 0};
	}

	USB_OUTEP(0U)->DOEPTSIZ = 0U;
	USB_OUTEP(0U)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19));
	USB_OUTEP(0U)->DOEPTSIZ |= (3U * 8U);
	USB_OUTEP(0U)->DOEPTSIZ |=  USB_OTG_DOEPTSIZ_STUPCNT;

	if (dma == 1U){
		USB_OUTEP(0U)->DOEPDMA = (uint32_t)setup;
		USB_OUTEP(0U)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_USBAEP;
	}

	return {OKE, 0};
}













































return_t usb_ll_dev_init(usb_driver_t *drv){
	return_t ret;
	uint32_t USB_BASE = (uint32_t)(drv -> usb);
	uint32_t i;

	for (i = 0U; i < 15U; i++){
		drv->usb->DIEPTXF[i] = 0U;
	}
#if defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)
	/* VBUS Sensing setup */
	if(drv->conf.vbus_sensing_enable == 0U){
		USB_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
		drv->usb->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
		drv->usb->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
		drv->usb->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
	}
	else{
		drv->usb->GCCFG |= USB_OTG_GCCFG_VBDEN;
	}
#else
	if (drv->conf.vbus_sensing_enable == 0U){
		USB_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
		drv->usb->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS;
		drv->usb->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
		drv->usb->GCCFG &= ~USB_OTG_GCCFG_VBUSASEN;
	}
	else{
		drv->usb->GCCFG &= ~USB_OTG_GCCFG_NOVBUSSENS;
		drv->usb->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;
	}
#endif /* defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx) */

	USB_PCGCCTL = 0U;

	USB_DEVICE->DCFG |= DCFG_FRAME_INTERVAL_80;

	if (drv->conf.phy_interface == USB_OTG_ULPI_PHY){
		if (drv->conf.speed == USBD_HS_SPEED){
			USB_DEVICE->DCFG |= USB_OTG_SPEED_HIGH;
		}
		else
		{
			USB_DEVICE->DCFG |= USB_OTG_SPEED_HIGH_IN_FULL;
		}
	}
	else{
		USB_DEVICE->DCFG |= USB_OTG_SPEED_FULL;
	}

	ret = usb_flush_tx_fifo(drv->usb, 0x10);
	if(is_timeout(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	ret = usb_flush_rx_fifo(drv->usb);
	if(is_timeout(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	USB_DEVICE->DIEPMSK = 0U;
	USB_DEVICE->DOEPMSK = 0U;
	USB_DEVICE->DAINTMSK = 0U;

	for (i = 0U; i < drv->conf.dev_endpoints; i++){
		if ((USB_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA){
			if (i == 0U)
				USB_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
			else
				USB_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
		}
		else
			USB_INEP(i)->DIEPCTL = 0U;

		USB_INEP(i)->DIEPTSIZ = 0U;
		USB_INEP(i)->DIEPINT  = 0xFB7FU;
	}

	for (i = 0U; i < drv->conf.dev_endpoints; i++){
		if ((USB_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA){
			if (i == 0U)
			    USB_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
			else
			    USB_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
		}
		else
			USB_OUTEP(i)->DOEPCTL = 0U;

		USB_OUTEP(i)->DOEPTSIZ = 0U;
		USB_OUTEP(i)->DOEPINT  = 0xFB7FU;
	}

	USB_DEVICE->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);

	drv->usb->GINTMSK = 0U;

	drv->usb->GINTSTS = 0xBFFFFFFFU;
	if (drv->conf.dma_enable == 0U)
		drv->usb->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;

	drv->usb->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
				   USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
				   USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_IISOIXFRM |
				   USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM;

	if (drv->conf.sof_enable != 0U)
		drv->usb->GINTMSK |= USB_OTG_GINTMSK_SOFM;


	if (drv->conf.vbus_sensing_enable == 1U)
		drv->usb->GINTMSK |= (USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT);


	return ret;
}

#endif /* ENABLE_USB */


