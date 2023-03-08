/*
 * usb_ll.h
 *
 *  Created on: Mar 6, 2023
 *      Author: anh
 */

#ifndef USB_LL_H_
#define USB_LL_H_

#include "periph_en.h"
#ifdef ENABLE_USB


#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx.h"
#include "status.h"



#define USB_OTG_ULPI_PHY                       1U
#define USB_OTG_EMBEDDED_PHY                   2U

#define EP_TYPE_CTRL                           0U
#define EP_TYPE_ISOC                           1U
#define EP_TYPE_BULK                           2U
#define EP_TYPE_INTR                           3U
#define EP_TYPE_MSK                            3U

#define DCFG_FRAME_INTERVAL_80                 0U
#define DCFG_FRAME_INTERVAL_85                 1U
#define DCFG_FRAME_INTERVAL_90                 2U
#define DCFG_FRAME_INTERVAL_95                 3U

#define USBD_HS_SPEED                          0U
#define USBD_HSINFS_SPEED                      1U
#define USBH_HS_SPEED                          0U
#define USBD_FS_SPEED                          2U
#define USBH_FSLS_SPEED                        1U

#define USB_OTG_SPEED_HIGH                     0U
#define USB_OTG_SPEED_HIGH_IN_FULL             1U
#define USB_OTG_SPEED_FULL                     3U

#define EP_ADDR_MSK                            0xFU


#define USB_OTG_CORE_ID_300A         		   0x4F54300AU
#define USB_OTG_CORE_ID_310A          		   0x4F54310AU


#define USB_PCGCCTL    *(__IO uint32_t *)((uint32_t)USB_BASE + USB_OTG_PCGCCTL_BASE)
#define USB_HPRT0      *(__IO uint32_t *)((uint32_t)USB_BASE + USB_OTG_HOST_PORT_BASE)
#define USB_DEVICE     ((USB_OTG_DeviceTypeDef *)(USB_BASE + USB_OTG_DEVICE_BASE))
#define USB_INEP(i)    ((USB_OTG_INEndpointTypeDef *)(USB_BASE + USB_OTG_IN_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USB_OUTEP(i)   ((USB_OTG_OUTEndpointTypeDef *)(USB_BASE + USB_OTG_OUT_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USB_DFIFO(i)   *(__IO uint32_t *)(USB_BASE + USB_OTG_FIFO_BASE + ((i) * USB_OTG_FIFO_SIZE))
#define USB_HOST       ((USB_OTG_HostTypeDef *)(USB_BASE + USB_OTG_HOST_BASE))
#define USB_HC(i)      ((USB_OTG_HostChannelTypeDef *)(USB_BASE + USB_OTG_HOST_CHANNEL_BASE + ((i) * USB_OTG_HOST_CHANNEL_SIZE)))

/**
 * PCD: Peripheral control driver.
 * LPM: Link power management.
 * HC: Host controller.
 * EP: End point device.
 * URB: USB request block.
 * PID: Product ID.
 * BCD: Binary coded decimal (device version).
 * PCD: Product class device.
 */

typedef enum{
	USB_DEVICE_MODE  = 0,
	USB_HOST_MODE    = 1,
	USB_DRD_MODE     = 2
} usb_otg_mode_t;

typedef enum{
	USB_URB_IDLE = 0,
	USB_URB_DONE,
	USB_URB_NOTREADY,
	USB_URB_NYET,
	USB_URB_ERROR,
	USB_URB_STALL
} usb_otg_urbstate_t;
typedef enum{
	USB_HC_IDLE = 0,
	USB_HC_XFRC,
	USB_HC_HALTED,
	USB_HC_NAK,
	USB_HC_NYET,
	USB_HC_STALL,
	USB_HC_XACTERR,
	USB_HC_BBLERR,
	USB_HC_DATATGLERR
} usb_otg_hcstate_t;

typedef enum{
	USB_STATE_RESET   = 0x00,
	USB_STATE_READY   = 0x01,
	USB_STATE_ERROR   = 0x02,
	USB_STATE_BUSY    = 0x03,
	USB_STATE_TIMEOUT = 0x04
} usb_state_t;

typedef enum {
	USB_LPM_L0 = 0x00, /* on */     /**< LPM_L0 */
	USB_LPM_L1 = 0x01, /* sleep */  /**< LPM_L1 */
	USB_LPM_L2 = 0x02, /* suspend *//**< LPM_L2 */
	USB_LPM_L3 = 0x03, /* off */    /**< LPM_L3 */
} usb_lpm_state_t;

typedef enum{
	USB_LPM_L0_ACTIVE = 0x00, /* on */
	USB_LPM_L1_ACTIVE = 0x01, /* sleep */
} usb_lpm_msg_t;

typedef enum{
	USB_BCD_ERROR                     = 0xFF,
	USB_BCD_CONTACT_DETECTION         = 0xFE,
	USB_BCD_STD_DOWNSTREAM_PORT       = 0xFD,
	USB_BCD_CHARGING_DOWNSTREAM_PORT  = 0xFC,
	USB_BCD_DEDICATED_CHARGING_PORT   = 0xFB,
	USB_BCD_DISCOVERY_COMPLETED       = 0x00,
} usb_bcd_msg_t;

typedef struct{
	uint8_t   num;
	uint8_t   is_in;
	uint8_t   is_stall;
	uint8_t   is_iso_incomplete;
	uint8_t   type;
	uint8_t   data_pid_start;
	uint8_t   even_odd_frame;
	uint16_t  tx_fifo_num;
	uint32_t  maxpacket;
	uint8_t   *xfer_buff;
	uint32_t  dma_addr;
	uint32_t  xfer_len;
	uint32_t  xfer_size;
	uint32_t  xfer_count;
} usb_ep_t;

typedef struct{
	uint8_t   dev_addr;
	uint8_t   ch_num;
	uint8_t   ep_num;
	uint8_t   ep_is_in;
	uint8_t   speed;
	uint8_t   do_ping;
	uint8_t   process_ping;
	uint8_t   ep_type;
	uint16_t  max_packet;
	uint8_t   data_pid;
	uint8_t   *xfer_buff;
	uint32_t  XferSize;
	uint32_t  xfer_len;
	uint32_t  xfer_count;
	uint8_t   toggle_in;
	uint8_t   toggle_out;
	uint32_t  dma_addr;
	uint32_t  ErrCnt;
	usb_otg_urbstate_t urb_state;
	usb_otg_hcstate_t state;
} usb_hc_t;


typedef struct{
	usb_otg_mode_t mode;
	uint32_t dev_endpoints;
	uint32_t host_channels;
	uint32_t speed;
	uint32_t dma_enable;
	uint32_t ep0_mps;
	uint32_t phy_interface;
	uint32_t sof_enable;
	uint32_t low_power_enable;
	uint32_t lpm_enable;
	uint32_t battery_charging_enable;
	uint32_t vbus_sensing_enable;
	uint32_t use_dedicated_ep1;
	uint32_t use_external_vbus;
} usb_config_t;

typedef USB_OTG_GlobalTypeDef  usb_t;


typedef struct {
	usb_t       	  *usb;
	usb_config_t       conf;
	__IO uint8_t       address;
	usb_ep_t           in_ep[16];
	usb_ep_t           out_ep[16];
	__IO  uint32_t     error;
	uint32_t           setup[12];
	usb_lpm_state_t    lpm_state;
	uint32_t           besl;
	uint32_t           framenumber;
	uint32_t 		   lpm_active;
	uint32_t           battery_charging_active;
	void               *pData;
	uint32_t 		   interruptpriority = 6;
} usb_driver_t;


return_t usb_core_init(usb_driver_t *drv);

return_t usb_core_reset(usb_t *usb);
return_t usb_set_curent_mode(usb_t *usb, usb_otg_mode_t mode);

return_t usb_flush_tx_fifo(usb_t *drv, uint32_t tx_fifo_size);
return_t usb_flush_rx_fifo(usb_t *drv);

return_t usb_ll_dev_init(usb_driver_t *drv);

return_t usb_ll_dev_connect(usb_t *usb);
return_t usb_ll_dev_disconnect(usb_t *usb);
return_t usb_ll_set_devaddress(usb_t *usb, uint8_t address);
return_t usb_ll_activate_ep(usb_t *usb, usb_ep_t *ep);
return_t usb_ll_deactivate_ep(usb_t *usb, usb_ep_t *ep);
return_t usb_ep0_startxfer(usb_t *usb, usb_ep_t *ep, uint8_t dma);
return_t usb_ep_startxfer(usb_t *usb, usb_ep_t *ep, uint8_t dma);
return_t usb_ll_ep_set_stall(usb_t *usb, usb_ep_t *ep);
return_t usb_ll_ep0_outstart(usb_t *usb, uint8_t dma, uint8_t *setup);



#if defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)
return_t usb_activate_lpm(usb_driver_t *drv);
#endif /* defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx) */

#ifdef __cplusplus
}
#endif

#endif /* ENABLE_USB */

#endif /* USB_LL_H_ */
