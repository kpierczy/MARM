/*
 * =====================================================================================
 *
 *       Filename:  config.h
 *
 *    Description:  Nixie CLOCK application config
 *
 *        Version:  1.0
 *        Created:  08.05.2016 18:36:36
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Lucjan Bryndza (LB), lbryndza.p(at)boff(dot)pl
 *   Organization:  BoFF
 *
 * =====================================================================================
 */
#pragma once
 
// --------  Hardware specific config ------------
#define  CONFIG_XTAL_HZ   25000000LU
#define  CONFIG_HCLK_HZ  168000000LU
#define	 CONFIG_PCLK1_HZ (CONFIG_HCLK_HZ/4)
#define  CONFIG_PCLK2_HZ (CONFIG_HCLK_HZ/2)

// --------  ISIX specific config ------------
//! Enable poweroff API  
//#define ISIX_CONFIG_SHUTDOWN_API
//! In F103 devs MPU is not present
#define ISIX_CONFIG_MEMORY_PROTECTION_MODEL ISIX_MPROT_LITE
#define ISIX_CONFIG_TASK_STACK_CHECK 1
//! Define log level for application
#define FOUNDATION_CONFIG_LOGLEVEL FOUNDATION_DBGLOG_DEBUG
#define ISIX_CONFIG_NUM_PRIORITIES 4

// ---- Ethernet specific config ----
#define ETH_DRV_USE_RMII  1
#define PHY_INT_USE_INTERRUPT 1
#define PHY_INT_EXTI_LINE_IRQ_N EXTI0_IRQn
#define PHY_INT_GPIO_PORT A
#define PHY_INT_EXTI_NUM 0
#define ETH_DRV_PHY_ADDR 0
#define ETH_PHY_DRIVER_NAME _ethernetif_phy_ksz8081_drv_

// Configure SD_DET_PIN
#define CONFIG_SD_DET_PORT GPIOC
#define CONFIG_SD_DET_PIN 6
#define CONFIG_SDIO_HOST_CLK_KHZ 24000


// FIXME I2C driver without DMA (due to conflict with DMA channel on transfer data)
#define CONFIG_ISIXDRV_I2C_USE_FIXED_I2C CONFIG_ISIXDRV_I2C_2


//!Configure usb stack
#if USB_HOST_EXAMPLE
#define CONFIG_USBHOST_VBUS_GPIO_N  E
#define CONFIG_USBHOST_VBUS_PIN   3
#define CONFIG_USBHOST_VBUS_ON      1
#define CONFIG_USBHOST_VBUS_OTYPE  GPIO_OTYPE_PP
#define CONFIG_USBLIB_US_TIM_N 2
#define CONFIG_USBHOST_USB_IRQ_MASK_VALUE 1
#endif
