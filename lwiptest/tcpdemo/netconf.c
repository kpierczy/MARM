/**
  ******************************************************************************
  * @file    netconf.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11/20/2009
  * @brief   Network connection configuration
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/tcp_impl.h"
#include "lwip/memp.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/init.h"
#include "lwip/tcpip.h"
#include "ethernetif.h"
#include "netconf.h"
#include <stdio.h>
#include <dbglog.h>
#include <stm32_eth.h>


#define MAX_DHCP_TRIES        4
#define SELECTED              1
#define NOT_SELECTED		  (!SELECTED)
#define CLIENTMAC6            2

#define sprintf(x,...) tiny_snprintf(x,sizeof(x),__VA_ARGS__)


static struct netif netif;



/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  *
  */

void LwIP_Init(void)
{
  struct ip_addr ipaddr;
  struct ip_addr netmask;
  struct ip_addr gw;
  uint8_t macaddress[6]={0x00,0xA4,0xA5,0x10,0x20,0x30};

  netif_init();
  tcpip_init( NULL, NULL );


#if LWIP_DHCP
  ipaddr.addr = 0;
  netmask.addr = 0;
  gw.addr = 0;

#else
  IP4_ADDR(&ipaddr, 192, 168, 16, 222);
  IP4_ADDR(&netmask, 255, 255, 255, 0);
  IP4_ADDR(&gw, 192, 168, 16, 1);
#endif

  set_mac_address(macaddress);

  /* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
            struct ip_addr *netmask, struct ip_addr *gw,
            void *state, err_t (* init)(struct netif *netif),
            err_t (* input)(struct pbuf *p, struct netif *netif))
    
   Adds your network interface to the netif_list. Allocate a struct
  netif and pass a pointer to this structure as the first argument.
  Give pointers to cleared ip_addr structures when using DHCP,
  or fill them with sane numbers otherwise. The state pointer may be NULL.

  The init function pointer must point to a initialization function for
  your ethernet netif interface. The following code illustrates it's use.*/
  netif_add(&netif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

  /*  Registers the default network interface.*/
  netif_set_default(&netif);

#if LWIP_DHCP
  /*  Creates a new DHCP client for this interface on the first call.
  Note: you must call dhcp_fine_tmr() and dhcp_coarse_tmr() at
  the predefined regular intervals after starting the client.
  You can peek in the netif->dhcp struct for the actual DHCP status.*/
  dhcp_start(&netif);
#endif

  /*  When the netif is fully configured this function must be called.*/
  netif_set_up(&netif);

}


#if 0

/**
  * @brief  LCD & LEDs periodic handling
  * @param  localtime: the current LocalTime value
  * @retval None
  */
void Display_Periodic_Handle(__IO uint32_t localtime)
{ 
  /* 250 ms */
  if (localtime - DisplayTimer >= LCD_TIMER_MSECS)
  {
    DisplayTimer = localtime;

    /* We have got a new IP address so update the display */
    if (IPaddress != netif.ip_addr.addr)
    {
      __IO uint8_t iptab[4];
      uint8_t iptxt[20];

      /* Read the new IP address */
      IPaddress = netif.ip_addr.addr;

      iptab[0] = (uint8_t)(IPaddress >> 24);
      iptab[1] = (uint8_t)(IPaddress >> 16);
      iptab[2] = (uint8_t)(IPaddress >> 8);
      iptab[3] = (uint8_t)(IPaddress);

      tiny_snprintf((char*)iptxt, sizeof(iptxt), "   %d.%d.%d.%d    ", iptab[3], iptab[2], iptab[1], iptab[0]);

      /* Display the new IP address */
#if LWIP_DHCP
      if (netif.flags & NETIF_FLAG_DHCP)
      {        
		/* Display the IP address */
		LCD_DisplayStringLine(Line7, "IP address assigned ");
        LCD_DisplayStringLine(Line8, "  by a DHCP server  ");
        LCD_DisplayStringLine(Line9, iptxt);
		Delay(LCD_DELAY);
		
		/** Start the client/server application: only when a dynamic IP address has been obtained  **/
	    /* Clear the LCD */
        LCD_Clear(Black);
	    LCD_SetBackColor(Black);
        LCD_SetTextColor(White);		   	
       
	    iptab[0] = (uint8_t)(IPaddress >> 24);
        iptab[1] = (uint8_t)(IPaddress >> 16);
        iptab[2] = (uint8_t)(IPaddress >> 8);
        iptab[3] = (uint8_t)(IPaddress);
        	   
	    sprintf((char*)iptxt, "is: %d.%d.%d.%d ", iptab[3], iptab[2], iptab[1], iptab[0]);		
       
	    LCD_DisplayStringLine(Line0, " You are configured ");
	    LCD_DisplayStringLine(Line2, iptxt);

	    if(Server)
	    {
	      LCD_DisplayStringLine(Line1, "as a server, your IP");
		 
		  /* Initialize the server application */
	      server_init(); 
	    }
	    else
	    {
	      LCD_DisplayStringLine(Line1, "as a client, your IP");
		 
		  /* Configure the IO Expander */
          IOE_Config(); 
      
          /* Enable the Touch Screen and Joystick interrupts */
          IOE_ITConfig(IOE_ITSRC_TSC);
		  
		  /* Initialize the client application */
	      client_init();
	    }	        
      }
      else
#endif
      {
        /* Display the IP address */
		dbprintf("  Static IP address   %s",iptxt);
      }           
    }

#if LWIP_DHCP
    
    else if (IPaddress == 0)
    {
      /* We still waiting for the DHCP server */
	  LCD_DisplayStringLine(Line4, "     Looking for    ");
      LCD_DisplayStringLine(Line5, "     DHCP server    ");
      LCD_DisplayStringLine(Line6, "     please wait... ");

      LedToggle &= 3;

      STM_EVAL_LEDToggle((Led_TypeDef)(LedToggle++));

      /* If no response from a DHCP server for MAX_DHCP_TRIES times */
	  /* stop the dhcp client and set a static IP address */
	  if (netif.dhcp->tries > MAX_DHCP_TRIES)
      {
        struct ip_addr ipaddr;
        struct ip_addr netmask;
        struct ip_addr gw;

        LCD_DisplayStringLine(Line7, "    DHCP timeout    ");        

        dhcp_stop(&netif);

        IP4_ADDR(&ipaddr, 192, 168, 0, 8);
        IP4_ADDR(&netmask, 255, 255, 255, 0);
        IP4_ADDR(&gw, 192, 168, 0, 1);

        netif_set_addr(&netif, &ipaddr , &netmask, &gw);

      }
    }
#endif
  } 
}

#endif
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
