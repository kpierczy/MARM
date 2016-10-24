#include <stdio.h>
#include <usb/core/usb_endianness.h>
#include <usb/core/usbd_api.h>
#include <usb/core/usbd_callbacks.h>
#include <foundation/dbglog.h>

/** Descriptors **/

#define BLK_BUFF_SIZE  MAX_FS_BULK_PACKET_SIZE
#define INT_BUFF_SIZE  16


#define VID 0x0483
#define PID 0x5750


static usb_device_descriptor_t const device_descriptor = {
  sizeof(usb_device_descriptor_t), /* bLength */
  DEVICE_DESCRIPTOR,               /* bDescriptorType */
  HTOUSBS(0x0200),                 /* bcdUSB */
  COMMUNICATION_DEVICE_CLASS,      /* bDeviceClass */
  0,                               /* bDeviceSubClass */
  0,                               /* bDeviceProtocol */
  64,                              /* bMaxPacketSize0 */
  HTOUSBS(VID),                    /* idVendor */
  HTOUSBS(PID + 2),                /* idProduct */
  HTOUSBS(0x0100),                 /* bcdDevice */
  1,                               /* iManufacturer */
  2,                               /* iProduct */
  3,                               /* iSerialNumber */
  1                                /* bNumConfigurations */
};

typedef struct {
  usb_configuration_descriptor_t       cnf_descr;
  usb_interface_descriptor_t           if0_descr;
  usb_cdc_header_descriptor_t          cdc_h_descr;
  usb_cdc_call_management_descriptor_t cdc_cm_descr;
  usb_cdc_acm_descriptor_t             cdc_acm_descr;
  usb_cdc_union_descriptor_t           cdc_u_descr;
  usb_endpoint_descriptor_t            ep2in_descr;
  usb_interface_descriptor_t           if1_descr;
  usb_endpoint_descriptor_t            ep1out_descr;
  usb_endpoint_descriptor_t            ep1in_descr;
} __packed usb_com_configuration_t;

#ifndef USB_BM_ATTRIBUTES
  #define USB_BM_ATTRIBUTES  (SELF_POWERED | D7_RESERVED)
#endif
#ifndef USB_B_MAX_POWER
  #define USB_B_MAX_POWER  1
#endif

static usb_com_configuration_t const com_configuration = {
  {
    sizeof(usb_configuration_descriptor_t),   /* bLength */
    CONFIGURATION_DESCRIPTOR,                 /* bDescriptorType */
    HTOUSBS(sizeof(usb_com_configuration_t)), /* wTotalLength */
    2,                                        /* bNumInterfaces */
    1,                                        /* bConfigurationValue */
    0,                                        /* iConfiguration */
    USB_BM_ATTRIBUTES,                        /* bmAttributes */
    USB_B_MAX_POWER                           /* bMaxPower */
  },
  {
    sizeof(usb_interface_descriptor_t), /* bLength */
    INTERFACE_DESCRIPTOR,               /* bDescriptorType */
    0,                                  /* bInterfaceNumber */
    0,                                  /* bAlternateSetting */
    1,                                  /* bNumEndpoints */
    COMMUNICATION_INTERFACE_CLASS,      /* bInterfaceClass */
    ABSTRACT_CONTROL_MODEL_SUBCLASS,    /* bInterfaceSubClass */
    0,                                  /* bInterfaceProtocol */
    0                                   /* iInterface */
  },
  {
    sizeof(usb_cdc_header_descriptor_t), /* bFunctionLength */
    CS_INTERFACE_DESCRIPTOR,             /* bDescriptorType */
    CDC_HEADER_DESCRIPTOR,               /* bDescriptorSubtype */
    HTOUSBS(0x120)                       /* bcdCDC */
  },
  {
    sizeof(usb_cdc_call_management_descriptor_t),/* bFunctionLength */
    CS_INTERFACE_DESCRIPTOR,                     /* bDescriptorType */
    CDC_CALL_MANAGEMENT_DESCRIPTOR,           /* bDescriptorSubtype */
    3,                                        /* bmCapabilities */
    1                                         /* bDataInterface */
  },
  {
    sizeof(usb_cdc_acm_descriptor_t), /* bFunctionLength */
    CS_INTERFACE_DESCRIPTOR,          /* bDescriptorType */
    CDC_ACM_DESCRIPTOR,               /* bDescriptorSubtype */
    2                                 /* bmCapabilities */
  },
  {
    sizeof(usb_cdc_union_descriptor_t), /* bFunctionLength */
    CS_INTERFACE_DESCRIPTOR,            /* bDescriptorType */
    CDC_UNION_DESCRIPTOR,               /* bDescriptorSubtype */
    0,                                  /* bControlInterface */
    1                                   /* bSubordinateInterface0 */
  },
  {
    sizeof(usb_endpoint_descriptor_t), /* bLength */
    ENDPOINT_DESCRIPTOR,               /* bDescriptorType */
    ENDP2 | ENDP_IN,                   /* bEndpointAddress */
    INTERRUPT_TRANSFER,                /* bmAttributes */
    HTOUSBS(INT_BUFF_SIZE),            /* wMaxPacketSize */
    3                                  /* bInterval */
  },
  {
    sizeof(usb_interface_descriptor_t), /* bLength */
    INTERFACE_DESCRIPTOR,               /* bDescriptorType */
    1,                                  /* bInterfaceNumber */
    0,                                  /* bAlternateSetting */
    2,                                  /* bNumEndpoints */
    DATA_INTERFACE_CLASS,               /* bInterfaceClass */
    0,                                  /* bInterfaceSubClass */
    0,                                  /* bInterfaceProtocol */
    0                                   /* iInterface */
  },
  {
    sizeof(usb_endpoint_descriptor_t), /* bLength */
    ENDPOINT_DESCRIPTOR,               /* bDescriptorType */
    ENDP1 | ENDP_OUT,                  /* bEndpointAddress */
    BULK_TRANSFER,                     /* bmAttributes */
    HTOUSBS(BLK_BUFF_SIZE),            /* wMaxPacketSize */
    0                                  /* bInterval */
  },
  {
    sizeof(usb_endpoint_descriptor_t), /* bLength */
    ENDPOINT_DESCRIPTOR,               /* bDescriptorType */
    ENDP1 | ENDP_IN,                   /* bEndpointAddress */
    BULK_TRANSFER,                     /* bmAttributes */
    HTOUSBS(BLK_BUFF_SIZE),            /* wMaxPacketSize */
    0                                  /* bInterval */
  }
};

static usb_string_descriptor_t(1) const string_lang = {
  sizeof(usb_string_descriptor_t(1)),
  STRING_DESCRIPTOR,
  {HTOUSBS(LANG_US_ENGLISH)}
};

static usb_string_descriptor_t(16) const string_manufacturer = {
  sizeof(usb_string_descriptor_t(16)),
  STRING_DESCRIPTOR,
  {
    HTOUSBS('M'), HTOUSBS('a'), HTOUSBS('r'), HTOUSBS('c'),
    HTOUSBS('i'), HTOUSBS('n'), HTOUSBS(' '), HTOUSBS('P'),
    HTOUSBS('e'), HTOUSBS('c'), HTOUSBS('z'), HTOUSBS('a'),
    HTOUSBS('r'), HTOUSBS('s'), HTOUSBS('k'), HTOUSBS('i')
  }
};

static usb_string_descriptor_t(31) const string_product = {
  sizeof(usb_string_descriptor_t(31)),
  STRING_DESCRIPTOR,
  {
    HTOUSBS('U'), HTOUSBS('S'), HTOUSBS('B'), HTOUSBS(' '),
    HTOUSBS('v'), HTOUSBS('i'), HTOUSBS('r'), HTOUSBS('t'),
    HTOUSBS('u'), HTOUSBS('a'), HTOUSBS('l'), HTOUSBS(' '),
    HTOUSBS('s'), HTOUSBS('e'), HTOUSBS('r'), HTOUSBS('i'),
    HTOUSBS('a'), HTOUSBS('l'), HTOUSBS(' '), HTOUSBS('p'),
    HTOUSBS('o'), HTOUSBS('r'), HTOUSBS('t'), HTOUSBS(' '),
    HTOUSBS('e'), HTOUSBS('x'), HTOUSBS('a'), HTOUSBS('m'),
    HTOUSBS('p'), HTOUSBS('l'), HTOUSBS('e')
  }
};

static usb_string_descriptor_t(10) const string_serial = {
  sizeof(usb_string_descriptor_t(10)),
  STRING_DESCRIPTOR,
  {
    HTOUSBS('0'), HTOUSBS('0'), HTOUSBS('0'), HTOUSBS('0'),
    HTOUSBS('0'), HTOUSBS('0'), HTOUSBS('0'), HTOUSBS('0'),
    HTOUSBS('0'), HTOUSBS('1')
  }
};

typedef struct {
  uint8_t const *data;
  uint16_t      length;
} string_table_t;

static string_table_t const strings[] = {
  {(uint8_t const*)&string_lang,         sizeof string_lang},
  {(uint8_t const*)&string_manufacturer, sizeof string_manufacturer},
  {(uint8_t const*)&string_product,      sizeof string_product},
  {(uint8_t const*)&string_serial,       sizeof string_serial}
};
static uint32_t const stringCount = sizeof(strings)/sizeof(strings[0]);

/** Callbacks **/

static int          Configure(void);
static uint8_t      Reset(usb_speed_t);
static usb_result_t GetDescriptor(uint16_t, uint16_t,
                                  uint8_t const **, uint16_t *);
static uint8_t      GetConfiguration(void);
static usb_result_t SetConfiguration(uint16_t);
static uint16_t     GetStatus(void);
static usb_result_t ClassNoDataSetup(usb_setup_packet_t const *);
static usb_result_t ClassInDataSetup(usb_setup_packet_t const *,
                                     uint8_t const **, uint16_t *);
static usb_result_t ClassOutDataSetup(usb_setup_packet_t const *,
                                      uint8_t **);
static void         ClassStatusIn(usb_setup_packet_t const *);
static void         EP1OUT(void);
static void         EP2IN(void);

static usbd_callback_list_t const ApplicationCallBacks = {
  Configure, Reset, 0, GetDescriptor, 0,
  GetConfiguration, SetConfiguration, GetStatus, 0, 0,
  0, 0,
  ClassNoDataSetup, ClassInDataSetup,
  ClassOutDataSetup, ClassStatusIn,
  {0, EP2IN, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {EP1OUT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  0, 0
};

/** COM implementation **/

static uint16_t ep2queue;
static uint8_t configuration, refresh, rs232state;
static usb_cdc_line_coding_t rs232coding;

/* rs232state bits */
#define DTR  0x01
#define DCD  0x02
#define DSR  0x04
#define RTS  0x08
#define CTS  0x10

/* Print diagnostic information on LCD. */
static void LCDrefresh(void) {
  if (refresh) {
     char *stop, parity;

    refresh = 0;
    if (rs232coding.bCharFormat == ONE_STOP_BIT)
      stop = "1  ";
    else if (rs232coding.bCharFormat == ONE_AND_HALF_STOP_BITS)
      stop = "1.5";
    else if (rs232coding.bCharFormat == TWO_STOP_BITS)
      stop = "2  ";
    else
      stop = "?  ";
    if (rs232coding.bParityType == NO_PARITY)
      parity = 'N';
    else if (rs232coding.bParityType == ODD_PARITY)
      parity = 'O';
    else if (rs232coding.bParityType == EVEN_PARITY)
      parity = 'E';
    else if (rs232coding.bParityType == MARK_PARITY)
      parity = 'M';
    else if (rs232coding.bParityType == SPACE_PARITY)
      parity = 'S';
    else
      parity = '?';
    dbprintf( "%6lu %u%c%s%2hu", rs232coding.dwDTERate,
            rs232coding.bDataBits, parity, stop, ep2queue);
    if (rs232state & RTS)
      dbprintf("RTS ");
    else
      dbprintf("    ");
    if (rs232state & CTS)
      dbprintf("CTS ");
    else
      dbprintf("    ");
    if (rs232state & DTR)
      dbprintf("DTR ");
    else
      dbprintf("    ");
    if (rs232state & DSR)
      dbprintf("DSR ");
    else
      dbprintf("    ");
    if (rs232state & DCD)
      dbprintf("DCD ");
    else
      dbprintf("    ");
  }
}

static void ResetState(void) {
  ep2queue = 0;
  configuration = 0;
  rs232coding.dwDTERate = 38400;
  rs232coding.bCharFormat = ONE_STOP_BIT;
  rs232coding.bParityType = NO_PARITY;
  rs232coding.bDataBits = 8;
  rs232state = 0;
  refresh = 1;
}

usbd_callback_list_t const * USBDgetApplicationCallbacks() {
  return &ApplicationCallBacks;
}

int Configure() {
  ResetState();
  return 0;
}

uint8_t Reset(usb_speed_t speed) {
  //ErrorResetable(speed == FULL_SPEED ? 0 : -1, 6);
  ResetState();

  /* Default control endpoint must be configured here. */
  if (USBDendPointConfigure(ENDP0, CONTROL_TRANSFER,
                            device_descriptor.bMaxPacketSize0,
                            device_descriptor.bMaxPacketSize0) !=
                                                      REQUEST_SUCCESS)
    //ErrorResetable(-1, 7);

  return device_descriptor.bMaxPacketSize0;
}

usb_result_t GetDescriptor(uint16_t wValue, uint16_t wIndex,
                           uint8_t const **data, uint16_t *length) {
  uint32_t index = wValue & 0xff;

  switch (wValue >> 8) {
    case DEVICE_DESCRIPTOR:
      if (index == 0 && wIndex == 0) {
        *data = (uint8_t const *)&device_descriptor;
        *length = sizeof(device_descriptor);
        return REQUEST_SUCCESS;
      }
      return REQUEST_ERROR;
    case CONFIGURATION_DESCRIPTOR:
      if (index == 0 && wIndex == 0) {
        *data = (uint8_t const *)&com_configuration;
        *length = sizeof(com_configuration);
        return REQUEST_SUCCESS;
      }
      return REQUEST_ERROR;
    case STRING_DESCRIPTOR:
      if (index < stringCount) {
        *data = strings[index].data;
        *length = strings[index].length;
        return REQUEST_SUCCESS;
      }
      return REQUEST_ERROR;
    default:
      return REQUEST_ERROR;
  }
}

uint8_t GetConfiguration() {
  return configuration;
}

usb_result_t SetConfiguration(uint16_t confValue) {
  if (confValue > device_descriptor.bNumConfigurations)
    return REQUEST_ERROR;

  configuration = confValue;
  USBDdisableAllNonControlEndPoints();
  if (confValue == com_configuration.cnf_descr.bConfigurationValue) {
    usb_result_t r1, r2;

    r1 = USBDendPointConfigure(ENDP1, BULK_TRANSFER,
                               BLK_BUFF_SIZE, BLK_BUFF_SIZE);
    r2 = USBDendPointConfigure(ENDP2, INTERRUPT_TRANSFER,
                               0, INT_BUFF_SIZE);
    if (r1 == REQUEST_SUCCESS && r2 == REQUEST_SUCCESS)
      return REQUEST_SUCCESS;
    else
      return REQUEST_ERROR;
  }

  return REQUEST_SUCCESS; /* confValue == 0 */
}

uint16_t GetStatus() {
  /* Current power setting should be reported. */
  if (com_configuration.cnf_descr.bmAttributes & SELF_POWERED)
    return STATUS_SELF_POWERED;
  else
    return 0;
}

static usb_cdc_serial_state_t state = {
  {DEVICE_TO_HOST | CLASS_REQUEST | INTERFACE_RECIPIENT,
   SERIAL_STATE, 0, 0, 2}, 0
};

usb_result_t ClassNoDataSetup(usb_setup_packet_t const *setup) {
  if (setup->bmRequestType == (HOST_TO_DEVICE |
                               CLASS_REQUEST |
                               INTERFACE_RECIPIENT) &&
      setup->bRequest == SET_CONTROL_LINE_STATE &&
      setup->wIndex == 0 &&
      setup->wLength == 0) {
    uint8_t new_rs232state;

    /* Host do device: DTR or RTS notification */
    new_rs232state = rs232state;
    if (setup->wValue & 1) /* DTR set */
      new_rs232state |= (DTR | DSR | DCD);
    else
      new_rs232state &= ~(DTR | DSR | DCD);
    if (setup->wValue & 2) /* RTS set */
      new_rs232state |= RTS | CTS;
    else
      new_rs232state &= ~(RTS | CTS);

    /* Device to host: DCD or DSR notification */
    if ((rs232state ^ new_rs232state) & (DCD | DSR)) {
      state.wData = 0;
      if (new_rs232state & DCD)
        state.wData |= 1;
      if (new_rs232state & DSR)
        state.wData |= 2;
      if (ep2queue == 0)
        USBDwrite(ENDP2, (uint8_t const *)&state, sizeof(state));
      if (ep2queue < 2)
        ++ep2queue;
    }

    /* Set new state. */
    if (rs232state != new_rs232state) {
      rs232state = new_rs232state;
      refresh = 1;
    }

    return REQUEST_SUCCESS;
  }
  return REQUEST_ERROR;
}

usb_result_t ClassInDataSetup(usb_setup_packet_t const *setup,
                              uint8_t const **data,
                              uint16_t *length) {
  if (setup->bmRequestType == (DEVICE_TO_HOST |
                               CLASS_REQUEST |
                               INTERFACE_RECIPIENT) &&
      setup->bRequest == GET_LINE_CODING &&
      setup->wValue == 0 &&
      setup->wIndex == 0) {
    *data = (const uint8_t *)&rs232coding;
    *length = sizeof(rs232coding);
    return REQUEST_SUCCESS;
  }
  return REQUEST_ERROR;
}

usb_result_t ClassOutDataSetup(usb_setup_packet_t const *setup,
                               uint8_t **data) {
  if (setup->bmRequestType == (HOST_TO_DEVICE |
                               CLASS_REQUEST |
                               INTERFACE_RECIPIENT) &&
      setup->bRequest == SET_LINE_CODING &&
      setup->wValue == 0 &&
      setup->wIndex == 0 &&
      setup->wLength == sizeof(rs232coding)) {
    *data = (uint8_t *)&rs232coding;
    return REQUEST_SUCCESS;
  }
  return REQUEST_ERROR;
}

void ClassStatusIn(usb_setup_packet_t const *setup) {
  if (setup->bmRequestType == (HOST_TO_DEVICE |
                               CLASS_REQUEST |
                               INTERFACE_RECIPIENT) &&
      setup->bRequest == SET_LINE_CODING &&
      setup->wValue == 0 &&
      setup->wIndex == 0 &&
      setup->wLength == sizeof(rs232coding)) {
    refresh = 1;
  }
}

void EP2IN() {
  if (ep2queue > 0)
    --ep2queue;
  if (ep2queue > 0)
    USBDwrite(ENDP2, (uint8_t const *)&state, sizeof(state));
  refresh = 1;
}

static uint8_t const help[] =
  "Press:\r\n"
  "  G to switch green LED on,\r\n"
  "  g to switch green LED off,\r\n"
  "  R to switch red LED on,\r\n"
  "  r to switch red LED off,\r\n"
  "  W to switch white LED on,\r\n"
  "  w to switch white LED off.\r\n";

void EP1OUT() {
  uint8_t buffer[BLK_BUFF_SIZE];
  uint16_t i, len;

  len = USBDread(ENDP1, buffer, BLK_BUFF_SIZE);
  for (i = 0; i < len; ++i) {
    switch (buffer[i]) {
#if 0
      case 'G':
        GreenLEDon();
        break;
      case 'g':
        GreenLEDoff();
        break;
      case 'R':
        RedLEDon();
        break;
      case 'r':
        RedLEDoff();
        break;
      case 'W':
        PowerLEDon();
        break;
      case 'w':
        PowerLEDoff();
        break;
      case ' ':
      case '\n':
      case '\r':
      case '\t':
        break;
#endif
      default:
        USBDwriteEx(ENDP1, help, sizeof(help) - 1);
        return;
    }
  }
}
