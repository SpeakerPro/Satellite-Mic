/******************************************************************************//**
 * @file     descriptors.c
 * @version  V0.10
 * @brief    M251 series USBD descriptor
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "NuMicro.h"
#include "GW_hid_Transfer.h"

/*!<USB HID Report Descriptor */
uint8_t HID_DeviceReportDescriptor[] =
{
    0x06, 0x00, 0xFF,   // Usage Page = 0xFF00 (Vendor Defined Page 1)
    0x09, 0x01,         // Usage (Vendor Usage 1)
    0xA1, 0x01,         // Collection (Application)
    0x19, 0x01,         // Usage Minimum
    0x29, 0x40,         // Usage Maximum //64 input usages total (0x01 to 0x40)
    0x15, 0x00,         // Logical Minimum (data bytes in the report may have minimum value = 0x00)
    0x26, 0xFF, 0x00,   // Logical Maximum (data bytes in the report may have maximum value = 0x00FF = unsigned 255)
    0x75, 0x08,         // Report Size: 8-bit field size
    0x95, 0x40,         // Report Count: Make sixty-four 8-bit fields (the next time the parser hits
    // an "Input", "Output", or "Feature" item)
    0x81, 0x00,         // Input (Data, Array, Abs): Instantiates input packet fields based on the
    // above report size, count, logical min/max, and usage.
    0x19, 0x01,         // Usage Minimum
    0x29, 0x40,         // Usage Maximum //64 output usages total (0x01 to 0x40)
    0x91, 0x00,         // Output (Data, Array, Abs): Instantiates output packet fields. Uses same
    // report size and count as "Input" fields, since nothing new/different was
    // specified to the parser since the "Input" item.
    0xC0                // End Collection
};

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
const uint8_t gu8DeviceDescriptor[] = {
    LEN_DEVICE,  /* bLength */
    DESC_DEVICE, /* bDescriptorType */
#ifdef SUPPORT_LPM
    0x01, 0x02, /* bcdUSB => 0x0201 to support LPM */
#else
    0x10, 0x01, /* bcdUSB */
#endif
    0x00,             /* bDeviceClass */
    0x00,             /* bDeviceSubClass */
    0x00,             /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE, /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF, (USBD_VID & 0xFF00) >> 8,
    /* idProduct */
    USBD_PID & 0x00FF, (USBD_PID & 0xFF00) >> 8,
    VERSION_MINOR_ID , VERSION_MAJOR_ID, /* bcdDevice */
    0x01,       /* iManufacture */
    0x02,       /* iProduct */
    0x03,       /* iSerialNumber */
    0x01 /* bNumConfigurations */
};

/*!<USB Configure Descriptor */
const uint8_t gu8ConfigDescriptor[] = {
    LEN_CONFIG,  /* bLength */
    DESC_CONFIG, /* bDescriptorType */
    /* wTotalLength */
    (LEN_CONFIG + LEN_INTERFACE + LEN_HID + LEN_ENDPOINT * 2) & 0x00FF,
    ((LEN_CONFIG + LEN_INTERFACE + LEN_HID + LEN_ENDPOINT * 2) & 0xFF00) >> 8,
    0x01, /* bNumInterfaces */
    0x01, /* bConfigurationValue */
    0x00, /* iConfiguration */
    0x80 | (USBD_SELF_POWERED << 6) |
        (USBD_REMOTE_WAKEUP << 5), /* bmAttributes */
    USBD_MAX_POWER,                /* MaxPower */

    /* I/F descr: HID */
    LEN_INTERFACE,  /* bLength */
    DESC_INTERFACE, /* bDescriptorType */
    0x00,           /* bInterfaceNumber */
    0x00,           /* bAlternateSetting */
    0x02,           /* bNumEndpoints */
    0x03,           /* bInterfaceClass */
    0x00,           /* bInterfaceSubClass */
    0x00,           /* bInterfaceProtocol */
    0x00,           /* iInterface */

    /* HID Descriptor */
    LEN_HID,      /* Size of this descriptor in UINT8s. */
    DESC_HID,     /* HID descriptor type. */
    0x10, 0x01,   /* HID Class Spec. release number. */
    0x00,         /* H/W target country. */
    0x01,         /* Number of HID class descriptors to follow. */
    DESC_HID_RPT, /* Descriptor type. */
    /* Total length of report descriptor. */
    sizeof(HID_DeviceReportDescriptor) & 0x00FF,
    (sizeof(HID_DeviceReportDescriptor) & 0xFF00) >> 8,

    /* EP Descriptor: interrupt in. */
    LEN_ENDPOINT,               /* bLength */
    DESC_ENDPOINT,              /* bDescriptorType */
    (_INT_IN_EP_NUM | EP_INPUT), /* bEndpointAddress */
    EP_INT,                     /* bmAttributes */
    /* wMaxPacketSize */
    EP2_MAX_PKT_SIZE & 0x00FF, (EP2_MAX_PKT_SIZE & 0xFF00) >> 8,
    HID_DEFAULT_INT_IN_INTERVAL, /* bInterval */

    /* EP Descriptor: interrupt out. */
    LEN_ENDPOINT,                 /* bLength */
    DESC_ENDPOINT,                /* bDescriptorType */
    (_INT_OUT_EP_NUM | EP_OUTPUT), /* bEndpointAddress */
    EP_INT,                       /* bmAttributes */
    /* wMaxPacketSize */
    EP3_MAX_PKT_SIZE & 0x00FF, (EP3_MAX_PKT_SIZE & 0xFF00) >> 8,
    HID_DEFAULT_INT_IN_INTERVAL /* bInterval */
};

const uint8_t gu8BosDescriptor[] =
{
    LEN_BOS,                         /* bLength         */
    DESC_BOS,                        /* bDescriptorType */
    ((LEN_BOS + LEN_DEVCAP) & 0xFF), /* wTotalLength    */
    ((LEN_BOS + LEN_DEVCAP) >> 8),   /* wTotalLength    */
    0x01,                            /* bNumDevcieCaps  */
    LEN_DEVCAP,                      /* bLength         */
    DESC_CAPABILITY,                 /* bDescriptorType */
    CAP_USB20_EXT,                   /* bDevCapabilityType, 0x02 is USB 2.0 Extension */
    0x06, 0x04, 0x00, 0x00           /* bmAttributs, 32 bits                          */
    /* bit 0     : Reserved. Must be 0.                                         */
    /* bit 1     : Set 1 to support LPM.                                        */
    /* bit 2     : Set 1 to support BSL & Alternative HIRD.                     */
    /* bit 3     : Set 1 to use Baseline BESL.                                  */
    /* bit 4     : Set 1 to use Deep BESL.                                      */
    /* bit 11:8  : Baseline BESL value. Ignore it if bit3 is zero.              */
    /* bit 15:12 : Deep BESL value. Ignore it if bit4 is zero.                  */
    /* bit 31:16 : Reserved. Must be 0.                                         */
};


/*!<USB Language String Descriptor */
const uint8_t gu8StringLang[4] =
{
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
const uint8_t gu8VendorStringDesc[] =
{
    14,
    DESC_STRING,
    'L', 0, 'e', 0, 'n', 0, 'o', 0, 'v', 0, 'o', 0
};

/*!<USB Product String Descriptor */
const uint8_t gu8ProductStringDesc[] =
{
    50,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'M', 0, 'e', 0, 'e', 0, 't', 0, 'i', 0, 'n', 0, 'g', 0, ' ', 0, 
		'S', 0, 'p', 0, 'e', 0, 'a', 0, 'k', 0, 'e', 0, 'r', 0, ' ', 0, 
		'P', 0, 'r', 0, 'o', 0, 'D', 0, 'S', 0, '3', 0, '0', 0, '0', 0
};


const uint8_t gu8StringSerial[26] =
{
    4,             // bLength
    DESC_STRING,    // bDescriptorType
    '0', 0, 
};

const uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    gu8StringSerial
};

const uint8_t *gpu8UsbHidReport[2] =
{
    HID_DeviceReportDescriptor,
    NULL
};

const uint32_t gu32UsbHidReportLen[2] =
{
    sizeof(HID_DeviceReportDescriptor),
    0
};

const uint32_t gu32ConfigHidDescIdx[2] =
{
    (LEN_CONFIG + LEN_INTERFACE),
    0
};

const S_USBD_INFO_T gsInfo =
{
    (uint8_t *) gu8DeviceDescriptor,
    (uint8_t *) gu8ConfigDescriptor,
    (uint8_t **)gpu8UsbString,
    (uint8_t **)gpu8UsbHidReport,
#ifdef SUPPORT_LPM
    (uint8_t *) gu8BosDescriptor,
#else
    0,
#endif
    (uint32_t *)gu32UsbHidReportLen,
    (uint32_t *)gu32ConfigHidDescIdx,
};

