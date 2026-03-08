/*
    (C) 2015-2016 flabbergast <s3+flabbergast@sdfeu.org>
    Based on ChibiOS USB_CDC_IAD demo - Copyright (C) 2006..2026 Giovanni Di Sirio
    Dual CDC ACM for RP2xxx by Szilveszter Zsigmond

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "hal.h"

extern void usb_sof_notify(void);

/*
 * Virtual serial ports over USB.
 */
SerialUSBDriver SDU1;
SerialUSBDriver SDU2;

/*
 * Endpoints.
 */
#define USB_INTERRUPT_REQUEST_EP_A      1   /* CDC1 control (Intr IN)  */
#define USB_DATA_AVAILABLE_EP_A         2   /* CDC1 data    (Bulk OUT) */
#define USB_DATA_REQUEST_EP_A           2   /* CDC1 data    (Bulk IN)  */
#define USB_INTERRUPT_REQUEST_EP_B      3   /* CDC2 control (Intr IN)  */
#define USB_DATA_AVAILABLE_EP_B         4   /* CDC2 data    (Bulk OUT) */
#define USB_DATA_REQUEST_EP_B           4   /* CDC2 data    (Bulk IN)  */

#define USB_INTERRUPT_REQUEST_SIZE      0x10
#define USB_DATA_SIZE                   0x40

/*
 * Interfaces.
 */
#define USB_NUM_INTERFACES              4
#define USB_CDC_CIF_NUM0                0   /* CDC1 communication interface */
#define USB_CDC_DIF_NUM0                1   /* CDC1 data interface          */
#define USB_CDC_CIF_NUM1                2   /* CDC2 communication interface */
#define USB_CDC_DIF_NUM1                3   /* CDC2 data interface          */

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[] = {
  USB_DESC_DEVICE(
    0x0200,                                 /* bcdUSB (2.0).                */
    0xEF,                                   /* bDeviceClass (misc).         */
    0x02,                                   /* bDeviceSubClass (common).    */
    0x01,                                   /* bDeviceProtocol (IAD).       */
    USB_DATA_SIZE,                          /* bMaxPacketSize.              */
    0x2E8A,                                 /* idVendor.                    */
    0x0003,                                 /* idProduct.                   */
    0x0200,                                 /* bcdDevice.                   */
    1,                                      /* iManufacturer.               */
    2,                                      /* iProduct.                    */
    3,                                      /* iSerialNumber.               */
    1)                                      /* bNumConfigurations.          */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

#define CDC_IF_DESC_SET_SIZE                                                \
  (USB_DESC_INTERFACE_SIZE + 5 + 5 + 4 + 5 + USB_DESC_ENDPOINT_SIZE +     \
   USB_DESC_INTERFACE_SIZE + (USB_DESC_ENDPOINT_SIZE * 2))

#define CDC_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp)    \
  /* Interface Descriptor.*/                                               \
  USB_DESC_INTERFACE(                                                      \
    comIfNum,                               /* bInterfaceNumber.       */  \
    0x00,                                   /* bAlternateSetting.      */  \
    0x01,                                   /* bNumEndpoints.          */  \
    CDC_COMMUNICATION_INTERFACE_CLASS,      /* bInterfaceClass.        */  \
    CDC_ABSTRACT_CONTROL_MODEL,             /* bInterfaceSubClass.     */  \
    0x01,                                   /* bInterfaceProtocol.     */  \
    0),                                     /* iInterface.             */  \
  /* Header Functional Descriptor (CDC section 5.2.3).*/                  \
  USB_DESC_BYTE     (5),                                                   \
  USB_DESC_BYTE     (CDC_CS_INTERFACE),                                    \
  USB_DESC_BYTE     (CDC_HEADER),                                          \
  USB_DESC_BCD      (0x0110),                                              \
  /* Call Management Functional Descriptor.*/                              \
  USB_DESC_BYTE     (5),                                                   \
  USB_DESC_BYTE     (CDC_CS_INTERFACE),                                    \
  USB_DESC_BYTE     (CDC_CALL_MANAGEMENT),                                 \
  USB_DESC_BYTE     (0x03),                                                \
  USB_DESC_BYTE     (datIfNum),                                            \
  /* Abstract Control Management Functional Descriptor.*/                  \
  USB_DESC_BYTE     (4),                                                   \
  USB_DESC_BYTE     (CDC_CS_INTERFACE),                                    \
  USB_DESC_BYTE     (CDC_ABSTRACT_CONTROL_MANAGEMENT),                     \
  USB_DESC_BYTE     (0x02),                                                \
  /* Union Functional Descriptor.*/                                        \
  USB_DESC_BYTE     (5),                                                   \
  USB_DESC_BYTE     (CDC_CS_INTERFACE),                                    \
  USB_DESC_BYTE     (CDC_UNION),                                           \
  USB_DESC_BYTE     (comIfNum),                                            \
  USB_DESC_BYTE     (datIfNum),                                            \
  /* Endpoint, Interrupt IN.*/                                             \
  USB_DESC_ENDPOINT (                                                      \
    comInEp,                                                               \
    USB_EP_MODE_TYPE_INTR,                  /* bmAttributes.           */  \
    USB_INTERRUPT_REQUEST_SIZE,             /* wMaxPacketSize.         */  \
    0x01),                                  /* bInterval.              */  \
  /* CDC Data Interface Descriptor.*/                                      \
  USB_DESC_INTERFACE(                                                      \
    datIfNum,                               /* bInterfaceNumber.       */  \
    0x00,                                   /* bAlternateSetting.      */  \
    0x02,                                   /* bNumEndpoints.          */  \
    CDC_DATA_INTERFACE_CLASS,               /* bInterfaceClass.        */  \
    0x00,                                   /* bInterfaceSubClass.     */  \
    0x00,                                   /* bInterfaceProtocol.     */  \
    0x00),                                  /* iInterface.             */  \
  /* Endpoint, Bulk OUT.*/                                                 \
  USB_DESC_ENDPOINT(                                                       \
    datOutEp,                               /* bEndpointAddress.       */  \
    USB_EP_MODE_TYPE_BULK,                  /* bmAttributes.           */  \
    USB_DATA_SIZE,                          /* wMaxPacketSize.         */  \
    0x00),                                  /* bInterval.              */  \
  /* Endpoint, Bulk IN.*/                                                  \
  USB_DESC_ENDPOINT(                                                       \
    datInEp,                                /* bEndpointAddress.       */  \
    USB_EP_MODE_TYPE_BULK,                  /* bmAttributes.           */  \
    USB_DATA_SIZE,                          /* wMaxPacketSize.         */  \
    0x00)                                   /* bInterval.              */

#define IAD_CDC_IF_DESC_SET_SIZE                                           \
  (USB_DESC_INTERFACE_ASSOCIATION_SIZE + CDC_IF_DESC_SET_SIZE)

#define IAD_CDC_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp)\
  /* Interface Association Descriptor.*/                                   \
  USB_DESC_INTERFACE_ASSOCIATION(                                          \
    comIfNum,                               /* bFirstInterface.        */  \
    2,                                      /* bInterfaceCount.        */  \
    CDC_COMMUNICATION_INTERFACE_CLASS,      /* bFunctionClass.         */  \
    CDC_ABSTRACT_CONTROL_MODEL,             /* bFunctionSubClass.      */  \
    1,                                      /* bFunctionProtocol.      */  \
    0                                       /* iInterface.             */  \
  ),                                                                       \
  CDC_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp)

/* Configuration Descriptor tree for dual CDC with IAD. */
static const uint8_t vcom_configuration_descriptor_data[] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(
    USB_DESC_CONFIGURATION_SIZE +
    (IAD_CDC_IF_DESC_SET_SIZE * 2),         /* wTotalLength.                */
    USB_NUM_INTERFACES,                     /* bNumInterfaces.              */
    0x01,                                   /* bConfigurationValue.         */
    0,                                      /* iConfiguration.              */
    0xC0,                                   /* bmAttributes (self powered). */
    50                                      /* bMaxPower (100mA).           */
  ),
  IAD_CDC_IF_DESC_SET(
    USB_CDC_CIF_NUM0,
    USB_CDC_DIF_NUM0,
    USB_ENDPOINT_IN(USB_INTERRUPT_REQUEST_EP_A),
    USB_ENDPOINT_OUT(USB_DATA_AVAILABLE_EP_A),
    USB_ENDPOINT_IN(USB_DATA_REQUEST_EP_A)
  ),
  IAD_CDC_IF_DESC_SET(
    USB_CDC_CIF_NUM1,
    USB_CDC_DIF_NUM1,
    USB_ENDPOINT_IN(USB_INTERRUPT_REQUEST_EP_B),
    USB_ENDPOINT_OUT(USB_DATA_AVAILABLE_EP_B),
    USB_ENDPOINT_IN(USB_DATA_REQUEST_EP_B)
  ),
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[] = {
  USB_DESC_BYTE(2+2*14),                    /* Length.                          */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* Descriptor Type.                 */
  'R', 0, 'P', 0, '2', 0, 'x', 0, 'x', 0, 'x', 0, '-', 0, 'C', 0,
  'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0
};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(2+2*9),                    /* Length.                          */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* Descriptor Type.                 */
  'A', 0, 'r', 0, 'd', 0, 'u', 0, 'p', 0, 'i', 0, 'l', 0, 'o', 0,
  't', 0
};

/*
 * Serial Number string.
 */
static const uint8_t vcom_string3[] = {
  USB_DESC_BYTE(8),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  '0' + CH_KERNEL_MAJOR, 0,
  '0' + CH_KERNEL_MINOR, 0,
  '0' + CH_KERNEL_PATCH, 0
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {
  (void)usbp;
  (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4)
      return &vcom_strings[dindex];
  }
  return NULL;
}

/**
 * @brief   EP1: CDC1 control — IN (Interrupt).
 */
static USBInEndpointState ep1instate;

static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  USB_INTERRUPT_REQUEST_SIZE,
  0x0000,
  &ep1instate,
  NULL,
};

/**
 * @brief   EP2: CDC1 data — IN + OUT (Bulk).
 */
static USBInEndpointState ep2instate;
static USBOutEndpointState ep2outstate;

static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  sduDataTransmitted,
  sduDataReceived,
  USB_DATA_SIZE,
  USB_DATA_SIZE,
  &ep2instate,
  &ep2outstate,
};

/**
 * @brief   EP3: CDC2 control — IN (Interrupt).
 */
static USBInEndpointState ep3instate;

static const USBEndpointConfig ep3config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  USB_INTERRUPT_REQUEST_SIZE,
  0x0000,
  &ep3instate,
  NULL,
};

/**
 * @brief   EP4: CDC2 data — IN + OUT (Bulk).
 */
static USBInEndpointState ep4instate;
static USBOutEndpointState ep4outstate;

static const USBEndpointConfig ep4config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  sduDataTransmitted,
  sduDataReceived,
  USB_DATA_SIZE,
  USB_DATA_SIZE,
  &ep4instate,
  &ep4outstate,
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {
  extern SerialUSBDriver SDU1;
  extern SerialUSBDriver SDU2;

  switch (event) {
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();

    if (usbp->state == USB_ACTIVE) {
      /* Enables the endpoints specified into the configuration.
         Note, this callback is invoked from an ISR so I-Class functions
         must be used. */
      usbInitEndpointI(usbp, USB_INTERRUPT_REQUEST_EP_A, &ep1config);
      usbInitEndpointI(usbp, USB_DATA_REQUEST_EP_A,      &ep2config);
      usbInitEndpointI(usbp, USB_INTERRUPT_REQUEST_EP_B, &ep3config);
      usbInitEndpointI(usbp, USB_DATA_REQUEST_EP_B,      &ep4config);

      /* Resetting the state of the CDC subsystem. */
      sduConfigureHookI(&SDU1);
      sduConfigureHookI(&SDU2);
    } else if (usbp->state == USB_SELECTED) {
      usbDisableEndpointsI(usbp);
    }

    chSysUnlockFromISR();
    return;
  case USB_EVENT_RESET:
    /* Falls into.*/
  case USB_EVENT_UNCONFIGURED:
    /* Falls into.*/
  case USB_EVENT_SUSPEND:
    chSysLockFromISR();

    /* Disconnection event on suspend.*/
    sduSuspendHookI(&SDU1);
    sduSuspendHookI(&SDU2);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_WAKEUP:
    chSysLockFromISR();

    /* Connection event on wakeup.*/
    sduWakeupHookI(&SDU1);
    sduWakeupHookI(&SDU2);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

/*
 * Handles the USB SOF event.
 */
static void sof_handler(USBDriver *usbp) {

  (void)usbp;

  osalSysLockFromISR();
  sduSOFHookI(&SDU1);
  sduSOFHookI(&SDU2);
  osalSysUnlockFromISR();
  usb_sof_notify();
}

static bool requests_hook(USBDriver *usbp) {
  return sduRequestsHook(usbp);
}

/*
 * USB driver configuration.
 */
const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  requests_hook,
  sof_handler
};

/*
 * Serial over USB driver configuration 1 — MAVLink (/dev/ttyACM0).
 */
const SerialUSBConfig serusbcfg1 = {
  &USBD1,
  USB_DATA_REQUEST_EP_A,
  USB_DATA_AVAILABLE_EP_A,
  USB_INTERRUPT_REQUEST_EP_A
};

/*
 * Serial over USB driver configuration 2 — debug console (/dev/ttyACM1).
 */
const SerialUSBConfig serusbcfg2 = {
  &USBD1,
  USB_DATA_REQUEST_EP_B,
  USB_DATA_AVAILABLE_EP_B,
  USB_INTERRUPT_REQUEST_EP_B
};
