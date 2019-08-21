/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                AUDIO Class  Description
  *          ===================================================================
 *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (limited to Mute control)
  *             - Audio Synchronization type: Asynchronous
  *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
  *          The current audio class version supports the following audio features:
  *             - Pulse Coded Modulation (PCM) format
  *             - sampling rate: 48KHz.
  *             - Bit resolution: 16
  *             - Number of channels: 2
  *             - No volume control
  *             - Mute/Unmute capability
  *             - Asynchronous Endpoints
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

  /* BSPDependencies
  - "stm32xxxxx_{eval}{discovery}.c"
  - "stm32xxxxx_{eval}{discovery}_io.c"
  - "stm32xxxxx_{eval}{discovery}_audio.c"
  EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_midi_cdc.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_AUDIO
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_MIDI_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_MIDI_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_MIDI_Private_Macros
  * @{
  */
/**
  * @}
  */
/** @defgroup USBD_MIDI_Private_FunctionPrototypes
  * @{
  */

static uint8_t  USBD_MIDI_CDC_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_MIDI_CDC_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx); 
static uint8_t  USBD_MIDI_CDC_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t  *USBD_MIDI_CDC_GetCfgDesc (uint16_t *length);
static uint8_t  *USBD_MIDI_CDC_GetDeviceQualifierDesc (uint16_t *length);
static uint8_t  USBD_MIDI_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_MIDI_CDC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_MIDI_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev);


static uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);





/**
  * @}
  */

/** @defgroup USBD_MIDI_Private_Variables
  * @{
  */

USBD_ClassTypeDef  USBD_MIDI_CDC =
{
  USBD_MIDI_CDC_Init,
  USBD_MIDI_CDC_DeInit,
  USBD_MIDI_CDC_Setup,
  NULL,
  USBD_MIDI_CDC_EP0_RxReady,
  USBD_MIDI_CDC_DataIn,
  USBD_MIDI_CDC_DataOut,
  NULL,
  NULL,
  NULL,
  USBD_MIDI_CDC_GetCfgDesc,
  USBD_MIDI_CDC_GetCfgDesc,
  USBD_MIDI_CDC_GetCfgDesc,
  USBD_MIDI_CDC_GetDeviceQualifierDesc,
};

/* USB AUDIO device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MIDI_CDC_CfgDesc[USB_MIDI_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration 1 */
  0x09,                                 /* bLength */
  USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
  LOBYTE(USB_MIDI_CONFIG_DESC_SIZ),    /* wTotalLength  195 bytes*/
  HIBYTE(USB_MIDI_CONFIG_DESC_SIZ),
  TOTAL_NUM_INTERFACES,                 /* bNumInterfaces */
  0x01,                                 /* bConfigurationValue */
  0x00,                                 /* iConfiguration */
  0xC0,                                 /* bmAttributes  BUS Powred*/
  0x32,                                 /* bMaxPower = 100 mA*/
  /* 09 byte*/

  /* USB-MIDI Synthesizer Standard AC Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  AUDIO_AC_INTERFACE_NUMBER,             /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  AUDIO_AC_NUM_ENDPOINTS,               /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_AUDIOCONTROL,          /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB-MIDI Synthesizer Class-specific AC Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,      /* bDescriptorType */
  AUDIO_CONTROL_HEADER,                 /* bDescriptorSubtype */
  0x00,          /* 1.00 */             /* bcdADC */
  0x01,
  0x09,                                 /* wTotalLength = 09*/
  0x00,
  0x01,                                 /* bInCollection */
  AUDIO_MS_INTERFACE_NUMBER,            /* baInterfaceNr */
  /* 09 byte*/

  /* USB-MIDI Synthesizer Standard MS Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  AUDIO_MS_INTERFACE_NUMBER,             /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  AUDIO_MS_NUM_ENDPOINTS,               /* bNumEndpoints */
  USB_DEVICE_CLASS_AUDIO,               /* bInterfaceClass */
  AUDIO_SUBCLASS_MIDISTREAMING,         /* bInterfaceSubClass */
  AUDIO_PROTOCOL_UNDEFINED,             /* bInterfaceProtocol */
  0x00,                                 /* iInterface */
  /* 09 byte*/

  /* USB-MIDI Synthesizer Class-specific MS Interface Descriptor */
  MS_CLASS_SPECIFIED_INTERFACE_DESC_SIZE,/* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,       /* bDescriptorType */
  AUDIO_CONTROL_HEADER,                  /* bDescriptorSubType */
  0x01,           /* 1.00 */             /* bcdMSC */
  0x00,
  0x25,                                  /* wTotalLength = 37 */
  0x00,
  /* 07 byte*/

  /* USB-MIDI Synthesizer MIDI IN Jack Descripter (Embedded) */
  MIDI_IN_JACK_DESC_SIZE,                /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,       /* bDescriptorType */
  MIDI_IN_JACK,                          /* bDescriptorSubType */
  JACKTYPE_EMBEDDED,                     /* bJackType */
  0x01,                                  /* bJackID */
  0x00,                                  /* iJack */
  /* 06 byte*/

  /* USB-MIDI Synthesizer MIDI IN Jack Descripter (External) */
  MIDI_IN_JACK_DESC_SIZE,                /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,       /* bDescriptorType */
  MIDI_IN_JACK,                          /* bDescriptorSubType */
  JACKTYPE_EXTERNAL,                     /* bJackType */
  0x02,                                  /* bJackID */
  0x00,                                  /* iJack */
  /* 06 byte*/
  
  /* USB-MIDI Synthesizer MIDI OUT Jack Descripter (Embedded) */
  MIDI_OUT_JACK_DESC_SIZE,               /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,       /* bDescriptorType */
  MIDI_OUT_JACK,                         /* bDescriptorSubType */
  JACKTYPE_EMBEDDED,                     /* bJackType */
  0x03,                                  /* bJackID */
  0x01,                                  /* bNrInputPins */
  0x02,                                  /* baSourceID(1) 0x02=Ext MIDI In Jack */
  0x01,                                  /* baSourcePin(1) */
  0x00,                                  /* iJack */
  /* 09 byte*/

  /* USB-MIDI Synthesizer MIDI OUT Jack Descripter (External) */
  MIDI_OUT_JACK_DESC_SIZE,                /* bLength */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,       /* bDescriptorType */
  MIDI_OUT_JACK,                         /* bDescriptorSubType */
  JACKTYPE_EXTERNAL,                     /* bJackType */
  0x04,                                  /* bJackID */
  0x01,                                  /* bNrInputPins */
  0x01,                                  /* baSourceID(1) 0x02=Ext MIDI In Jack */
  0x01,                                  /* baSourcePin(1) */
  0x00,                                  /* iJack */
  /* 09 byte*/

  /* USB-MIDI Synthesizer Standard Bulk OUT EndPoint Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
  MIDI_OUT_EP,                         /* bEndpointAddress 1 out endpoint*/
  USBD_EP_TYPE_BULK,                    /* bmAttributes */
  LOBYTE(BULK_SEND_PACKET),             /* wMaxPacketSize */
  HIBYTE(BULK_SEND_PACKET),
  0x01,                                 /* bInterval */
  0x00,                                 /* bRefresh */
  0x00,                                 /* bSynchAddress */
  /* 09 byte*/

  /* USB-MIDI Synthesizer Class-specific Bulk OUT EndPoint Descriptor */
  AUDIO_CLASS_SPECIFIED_ENDPOINT_DESC_SIZE, /* bLength */
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
  MS_GENERAL,                           /* bDescriptorSubType */
  0x01,                                 /* bNumEmbMIDIJack */
  0x01,                                 /* baAssocJackID */
  /* 05 byte*/

  /* USB-MIDI Synthesizer Standard Bulk IN EndPoint Descriptor */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    /* bLength */
  USB_DESC_TYPE_ENDPOINT,               /* bDescriptorType */
  MIDI_IN_EP,                          /* bEndpointAddress 1 out endpoint*/
  USBD_EP_TYPE_BULK,                    /* bmAttributes */
  LOBYTE(BULK_SEND_PACKET),             /* wMaxPacketSize */
  HIBYTE(BULK_SEND_PACKET),
  0x00,                                 /* bInterval */
  0x00,                                 /* bRefresh */
  0x00,                                 /* bSynchAddress */
  /* 09 byte*/

  /* USB-MIDI Synthesizer Class-specific Bulk IN EndPoint Descriptor */
  AUDIO_CLASS_SPECIFIED_ENDPOINT_DESC_SIZE, /* bLength */
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,       /* bDescriptorType */
  MS_GENERAL,                           /* bDescriptorSubType */
  0x01,                                 /* bNumEmbMIDIJack */
  0x03,                                 /* baAssocJackID */
  /* 05 byte*/
 
  /*---------------------------------------------------------------------------*/
  0x08,// 75 bLength: Interface Descriptor size
  0x0B,								// 76 bDescriptorType: IAD
  CDC_CDC_INTERFACE_NUMBER,		// 77 bFirstInterface
  CDC_NUM_INTERFACES,					// 78 bInterfaceCount
  0x02,								// 79 bFunctionClass: CDC
  0x02,								// 80 bFunctionSubClass
  0x01,								// 81 bFunctionProtocol
  0x02,	

  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  CDC_CDC_INTERFACE_NUMBER,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  CDC_CDC_NUM_ENDPOINTS,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */

  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,

  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */

  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */

  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */

  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SIZE),
  CDC_FS_BINTERVAL,                           /* bInterval: */
  /*---------------------------------------------------------------------------*/

  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  CDC_DATA_INTERFACE_NUMBER,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  CDC_DATA_NUM_ENDPOINTS,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */

  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */

  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  CDC_IN_EP,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
  0x00,  

};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MIDI_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END=
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */

/** @defgroup USBD_MIDI_Private_Functions
  * @{
  */

/**
  * @brief  USBD_MIDI_CDC_Init
  *         Initialize the MIDI and the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_MIDI_CDC_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_MIDI_CDC_HandleTypeDef   *hcmpst;
  USBD_MIDI_CDC_ItfTypeDef      *fops;

  (void)cfgidx;

  // MIDI
  /* Open EP OUT */
  USBD_LL_OpenEP(pdev, MIDI_OUT_EP, USBD_EP_TYPE_BULK, BULK_SEND_PACKET);
  pdev->ep_out[MIDI_OUT_EP & 0xFU].is_used = 1U;
  /* Open EP IN */
  // NOTE:Unused but leave enabled for future.
  USBD_LL_OpenEP(pdev, MIDI_IN_EP, USBD_EP_TYPE_BULK, BULK_SEND_PACKET);
  pdev->ep_in[MIDI_IN_EP & 0xFU].is_used = 1U;

  // CDC
  /* Open EP IN */
  USBD_LL_OpenEP(pdev, CDC_IN_EP, USBD_EP_TYPE_BULK, CDC_DATA_FS_IN_PACKET_SIZE);
  pdev->ep_in[CDC_IN_EP & 0xFU].is_used = 1U;
  /* Open EP OUT */
  USBD_LL_OpenEP(pdev, CDC_OUT_EP, USBD_EP_TYPE_BULK, CDC_DATA_FS_OUT_PACKET_SIZE); 
  pdev->ep_out[CDC_OUT_EP & 0xFU].is_used = 1U;
  /* Open Command IN EP */
  USBD_LL_OpenEP(pdev, CDC_CMD_EP, USBD_EP_TYPE_INTR, CDC_CMD_PACKET_SIZE);
  pdev->ep_in[CDC_CMD_EP & 0xFU].is_used = 1U;


  /* Allocate Audio structure */
  pdev->pClassData = USBD_malloc(sizeof (USBD_MIDI_CDC_HandleTypeDef));

  if(pdev->pClassData == NULL)
  {
    return USBD_FAIL;
  }
  else
  {
    hcmpst = (USBD_MIDI_CDC_HandleTypeDef*) pdev->pClassData;
    
    // clear
    memset(&hcmpst->hmidi, 0, sizeof(hcmpst->hmidi));
    memset(&hcmpst->hcdc, 0, sizeof(hcmpst->hcdc));
    
    fops = (USBD_MIDI_CDC_ItfTypeDef *)pdev->pUserData;
    /* Init  physical Interface components */
    if ( fops != NULL ) {
      fops->ifmidi.Init();
      fops->ifcdc.Init();
    }
    
    /* Prepare Out endpoint to receive 1st packet */
    USBD_LL_PrepareReceive(pdev, MIDI_OUT_EP, hcmpst->hmidi.RxBuffer, BULK_SEND_PACKET);
    USBD_LL_PrepareReceive(pdev, CDC_OUT_EP,  hcmpst->hcdc.RxBuffer, CDC_DATA_FS_OUT_PACKET_SIZE);

    return USBD_OK;
  }
}

/**
  * @brief  USBD_MIDI_CDC_DeInit
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_MIDI_CDC_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx)
{
  (void)cfgidx;

  USBD_MIDI_CDC_ItfTypeDef      *fops;
  // MIDI
  /* Close EP OUT */
  USBD_LL_CloseEP(pdev, MIDI_OUT_EP);
  pdev->ep_out[MIDI_OUT_EP & 0xFU].is_used = 0U;
  /* Close EP IN */
  USBD_LL_CloseEP(pdev, MIDI_IN_EP);
  pdev->ep_in[MIDI_IN_EP & 0xFU].is_used = 0U;
  
  // CDC
  /* Close EP IN */
  USBD_LL_CloseEP(pdev, CDC_IN_EP);
  pdev->ep_in[CDC_IN_EP & 0xFU].is_used = 0U;
  /* Close EP OUT */
  USBD_LL_CloseEP(pdev, CDC_OUT_EP);
  pdev->ep_out[CDC_OUT_EP & 0xFU].is_used = 0U;
  /* Close Command IN EP */
  USBD_LL_CloseEP(pdev, CDC_CMD_EP);
  pdev->ep_in[CDC_CMD_EP & 0xFU].is_used = 0U;


  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    fops = pdev->pUserData;
    if ( fops != NULL ) {
      fops->ifmidi.DeInit();
      fops->ifcdc.DeInit();
    }
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_CDC_Setup
  *         Handle the AUDIO and the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_MIDI_CDC_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
  uint8_t ret = USBD_OK;
  
  switch (req->wIndex) {
    case CDC_CDC_INTERFACE_NUMBER:
    USBD_CDC_Setup(pdev, req);
    break;

    case CDC_DATA_INTERFACE_NUMBER:
    // TODO
    break;

    case AUDIO_AC_INTERFACE_NUMBER:
    // TODO
    break;

    case AUDIO_MS_INTERFACE_NUMBER:
    // TODO
    break;

    default:
    break;
  }

  return ret;
}

/**
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_CDC_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{

  USBD_MIDI_CDC_HandleTypeDef   *hcmpst = (USBD_MIDI_CDC_HandleTypeDef*) pdev->pClassData;
  USBD_MIDI_CDC_ItfTypeDef      *fops   = (USBD_MIDI_CDC_ItfTypeDef   *) pdev->pUserData;
  uint8_t ifalt = 0U;
  uint16_t status_info = 0U;
  uint8_t ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    if (req->wLength)
    {
      if (req->bmRequest & 0x80U)
      {
        fops->ifcdc.Control(req->bRequest, (uint8_t *)(void *)hcmpst->hcdc.data, req->wLength);
        USBD_CtlSendData (pdev, (uint8_t *)(void *)hcmpst->hcdc.data, req->wLength);
      }
      else
      {
        hcmpst->hcdc.CmdOpCode = req->bRequest;
        hcmpst->hcdc.CmdLength = (uint8_t)req->wLength;

        USBD_CtlPrepareRx (pdev, (uint8_t *)(void *)hcmpst->hcdc.data, req->wLength);
      }
    }
    else
    {
      ((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                        (uint8_t *)(void *)req, 0U);
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_STATUS:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        USBD_CtlSendData (pdev, (uint8_t *)(void *)&status_info, 2U);
      }
      else
      {
        USBD_CtlError (pdev, req);
	ret = USBD_FAIL;
      }
      break;

    case USB_REQ_GET_INTERFACE:
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        USBD_CtlSendData (pdev, &ifalt, 1U);
      }
      else
      {
        USBD_CtlError (pdev, req);
	ret = USBD_FAIL;
      }
      break;

    case USB_REQ_SET_INTERFACE:
      if (pdev->dev_state != USBD_STATE_CONFIGURED)
      {
        USBD_CtlError (pdev, req);
	ret = USBD_FAIL;
      }
      break;

    default:
      USBD_CtlError (pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;

  default:
    USBD_CtlError (pdev, req);
    ret = USBD_FAIL;
    break;
  }

  return ret;
}


/**
  * @brief  USBD_CDC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_CDC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_CDC_HandleTypeDef *hcdc = &((USBD_MIDI_CDC_HandleTypeDef*)pdev->pClassData)->hcdc;
  PCD_HandleTypeDef *hpcd = pdev->pData;

  if(pdev->pClassData != NULL)
  {
    if((pdev->ep_in[epnum].total_length > 0U) && ((pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0U))
    {
      /* Update the packet total length */
      pdev->ep_in[epnum].total_length = 0U;

      /* Send ZLP */
      USBD_LL_Transmit (pdev, epnum, NULL, 0U);
    }
    else
    {
      hcdc->TxState = 0U;
    }
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
  * @brief  USBD_MIDI_CDC_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_MIDI_CDC_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_MIDI_CDC_CfgDesc);
  return USBD_MIDI_CDC_CfgDesc;
}

/**
  * @brief  USBD_MIDI_CDC_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MIDI_CDC_DataIn (USBD_HandleTypeDef *pdev,
                              uint8_t epnum)
{
  
  switch (epnum) {
  case CDC_IN_EP:
    USBD_CDC_DataIn(pdev, epnum);
    break;
  default:
    break;  
  }

  /* Only OUT data are processed */
  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_CDC_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_MIDI_CDC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef   *hcdc = &((USBD_MIDI_CDC_HandleTypeDef*) pdev->pClassData)->hcdc;

  if((pdev->pUserData != NULL) && (hcdc->CmdOpCode != 0xFFU))
  {
    (((USBD_MIDI_CDC_ItfTypeDef *)pdev->pUserData))->ifcdc.Control(hcdc->CmdOpCode,
                                                      (uint8_t *)(void *)hcdc->data,
                                                      (uint16_t)hcdc->CmdLength);
      hcdc->CmdOpCode = 0xFFU;

  }
  
  // TODO:mute, volume control

  return USBD_OK;
}


/**
  * @brief  USBD_MIDI_CDC_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MIDI_CDC_DataOut (USBD_HandleTypeDef *pdev,
                              uint8_t epnum)
{

  uint32_t size = 0;
  USBD_MIDI_CDC_HandleTypeDef   *hcmpst = NULL;
  USBD_MIDI_CDC_ItfTypeDef          *fops   = NULL;
  
  hcmpst = (USBD_MIDI_CDC_HandleTypeDef*) pdev->pClassData;  
  fops   = (USBD_MIDI_CDC_ItfTypeDef   *) pdev->pUserData;

  if ( ( hcmpst != NULL ) && ( fops != NULL ) ) {
    
    size = USBD_LL_GetRxDataSize(pdev,  epnum);

    switch(epnum) {
    case MIDI_OUT_EP:
      hcmpst->hmidi.RxLength = size;
      fops->ifmidi.Receive(hcmpst->hmidi.RxBuffer, &hcmpst->hmidi.RxLength);
      USBD_LL_PrepareReceive(pdev, MIDI_OUT_EP, &hcmpst->hmidi.RxBuffer[0], BULK_SEND_PACKET);
      break;

    case CDC_OUT_EP:
      hcmpst->hcdc.RxLength = size;
      fops->ifcdc.Receive(hcmpst->hcdc.RxBuffer, &hcmpst->hcdc.RxLength);
      //USBD_LL_PrepareReceive(pdev, CDC_OUT_EP, &hcmpst->hcdc.RxBuffer[0], BULK_SEND_PACKET);
      break;
    
    default:
      break;
    }

    return USBD_OK;
  }
  else {
    
    return USBD_FAIL;
  }
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_MIDI_CDC_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_MIDI_CDC_DeviceQualifierDesc);
  return USBD_MIDI_CDC_DeviceQualifierDesc;
}

/**
* @brief  USBD_MIDI_CDC_RegisterInterface
* @param  fops: Audio interface callback
* @retval status
*/
uint8_t  USBD_MIDI_CDC_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                        USBD_MIDI_CDC_ItfTypeDef *fops)
{
  if(fops != NULL)
  {
    pdev->pUserData= fops;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev, uint8_t  *pbuff, uint16_t length)
{
  USBD_CDC_HandleTypeDef   *hcdc = &((USBD_MIDI_CDC_HandleTypeDef*) pdev->pClassData)->hcdc;

  hcdc->TxBuffer = pbuff;
  hcdc->TxLength = length;

  return USBD_OK;
}


/**
  * @brief  USBD_CDC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetRxBuffer  (USBD_HandleTypeDef   *pdev, uint8_t  *pbuff)
{
  USBD_CDC_HandleTypeDef   *hcdc = &((USBD_MIDI_CDC_HandleTypeDef*) pdev->pClassData)->hcdc;

  hcdc->RxBuffer = pbuff;

  return USBD_OK;
}


/**
  * @brief  USBD_MIDI_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_MIDI_SetRxBuffer  (USBD_HandleTypeDef   *pdev, uint8_t  *pbuff)
{
  USBD_MIDI_HandleTypeDef   *hmidi = &((USBD_MIDI_CDC_HandleTypeDef*) pdev->pClassData)->hmidi;

  hmidi->RxBuffer = pbuff;

  return USBD_OK;
}

/**
  * @brief  USBD_CDC_TransmitPacket
  *         Transmit packet on IN endpoint
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef   *hcdc = &((USBD_MIDI_CDC_HandleTypeDef*) pdev->pClassData)->hcdc;

  if(pdev->pClassData != NULL)
  {
    if(hcdc->TxState == 0U)
    {
      /* Tx Transfer in progress */
      hcdc->TxState = 1U;

      /* Update the packet total length */
      pdev->ep_in[CDC_IN_EP & 0xFU].total_length = hcdc->TxLength;

      /* Transmit next packet */
      USBD_LL_Transmit(pdev, CDC_IN_EP, hcdc->TxBuffer,
                       (uint16_t)hcdc->TxLength);

      return USBD_OK;
    }
    else
    {
      return USBD_BUSY;
    }
  }
  else
  {
    return USBD_FAIL;
  }
}


/**
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{
  USBD_CDC_HandleTypeDef   *hcdc = &((USBD_MIDI_CDC_HandleTypeDef*) pdev->pClassData)->hcdc;

  /* Suspend or Resume USB Out process */
  if(pdev->pClassData != NULL)
  {

    /* Prepare Out endpoint to receive next packet */
    USBD_LL_PrepareReceive(pdev,
                           CDC_OUT_EP,
                           hcdc->RxBuffer,
                           CDC_DATA_FS_OUT_PACKET_SIZE);
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
