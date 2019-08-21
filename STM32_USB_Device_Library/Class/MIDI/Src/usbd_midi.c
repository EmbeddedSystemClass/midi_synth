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
#include "usbd_midi.h"
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
#define AUDIO_SAMPLE_FREQ(frq)      (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_PACKET_SZE(frq)          (uint8_t)(((frq * 2U * 2U)/1000U) & 0xFFU), \
                                       (uint8_t)((((frq * 2U * 2U)/1000U) >> 8) & 0xFFU)

/**
  * @}
  */




/** @defgroup USBD_MIDI_Private_FunctionPrototypes
  * @{
  */


static uint8_t  USBD_MIDI_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx);

static uint8_t  USBD_MIDI_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx);

static uint8_t  USBD_MIDI_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);

static uint8_t  *USBD_MIDI_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_MIDI_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t  USBD_MIDI_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_MIDI_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_MIDI_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_MIDI_EP0_TxReady (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_MIDI_SOF (USBD_HandleTypeDef *pdev);

static uint8_t  USBD_MIDI_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  USBD_MIDI_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

/**
  * @}
  */

/** @defgroup USBD_MIDI_Private_Variables
  * @{
  */

USBD_ClassTypeDef  USBD_MIDI =
{
  USBD_MIDI_Init,
  USBD_MIDI_DeInit,
  USBD_MIDI_Setup,
  USBD_MIDI_EP0_TxReady,
  USBD_MIDI_EP0_RxReady,
  USBD_MIDI_DataIn,
  USBD_MIDI_DataOut,
  USBD_MIDI_SOF,
  USBD_MIDI_IsoINIncomplete,
  USBD_MIDI_IsoOutIncomplete,
  USBD_MIDI_GetCfgDesc,
  USBD_MIDI_GetCfgDesc,
  USBD_MIDI_GetCfgDesc,
  USBD_MIDI_GetDeviceQualifierDesc,
};

/* USB AUDIO device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MIDI_CfgDesc[USB_MIDI_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration 1 */
  0x09,                                 /* bLength */
  USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
  LOBYTE(USB_MIDI_CONFIG_DESC_SIZ),    /* wTotalLength  101 bytes*/
  HIBYTE(USB_MIDI_CONFIG_DESC_SIZ),
  0x02,                                 /* bNumInterfaces */
  0x01,                                 /* bConfigurationValue */
  0x00,                                 /* iConfiguration */
  0xC0,                                 /* bmAttributes  BUS Powred*/
  0x32,                                 /* bMaxPower = 100 mA*/
  /* 09 byte*/

  /* USB-MIDI Synthesizer Standard AC Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  0x00,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x00,                                 /* bNumEndpoints */
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
  0x01,                                 /* baInterfaceNr */
  /* 09 byte*/

  /* USB-MIDI Synthesizer Standard MS Interface Descriptor */
  AUDIO_INTERFACE_DESC_SIZE,            /* bLength */
  USB_DESC_TYPE_INTERFACE,              /* bDescriptorType */
  0x01,                                 /* bInterfaceNumber */
  0x00,                                 /* bAlternateSetting */
  0x02,                                 /* bNumEndpoints */
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
  AUDIO_OUT_EP,                         /* bEndpointAddress 1 out endpoint*/
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
  AUDIO_IN_EP,                          /* bEndpointAddress 1 out endpoint*/
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
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MIDI_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END=
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
  * @brief  USBD_MIDI_Init
  *         Initialize the MIDI interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_MIDI_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  USBD_MIDI_HandleTypeDef   *haudio;

  /* Open EP OUT */
  USBD_LL_OpenEP(pdev, AUDIO_OUT_EP, USBD_EP_TYPE_BULK, BULK_SEND_PACKET);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 1U;

  /* Allocate Audio structure */
  pdev->pClassData = USBD_malloc(sizeof (USBD_MIDI_HandleTypeDef));

  if(pdev->pClassData == NULL)
  {
    return USBD_FAIL;
  }
  else
  {
    haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;
    haudio->alt_setting = 0U;
    haudio->offset = AUDIO_OFFSET_UNKNOWN;
    haudio->wr_ptr = 0U;
    haudio->rd_ptr = 0U;
    haudio->rd_enable = 0U;

    /* Initialize the Audio output Hardware layer */
    if (((USBD_MIDI_ItfTypeDef *)pdev->pUserData)->Init(USBD_MIDI_FREQ,
                                                         AUDIO_DEFAULT_VOLUME,
                                                         0U) != 0)
    {
      return USBD_FAIL;
    }

    /* Prepare Out endpoint to receive 1st packet */
    USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, haudio->buffer,
                           BULK_SEND_PACKET);
  }
  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_Init
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_MIDI_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx)
{

  /* Open EP OUT */
  USBD_LL_CloseEP(pdev, AUDIO_OUT_EP);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 0U;

  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
   ((USBD_MIDI_ItfTypeDef *)pdev->pUserData)->DeInit(0U);
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_MIDI_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
  USBD_MIDI_HandleTypeDef *haudio;
  uint16_t len;
  uint8_t *pbuf;
  uint16_t status_info = 0U;
  uint8_t ret = USBD_OK;

  haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    switch (req->bRequest)
    {
    case AUDIO_REQ_GET_CUR:
      AUDIO_REQ_GetCurrent(pdev, req);
      break;

    case AUDIO_REQ_SET_CUR:
      AUDIO_REQ_SetCurrent(pdev, req);
      break;

    default:
      USBD_CtlError (pdev, req);
      ret = USBD_FAIL;
      break;
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

    case USB_REQ_GET_DESCRIPTOR:
      if( (req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
      {
        pbuf = USBD_MIDI_CfgDesc + 18;
        len = MIN(USB_MIDI_DESC_SIZ , req->wLength);

        USBD_CtlSendData (pdev, pbuf, len);
      }
      break;

    case USB_REQ_GET_INTERFACE :
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
        USBD_CtlSendData (pdev, (uint8_t *)(void *)&haudio->alt_setting, 1U);
      }
      else
      {
        USBD_CtlError (pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_SET_INTERFACE :
      if (pdev->dev_state == USBD_STATE_CONFIGURED)
      {
         if ((uint8_t)(req->wValue) <= USBD_MAX_NUM_INTERFACES)
         {
           haudio->alt_setting = (uint8_t)(req->wValue);
         }
         else
         {
           /* Call the error management function (command will be nacked */
           USBD_CtlError (pdev, req);
           ret = USBD_FAIL;
         }
      }
      else
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
  * @brief  USBD_MIDI_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_MIDI_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_MIDI_CfgDesc);
  return USBD_MIDI_CfgDesc;
}

/**
  * @brief  USBD_MIDI_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MIDI_DataIn (USBD_HandleTypeDef *pdev,
                              uint8_t epnum)
{

  /* Only OUT data are processed */
  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_MIDI_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  USBD_MIDI_HandleTypeDef   *haudio;
  haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

  if (haudio->control.cmd == AUDIO_REQ_SET_CUR)
  {/* In this driver, to simplify code, only SET_CUR request is managed */

    if (haudio->control.unit == AUDIO_OUT_STREAMING_CTRL)
    {
     ((USBD_MIDI_ItfTypeDef *)pdev->pUserData)->MuteCtl(haudio->control.data[0]);
      haudio->control.cmd = 0U;
      haudio->control.len = 0U;
    }
  }

  return USBD_OK;
}
/**
  * @brief  USBD_MIDI_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_MIDI_EP0_TxReady (USBD_HandleTypeDef *pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}
/**
  * @brief  USBD_MIDI_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t  USBD_MIDI_SOF (USBD_HandleTypeDef *pdev)
{
  return USBD_OK;
}

/**
  * @brief  USBD_MIDI_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
void  USBD_MIDI_Sync (USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset)
{
  uint32_t cmd = 0U;
  USBD_MIDI_HandleTypeDef   *haudio;
  haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

  haudio->offset =  offset;

  if(haudio->rd_enable == 1U)
  {
    haudio->rd_ptr += (uint16_t)(AUDIO_TOTAL_BUF_SIZE / 2U);

    if (haudio->rd_ptr == AUDIO_TOTAL_BUF_SIZE)
    {
      /* roll back */
      haudio->rd_ptr = 0U;
    }
  }

  if(haudio->rd_ptr > haudio->wr_ptr)
  {
    if((haudio->rd_ptr - haudio->wr_ptr) < BULK_SEND_PACKET)
    {
      cmd = AUDIO_TOTAL_BUF_SIZE / 2U + 4U;
    }
    else
    {
      if((haudio->rd_ptr - haudio->wr_ptr) > (AUDIO_TOTAL_BUF_SIZE - BULK_SEND_PACKET))
      {
        cmd = AUDIO_TOTAL_BUF_SIZE / 2U - 4U;
      }
    }
  }
  else
  {
    if((haudio->wr_ptr - haudio->rd_ptr) < BULK_SEND_PACKET)
    {
      cmd = AUDIO_TOTAL_BUF_SIZE / 2U - 4U;
    }
    else
    {
      if((haudio->wr_ptr - haudio->rd_ptr) > (AUDIO_TOTAL_BUF_SIZE - BULK_SEND_PACKET))
      {
        cmd = AUDIO_TOTAL_BUF_SIZE / 2U + 4U;
      }
    }
  }

  if(haudio->offset == AUDIO_OFFSET_FULL)
  {
    ((USBD_MIDI_ItfTypeDef *)pdev->pUserData)->AudioCmd(&haudio->buffer[0],
                                                         cmd,
                                                         AUDIO_CMD_PLAY);
      haudio->offset = AUDIO_OFFSET_NONE;
  }
}

/**
  * @brief  USBD_MIDI_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MIDI_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return USBD_OK;
}
/**
  * @brief  USBD_MIDI_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MIDI_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{

  return USBD_OK;
}
/**
  * @brief  USBD_MIDI_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_MIDI_DataOut (USBD_HandleTypeDef *pdev,
                              uint8_t epnum)
{

#if 0
  USBD_MIDI_HandleTypeDef   *haudio;
  haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

  if (epnum == AUDIO_OUT_EP)
  {
    /* Increment the Buffer pointer or roll it back when all buffers are full */

    haudio->wr_ptr += BULK_SEND_PACKET;

    if (haudio->wr_ptr == AUDIO_TOTAL_BUF_SIZE)
    {
      /* All buffers are full: roll back */
      haudio->wr_ptr = 0U;

      if(haudio->offset == AUDIO_OFFSET_UNKNOWN)
      {
        ((USBD_MIDI_ItfTypeDef *)pdev->pUserData)->AudioCmd(&haudio->buffer[0],
                                                             AUDIO_TOTAL_BUF_SIZE / 2U,
                                                             AUDIO_CMD_START);
          haudio->offset = AUDIO_OFFSET_NONE;
      }
    }

    if(haudio->rd_enable == 0U)
    {
      if (haudio->wr_ptr == (AUDIO_TOTAL_BUF_SIZE / 2U))
      {
        haudio->rd_enable = 1U;
      }
    }

    /* Prepare Out endpoint to receive next audio packet */
    USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, &haudio->buffer[haudio->wr_ptr],
                           BULK_SEND_PACKET);
  }
#elif 0
  USBD_MIDI_HandleTypeDef   *haudio;
  haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

  if (epnum == AUDIO_OUT_EP)
  {
    /* Increment the Buffer pointer or roll it back when all buffers are full */

    haudio->wr_ptr += BULK_SEND_PACKET;

    if (haudio->wr_ptr == AUDIO_TOTAL_BUF_SIZE)
    {
      /* All buffers are full: roll back */
      haudio->wr_ptr = 0U;

      if(haudio->offset == AUDIO_OFFSET_UNKNOWN)
      {
        ((USBD_MIDI_ItfTypeDef *)pdev->pUserData)->AudioCmd(&haudio->buffer[0],
                                                             AUDIO_TOTAL_BUF_SIZE / 2U,
                                                             AUDIO_CMD_START);
          haudio->offset = AUDIO_OFFSET_NONE;
      }
    }

    if(haudio->rd_enable == 0U)
    {
      if (haudio->wr_ptr == (AUDIO_TOTAL_BUF_SIZE / 2U))
      {
        haudio->rd_enable = 1U;
      }
    }

    /* Prepare Out endpoint to receive next audio packet */
    USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, &haudio->buffer[haudio->wr_ptr],
                           BULK_SEND_PACKET);
#else
  uint32_t size = 0;
  USBD_MIDI_HandleTypeDef   *haudio;

  size = USBD_LL_GetRxDataSize(pdev,  epnum);
  haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;  

  if ( pdev->pClass != NULL ) {
    ((USBD_MIDI_ItfTypeDef *)pdev->pUserData)->AudioCmd(&haudio->buffer[0], size, 0);
    /* Prepare Out endpoint to receive next audio packet */
    USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, &haudio->buffer[0],
                           BULK_SEND_PACKET);
    return USBD_OK;
  }
  else {
    return USBD_FAIL;
  }


#endif
  return USBD_OK;
}

/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_MIDI_HandleTypeDef   *haudio;
  haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

  memset(haudio->control.data, 0, 64U);

  /* Send the current mute state */
  USBD_CtlSendData (pdev, haudio->control.data, req->wLength);
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_MIDI_HandleTypeDef   *haudio;
  haudio = (USBD_MIDI_HandleTypeDef*) pdev->pClassData;

  if (req->wLength)
  {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx (pdev,
                       haudio->control.data,
                       req->wLength);

    haudio->control.cmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
    haudio->control.len = (uint8_t)req->wLength; /* Set the request data length */
    haudio->control.unit = HIBYTE(req->wIndex);  /* Set the request target unit */
  }
}


/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t  *USBD_MIDI_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_MIDI_DeviceQualifierDesc);
  return USBD_MIDI_DeviceQualifierDesc;
}

/**
* @brief  USBD_MIDI_RegisterInterface
* @param  fops: Audio interface callback
* @retval status
*/
uint8_t  USBD_MIDI_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                        USBD_MIDI_ItfTypeDef *fops)
{
  if(fops != NULL)
  {
    pdev->pUserData= fops;
  }
  return USBD_OK;
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
