/**
  ******************************************************************************
  * @file    usbd_audio.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_audio.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_MIDI_CDC_H
#define __USB_MIDI_CDC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_AUDIO
  * @brief This file is the Header file for usbd_audio.c
  * @{
  */


/** @defgroup USBD_MIDI_Exported_Defines
  * @{
  */
#ifndef USBD_MIDI_FREQ
/* AUDIO Class Config */
#define USBD_MIDI_FREQ                               48000U
#endif /* USBD_MIDI_FREQ */


// bInterfaceNumber
#define AUDIO_AC_INTERFACE_NUMBER                    0x00U
#define AUDIO_MS_INTERFACE_NUMBER                    0x01U
#define CDC_CDC_INTERFACE_NUMBER                    0x02U
#define CDC_DATA_INTERFACE_NUMBER                   0x03U

// bNumInterfaces
#define AUDIO_NUM_INTERFACES                         0x02U
#define CDC_NUM_INTERFACES                          0x02U
#define TOTAL_NUM_INTERFACES                        (AUDIO_NUM_INTERFACES + CDC_NUM_INTERFACES)

// bEndpointAddress
#if 0
#define MIDI_OUT_EP                                 0x01U
#define MIDI_IN_EP                                  0x81U
#define CDC_OUT_EP                                  0x02U
#define CDC_CMD_EP                                  0x82U
#define CDC_IN_EP                                   0x83U
#else
#define CDC_OUT_EP                                  0x01U
#define CDC_CMD_EP                                  0x82U
#define CDC_IN_EP                                   0x81U
#define MIDI_OUT_EP                                 0x02U
#define MIDI_IN_EP                                  0x83U


#endif

// bNumEndpoint
#define AUDIO_AC_NUM_ENDPOINTS                       0x00U
#define AUDIO_MS_NUM_ENDPOINTS                       0x02U
#define CDC_CDC_NUM_ENDPOINTS                        0x01U
#define CDC_DATA_NUM_ENDPOINTS                       0x02U


#define CDC_DATA_FS_MAX_PACKET_SIZE                 64U  /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE                         8U  /* Control Endpoint Packet size */
#ifndef CDC_FS_BINTERVAL
  #define CDC_FS_BINTERVAL                          0x10U
#endif /* CDC_FS_BINTERVAL */

#define CDC_DATA_FS_IN_PACKET_SIZE                  CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE                 CDC_DATA_FS_MAX_PACKET_SIZE

#ifndef USBD_MAX_NUM_INTERFACES
#define USBD_MAX_NUM_INTERFACES                       1U
#endif /* USBD_MIDI_FREQ */

#define USB_MIDI_CONFIG_DESC_SIZ                     0xCBU
#define AUDIO_INTERFACE_DESC_SIZE                     0x09U
#define MS_CLASS_SPECIFIED_INTERFACE_DESC_SIZE        0x07U
#define USB_MIDI_DESC_SIZ                            0x09U
#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE             0x09U
#define AUDIO_CLASS_SPECIFIED_ENDPOINT_DESC_SIZE      0x05U
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE            0x07U
#define MIDI_IN_JACK_DESC_SIZE                        0x06U
#define MIDI_OUT_JACK_DESC_SIZE                       0x09U

#define AUDIO_DESCRIPTOR_TYPE                         0x21U
#define USB_DEVICE_CLASS_AUDIO                        0x01U
#define AUDIO_SUBCLASS_AUDIOCONTROL                   0x01U
#define AUDIO_SUBCLASS_AUDIOSTREAMING                 0x02U
#define AUDIO_SUBCLASS_MIDISTREAMING                  0x03U
#define AUDIO_PROTOCOL_UNDEFINED                      0x00U
#define AUDIO_STREAMING_GENERAL                       0x01U
#define AUDIO_STREAMING_FORMAT_TYPE                   0x02U

/* Audio Descriptor Types */
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE               0x24U
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE                0x25U

/* Audio Control Interface Descriptor Subtypes */
#define AUDIO_CONTROL_HEADER                          0x01U
#define AUDIO_CONTROL_INPUT_TERMINAL                  0x02U
#define AUDIO_CONTROL_OUTPUT_TERMINAL                 0x03U
#define AUDIO_CONTROL_FEATURE_UNIT                    0x06U

#define MIDI_IN_JACK                                  0x02U
#define MIDI_OUT_JACK                                 0x03U

#define JACKTYPE_EMBEDDED                             0x01U
#define JACKTYPE_EXTERNAL                             0x02U

#define MS_GENERAL                                    0x01U

#define AUDIO_REQ_GET_CUR                             0x81U
#define AUDIO_REQ_SET_CUR                             0x01U

#define BULK_SEND_PACKET                              64U


/*---------------------------------------------------------------------*/
/*  CDC definitions                                                    */
/*---------------------------------------------------------------------*/
#define CDC_SEND_ENCAPSULATED_COMMAND               0x00U
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01U
#define CDC_SET_COMM_FEATURE                        0x02U
#define CDC_GET_COMM_FEATURE                        0x03U
#define CDC_CLEAR_COMM_FEATURE                      0x04U
#define CDC_SET_LINE_CODING                         0x20U
#define CDC_GET_LINE_CODING                         0x21U
#define CDC_SET_CONTROL_LINE_STATE                  0x22U
#define CDC_SEND_BREAK                              0x23U


/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
 typedef struct
{
   uint8_t cmd;
   uint8_t data[USB_MAX_EP0_SIZE];
   uint8_t len;
   uint8_t unit;
}
USBD_MIDI_ControlTypeDef;



typedef struct
{
  uint8_t  *RxBuffer;
  uint32_t RxLength;
  __IO uint32_t RxState;
  USBD_MIDI_ControlTypeDef control;
}
USBD_MIDI_HandleTypeDef;


typedef struct
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t cmd, uint8_t* pbuf, uint16_t length);
  int8_t (* Receive)       (uint8_t* Buf, uint32_t *Len);
}USBD_MIDI_ItfTypeDef;

typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}USBD_CDC_LineCodingTypeDef;


typedef struct _USBD_CDC_Itf
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t cmd, uint8_t* pbuf, uint16_t length);
  int8_t (* Receive)       (uint8_t* Buf, uint32_t *Len);
}USBD_CDC_ItfTypeDef;


typedef struct
{
  USBD_MIDI_ItfTypeDef  ifmidi;
  USBD_CDC_ItfTypeDef   ifcdc;
}USBD_MIDI_CDC_ItfTypeDef;




typedef struct
{
  uint32_t data[512U / 4U];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;
  uint8_t  *RxBuffer;
  uint8_t  *TxBuffer;
  uint32_t RxLength;
  uint32_t TxLength;

  __IO uint32_t TxState;
  __IO uint32_t RxState;
}
USBD_CDC_HandleTypeDef;


typedef struct
{
  USBD_MIDI_HandleTypeDef hmidi;
  USBD_CDC_HandleTypeDef  hcdc;
}USBD_MIDI_CDC_HandleTypeDef;

/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_MIDI_CDC;
#define USBD_MIDI_CDC_CLASS    &USBD_MIDI_CDC
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
extern uint8_t  USBD_MIDI_CDC_RegisterInterface  (USBD_HandleTypeDef   *pdev, USBD_MIDI_CDC_ItfTypeDef *fops);
extern uint8_t  USBD_MIDI_SetRxBuffer  (USBD_HandleTypeDef   *pdev, uint8_t  *pbuff);
extern uint8_t  USBD_CDC_SetTxBuffer  (USBD_HandleTypeDef   *pdev, uint8_t  *pbuff, uint16_t length);
extern uint8_t  USBD_CDC_SetRxBuffer  (USBD_HandleTypeDef   *pdev, uint8_t  *pbuff);
extern uint8_t  USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev);
extern uint8_t  USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_MIDI_CDC_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
