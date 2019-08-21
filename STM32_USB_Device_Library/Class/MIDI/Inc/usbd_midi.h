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
#ifndef __USB_MIDI_H
#define __USB_MIDI_H

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

#ifndef USBD_MAX_NUM_INTERFACES
#define USBD_MAX_NUM_INTERFACES                       1U
#endif /* USBD_MIDI_FREQ */

#define AUDIO_OUT_EP                                  0x01U
#define AUDIO_IN_EP                                   0x81U
#define USB_MIDI_CONFIG_DESC_SIZ                     0x65U
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

#define AUDIO_INPUT_TERMINAL_DESC_SIZE                0x0CU
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE               0x09U
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE           0x07U

#define AUDIO_CONTROL_MUTE                            0x0001U

#define AUDIO_FORMAT_TYPE_I                           0x01U
#define AUDIO_FORMAT_TYPE_III                         0x03U

#define AUDIO_ENDPOINT_GENERAL                        0x01U

#define AUDIO_REQ_GET_CUR                             0x81U
#define AUDIO_REQ_SET_CUR                             0x01U

#define AUDIO_OUT_STREAMING_CTRL                      0x02U


//#define AUDIO_OUT_PACKET                              (uint16_t)(((USBD_MIDI_FREQ * 2U * 2U) / 1000U))
#define AUDIO_DEFAULT_VOLUME                          70U

#define BULK_SEND_PACKET                              64U


/* Number of sub-packets in the audio transfer buffer. You can modify this value but always make sure
  that it is an even number and higher than 3 */
#define AUDIO_OUT_PACKET_NUM                          80U
/* Total size of the audio transfer buffer */
//#define AUDIO_TOTAL_BUF_SIZE                          ((uint16_t)(AUDIO_OUT_PACKET * AUDIO_OUT_PACKET_NUM))

#define AUDIO_TOTAL_BUF_SIZE                          ((uint16_t)(BULK_SEND_PACKET * AUDIO_OUT_PACKET_NUM))

    /* Audio Commands enumeration */
typedef enum
{
  AUDIO_CMD_START = 1,
  AUDIO_CMD_PLAY,
  AUDIO_CMD_STOP,
}AUDIO_CMD_TypeDef;


typedef enum
{
  AUDIO_OFFSET_NONE = 0,
  AUDIO_OFFSET_HALF,
  AUDIO_OFFSET_FULL,
  AUDIO_OFFSET_UNKNOWN,
}
AUDIO_OffsetTypeDef;
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
  uint32_t                  alt_setting;
  uint8_t                   buffer[AUDIO_TOTAL_BUF_SIZE];
  AUDIO_OffsetTypeDef       offset;
  uint8_t                    rd_enable;
  uint16_t                   rd_ptr;
  uint16_t                   wr_ptr;
  USBD_MIDI_ControlTypeDef control;
}
USBD_MIDI_HandleTypeDef;


typedef struct
{
    int8_t  (*Init)         (uint32_t  AudioFreq, uint32_t Volume, uint32_t options);
    int8_t  (*DeInit)       (uint32_t options);
    int8_t  (*AudioCmd)     (uint8_t* pbuf, uint32_t size, uint8_t cmd);
    int8_t  (*VolumeCtl)    (uint8_t vol);
    int8_t  (*MuteCtl)      (uint8_t cmd);
    int8_t  (*PeriodicTC)   (uint8_t cmd);
    int8_t  (*GetState)     (void);
}USBD_MIDI_ItfTypeDef;
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

extern USBD_ClassTypeDef  USBD_MIDI;
#define USBD_MIDI_CLASS    &USBD_MIDI
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_MIDI_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                        USBD_MIDI_ItfTypeDef *fops);

void  USBD_MIDI_Sync (USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_MIDI_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
