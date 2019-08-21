/**
  ******************************************************************************
  * @file    usbd_cdc_if_template.c
  * @author  MCD Application Team
  * @brief   Generic media access Layer.
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
  - "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
  - "stm32xxxxx_{eval}{discovery}_io.c"
  EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "usbd_midi_cdc_if.h"
#include "cmdif.h"


extern USBD_HandleTypeDef USBD_Device;

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CDC
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_MIDI_CDC_Private_TypesDefinitions
  * @{
  */
typedef struct _MIDI_EventPacket {
  uint8_t Header;
  uint8_t MIDI_0;
  uint8_t MIDI_1;
  uint8_t MIDI_2;
}MIDI_EventPacket_t;
/**
  * @}
  */


/** @defgroup USBD_MIDI_CDC_Private_Defines
  * @{
  */
#ifdef DEBUG_USB_MIDI
#define USB_MIDI_PRINTF printf
#else
#define USB_MIDI_PRINTF(...) 
#endif

#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

#define MIDI_IN_BUF_SIZE  128

/**
  * @}
  */


/** @defgroup USBD_MIDI_CDC_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_MIDI_CDC_Private_FunctionPrototypes
  * @{
  */

static int8_t MIDIClass_Init     (void);
static int8_t MIDIClass_DeInit   (void);
static int8_t MIDIClass_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t MIDIClass_Receive  (uint8_t* pbuf, uint32_t *Len);

static int8_t CDC_Init     (void);
static int8_t CDC_DeInit   (void);
static int8_t CDC_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive  (uint8_t* pbuf, uint32_t *Len);

static void TIM_Config(void);
static void SetTxData(const uint8_t *p_recvdat, size_t recv_len);

static USB_MIDI_RECEIVE_CALLBACK_t _usb_midi_receive_callback = NULL;

USBD_MIDI_CDC_ItfTypeDef USBD_MIDI_CDC_fops =
{
  // USBD_MIDI_ItfTypeDef
  {
    MIDIClass_Init,
    MIDIClass_DeInit,
    MIDIClass_Control,
    MIDIClass_Receive
  },
  // USBD_CDC_ItfTypeDef
  {
    CDC_Init,
    CDC_DeInit,
    CDC_Control,
    CDC_Receive    
  }
};


static USBD_CDC_LineCodingTypeDef linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };

/* TIM handler declaration */
TIM_HandleTypeDef  TimHandle;


// CIN vs MIDI_x Size
static const size_t _cin_midi_x_size_tbl[16] = {
  0, 0, 2, 3, 3, 1, 2, 3, 3, 3, 3, 3, 2, 2, 3, 1
};

static uint8_t MIDIINBuffer[MIDI_IN_BUF_SIZE];
static uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
static uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
static uint32_t UserTxBufPtrIn = 0;/* Increment this pointer or roll it back to start address when data are received over USART */
static uint32_t UserTxBufPtrOut = 0; /* Increment this pointer or roll it back to start address when data are sent over USB */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  MIDI_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t MIDIClass_Init(void)
{
  CmdIF_Init();
  CmdIF_RegistReceiveCallbackFunc(SetTxData);
  USBD_MIDI_SetRxBuffer(&USBD_Device, MIDIINBuffer);
  return (0);
}

/**
  * @brief  MIDI_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t MIDIClass_DeInit(void)
{
  /*
     Add your deinitialization code here
  */
  return (0);
}


/**
  * @brief  MIDI_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t MIDIClass_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  (void)cmd;
  (void)pbuf;
  (void)length;

  return (0);
}

void Regist_USB_MIDI_CallbackFunc(USB_MIDI_RECEIVE_CALLBACK_t callback) {
  if ( callback != NULL ) {
    _usb_midi_receive_callback = callback;
  }
}
static int32_t USBMIDI_Proc(const uint8_t *mid_msg,  size_t len) {

  uint32_t i = 0;
  size_t midi_x_size = 0;
  MIDI_EventPacket_t *packet = NULL;
  uint8_t cin = 0;
  uint8_t cn = 0;

  if ( len % 4 != 0 ) {
    return -1;
  }
  for ( i = 0; i < len; i += 4 ) {
    packet = (MIDI_EventPacket_t *)&mid_msg[i];
    cn  = (packet->Header >> 4) & 0x0F;
    cin = packet->Header & 0x0F;
    midi_x_size = _cin_midi_x_size_tbl[cin];
    if ( midi_x_size != 0 ) {
      if ( _usb_midi_receive_callback != NULL ) {
        _usb_midi_receive_callback(cn, &packet->MIDI_0, midi_x_size);
      }
    }
  }

  return 0;
}
/**
  * @brief  MIDI_Receive
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint untill exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t MIDIClass_Receive (uint8_t* Buf, uint32_t *Len)
{
  uint32_t i = 0;
  
  for ( i = 0; i < *Len; i++ ) {
    USB_MIDI_PRINTF("%02X", Buf[i]);
  }

  USBMIDI_Proc(Buf, *Len);

  USB_MIDI_PRINTF("\n");
  return (0);
}


/**
  * @brief  CDC_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init(void)
{  
  /* USB handler declaration */
  extern USBD_HandleTypeDef  USBD_Device;


  TIM_Config();
  
  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    while(1) {
      __asm volatile ("nop");
    }
  }

  USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer, 0);
  USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);

  return (0);
}

/**
  * @brief  CDC_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit(void)
{
  /*
     Add your deinitialization code here
  */
  return (0);
}


/**
  * @brief  CDC_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  (void)length;

  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_SET_LINE_CODING:
    linecoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                            (pbuf[2] << 16) | (pbuf[3] << 24));
    linecoding.format     = pbuf[4];
    linecoding.paritytype = pbuf[5];
    linecoding.datatype   = pbuf[6];

    /* Add your code here */
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(linecoding.bitrate);
    pbuf[1] = (uint8_t)(linecoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(linecoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(linecoding.bitrate >> 24);
    pbuf[4] = linecoding.format;
    pbuf[5] = linecoding.paritytype;
    pbuf[6] = linecoding.datatype;

    /* Add your code here */
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Add your code here */
    break;

  case CDC_SEND_BREAK:
     /* Add your code here */
    break;

  default:
    break;
  }

  return (0);
}

/**
  * @brief  CDC_Receive
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint untill exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive (uint8_t* Buf, uint32_t *Len)
{
  
  CmdIF_CommandProc(Buf, *Len);
  USBD_CDC_ReceivePacket(&USBD_Device);

  return (0);
}



/**
  * @brief  TIM_Config: Configure TIMx timer
  * @param  None.
  * @retval None
  */
static void TIM_Config(void)
{  
  /* Set TIMx instance */
  TimHandle.Instance = TIMx;
  
  /* Initialize TIM3 peripheral as follow:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period = (CDC_POLLING_INTERVAL*1000) - 1;
  TimHandle.Init.Prescaler = 84-1;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    while(1) {
      __asm volatile("nop");
    }
  }
}


/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t buffptr;
  uint32_t buffsize;

  (void)htim;
  
  if(UserTxBufPtrOut != UserTxBufPtrIn)
  {
    if(UserTxBufPtrOut > UserTxBufPtrIn) /* Roll-back */
    {
      buffsize = APP_RX_DATA_SIZE - UserTxBufPtrOut;
    }
    else 
    {
      buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
    }
    
    buffptr = UserTxBufPtrOut;
    
    USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t*)&UserTxBuffer[buffptr], buffsize);
    
    if(USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK)
    {
      UserTxBufPtrOut += buffsize;
      if (UserTxBufPtrOut == APP_RX_DATA_SIZE)
      {
        UserTxBufPtrOut = 0;
      }
    }
  }
}


static void SetTxData(const uint8_t *p_recvdat, size_t recv_len) {

  char asciihex[3] = {0, 0, 0};
  uint32_t i = 0;
  uint32_t j = 0;

  for ( i = 0; i < recv_len; i++) {
    snprintf(asciihex, sizeof(asciihex), "%02X", p_recvdat[i]);

    for ( j = 0; j < sizeof(asciihex)-1; j++ ) {
      UserTxBuffer[UserTxBufPtrIn++] = asciihex[j];
      if ( UserTxBufPtrIn >= APP_RX_DATA_SIZE)  {
        UserTxBufPtrIn = 0;
      }
    }
    UserTxBuffer[UserTxBufPtrIn++] = '\r';
    if ( UserTxBufPtrIn >= APP_RX_DATA_SIZE)  {
      UserTxBufPtrIn = 0;
    }

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

