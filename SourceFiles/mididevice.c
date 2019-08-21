/*
  MIT License

  Copyright (c) 2019 nyannkov

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
#include "mididevice.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_midi_cdc_if.h"
#include "mode4_ymf825.h"

USBD_HandleTypeDef USBD_Device;
static MIDI_Handle_t *phMIDI;

static int32_t USBMIDI_ReceiveProc(uint8_t cable_number, const uint8_t *midi_msg, size_t len);

void MIDIDevice_Create(void) {

	phMIDI = MIDI_Mode4_YMF825_Init();

	Regist_USB_MIDI_CallbackFunc(USBMIDI_ReceiveProc);
	
	/* Init Device Library */
	USBD_Init(&USBD_Device, &MIDI_SYNTHESIZER_Desc, 0);

	/* Add Supported Class */
	USBD_RegisterClass(&USBD_Device, USBD_MIDI_CDC_CLASS);

	/* Add Interface callbacks for MIDI Class */
	USBD_MIDI_CDC_RegisterInterface(&USBD_Device, &USBD_MIDI_CDC_fops);

	/* Start Device Process */
	USBD_Start(&USBD_Device);
}

void MIDIDevice_Delete(void) {

	USBD_Stop(&USBD_Device);

	USBD_DeInit(&USBD_Device);

	MIDI_Mode4_YMF825_DeInit(phMIDI);
}

static int32_t USBMIDI_ReceiveProc(uint8_t cable_number, const uint8_t *midi_msg, size_t len) {

	(void)cable_number;

	return MIDI_Play(phMIDI, midi_msg, len);
}