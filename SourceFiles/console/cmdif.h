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
#ifndef __CMDIF_H__
#define __CMDIF_H__

#include <stdint.h>
#include <stddef.h>

typedef void (*CMDIF_RECEIVE_CALLBACK_t)(const uint8_t *p_recvdat, size_t recv_len);

extern void CmdIF_Init(void);
extern int32_t CmdIF_CommandProc(const uint8_t *recv_dat, size_t recv_len); 
extern void CmdIF_RegistReceiveCallbackFunc(const CMDIF_RECEIVE_CALLBACK_t p_callback); 

#endif // __CMDIF_H__
