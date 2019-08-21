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
#include "cmdif.h"
#include "ymf825.h"

#ifndef NULL
#define NULL						((void *)0)
#endif

#define MAX_STRING_LENGTH_SIZE				(1024)
#define MAX_BYTE_ARRAY_SIZE			(MAX_STRING_LENGTH_SIZE/2)

typedef struct _CharArray {
	size_t 		length;	
	uint8_t	 	string[MAX_STRING_LENGTH_SIZE];
}CharArray_t;

typedef struct _ByteArray {
	size_t 		num;
	uint8_t		bytes[MAX_BYTE_ARRAY_SIZE];
}ByteArray_t;

static CharArray_t 		_char_array;
static ByteArray_t		_byte_array;
static CMDIF_RECEIVE_CALLBACK_t _p_recv_callback = NULL;


static void _ExecCommand(const CharArray_t *p_char_array);
static int32_t _ParseByteArray(ByteArray_t *p_byte_array, const CharArray_t *p_char_array);
static int32_t _SendRecvBytes(const ByteArray_t * p_byte_array);




static inline int8_t _ParseHexNumber(uint8_t chex) {

	int8_t hex = 0;

	if ( ( '0' <= chex ) && ( chex <= '9' ) ) {
		hex = chex - (int8_t)'0';
	}
	else if ( ( 'a' <= chex ) && ( chex <= 'f' ) ) {
		hex = chex - (int8_t)'a' + 0x0A;
	}
	else if ( ( 'A' <= chex ) && ( chex <= 'F' ) ) {
		hex = chex - (int8_t)'A' + 0x0A;
	}
	else {
		return (-1);
	}

	return hex;
}


void CmdIF_Init(void) {
	int32_t i = 0;

	for ( i = 0; i < MAX_STRING_LENGTH_SIZE; i++ ) {
		((uint8_t *)&_char_array)[i] = 0;
	}	
	for ( i = 0; i < MAX_BYTE_ARRAY_SIZE; i++ ) {
		((uint8_t *)&_byte_array)[i] = 0;
	}	
	return;
}

void CmdIF_RegistReceiveCallbackFunc(const CMDIF_RECEIVE_CALLBACK_t p_callback) {
	if ( p_callback != NULL ) {
		_p_recv_callback = p_callback;
	}
}

int32_t CmdIF_CommandProc(const uint8_t *recv_dat, size_t recv_len) {
	uint32_t i = 0;
	uint8_t * p_str = NULL;
	int32_t cmd_proc_result = 0;

	if ( recv_dat == NULL ) {
		cmd_proc_result = -1;
	}
	else if ( _char_array.length + recv_len >= MAX_STRING_LENGTH_SIZE ) {
		cmd_proc_result = -2;
	}
	else {
		p_str = &_char_array.string[_char_array.length];
		for ( i = 0; i < recv_len; i++ ) {
			if ( recv_dat[i] == '\n' ) {
				continue;
			}
			else if ( recv_dat[i] == '\r' ) {
				p_str[i] = '\0'; 
				_ExecCommand(&_char_array);
				CmdIF_Init();
				p_str = &_char_array.string[_char_array.length];
			}
			else {
				p_str[i] = recv_dat[i];
				_char_array.length++;
			}
		}
	}

	if ( cmd_proc_result != 0 ) {
		CmdIF_Init();
	}

	return 0;
}

static void _ExecCommand(const CharArray_t *p_char_array) {

	int32_t parse_bin_result = 0;

	if ( p_char_array->length == 0 ) {
		return;
	}

	if ( p_char_array->string[0] == ':') {
		// TODO;
	}
	else {
		parse_bin_result = _ParseByteArray(&_byte_array, &_char_array);	
		if ( parse_bin_result == 0 ) {
			_SendRecvBytes(&_byte_array);
		}
	}
}

static int32_t _ParseByteArray(ByteArray_t *p_byte_array, const CharArray_t *p_char_array) {

	uint32_t i = 0;
	int8_t 	hex = 0;
	uint8_t *p_bytes = NULL;

	if ( (p_char_array->length % 2) != 0 ) { 
		return (-1);
	}
	if ( p_char_array->length == 0 ) {
		return (-2);
	}
	
	p_bytes = &p_byte_array->bytes[0];
	p_byte_array->num = 0;
	for ( i = 0; i < p_char_array->length; i++ ) {
		hex = _ParseHexNumber(p_char_array->string[i]);
		if ( hex < 0 ) {
			return (-3);
		}
		if ( (i % 2) == 0 ) { 
			*p_bytes  = hex << 4;
		}
		else {
			*p_bytes |= hex;
			p_byte_array->num++;
			p_bytes++;
		}
	}

	return 0;
}

static int32_t _SendRecvBytes(const ByteArray_t * p_byte_array) {

	int32_t readflg = 0;
	uint8_t recvdat = 0;
	readflg = ( p_byte_array->bytes[0] & 0x80 ) == 0x80 ? 1 : 0;

	if (readflg == 0) {
		if (p_byte_array->num > 1) {
			if_write(p_byte_array->bytes[0], &p_byte_array->bytes[1], p_byte_array->num-1);
		}
		else {
			return -1;
		}
	}
	else {
		recvdat = if_s_read(p_byte_array->bytes[0]);
		if ( _p_recv_callback != NULL ) {
			_p_recv_callback(&recvdat, 1);
		}
	}

	return 0;
}
