/*
 
 This code was created by modifying ymf825board_sample1.ino.
 The author and license are as follows.

  MIT License

  Copyright (c) 2017 Yamaha Corporation

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
#include "ymf825.h"
#include "main.h"

#define OUTPUT_power 1
#define SPI_TRANSMIT_TIMEOUT (100)
#define SPI_RECEIVE_TIMEOUT (100)

#define delay HAL_Delay

static uint8_t tone_data_head[1] ={
	(0x80+16),//header
};
static uint8_t tone_data_tail[4] ={
	0x80,0x03,0x81,0x80,
};

static SPI_HandleTypeDef SpiHandle;

static void set_ss_low(void);
static void set_ss_high(void);
static void set_rst_low(void);
static void set_rst_high(void);
static void setup(void);


int32_t YMF825_Init(void) {

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;

	GPIOD->MODER |= GPIO_MODER_MODER14_0; // SS
	GPIOF->MODER |= GPIO_MODER_MODER12_0; // RST_N

	set_ss_high();
	set_rst_low();

	/*##-1- Configure the SPI peripheral #######################################*/
	/* Set the SPI parameters */
	SpiHandle.Instance				= SPIx;
	SpiHandle.Init.BaudRatePrescaler= SPI_BAUDRATEPRESCALER_16;
	SpiHandle.Init.Direction		= SPI_DIRECTION_2LINES;
	SpiHandle.Init.CLKPhase			= SPI_PHASE_1EDGE;
	SpiHandle.Init.CLKPolarity		= SPI_POLARITY_LOW;
	SpiHandle.Init.DataSize			= SPI_DATASIZE_8BIT;
	SpiHandle.Init.FirstBit			= SPI_FIRSTBIT_MSB;
	SpiHandle.Init.TIMode			= SPI_TIMODE_DISABLE;
	SpiHandle.Init.CRCCalculation	= SPI_CRCCALCULATION_DISABLE;
	SpiHandle.Init.CRCPolynomial	= 7;
	SpiHandle.Init.NSS				= SPI_NSS_SOFT;
	SpiHandle.Init.Mode				= SPI_MODE_MASTER;

	if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
	{
		/* Initialization Error */
		return (-1);
	}

	setup();

	return 0;
}

void YMF825_DeInit(void) {

	HAL_SPI_DeInit(&SpiHandle);

	RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
	RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOFEN;

}

void if_write(uint8_t addr, const uint8_t* data, uint16_t size){
	set_ss_low();
	HAL_SPI_Transmit(&SpiHandle, &addr, 1, SPI_TRANSMIT_TIMEOUT);
	HAL_SPI_Transmit(&SpiHandle, (uint8_t*)data, size, SPI_TRANSMIT_TIMEOUT);
	set_ss_high();	
}

void if_s_write(uint8_t addr,uint8_t data){
	if_write(addr,&data,1);
}

uint8_t if_s_read(uint8_t addr){

	uint8_t readaddr = 0;
	uint8_t rcv = 0;
	readaddr = 0x80|addr;

	set_ss_low();
	HAL_SPI_Transmit(&SpiHandle, &readaddr, 1, SPI_TRANSMIT_TIMEOUT);
	HAL_SPI_Receive(&SpiHandle, &rcv, 1, SPI_RECEIVE_TIMEOUT);
	set_ss_high();
	return rcv;	
}


void YMF825_SelectChannel(uint8_t ch) {

	if_s_write(0x0B, (ch&0x0F));
}

void YMF825_ChangeVoVol(uint8_t VoVol) {

	if_s_write(0x0C, ((VoVol&0x1F) << 2));
}

void YMF825_ChangeChVol(uint8_t ChVol) {

	if_s_write(0x10, ((ChVol&0x1F) << 2));
}

void YMF825_SelectNoteNumber(uint16_t fnum, uint16_t block) {

	uint8_t dat[2] = {0x00, 0x00};
	dat[0] = ((fnum & 0x380) >> 4) | (block&0x07);
	dat[1] = fnum & 0x7F;
	if_s_write(0x0D, dat[0]);
	if_s_write(0x0E, dat[1]);
}

void YMF825_KeyOn(uint8_t tone_num) {

	if_s_write(0x0F, (0x40|(tone_num&0x0F)));
}

void YMF825_KeyOff(uint8_t tone_num) {

	if_s_write(0x0F, (0x00|(tone_num&0x0F)));
}

void YMF825_ChangeMASTER_VOL(uint8_t master_vol) {

	if_s_write( 0x19, ((master_vol&0x3F) << 2) );
}

void YMF825_ChangePitch(uint16_t INT, uint16_t FRAC) {

	if_s_write(0x12, (INT<<3) | ((FRAC>>6)&0x07));
	if_s_write(0x13, (FRAC&0x3F)<<1);
}

void YMF825_SetToneParameter(uint8_t tone_matrix[16][30]) {

	uint8_t addr = 0x07;
	int32_t i = 0;
	if_s_write( 0x08, 0xF6 );
	delay(1);
	if_s_write( 0x08, 0x00 );
	set_ss_low();

	HAL_SPI_Transmit(&SpiHandle, &addr, 1, SPI_TRANSMIT_TIMEOUT);
	HAL_SPI_Transmit(&SpiHandle, &tone_data_head[0], sizeof(tone_data_head), SPI_TRANSMIT_TIMEOUT);

	for ( i = 0; i < 16; i++ ) {
		HAL_SPI_Transmit(&SpiHandle, tone_matrix[i], 30, SPI_TRANSMIT_TIMEOUT);
	}

	HAL_SPI_Transmit(&SpiHandle, &tone_data_tail[0], sizeof(tone_data_tail), SPI_TRANSMIT_TIMEOUT);

	set_ss_high();	
}

static void set_ss_low(void) {
	GPIOD->ODR &= ~GPIO_ODR_ODR_14;
}

static void set_ss_high(void) {
	GPIOD->ODR |= GPIO_ODR_ODR_14;
}

static void set_rst_low(void) {
	GPIOF->ODR &= ~GPIO_ODR_ODR_12;
}

static void set_rst_high(void) {
	GPIOF->ODR |= GPIO_ODR_ODR_12;
}
static void setup(void) {
	set_rst_low();
	delay(1);
	set_rst_high();
	if_s_write( 0x1D, OUTPUT_power );
	if_s_write( 0x02, 0x0E );
	delay(1);
	if_s_write( 0x00, 0x01 );//CLKEN
	if_s_write( 0x01, 0x00 ); //AKRST
	if_s_write( 0x1A, 0xA3 );
	delay(1);
	if_s_write( 0x1A, 0x00 );
	delay(30);
	if_s_write( 0x02, 0x04 );//AP1,AP3
	delay(1);
	if_s_write( 0x02, 0x00 );
	//add
	if_s_write( 0x19, (40 << 2) );//MASTER VOL
	if_s_write( 0x1B, 0x3F );//interpolation
	if_s_write( 0x14, 0x00 );//interpolation
	if_s_write( 0x03, 0x01 );//Analog Gain
	
	if_s_write( 0x08, 0xF6 );
	delay(21);
	if_s_write( 0x08, 0x00 );
	if_s_write( 0x09, 0xF8 );
	if_s_write( 0x0A, 0x00 );
	
	if_s_write( 0x17, 0x40 );//MS_S
	if_s_write( 0x18, 0x00 );
}