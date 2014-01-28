/*************************************************************************
Title:    XIO I2C Driver
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     xio-driver.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2014 Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

/* 0x00-0x04 - input registers */
/* 0x08-0x0C - output registers */
/* 0x18-0x1C - direction registers - 0 is output, 1 is input */
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

#include "avr-i2c-master.h"
#include "xio-driver.h"

#define PORT_(port) PORT ## port
#define DDR_(port)  DDR  ## port
#define PIN_(port)  PIN  ## port

#define PORT(port) PORT_(port)
#define DDR(port)  DDR_(port)
#define PIN(port)  PIN_(port)

void xioDirectionSet(XIOControl* xio)
{
	uint8_t i2cBuf[8], i;
	i2cBuf[0] = xio->address;
	i2cBuf[1] = 0x80 | 0x18;  // 0x80 is auto-increment
	for(i=0; i<5; i++)
		i2cBuf[2+i] = xio->direction[i];
	i2c_transmit(i2cBuf, 7, 1);
	while(i2c_busy());
}

// xioPinDirections is an array of 5 bytes corresponding to IO0_0 (byte 0, bit 0) through IO4_7 (byte 4, bit 7)
// Set xioPinDirections bit to 0 for XIO pin as an output
// Set xioPinDirections bit to 1 for XIO pin as an input

void xioInitialize(XIOControl* xio, uint8_t xioAddress, const uint8_t* xioPinDirections)
{
	memset(xio, 0, sizeof(XIOControl));
		
	xio->address = xioAddress;
	xio->status |= XIO_I2C_ERROR;

	memcpy(xio->direction, xioPinDirections, 5);

	PORT(I2C_RESET_PORT) &= ~_BV(I2C_RESET);
	PORT(I2C_OUT_EN_PORT) |= _BV(I2C_OUT_EN);

	DDR(I2C_RESET_PORT) |= _BV(I2C_RESET);
	DDR(I2C_OUT_EN_PORT) |= _BV(I2C_OUT_EN);
	_delay_us(1);

	PORT(I2C_OUT_EN_PORT) &= ~_BV(I2C_OUT_EN);
	PORT(I2C_RESET_PORT) |= _BV(I2C_RESET);
	_delay_us(1);

	xioDirectionSet(xio);
	
	if (i2c_transaction_successful())
	{
		xio->status &= ~(XIO_I2C_ERROR);
		xio->status |= XIO_INITIALIZED;
	}
}

void xioOutputWrite(XIOControl* xio)
{
	uint8_t i2cBuf[8];
	uint8_t i;
	uint8_t baseOutputRegister = 0x08;

	while (0xFF == xio->direction[baseOutputRegister - 0x08] && baseOutputRegister < (0x08 + 5))
		baseOutputRegister++;

	// If we're at 5, there are no bits set as output
	if ((0x08 + 5) == baseOutputRegister)
		return;

	// Reinforce direction
	xioDirectionSet(xio);

	while(i2c_busy());

	if (!i2c_transaction_successful())
		xio->status |= XIO_I2C_ERROR;

	i2cBuf[0] = xio->address;
	i2cBuf[1] = 0x80 | baseOutputRegister;  // 0x80 is auto-increment, 0x08 is the base of the output registers
	for(i=baseOutputRegister - 0x08; i<5; i++)
	{
		i2cBuf[2+i] = xio->io[i] & ~xio->direction[i];
	}

	i2c_transmit(i2cBuf, 2+((0x08+5) - baseOutputRegister), 1);
}

void xioSetIO(XIOControl* xio, uint8_t ioNum, uint8_t state)
{
	if (ioNum >= 40)
		return;

	// Don't set inputs
	if (xio->direction[ioNum/8] & (1<<(ioNum%8)))
		return;
	
	if (state)
		xio->io[ioNum/8] |= 1<<(ioNum%8);
	else
		xio->io[ioNum/8] &= ~(1<<(ioNum%8));
}

uint8_t xioGetIO(XIOControl* xio, uint8_t ioNum)
{
	if (ioNum >= 40)
		return(0);
	
	return ((xio->io[ioNum/8] & (1<<(ioNum % 8)))?1:0);
}

void xioInputRead(XIOControl *xio)
{
	uint8_t i2cBuf[8];
	uint8_t successful = 0;
	uint8_t baseInputRegister = 0;

	while (0 == xio->direction[baseInputRegister] && baseInputRegister < 5)
		baseInputRegister++;

	// If we're at 5, there are no bits set as input
	if (5 == baseInputRegister)
		return;

	while(i2c_busy());

	if (!i2c_transaction_successful())
		xio->status |= XIO_I2C_ERROR;

	i2cBuf[0] = xio->address;
	i2cBuf[1] = 0x80 | baseInputRegister;  // 0x80 is auto-increment, 0x00 is base of the input registers
	i2c_transmit(i2cBuf, 2, 0);
	i2cBuf[0] = xio->address | 0x01;
	i2c_transmit(i2cBuf, (5 - baseInputRegister) + 1, 1);
	while(i2c_busy());
	
	successful = i2c_receive(i2cBuf, (5 - baseInputRegister) + 1);

	if (!successful)
		// In the event of a read hose-out, don't put crap in the input buffer
		xio->status |= XIO_I2C_ERROR;
	else
	{
		uint8_t i;
		for(i=baseInputRegister; i<5; i++)
		{
			// Clear all things marked as inputs, leave outputs alone
			xio->io[i] &= ~xio->direction[i];
			// Only set anything that's high and marked as an input
			xio->io[i] |= (xio->direction[i] | i2cBuf[1+i]);
		}
	}
}



