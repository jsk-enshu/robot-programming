/* 
* KoCustomSoftSerial.cpp
*
* Created: 2017/06/17 10:33:09
* Author: 5636m_000
*
* Arduinoライブラリの中のSoftSerialを115200bpsで使えるように改造しました。
* It was remodeled in order to use SoftSerial in the Arduino library by 115200bps.
*
* 下記コメントはArduinoのSoftSetialを引用
* Following comment quotes SoftSetial in Arduino.
* 
* -----------------------------------------------------------
*
* SoftwareSerial.h (formerly NewSoftSerial.h) - 
* Multi-instance software serial library for Arduino/Wiring
* -- Interrupt-driven receive and other improvements by ladyada
*   (http://ladyada.net)
* -- Tuning, circular buffer, derivation from class Print/Stream,
*   multi-instance support, porting to 8MHz processors,
*   various optimizations, PROGMEM delay tables, inverse logic and 
*   direct port writing by Mikal Hart (http://www.arduiniana.org)
* -- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
* -- 20MHz processor support by Garrett Mace (http://www.macetech.com)
* -- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*
* The latest version of this library can always be found at
* http://arduiniana.org.
* ------------------------------------------------------------
*/


//#define _DEBUG 1
#define _DEBUG_PIN1 11
#define _DEBUG_PIN2 13
//
// Includes
//
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <Stream.h>
#include "KoCustomSoftSerial.h"
#include <util/delay_basic.h>

//
// Statics
//
KoCustomSoftSerial *KoCustomSoftSerial::active_object = 0;
uint8_t				KoCustomSoftSerial::_receive_buffer[_SS_MAX_RX_BUFF];
volatile uint8_t	KoCustomSoftSerial::_receive_buffer_tail = 0;
volatile uint8_t	KoCustomSoftSerial::_receive_buffer_head = 0;
//volatile uint8_t	KoCustomSoftSerial::_receive_lengt = 0;

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.

volatile uint8_t	*_debugpport;
volatile uint8_t	_debugBitMask;


//
// Private methods
//

/* static */
inline void KoCustomSoftSerial::tunedDelay(uint16_t delay)
{
	_delay_loop_2(delay);
}

bool KoCustomSoftSerial::listen()
{
	if (!_tx_delay)
	{
		return false;
	}
	if (active_object != this)
	{
		if (active_object)
			active_object->stopListening();

		_buffer_overflow = false;
		_rx_parity_error = false;
		_receive_buffer_head = _receive_buffer_tail = 0;
		active_object = this;

		setRxIntMsk(true);
		return true;
	}

	return false;
}

bool KoCustomSoftSerial::stopListening()
{
	if (active_object == this)
	{
		setRxIntMsk(false);
		active_object = NULL;
		return true;
	}
	return false;
}

const	uint8_t		fakebuf;
//
// The receive routine called by the interrupt handler
//
void KoCustomSoftSerial::recv()
{

	#if GCC_VERSION < 40302
	asm volatile(
		"push r18 \n\t"
		"push r19 \n\t"
		"push r20 \n\t"
		"push r21 \n\t"
		"push r22 \n\t"
		"push r23 \n\t"
		"push r26 \n\t"
		"push r27 \n\t"
	::);
	#endif

	uint8_t		d = 0;
	uint8_t		getptr = _receive_length * 6;

//	uint8_t		fakebuf;

#if _DEBUG
//	volatile uint8_t	*_debugpport_1;
//	volatile uint8_t	_debugBitMask_1;
//	_debugpport_1  = portOutputRegister(digitalPinToPort(_DEBUG_PIN1));
//	_debugBitMask_1 = digitalPinToBitMask(_DEBUG_PIN1);
//	uint16_t	val_1 = *_debugpport;
//	_debugpport  = portOutputRegister(digitalPinToPort(_DEBUG_PIN2));
#endif



//	_debugpport = &fakebuf;
	uint16_t	val = *_debugpport; 
	
	setRxIntMsk(false);

	if(_get_length == 0)
	{
		_get_length = 0;

		do{
			d = 0;
			for (uint8_t i=8; i > 0; --i)
			{
				tunedDelay(_rx_delay_centering);		//
//		*_debugpport_1 |= _debugBitMask_1;
//		*_debugpport_1 = val_1;
		*_debugpport |= _debugBitMask;
		*_debugpport = val;
//	asm volatile("nop \n\t""nop \n\t""nop \n\t""nop \n\t""nop \n\t"	::);
		
				d >>= 1;
				if (rx_pin_read())
				{
					d |= 0x80;
				}
			}
			_receive_buffer[_get_length++] = d;


		}while(--getptr);
	}
	setRxIntMsk(true);

	#if GCC_VERSION < 40302
	asm volatile(
		"pop r27 \n\t"
		"pop r26 \n\t"
		"pop r23 \n\t"
		"pop r22 \n\t"
		"pop r21 \n\t"
		"pop r20 \n\t"
		"pop r19 \n\t"
		"pop r18 \n\t"
	::);
	#endif
}

uint8_t KoCustomSoftSerial::rx_pin_read()
{
	return *_receivePortRegister & _receiveBitMask;
}

//
// Interrupt handling
//

/* static */
inline void KoCustomSoftSerial::handle_interrupt()
{
	if (active_object)
	{
		active_object->recv();
	}
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
	KoCustomSoftSerial::handle_interrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect, ISR_ALIASOF(PCINT0_vect));
#endif

//
// Constructor
//
KoCustomSoftSerial::KoCustomSoftSerial(uint8_t receivePin, uint8_t transmitPin) :
	_rx_delay_centering(0),
	_tx_delay(0),
	_buffer_overflow(false),
	_rx_parity_error(false),
	_rx_timeout_error(false)
{
	setTX(transmitPin);
	setRX(receivePin);
}

//
// Destructor
//
KoCustomSoftSerial::~KoCustomSoftSerial()
{
	end();
}

void KoCustomSoftSerial::setTX(uint8_t tx)
{
	digitalWrite(tx, HIGH);
	pinMode(tx, OUTPUT);
	_transmitBitMask = digitalPinToBitMask(tx);
	uint8_t port = digitalPinToPort(tx);
	_transmitPortRegister = portOutputRegister(port);

#if _DEBUG
	_debugpport  = portOutputRegister(digitalPinToPort(_DEBUG_PIN2));
	_debugBitMask = digitalPinToBitMask(_DEBUG_PIN2);


#endif
	_debugpport = &fakebuf;

}

void KoCustomSoftSerial::setRX(uint8_t rx)
{
	pinMode(rx, INPUT);
	digitalWrite(rx, HIGH);  // pullup for normal logic!
	_receivePin = rx;
	_receiveBitMask = digitalPinToBitMask(rx);
	uint8_t port = digitalPinToPort(rx);
	_receivePortRegister = portInputRegister(port);
}

uint16_t KoCustomSoftSerial::subtract_cap(uint16_t num, uint16_t sub) {
	if (num > sub)
		return num - sub;
	else
		return 1;
}


//
// Public methods
//

void KoCustomSoftSerial::begin(long speed)
{
	_rx_delay_centering =0;
	_tx_delay = 0;

	uint16_t bit_delay = (F_CPU / speed) / 4;
	_tx_delay = subtract_cap(bit_delay, 15 / 4);

	// Only setup rx when we have a valid PCINT for this pin
	if (digitalPinToPCICR(_receivePin)) {
		#if GCC_VERSION > 40800
			_rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);
		#else // Timings counted from gcc 4.3.2 output
			_rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4);
		#endif

		*digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
		_pcint_maskreg = digitalPinToPCMSK(_receivePin);
		_pcint_maskvalue = _BV(digitalPinToPCMSKbit(_receivePin));

		tunedDelay(_tx_delay); // if we were low this establishes the end
	}

	#if _DEBUG
	pinMode(_DEBUG_PIN1, OUTPUT);
	pinMode(_DEBUG_PIN2, OUTPUT);
	#endif

	listen();
}

void KoCustomSoftSerial::begin(long speed,int timeout)
{
	_rx_delay_centering =0;
	_tx_delay = 0;

	uint16_t bit_delay = (F_CPU / speed) / 4;
	_tx_delay = subtract_cap(bit_delay, 15 / 4);

	// Only setup rx when we have a valid PCINT for this pin
	if (digitalPinToPCICR(_receivePin)) {
		#if GCC_VERSION > 40800
		_rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);
		#else // Timings counted from gcc 4.3.2 output
		_rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4);
		#endif

		*digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
		_pcint_maskreg = digitalPinToPCMSK(_receivePin);
		_pcint_maskvalue = _BV(digitalPinToPCMSKbit(_receivePin));

		tunedDelay(_tx_delay); // if we were low this establishes the end
	}

	#if _DEBUG
	pinMode(_DEBUG_PIN1, OUTPUT);
	pinMode(_DEBUG_PIN2, OUTPUT);
	#endif

	listen();
}

void KoCustomSoftSerial::setRxIntMsk(bool enable)
{
	if (enable)
		*_pcint_maskreg |= _pcint_maskvalue;
	else
		*_pcint_maskreg &= ~_pcint_maskvalue;
}

void KoCustomSoftSerial::end()
{
	stopListening();
}


// Read data from buffer
int KoCustomSoftSerial::read()
{
	if (!isListening())
	return -2;

	// Empty buffer?
	if (_receive_buffer_head == _receive_buffer_tail)
	return -3;

	// Read from "head"
	uint8_t d = _receive_buffer[_receive_buffer_head++]; // grab next byte
	if (_receive_buffer_head == _receive_buffer_tail)
	{
		_receive_buffer_head = 0;
		_receive_buffer_tail = 0;
 	}
	return d;
}


                                   //0 1  2   3  4  5  6  7  8  9 10 11 
const	uint8_t		SamplingDef[] = {0,5, 9 ,13,17,20,24,28,32,34,36,43,255};

//const	uint8_t		SamplingDef[] = {0,5, 9 ,13,17,20,26,31,36,41,46,51,255};

int KoCustomSoftSerial::available()
{
	if (!isListening())
		return 0;

	if(_receive_length && _get_length)
	{
		uint8_t		d = 0;
		uint8_t		bit_cnt = 8 ;
		uint8_t		byte_cnt= 0;
		uint8_t		cnt = 1;
		uint8_t		shiftcnt  = 0;
		uint8_t		dat = 0;
		uint8_t		getcnt = 0;
		bool		old_bit = 0;
		bool		new_bit = 0;


#if _DEBUG
	volatile uint8_t	*_debugpport_1;
	volatile uint8_t	_debugBitMask_1;
	_debugpport_1  = portOutputRegister(digitalPinToPort(_DEBUG_PIN1));
	_debugBitMask_1 = digitalPinToBitMask(_DEBUG_PIN1);
	uint16_t	val_1 = *_debugpport;

	_debugpport  = portOutputRegister(digitalPinToPort(_DEBUG_PIN2));
	uint16_t	val = *_debugpport;
	*_debugpport |= _debugBitMask;
	*_debugpport = val;
#endif

		for(uint16_t	 i = 0; i < (uint16_t)_receive_length * 6 * 9 + 100; i++)
		{
			if(bit_cnt == 8)
			{
				d = _receive_buffer[byte_cnt++];
				bit_cnt = 0;
			}
			new_bit = (bool)(d & 0x01); 
			d >>= 1;
			if(old_bit ^ new_bit)
			{
				uint8_t		cc;
				for (cc = 0; cc < 13; cc++)
				{
#if _DEBUG
	*_debugpport_1 |= _debugBitMask_1;
	*_debugpport_1 = val_1;
#endif

					if(cnt < SamplingDef[cc])
					{
						for (uint8_t  c = 0; c < cc;	c++)
						{
							dat >>= 1;
							if(old_bit)	dat |= 0x80;
							shiftcnt++;
							if(shiftcnt ==8)
							{
								_receive_buffer[getcnt++] = dat;
								getcnt &= 0x7f;
								dat = 0;
								shiftcnt = 0;
							}
						}
						
						break;
					}
				}
#if _DEBUG
	*_debugpport |= _debugBitMask;
	*_debugpport = val;
#endif
				cnt = 1;
			}
			else
			{
				cnt++;
				if(cnt > 55)
				{
					for (uint8_t  c = 0; c < 13;	c++)
					{
						dat >>= 1;
						dat |= 0x80;
						shiftcnt++;
						if(shiftcnt ==8)
						{
							_receive_buffer[getcnt++] = dat;
							getcnt &= 0x7f;
							dat = 0;
							shiftcnt = 0;
#if _DEBUG
	*_debugpport |= _debugBitMask;
	*_debugpport = val;
#endif

						}
					}
					cnt = 0;
				}

			}
			old_bit = new_bit;
			bit_cnt++;
		}

		uint8_t		dcnt = 0;
		bool		parity = 0;
		uint8_t		getdat = 0;
		uint8_t		phase = 0;

		getcnt = 0;
		byte_cnt = 0;
		bit_cnt = 8;


		for(uint16_t	 i = 0; i < (uint16_t)_receive_length * 2 * 8; i++)
		{
			if(bit_cnt == 8)
			{
				d = _receive_buffer[byte_cnt++];
				bit_cnt = 0;
			}
			new_bit = (bool)(d & 0x01);

			switch(phase)
			{
				case	0://
					if(!new_bit)
					{//start bit
						phase++;
						dcnt = 0;
						parity = 0;
						getdat = 0;
					}
				break;
				case	1://
					getdat >>=1;
					parity ^= new_bit;
					if(new_bit)	getdat |= 0x80;
					dcnt++;
					if(dcnt == 8)
						phase++;

				break;
				case	2://
					parity ^= new_bit;
					if(!parity)
					{
						_receive_buffer[getcnt++] = getdat;
#if _DEBUG
	*_debugpport |= _debugBitMask;
	*_debugpport = val;
#endif
					}
					else
					{
						_receive_buffer[getcnt++] = 0x55;
						return -1;
					}				
		
					phase++;

				break;
				case	3://
					if(new_bit)
					{//stop bit
						phase = 0;
						dcnt = 0;
					}
				break;
			}
			d >>= 1;
			bit_cnt++;

		}
		_receive_buffer_tail = getcnt;	//_receive_buffer[0];
		_receive_buffer_head = 0;
		_receive_length = 0;
		_get_length = 0;

#if _DEBUG
/*
	*_debugpport |= _debugBitMask;
	*_debugpport = val;
	Serial.write(_receive_buffer[0]);
	Serial.write(_receive_buffer[1]);
	Serial.write(_receive_buffer[2]);
	Serial.write(_receive_buffer[3]);
*/
#endif

				
		return getcnt;		//_receive_buffer[0];
	}
	else if(_receive_buffer_tail > _receive_buffer_head)
	{
		return _receive_buffer_tail - _receive_buffer_head;
	}
	else
	{
		return 0;
	}
}



size_t KoCustomSoftSerial::write(uint8_t b)
{
	if (_tx_delay == 0) {
		setWriteError();
		return 0;
	}

	volatile uint8_t *reg = _transmitPortRegister;
	uint8_t		reg_mask =	 _transmitBitMask;
	uint8_t		inv_mask =	~_transmitBitMask;
	uint8_t		oldSREG = SREG;
	uint16_t	delay = _tx_delay;

	uint8_t parity = 0;

	cli();  // turn off interrupts for a clean txmit

	// Write the start bit
	*reg &= inv_mask;

	tunedDelay(delay);

	// Write each of the 8 bits
	for (uint8_t i = 8; i > 0; --i)
	{
		if (b & 1) // choose bit
			*reg |= reg_mask; // send 1
		else
			*reg &= inv_mask; // send 0
		parity ^= b;
		tunedDelay(delay);
		b >>= 1;
	}
	//
	if (parity & 1) // choose bit
		*reg |= reg_mask; // send 1
	else
		*reg &= inv_mask; // send 0
	tunedDelay(delay);
	

	// restore pin to natural state
	*reg |= reg_mask;

	SREG = oldSREG; // turn interrupts back on
	tunedDelay(_tx_delay);
	
	return 1;
}

void KoCustomSoftSerial::flush()
{
	// There is no tx buffering, simply return
}

int KoCustomSoftSerial::peek()
{
	if (!isListening())
		return -1;

	// Empty buffer?
	if (_receive_buffer_head == _receive_buffer_tail)
		return -1;

	// Read from "head"
	return _receive_buffer[_receive_buffer_head];
}
