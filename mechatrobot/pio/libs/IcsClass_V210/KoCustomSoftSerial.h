/* 
* KoCustomSoftSerial.h
*
* Created: 2017/06/17 10:33:10
* Author: 5636m_000
* 
* Arduino���C�u�����̒���SoftSerial��115200bps�Ŏg����悤�ɉ������܂����B
* It was remodeled in order to use SoftSerial in the Arduino library by 115200bps.
*
* ���L�R�����g��Arduino��SoftSetial�����p
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

#ifndef KoCustomSoftSerial_h
#define KoCustomSoftSerial_h

#include <inttypes.h>
#include <Stream.h>

/******************************************************************************
* Definitions
******************************************************************************/

#ifndef _SS_MAX_RX_BUFF
#define _SS_MAX_RX_BUFF 130 // RX buffer size
#endif

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif


#define PARITY_NONE		0x00
#define PARITY_ODD		0x03
#define PARITY_EVEN		0x02

class KoCustomSoftSerial : public Stream
{
	private:
	// per object data
				uint8_t		_receivePin;
				uint8_t		_receiveBitMask;
	volatile	uint8_t		*_receivePortRegister;
				uint8_t		_transmitBitMask;
	volatile	uint8_t		*_transmitPortRegister;
	volatile	uint8_t		*_pcint_maskreg;
				uint8_t		_pcint_maskvalue;

	// Expressed as 4-cycle delays (must never be 0!)
	uint16_t	_rx_delay_centering;
	uint16_t	_tx_delay;

	uint16_t	_buffer_overflow:1;
	uint16_t	_rx_parity_error:1;
	uint16_t	_rx_timeout_error:1;

	uint16_t	_inverse_logic:1;

	
	// static data
	static			uint8_t		_receive_buffer[_SS_MAX_RX_BUFF];
	static volatile uint8_t		_receive_buffer_tail;
	static volatile uint8_t		_receive_buffer_head;
	static KoCustomSoftSerial *active_object;
	
	uint8_t		_receive_length;
	uint8_t		_get_length;


	// private methods
	inline void recv() __attribute__((__always_inline__));
	uint8_t rx_pin_read();
	inline void setRxIntMsk(bool enable) __attribute__((__always_inline__));

	// Return num - sub, or 1 if the result would be < 1
	static uint16_t subtract_cap(uint16_t num, uint16_t sub);

	// private static method for timing
	static inline void tunedDelay(uint16_t delay);

	KoCustomSoftSerial( const KoCustomSoftSerial &c );
	KoCustomSoftSerial& operator=( const KoCustomSoftSerial &c );


	public:


	// public methods
	KoCustomSoftSerial(uint8_t receivePin, uint8_t transmitPin);
	~KoCustomSoftSerial();
	void begin(long speed);
	void begin(long speed,int timeout);

	bool listen();
	void end();
	bool isListening() { return this == active_object; }
	bool stopListening();
	bool overflow() { bool ret = _buffer_overflow; if (ret) _buffer_overflow = false; return ret; }
	int peek();

	void setTX(uint8_t transmitPin);
	void setRX(uint8_t receivePin);

	bool prityerr() { bool ret = _rx_parity_error; if (ret) _rx_parity_error = false; return ret; }

	void	set_recv_length(byte length){_receive_length = length; _receive_buffer_tail = 0; _get_length = 0;}

	virtual size_t write(uint8_t byte);
	virtual int read();
	virtual int available();
	virtual void flush();
	operator bool() { return true; }
	
	using Print::write;

	// public only for easy access by interrupt handlers
	static inline void handle_interrupt() __attribute__((__always_inline__));
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round

#endif
