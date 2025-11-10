/*
NOTE: Modified version of SoftwareSerial.cpp (formerly NewSoftSerial.cpp)
NOTE: Modifications marked with NON_STANDARD (if you want to port them to a newer version of SoftwareSerial)
 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://arduiniana.org.
*/

// When set, _DEBUG co-opts pins 14 and 12 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 1
#define _DEBUG_PIN1 14  // D5
#define _DEBUG_PIN2 12  // D6
// 
// Includes
// 
#include <Arduino.h>
#include "ESPSoftwareSerial.h"       // our custom version of standard SoftwareSerial

// ESPHOME/ESP8266 MODIFICATION: ESP8266 direct GPIO register access definitions
// These registers control the output value (OUT), set output high (OUT_W1TS), 
// set output low (OUT_W1TC), and read the input value (IN).
#define GPIO_OUT_REG_ADDRESS 0x6000030C // GPIO_OUT
#define GPIO_OUT_W1TS_REG_ADDRESS 0x60000308 // GPIO_OUT_W1TS (Set High)
#define GPIO_OUT_W1TC_REG_ADDRESS 0x6000030C // GPIO_OUT_W1TC (Set Low)
#define GPIO_IN_REG_ADDRESS 0x60000318 // GPIO_IN

//
// Statics
//
SoftwareSerial *SoftwareSerial::active_object = 0;
uint8_t SoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF]; 
volatile uint8_t SoftwareSerial::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSerial::_receive_buffer_head = 0;

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
#if _DEBUG
inline void IRAM_ATTR DebugPulse(uint8_t pin, uint8_t count)
{
  // uint32_t pinMask = (1 << pin);
  // ESPHOME/ESP8266 MODIFICATION: Use OUT_W1TC to set low
  // WRITE_PERI_REG(GPIO_OUT_W1TC_REG_ADDRESS, pinMask);
  digitalWrite(pin, LOW);
  delayMicroseconds(count);
  // ESPHOME/ESP8266 MODIFICATION: Use OUT_W1TS to set high
  // WRITE_PERI_REG(GPIO_OUT_W1TS_REG_ADDRESS, pinMask);
  digitalWrite(pin, HIGH);
}
#else
inline void IRAM_ATTR DebugPulse(uint8_t, uint8_t) {}
#endif

//
// Private methods
//

/* static */ 
inline void SoftwareSerial::tunedDelay(uint16_t delay_us) { 
  if(delay_us > 0)  {
    delayMicroseconds(delay_us);
  }
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another 
bool SoftwareSerial::listen()
{
  // if (!_rx_delay_stopbit)
  if (_rx_delay_stopbit_us == 0)
    return false;

  if (active_object != this)
  {
    if (active_object)
      active_object->stopListening();

    _buffer_overflow = false;
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;

    // attachInterrupt(digitalPinToInterrupt(_receivePin), sws_static_recv, FALLING);
    return true;
  }

  return false;
}

// Stop listening. Returns true if we were actually listening.
bool SoftwareSerial::stopListening()
{
  if (active_object == this)
  {
    // setRxIntMsk(false);
    detachInterrupt(digitalPinToInterrupt(_receivePin));
    active_object = NULL;
    return true;
  }
  return false;
}

//
// The receive routine called by the interrupt handler
//
void IRAM_ATTR SoftwareSerial::recv()
{
  uint8_t d = 0;
  // noInterrupts();
  // tunedDelay(_rx_delay_centering_us);
  DebugPulse(_DEBUG_PIN1, 1);

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (_inverse_logic ? rx_pin_read() : !rx_pin_read())
  {
    // Disable further interrupts during reception, this prevents
    // triggering another interrupt directly after we return, which can
    // cause problems at higher baudrates.
    // setRxIntMsk(false);
    noInterrupts();  // set earlier

    // Wait approximately 1/2 of a bit width to "center" the sample
    tunedDelay(_rx_delay_centering_us); // Before debounce
    DebugPulse(_DEBUG_PIN2, 1);  // Pulsed before debounce

    // Read each of the 8 bits
    for (uint8_t i=8; i > 0; --i)
    {
      tunedDelay(_rx_delay_intrabit_us);
      d >>= 1;
      DebugPulse(_DEBUG_PIN2, 1);
      if (rx_pin_read()){
        d |= 0x80;
        // Serial.println("Someone wants to talk");
      }
    }

    if (_inverse_logic)
      d = ~d;

    // if buffer full, set the overflow flag and return
    uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    if (next != _receive_buffer_head)
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = next;
    } 
    else 
    {
      // DebugPulse(_DEBUG_PIN1, 1);
      _buffer_overflow = true;
      // Serial.println("Buffer Overflow");
    }

    // NON_STANDARD - support parity bit
    if (_parity) 
    {
      tunedDelay(_rx_delay_stopbit_us);
      DebugPulse(_DEBUG_PIN2, 1);
    }

    // skip the stop bit
    tunedDelay(_rx_delay_stopbit_us);
    DebugPulse(_DEBUG_PIN2, 1);

    // Re-enable interrupts when we're sure to be inside the stop bit
    // interrupts();

    // Serial.print(d, HEX);
  }
}

uint8_t IRAM_ATTR SoftwareSerial::rx_pin_read()
{
  // uint32_t pinMask = (1 << _receivePin);
  // return (READ_PERI_REG(GPIO_IN_REG_ADDRESS) & pinMask) ? 1 : 0;
  return digitalRead(_receivePin);
}

//
// Static interrupt handler needed by attachInterrupt
//
// static void IRAM_ATTR sws_static_recv()
// {
//   if (SoftwareSerial::active_object)
//   {
//     // ESPHOME/ESP8266 MODIFICATION: Detach interrupt immediately to disable 
//     // further interrupts during the bit-banging timing section.
//     detachInterrupt(digitalPinToInterrupt(SoftwareSerial::active_object->_receivePin));
//     SoftwareSerial::active_object->recv();
//   }
// }

//
// Constructor
//
SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /* = false */) : 
  _rx_delay_centering_us(0),
  _rx_delay_intrabit_us(0),
  _rx_delay_stopbit_us(0),
  _tx_delay_us(0),
  _buffer_overflow(false),
  _inverse_logic(inverse_logic),
  _parity(false)                   // NON_STANDARD - init parity mode to false
{
  setTX(transmitPin);
  setRX(receivePin);
}

//
// Destructor
//
SoftwareSerial::~SoftwareSerial()
{
  end();
}

void SoftwareSerial::setTX(uint8_t tx)
{
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output high. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.
  digitalWrite(tx, _inverse_logic ? LOW : HIGH);
  pinMode(tx, OUTPUT);
  tx_pin_write(_inverse_logic ? LOW : HIGH);

  _transmitPin = tx;  // added by Gemini ???
}

void SoftwareSerial::setRX(uint8_t rx)
{
  pinMode(rx, INPUT);
  // if (!_inverse_logic)
  //   digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin = rx;  
}

uint16_t SoftwareSerial::subtract_cap(uint16_t num, uint16_t sub) {
  if (num > sub)
    return num - sub;
  else
    return 1;
}

//
// Public methods
//

// NON_STANDARD (allow enabling/disabling even parity)
void SoftwareSerial::setParity(bool parity)
{
    _parity = parity;
}

void SoftwareSerial::begin(long speed)
{
  _rx_delay_centering_us = _rx_delay_intrabit_us = _rx_delay_stopbit_us = _tx_delay_us = 0;

  uint16_t bit_delay_us = 1000000L / speed; // How many uS are in one bit

  _tx_delay_us = subtract_cap(bit_delay_us, 3);  // How long to wait before commmanding to adjust line

  _rx_delay_centering_us = subtract_cap(bit_delay_us / 2, 8);  // Time from beginning of bit to middle of bit

  _rx_delay_intrabit_us = subtract_cap(bit_delay_us, 5);  // Time to wait between reading at middle of bits

  _rx_delay_stopbit_us = subtract_cap(bit_delay_us, 8);  // Time to catch stop bit

  //tunedDelay(_tx_delay_uS); // if we were low this establishes the end

#if _DEBUG
  pinMode(_DEBUG_PIN1, OUTPUT);
  pinMode(_DEBUG_PIN2, OUTPUT);
#endif

  listen();
}

void SoftwareSerial::end()
{
  stopListening();
}


// Read data from buffer
int SoftwareSerial::read()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int SoftwareSerial::available()
{
  if (!isListening())
    return 0;

  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(uint8_t b)
{
  if (_tx_delay_us == 0) {
    setWriteError();
    return 0;
  }

  //uint32_t reg_mask = (1 << _transmitPin); // Pin mask (e.g., 0x00000004 for GPIO2)

  bool inv = _inverse_logic;
  //uint16_t delay_us = _tx_delay_us;

  uint8_t parity = 0;

  if (inv)
    b = ~b;

  noInterrupts();  // turn off interrupts for a clean txmit

  // Write the start bit
  tx_pin_write(inv ? HIGH : LOW);

  tunedDelay(_tx_delay_us);

  // Serial.print(b, HEX);  // debug

  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i)
  {
    if (b & 1) // choose bit
    {
      //*reg |= reg_mask; // send 1
      tx_pin_write(HIGH);
      parity ^= 0x01;        // NON_STANDARD
    } 
    else
    {
      //*reg &= inv_mask; // send 0
      tx_pin_write(LOW);
      parity ^= 0x00;        // NON_STANDARD
    }
    tunedDelay(_tx_delay_us);
    b >>= 1;
  }

  // Write even parity stop bit
  if (_parity) 
  {
    if (parity == 0)                   // even number of 1s
      tx_pin_write(inv ? HIGH : LOW);  // send 0
    else                               // odd number of 1s
      tx_pin_write(inv ? LOW : HIGH);  // send 1 
    tunedDelay(_tx_delay_us);
  }

  // restore pin to natural state
  tx_pin_write(inv ? LOW : HIGH);

  interrupts();
  tunedDelay(_tx_delay_us);
  
  return 1;
}

void SoftwareSerial::flush()
{
  // There is no tx buffering, simply return
}

int SoftwareSerial::peek()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}

// NON_STANDARD
void IRAM_ATTR SoftwareSerial::tx_pin_write(uint8_t pin_state)
{
  //uint32_t pinMask = (1 << _transmitPin);
  if (pin_state == LOW)
    // ESPHOME/ESP8266 MODIFICATION: Use OUT_W1TC to set low
    //WRITE_PERI_REG(GPIO_OUT_W1TC_REG_ADDRESS, pinMask);
    digitalWrite(_transmitPin, LOW);
  else
    // ESPHOME/ESP8266 MODIFICATION: Use OUT_W1TS to set high
    //WRITE_PERI_REG(GPIO_OUT_W1TS_REG_ADDRESS, pinMask);
    digitalWrite(_transmitPin, HIGH);
}

