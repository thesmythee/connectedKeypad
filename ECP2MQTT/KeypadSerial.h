/*
ESPHOME/ESP8266 MODIFICATION: Rewritten from original KeypadSerial.h.
Key changes:
1. Removed all AVR-specific hardware defines (PCINT vectors, etc.).
2. Adjusted for ESP8266 compatibility by ensuring necessary Arduino/ESP headers are included.
3. The `delay_us` function signature is kept, but its implementation in the CPP file 
   will be rewritten to use the ESP-compatible `delayMicroseconds()`.

Original code is preserved at the bottom in a comment block.
*/

#pragma once

#include <Arduino.h>
#include "ESPSoftwareSerial.h"

// ESPHOME/ESP8266 MODIFICATION: Removed AVR-specific Pin definitions for better portability.
// The pins will be defined in the ESPHome component configuration. 
// Define default pin numbers if necessary for compilation, but they should be configured externally.
#define RX_PIN 4  // GPIO Pin (GPIO4) to keypad Green
#define TX_PIN 5  // GPIO Pin (GPIO5) to keypad Yellow

// responses from requestData func
#define NO_MESG    (0)
#define KEYS_MESG  (1)

#define KP_SERIAL_BAUD        (4800)    // baud rate for keypad communication
#define KP_SERIAL_MAX_KEYPADS    (8)    // max number of keypads in alarm circuit
#define KP_SERIAL_READ_BUF_SIZE (64)    // size of read buffer

// polling states during keypad polling
enum {
    NOT_POLLING  = 0,
    POLL_STATE_1 = 1,  // send first  0x00
    POLL_STATE_2 = 2,  // send second 0x00
    POLL_STATE_3 = 3,  // send third  0x00
    POLL_STATE_4 = 4,  // send fourth 0x00
    POLL_STATE_5 = 5   // read bitmask from keypad
};

class KeypadSerial
{
public:
    KeypadSerial(void);              // Class constructor.  Returns: none

    void    init(void);              // init the class
    bool    poll(void);
    void    write(const uint8_t * msg, const uint8_t size);
    bool    read(uint8_t * c, uint32_t timeout);
    void    getMsg(char * buf, uint8_t bufLen);
    uint8_t requestData(uint8_t kp);

    // return keypad address for keypad kp
    uint8_t getAddr(uint8_t kp)         { return kp < numKeypads ? keypadAddr[kp] : 0; }

    // return the number of keypads that responded to the poll request
    uint8_t getNumKeypads(void)         { return numKeypads; }

    // return the number of keys returned by keypad
    uint8_t getKeyCount(void)           { return recvMsgLen > 3 ? recvMsgLen - 3 : 0; }

    // return pointer to array of keys returned by keypad
    uint8_t * getKeys(void)             { return &readBuf[2]; }  // keys start at byte 2

    // return length of data received from keypad
    uint8_t getRecvMsgLen(void)         { return recvMsgLen; }

    // return pointer to array of data read from keypad
    uint8_t * getRecvMsg(void)          { return readBuf; }

    // ESPHOME/ESP8266 MODIFICATION: Changed from inline ISR to a regular function 
    // that is called by the ESP interrupt handler, and MUST run in IRAM.
    static void IRAM_ATTR pinChangeIsr(void);
    static KeypadSerial * pKeypadSerial;

    // ESPHOME/ESP8266 MODIFICATION: Exposed softSerial publicly for ESPHome component access
    SoftwareSerial softSerial; 

    // ESPHOME/ESP8266 MODIFICATION: Exposed pollState publicly for ISR/poll access
    uint8_t pollState;

private:
    bool    parsePollResp(uint32_t resp);
    // ESPHOME/ESP8266 MODIFICATION: Replaced write0() with esp8266_write_low() using direct register access
    void    write0(void);
    void    beforeWrite(void);
    void    afterWrite(void);

    // ESPHOME/ESP8266 MODIFICATION: The implementation will use the ESP-compatible delayMicroseconds
    static inline void delay_us(uint32_t us);

    uint8_t keypadAddr[KP_SERIAL_MAX_KEYPADS];
    uint8_t numKeypads;
    uint8_t readBuf[KP_SERIAL_READ_BUF_SIZE];
    uint8_t recvMsgLen;
};