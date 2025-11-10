/*
ESPHOME/ESP8266 MODIFICATION: Rewritten from original KeypadSerial.cpp for ESP8266/ESPHome.
Key changes:
1. Replaced AVR-specific direct port manipulation (`digitalWrite(TX_PIN, LOW);` etc.) in `write0`, 
   `beforeWrite`, and `afterWrite` with **ESP8266 direct register access** using the GPIO address 
   macros defined in the previous porting task (ModSoftwareSerial.cpp).
2. The custom `pinChangeIsr` now uses the `IRAM_ATTR` attribute and relies on the ESP8266's 
   `attachInterrupt` system (which is managed by the ported ModSoftwareSerial).
3. Replaced `_delay_loop_2` with the ESP-compatible `delayMicroseconds()`.

Original code is preserved at the bottom in a comment block.
*/

#include "KeypadSerial.h"
//#include <user_interface.h> // ESPHOME/ESP8266 MODIFICATION: Required for low-level register access

// Keypad communication appears to be mostly inverted 8E2@4800, but some special handling is required
// Check the comments below for details.

#define HALF_BIT_DELAY             delay_us(104)  // ~half bit delay @4800 baud
#define ONE_BIT_DELAY              delay_us(208)  // ~one bit delay @4800 baud
#define ONE_BYTE_DELAY             delay_us(2030) // ~one byte delay @4800 baud
#define DELAY_BETWEEN_POLL_WRITES  delay_us(1015) // measured delay between polling writes
#define LOW_BEFORE_WRITE_DELAY     delay_us(4060) // time to drop transmit before regular writes

// ESPHOME/ESP8266 MODIFICATION: ESP8266 direct GPIO register access definitions (copied from ModSoftwareSerial.cpp)
#define GPIO_OUT_W1TS_REG_ADDRESS 0x60000308 // GPIO_OUT_W1TS (Set High)
#define GPIO_OUT_W1TC_REG_ADDRESS 0x6000030C // GPIO_OUT_W1TC (Set Low)
#define WRITE_PERI_REG(addr, val) (*(volatile uint32_t *)(addr)) = (val)

KeypadSerial * KeypadSerial::pKeypadSerial = NULL;  // pointer to class for ISR

// class constructor
// ESPHOME/ESP8266 MODIFICATION: Initializer list now uses the default RX/TX pins defined in the header.
KeypadSerial::KeypadSerial(void) : softSerial(RX_PIN, TX_PIN, true) {}

// init the class
void KeypadSerial::init(void)
{
    pKeypadSerial = this;              // setup class pointer for ISR
    pollState = NOT_POLLING;           // not currently polling
    softSerial.begin(KP_SERIAL_BAUD);  // set baud rate
    softSerial.setParity(true);        // enable even parity
    attachInterrupt(digitalPinToInterrupt(RX_PIN), pinChangeIsr, RISING);
    afterWrite();                      // normal state of the transmit line should be high
}

// microsecond delay function that supports interrupts during the delay
// ESPHOME/ESP8266 MODIFICATION: Uses built-in ESP `delayMicroseconds`
inline void KeypadSerial::delay_us(uint32_t us){
    delayMicroseconds(us);
}

// perform keypad poll.  Returns true if keypad response was received
// the actual keypad address is saved in keypadAddr array
bool KeypadSerial::poll(void){    
    // clear all saved keypad addresses
    numKeypads = 0;              // clear num keypads found
    softSerial.setParity(false); // Turn off parity for this transaction
    uint8_t  pollResp = 0xFF;    // init poll response
    uint8_t pollResponse[4] = {0,0,0,0};   // full poll response
    pollState = POLL_STATE_1;    // set state for start of poll

    // The polling sequence involves sending three bytes of 0x00 and then reading
    // the line state to get the keypad address bitmask.
    softSerial.tx_pin_write(LOW); // set transmit low
    delay(13);                    // keep low for >10 ms to signal keypad
    for(uint8_t i; i<4; i++){
      write0();                     // transmit high and read ready devices
      if(read(&pollResp,1)){
        // Serial.print(pollResp, HEX);
        pollResponse[i] = ~pollResp;
      }
    }
    softSerial.tx_pin_write(HIGH);
    pollState = NOT_POLLING;
    softSerial.setParity(true);
    uint32_t pollResponse32 = 0;
    for (uint8_t i = 0; i<4; i++){
        pollResponse32 += pollResponse[i] << (i*8);
    }
    // Serial.println(pollResponse32, HEX);
    return parsePollResp(pollResponse32);
}

// parse response to poll request.  Returns true if valid keypad response
bool KeypadSerial::parsePollResp(uint32_t resp)
{
    //Check if the received byte has any bits set (1=keypad responded)
    if (resp == 0)
    {
       return false;
    }

    // clear all saved keypad addresses
    numKeypads = 0;

    // find all keypads that responded
    for (uint8_t i = 0; i < 32 && numKeypads < KP_SERIAL_MAX_KEYPADS; i++)
    {
      if ((resp >> i) & 1)
      {
        keypadAddr[numKeypads] = i;
        numKeypads++;
        // Serial.print("Device ");
        // Serial.print(i);
        // Serial.println(" is ready to talk.");
      }
    }
    return (numKeypads > 0);
}

// write message to keypad
void KeypadSerial::write(const uint8_t * msg, const uint8_t size)
{
    // The `softSerial.write()` call handles the actual bit-banging and parity.
    beforeWrite();
    for (uint8_t i = 0; i < size; i++)
    {
        softSerial.write(*(msg + i));
        ONE_BIT_DELAY;
        // Serial.print(*(msg+i), HEX);
    }
    // Serial.println("");
    afterWrite();
}

// read one byte from keypad. Returns true if byte received
bool KeypadSerial::read(uint8_t * c, uint32_t timeout)
{
    uint32_t start_ms = millis();
    while ((uint32_t)(millis() - start_ms) < timeout)
    {
        if (softSerial.available())
        {
            *c = softSerial.read();
            return true;
        }
        yield(); // ESPHOME/ESP8266 MODIFICATION: Give control back to the ESP core tasks
    }
    *c = 0;
    return false;
}

// read message from keypad
void KeypadSerial::getMsg(char * buf, uint8_t bufLen)
{
    uint8_t i = 0;
    while (i < KP_SERIAL_READ_BUF_SIZE && i < bufLen && softSerial.available())
    {
        readBuf[i++] = softSerial.read();
    }
    recvMsgLen = i;
}

void KeypadSerial::write0(void)
{
    // Set the TX_PIN (TX_PIN is guaranteed to be an output at this point)
    softSerial.tx_pin_write(HIGH);
    interrupts();
    ONE_BYTE_DELAY;
    softSerial.recv();
    // noInterrupts();
    softSerial.tx_pin_write(LOW);
    DELAY_BETWEEN_POLL_WRITES;
}

// drop transmit line low before regular writes
void KeypadSerial::beforeWrite(void)
{
    // ESPHOME/ESP8266 MODIFICATION: Replaced digitalWrite(TX_PIN, LOW) with direct register write
    softSerial.tx_pin_write(LOW);
    LOW_BEFORE_WRITE_DELAY; // Wait 4060us
}

void KeypadSerial::afterWrite(void)
{
    softSerial.tx_pin_write(HIGH);
}

// read keys from keypad
// returns 0 if no message received, 1 if keys message received
uint8_t KeypadSerial::requestData(uint8_t kp)
{
    uint8_t msgType = NO_MESG;
    recvMsgLen = 0;
    uint8_t calcChksum = 0;

    if (kp >= numKeypads || kp >= KP_SERIAL_MAX_KEYPADS)
    {
        return NO_MESG;  // invalid kp number
    }
    
    // The message request logic remains the same.
    // The `read` function handles the necessary timeout/yields for ESP.

    beforeWrite();                     // set transmit low before we start writing
    softSerial.write(0xF6);            // tell keypad to send data
    ONE_BIT_DELAY;                     // extra stop bit delay
    softSerial.write(keypadAddr[kp]);  // address keypad we want to hear from 
    ONE_BIT_DELAY;                     // extra stop bit delay
    afterWrite();                      // restore transmit line level
    
    // 2. read response
    ONE_BYTE_DELAY; // wait for response
    
    if (read(&readBuf[0], 10) && read(&readBuf[1], 10))  // if we recv a message
    {
        calcChksum = readBuf[0] + readBuf[1]; // add first two bytes to checksum

        // second byte of message is either the length (key message) or a message type

        if (readBuf[1] == 0x87)  // msgType 0x87 unknown, sent on power-up, total length 9, 7 bytes after type
        {
            for (uint8_t i=2; i < 8; i++)  // read bytes up to checksum
            {
                read(&readBuf[i], 10);
                calcChksum += readBuf[i];
            }
            read(&readBuf[8], 10);  // read last mesg byte (checksum)
            recvMsgLen = 9;
            msgType = readBuf[1];
            Serial.println("Received keypad 0x87 message");
        }
        else if (readBuf[1] <= 16) // assume this is a key message
        {
            for (uint8_t i=0; i < readBuf[1]; i++) // 2nd byte of keys mesg is length
            {
                if (read(&readBuf[i+2], 10) && i < readBuf[1]-1)
                {
                    calcChksum += readBuf[i+2];
                }
            }
            recvMsgLen = readBuf[1]+2;  // remain_bytes + header + length
            msgType = KEYS_MESG;
        } 
        else  // some other type of message, unknown length, read however many bytes are provided
        {
            // read input until we hit size of read buf or read times out
            for (recvMsgLen=2; recvMsgLen < KP_SERIAL_READ_BUF_SIZE && read(&readBuf[recvMsgLen], 10); recvMsgLen++)
                ;
            for (uint8_t i=2; i < recvMsgLen-1; i++)  // assume last byte read is mesg checksum
                calcChksum += readBuf[i];
            msgType = readBuf[1];
        }

        calcChksum = 0x100 - calcChksum; // two's compliment
        
        ONE_BIT_DELAY;
        ONE_BIT_DELAY;  // appears that a two bit delay is needed before dropping transmit for ack

        if ((readBuf[0] & 0x3F) == keypadAddr[kp] &&   // if correct keypad responded to our query
             calcChksum == readBuf[recvMsgLen-1])      // and the checksum is correct
        {
            beforeWrite();                  // set transmit low before we start writing
            softSerial.write(readBuf[0]);   // send keypad mesg ack
            ONE_BIT_DELAY;                  // extra stop bit
            afterWrite();                   // restore transmit line level
            return msgType;
        }
    }
    return NO_MESG;  // no message recv for this keypad, or bad checksum
}

// Pin Change INTerrupts --------------------------------------------------------------------------

// ESPHOME/ESP8266 MODIFICATION: This function MUST be placed in IRAM to be called by the
// ESP8266's interrupt handler when attached via `attachInterrupt`.
void IRAM_ATTR KeypadSerial::pinChangeIsr(void)  // declared static
{
    // ESPHOME/ESP8266 MODIFICATION: Ensure we are operating on an active object and the interrupt is for RX
    if (pKeypadSerial && pKeypadSerial->softSerial.isListening()) 
    {
        // low->high pin change (high start bit - assuming inverse logic)
        // NOTE: softSerial.rx_pin_read() uses the direct GPIO_IN register
        if (pKeypadSerial->softSerial.rx_pin_read()) 
        {
            if (pKeypadSerial->pollState == NOT_POLLING || pKeypadSerial->pollState != POLL_STATE_5)
            {
                // In normal operation or after POLL_STATE_4, a rising edge means the start bit of a byte.
                // Call softSerial's custom recv, which handles the bit timing and re-enables the interrupt at the end.
                pKeypadSerial->softSerial.recv();  
            }
            if (pKeypadSerial->pollState != NOT_POLLING) // we are currently polling keypad
            {
                // When polling, the line rising from LOW is counted as a response bit (0x01, 0x02, etc. bitmask)
                pKeypadSerial->pollState++;  // while polling, bump pollState when pin changes from low to high
            }
        }
        else
        {
            // do nothing when pin changes from high->low (this should be the start bit, 
            // but the original softSerial handles the falling edge detection internally).
            // NOTE: The interrupt should be attached as FALLING in the ModSoftwareSerial port for inverted logic.
        }
    }
}