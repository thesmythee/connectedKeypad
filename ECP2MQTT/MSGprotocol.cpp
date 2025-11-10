// file MSGprotocol.cpp - methods for converting data into plain text strings transferred over USB serial

// if you want to change the format of the messages exchanged with your CPU via the USB serial port, update 
// this class

#include "MSGprotocol.h"
#include "KeypadSerial.h"

// when arduino code inits, use these initial keypad values
#define INIT_MSG   "F7 z=00 t=1 c=0 r=1 a=0 s=0 p=1 b=1 1=Arduino Init     2=Completed  v1.01"
// #define INIT_MSGA "F7A z=01 t=2 c=0 r=0 a=1 s=1 p=1 b=1 1=Arduino Alternat 2=Completed  v1.01"
// #define FULL_MSG    "F7 z=00 t=0 n=0 x=0 f=0 k=0 y=0 o=0 r=1 l=0 e=0 s=0 w=0 a=0 p=0 d=0 c=0 i=0 g=0 j=0 b=1 1=TEST    TEST     2=    TEST    TEST"
// #define FULL_MSGA  "F7A z=88 t=0 n=0 x=0 f=0 k=0 y=0 o=0 r=1 l=0 e=0 s=0 w=0 a=0 p=0 d=0 c=0 i=0 g=0 j=0 b=1 1=    TEST    TEST 2=TEST    TEST    "

// macros to determine if command starts with 'F7' or 'F7A'
#define F7_MSG_ALTZ(s)       (*((s)+0) == 'F' && *((s)+1) == '7' && *((s)+2) == 'Z')
#define F7_MSG_ALT(s)        (*((s)+0) == 'F' && *((s)+1) == '7' && *((s)+2) == 'A')
#define F7_MSG(s)            (*((s)+0) == 'F' && *((s)+1) == '7')

// init class
void MSGprotocol::init(void)
{
    count = 0;
    altMsgActive = false;
    
    // init F7 message structs
    initF7(&msgF7[0]);
    initF7(&msgF7[1]);

    parseRecv(INIT_MSG, strlen(INIT_MSG));
    parseRecv(INIT_MSGA, strlen(INIT_MSGA));
}

// initialize F7 message struct
void MSGprotocol::initF7(t_MesgF7 * pMsgF7)
{
    memset((void *)pMsgF7, 0, sizeof(t_MesgF7));

    pMsgF7->type    = 0xF7;  // set F7 type
    pMsgF7->addr1   = 0x00;  // send to devices addressed 0-7
    pMsgF7->addr2   = 0x00;  // send to devices addressed 8-15
    pMsgF7->keypads = 0xFF;  // send to keypads addressed 16-23
    pMsgF7->addr4   = 0x00;  // send to devices addressed 34-31?
    pMsgF7->prog    = 0x00;  // programming mode (not used)
    pMsgF7->zone    = 0x00;  // # displayed on 6150 keypads seven segment displays
}

void MSGprotocol::toneCheck(){
    t_MesgF7 newF7;
    t_MesgF7 * pNewF7 = &newF7;
    memcpy(pNewF7, &msgF7[0], sizeof(t_MesgF7)); // copy existing F7 mesg struct
    // If F7 message has tone 1, 2, or 3, then set tone to 0
    uint8_t toneRead = pNewF7->byte1 & 0x07;
    if( toneRead == 0x01 | toneRead == 0x02 | toneRead == 0x03 ){
        toneRead = 0;
        uint8_t toneByte = pNewF7->byte1 & 0xF8;
        pNewF7->byte1 = toneRead & toneByte;
        memcpy(&msgF7[0], pNewF7, sizeof(t_MesgF7)); // copy existing F7 mesg struct
    }
    // Verify alternate tone is same as primary
    if (altMsgActive){
        memcpy(pNewF7, &msgF7[1], sizeof(t_MesgF7)); // copy existing F7 mesg struct
        uint8_t toneByte = pNewF7->byte1 & 0xF8;
        pNewF7->byte1 = toneRead & toneByte;
        memcpy(&msgF7[1], pNewF7, sizeof(t_MesgF7)); // copy existing F7 mesg struct
    }
}

// parse received command string
uint8_t MSGprotocol::parseRecv(const char * msg, const uint8_t len)
{
    if (len > 4 && F7_MSG_ALT(msg)) // an alt F7 command only updates the secondary F7 message
    {
        altMsgActive = true;
        return parseF7(msg+4, len-4, &msgF7[1]);  // parse the command after 'F7A '
    }
    else if (len > 4 && F7_MSG_ALTZ(msg)) // a primary F7 command sets altMsg false, so send F7A msg second if needed
    {
        count = 0;  // zero count so primary F7 msg is the next one displayed
        altMsgActive = false;
        return parseF7Z(msg+4, len-4, &msgF7[0]);  // parse the command after 'F7Z '
    }
    else if (len > 4 && F7_MSG(msg)) // a primary F7 command sets altMsg false, so send F7A msg second if needed
    {
        count = 0;  // zero count so primary F7 msg is the next one displayed
        altMsgActive = false;
        return parseF7(msg+3, len-3, &msgF7[0]);  // parse the command after 'F7 '
    }
    return 0x0;  // received unknown command
}

// generate message from data received from keypad
const char * MSGprotocol::keyMsg(char * buf, uint8_t bufLen, uint8_t addr, uint8_t len, uint8_t * pData, uint8_t type)
{
    // format of message is KEYS_XX[N] key0 key1 ... keyN-1, where XX is keypad number, N is key count
    // or                   UNK__XX[N] byte0 byte1 .. byteN-1 for unknown message from keypad XX with N bytes

    uint8_t idx = 0;
    idx += sprintf(buf+idx, "%s_%2d[%02d] ", type == KEYS_MESG ? "KEYS" : "UNK_", addr, len);
    for (uint8_t i=0; i < len && bufLen - idx > 6; i++)
    {
        idx += sprintf(buf+idx, "0x%02x ", *(pData+i));
    }
    sprintf(buf+idx-1, "\n");
    return (const char *)buf;
}

// parse F7 command, form is F7[A] z=FC t=0 c=1 r=0 a=0 s=0 p=1 b=1 1=1234567890123456 2=ABCDEFGHIJKLMNOP
//   z - zone             (byte arg)
//   t - tone             (nibble arg)   byte 1  bits 012
//   n - night            (bool arg)     byte 1  bit 4
//   x - canceled         (bool arg)     byte 1  bit 5
//   f - fire beeping     (bool arg)     byte 2  bit 0 (0x01)
//   k - lcd "check"      (bool arg)     byte 2  bit 1 (0x02)
//   y - hide R digit     (bool arg)     byte 2  bit 2 (0x04)
//   o - hide L digit     (bool arg)     byte 2  bit 3 (0x08)
//   r - lcd "ready"      (bool arg)     byte 2  bit 4 (0x10)
//   l - lcd "fire"       (bool arg)     byte 2  bit 5 (0x20)
//   e - battery          (bool arg)     byte 2  bit 6 (0x40)
//   s - arm-stay         (bool arg)     byte 2  bit 7 (0x80)
//   w - alarm            (bool arg)     byte 3  bit 1
//   a - arm-away         (bool arg)     byte 3  bit 2
//   p - ac               (bool arg)     byte 3  bit 3
//   d - bypass           (bool arg)     byte 3  bit 4
//   c - chime            (bool arg)     byte 3  bit 5
//   i - instant          (bool arg)     byte 3  bit 7
//   g - 100 zone         (bool arg)     byte46  bit 0
//   j - 200 zone         (bool arg)     byte46  bit 1
//   m - test             (bool arg)     byte46  bit 4
//   h - phone            (bool arg)     byte46  bit 5
//   b - lcd-backlight-on (bool arg)
//   1 - line1 text       (16-chars)
//   2 - line2 text       (16-chars)

// BYTE1 tone notes
//   00-03 - low two bits define chime count for each F7 msg (0 none, 1,2,3 chime count per msg)
//   04    - fast pulsing tone (like there is an error, or timeout almost done)
//   05-06 - slow pulsing tone (like when alarm is in arm-delay and it is time to leave)
//   07    - continous tone (not pulsing)
//   0x40 bit causes incompat. con. error

// BYTE2 notes: bit(0x80) 1 -> ARMED-STAY, bit(0x10) 1 -> READY (1 when ok, 0 when exit delay)
// BYTE3 notes: bit(0x20) 1 -> chime on, bit(0x08) 1 -> ac power ok, bit(0x04) 1 -> ARMED_AWAY

uint8_t MSGprotocol::parseF7(const char * msg, uint8_t len, t_MesgF7 * pMsgF7)
{   
    bool success = true;
    bool lcd_backlight = false;

    t_MesgF7 newF7;
    t_MesgF7 * pNewF7 = &newF7;
    memcpy((void *)pNewF7, (const void *)pMsgF7, sizeof(t_MesgF7)); // copy existing F7 mesg struct
    // memset((void *)pNewF7, 0, sizeof(t_MesgF7)); // zero out contents

    for (uint8_t i=0; i < len && *(msg+i) != '\0'; i++)  // msg pointer starts after 'F7 ' or 'F7A '
    {
        if (*(msg+i) != ' ')  // skip over spaces
        {
            char parm = *(msg+i);
            i += 2;  // move past parm and '=', msg+i now points at arg 

            switch (parm)
            {
            case 'z': //set zone number
                pNewF7->zone = GET_BYTE(*(msg+i), *(msg+i+1)); i += 2;
                if(pNewF7->zone == 0x00){
                    pNewF7->byte2 = SET_BIT2(pNewF7->byte2, 1 );
                    pNewF7->byte2 = SET_BIT3(pNewF7->byte2, 1 );
                } else {
                    pNewF7->byte2 = SET_BIT2(pNewF7->byte2, 0 );
                    pNewF7->byte2 = SET_BIT3(pNewF7->byte2, 0 );
                }
                break;
            case 't': //set tone rhythm
                pNewF7->byte1 = GET_NIBBLE(*(msg+i)); i++;
                break;
            case 'n': //set NIGHT toggle
                pNewF7->byte1 = SET_BIT4(pNewF7->byte1, GET_BOOL(*(msg+i))); i++;
                break;
            case 'x': //set CANCELED toggle
                pNewF7->byte1 = SET_BIT5(pNewF7->byte1, GET_BOOL(*(msg+i))); i++;
                break;
            // case '?': //set testing toggle
            //     pNewF7->byte3 = SET_BIT7(pNewF7->byte3, GET_BOOL(*(msg+i))); i++;
            //     break;
            case 'f': //set FIRE lcd toggle
                pNewF7->byte2 = SET_BIT0(pNewF7->byte2, GET_BOOL(*(msg+i))); i++;
                break;
            case 'k': //set CHECK lcd toggle
                pNewF7->byte2 = SET_BIT1(pNewF7->byte2, GET_BOOL(*(msg+i))); i++;
                break;
            case 'y': //set SYSTEM lcd toggle
                pNewF7->byte2 = SET_BIT2(pNewF7->byte2, GET_BOOL(*(msg+i))); i++;
                break;
            case 'o': //set no-ac lcd toggle (and maybe light?)
                pNewF7->byte2 = SET_BIT3(pNewF7->byte2, GET_BOOL(*(msg+i))); i++;
                break;
            case 'r': //set ready lcd toggle (and maybe light?)
                pNewF7->byte2 = SET_BIT4(pNewF7->byte2, GET_BOOL(*(msg+i))); i++;
                break;
            case 'l': //set fire lcd toggle
                pNewF7->byte2 = SET_BIT5(pNewF7->byte2, GET_BOOL(*(msg+i))); i++;
                break;
            case 'e': //set bat lcd toggle
                pNewF7->byte2 = SET_BIT6(pNewF7->byte2, GET_BOOL(*(msg+i))); i++;
                break;
            case 's': //set arm-stay lcd toggle
                pNewF7->byte2 = SET_BIT7(pNewF7->byte2, GET_BOOL(*(msg+i))); i++;
                break;
            case 'w': //set alarm lcd toggle
                pNewF7->byte3 = SET_BIT1(pNewF7->byte3, GET_BOOL(*(msg+i))); i++;
                break;
            case 'a': //set arm-away lcd toggle
                pNewF7->byte3 = SET_BIT2(pNewF7->byte3, GET_BOOL(*(msg+i))); i++;
                break;
            case 'p': //set power-on lcd toggle
                // pNewF7->byte3 = SET_BIT3(pNewF7->byte3, GET_BOOL(*(msg+i))); i++;
                pNewF7->byte3 = SET_BIT3(pNewF7->byte3, GET_BOOL(*(msg+i))); i++;
                break;
            case 'd': //set bypass lcd toggle
                pNewF7->byte3 = SET_BIT4(pNewF7->byte3, GET_BOOL(*(msg+i))); i++;
                break;
            case 'c': //set chime lcd toggle
                pNewF7->byte3 = SET_BIT5(pNewF7->byte3, GET_BOOL(*(msg+i))); i++;
                break;
            case 'i': //set instant lcd toggle
                pNewF7->byte3 = SET_BIT7(pNewF7->byte3, GET_BOOL(*(msg+i))); i++;
                break;
            case 'g': //set 100 lcd toggle
                pNewF7->byte46 = SET_BIT0(pNewF7->byte46, GET_BOOL(*(msg+i))); i++;
                break;
            case 'j': //set 200 lcd toggle
                pNewF7->byte46 = SET_BIT1(pNewF7->byte46, GET_BOOL(*(msg+i))); i++;
                break;
            case 'm': //set test lcd toggle
                pNewF7->byte46 = SET_BIT4(pNewF7->byte46, GET_BOOL(*(msg+i))); i++;
                break;
            case 'h': //set phone lcd toggle
                pNewF7->byte46 = SET_BIT5(pNewF7->byte46, GET_BOOL(*(msg+i))); i++;
                break;
            case 'b': //set lcd backlight illuminance
                lcd_backlight = GET_BOOL(*(msg+i)); i++;
                break;
            case '1':  // line1 arg must occur after 'b' parameter for this code to work
                memset(pNewF7->line1, 0, LCD_LINE_LEN);
                for (uint8_t j=0; j < LCD_LINE_LEN && i < len; j++)
                {
                    pNewF7->line1[j] = *(msg+i) & 0x7f; 
                    i++;
                }
                pNewF7->line1[0] |= lcd_backlight ? 0x80 : 0x00;  // or in backlight bit
                break;
            case '2':
                memset(pNewF7->line2, 0, LCD_LINE_LEN);
                for (uint8_t j=0; j < LCD_LINE_LEN && i < len; j++)
                {
                    pNewF7->line2[j] = *(msg+i) & 0x7f; 
                    i++;
                }
                break;
            default:
                success = false;
                Serial.println("Failed to parse F7");
                break;
            }
        }
    }

    if (success)
    {
        memcpy(pMsgF7, pNewF7, sizeof(t_MesgF7)); // replace existing F7 mesg with updated version

        pMsgF7->chksum = 0;

        for (uint8_t i=0; i < 44; i++)
        {
            pMsgF7->chksum += *(((uint8_t *)pMsgF7) + i);
        }

        pMsgF7->chksum = 0x100 - pMsgF7->chksum;  // two's compliment

        return 0xF7;
    }
    return 0;  // failed to parse message
}

uint8_t MSGprotocol::parseF7Z(const char * msg, uint8_t len, t_MesgF7 * pMsgF7)
{   
    bool success = true;

    t_MesgF7 newF7;
    t_MesgF7 * pNewF7 = &newF7;
    memset((void *)pNewF7, 0, sizeof(t_MesgF7)); // zero out contents
    
    pNewF7->type    = 0xF7;  // set F7 type
    if(len == 60){
        pNewF7->addr1    = GET_BYTE(*(msg+0), *(msg+1));
        pNewF7->addr2    = GET_BYTE(*(msg+2), *(msg+3));
        pNewF7->keypads  = GET_BYTE(*(msg+4), *(msg+5));
        pNewF7->addr4    = GET_BYTE(*(msg+6), *(msg+7));
        pNewF7->zone     = GET_BYTE(*(msg+8), *(msg+9));
        pNewF7->byte1    = GET_BYTE(*(msg+10), *(msg+11));
        pNewF7->byte2    = GET_BYTE(*(msg+12), *(msg+13));
        pNewF7->byte3    = GET_BYTE(*(msg+14), *(msg+15));
        pNewF7->prog     = GET_BYTE(*(msg+16), *(msg+17));
        pNewF7->prompt   = GET_BYTE(*(msg+18), *(msg+19));
        pNewF7->pad1     = GET_BYTE(*(msg+20), *(msg+21));
        memset(pNewF7->line1, 0, LCD_LINE_LEN);
        for (uint8_t j=22; j < (22 + 16); j++){ pNewF7->line1[j-22] = *(msg+j) & 0X7F; }
        memset(pNewF7->line2, 0, LCD_LINE_LEN);
        for (uint8_t j=38; j < (38 + 16); j++){ pNewF7->line2[j-38] = *(msg+j) & 0X7F; }
        pNewF7->byte46   = GET_BYTE(*(msg+54), *(msg+55));
        pNewF7->byte47   = GET_BYTE(*(msg+56), *(msg+57));
        pNewF7->byte48   = GET_BYTE(*(msg+58), *(msg+59));
    } else {
        success = false;
        Serial.println("Failed to parse F7Z ");
        // Serial.print("F7Z length");
        // Serial.println(len);
    }

    if (success)
    {
        memcpy(pMsgF7, pNewF7, sizeof(t_MesgF7)); // replace existing F7 mesg with updated version

        pMsgF7->chksum = 0;

        for (uint8_t i=0; i < 44; i++)
        {
            pMsgF7->chksum += *(((uint8_t *)pMsgF7) + i);
        }

        pMsgF7->chksum = 0x100 - pMsgF7->chksum;  // two's compliment

        return 0xF7;
    }
    return 0;  // failed to parse message
}

// returned mesg always alternates between 2 stored messages (which may be the same)
const uint8_t * MSGprotocol::getF7(void)
{ 
    printF7();
    if (altMsgActive)
        return (const uint8_t *)&(msgF7[count++ & 0x1]);
    else
        return (const uint8_t *)&(msgF7[0]);
}


#if _DEBUG
// debug: print message struct into buf (broken, needs to be updated)
// const char * MSGprotocol::printF7(char * buf)
void MSGprotocol::printF7()
{
  const uint8_t * tempMessage = (const uint8_t *)&(msgF7[0]);
  for(int i = 0; i < 11; i++){
    Serial.print((char)(*(tempMessage + i)), HEX); 
    Serial.print(" ");
  }
  Serial.println();
  
}
#else
void MSGprotocol::printF7(){}
#endif

