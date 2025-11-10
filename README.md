# connectedKeypad
Connecting a common brand keypad to make available for use on Home Assistant systems.

The purpose of this is to be able to use Honeywell alarm keypads with the Home Assistant alarm control panel integration.

This is set up to use Home Assistant integration Manual MQTT Alarm Control Panel.
Set the Command topic to be the topic that the keypad uses to send messages to Home Assistant.
Set the State topic to be the topic that the HA Alarm control panel uses to return its state.
"F7" messages can be sent to the MQTT state topic to configure keypad output.
Keypresses from keypad can be read in the MQTT command topic.

Honeywell keypad models 6150 and 6160 can be used with this.  The 6150 is a fixed message display and the 6160 is a 2x16 character alphanumeric display.

Update the .ino file with your wifi credentials, MQTT credentials, and ALARM_CODE. 

For the hardware side, I used an ESP8266.  ESP32 should also work.  The ESP8266 needs a buck converter for a power supply and logic level conversion.  I'm currently using an optoisolator for uC to keypad communication and a resistor divider for keypad to uC comms.  See https://github.com/Dilbert66/esphome-vistaECP for connection methods between the uC and keypad.  The uC transmit is currently set for GPIO5 and uC receive is set for GPIO4.  These are declared in KeypadSerial.h.
