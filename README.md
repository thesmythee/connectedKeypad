# ECP2MQTT
Connecting a common brand keypad to make available for use on MQTT systems.

The purpose of this is to be able to use Honeywell alarm keypads with the Home Assistant alarm control panel integration.  Honeywell keypad models 6150 and 6160 can be used with this.  The 6150 is a fixed message display and the 6160 is a 2x16 character alphanumeric display.

This is set up to use Home Assistant integration Manual MQTT Alarm Control Panel.

HOW TO USE:
Update the .ino file with your wifi credentials, MQTT credentials, and ALARM_CODE. 
Set the Command topic to be the topic that the keypad uses to send messages to Home Assistant.
Set the State topic to be the topic that the HA Alarm control panel uses to return its state.
"F7" messages can be sent to the MQTT state topic to configure keypad output.
Keypresses from keypad can be read in the MQTT command topic.

For the hardware side, a microcontroller (uC) is required.  I used an ESP8266.  ESP32 should also work with little to no code change.  
The uC needs a buck converter for a power supply and logic level conversion.  
An optocoupler for uC to keypad communication and a resistor divider for keypad to uC comms. 
The uC transmit is currently set for GPIO5 and uC receive is set for GPIO4.  These are declared in KeypadSerial.h.
