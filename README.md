# ArduinoENC_Artnet-WS2811
### This Project is using an Arduino Nano to receive Artnet Data and drive WS2811/WS2812 Leds. 
This Project was started in January 2017, it is working and still in development.

The Arduino is in Charge of reading Artnet Packets received by an ENC28J60 Ethernet Shield, prepare the data and output the WS2811 Protocol by using the Adafruit Neopixel library via normal I/O-Pins. For my Installation i added a Screw Terminal Shield for easier wiring.

## Instructions
- Copy the Folder as a *Projectfolder* to your Sketchbook
- Copy the Content from *Projectfolder*/libraries to your Arduino User Library location
- Leave the folder *src* as it is (Inside is a Modified Version from the Adafruit Neopixel library)
- *DATAPIN1* and *DATAPIN2* select pins which are used to connect to the WS2811/12 Data input line (If changed, check *CONNECTPIN1* and *CONNECTPIN2*)
- *ARDUINO_NR* can be used to select the IP and MAC Adress configuration for multiple Arduinos in one Network
Remember!! do NOT use the Arduino internal Voltage regulator (5V Output) as Leds power supply and add a small resistor in series to the Dataline for example 50-120Ohms. Connect all grounds together (Arduino GND and power supply GND)

## Changelog

### V1.1 - 2019.02.19
- 2019.02.19  Modified imports - Moved modified Neopixel library in Subfolder *PROJECTFOLDER*/src
- Optimized Code readability
- added Intructions to README.md
### V1.0 - 2018.05.07
- 2 Universe Artnet to Ws2811 on 2 Output Pins (1 Universe per Pin)
