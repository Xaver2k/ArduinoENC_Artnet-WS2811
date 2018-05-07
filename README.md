# ArduinoENC_Artnet-WS2811
### This Project is using an Arduino Nano to receive Artnet Data and drive WS2811/WS2812 Digital Leds. This Project was started in January 2017, it is working and still in development.

The Arduino is in Charge of reading Artnet Packets received by an ENC28J60 Ethernet Shield, prepare the data and output the WS2811 Protocol by using the Adafruit Neopixel library via normal I/O-Pins. For my Installation i added a Screw Terminal Shield for easier wiring.

Remember do not use the Arduino internal Power Supply for Leds supply and add a small resistor in series to the Dataline for example 50-120Ohms. Connect all grounds together (Arduino GND and power supply GND)

Changelog

- 2018.05.07  2 Universe Artnet to Ws2811 on 2 Output Pins (1 Universe per Pin)
