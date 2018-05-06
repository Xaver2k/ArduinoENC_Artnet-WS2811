/*
This Project will receive multiple universes via Artnet and control a strip of ws2811 leds via 
Adafruit's NeoPixel library: https://github.com/adafruit/Adafruit_NeoPixel
This example may be copied under the terms of the MIT license, see the LICENSE file for details

Components per unit
1x Arduino Nano 
1x Deek-Robot Screw Terminal shield 
1x ENC28J60 for Nano Shield

This Code is written for Maximum Ram saving and fps, without modifying the libraries!!
*/

// Arduino Nano with Deek-Robot and ENC28J60 Shield

#include <SPI.h> // Nesseccary for Ethernet Library
#include <Adafruit_NeoPixel.h> // Library for Led protocol

// Ethernet Librarys
#include <UIPEthernet.h>
#include <UIPUdp.h>

// Artnet Specific
#define ART_NET_PORT 6454       // UDP specific
#define ART_DMX 0x5000          // Opcode Artnet DMX Paket
#define MAX_BUFFER_ARTNET 530   // Artnet Buffer Size
#define ART_NET_ID "Art-Net\0"  // Artnet Paket ID
#define ART_DMX_START 18        // Artnet LED Offset

// Led Data
#define NUMLEDS 171 // Number of LEDs per Universe/Output + 18 Byte Artnet Header
#define DATAPIN1 2  // Output Pin 1th Universe
#define DATAPIN2 4  // Output Pin 2nd Universe

// Select Arduino's IP, MAC and Universe configuration. Prepared for 5 Arduinos
#define ARDUINO_NR 1  // Define Arduino Number 1, 2, 3, 4, 5

// Frames per second counter, uncomment for statistics
//#define FPS_COUNTER

/*  
 *  To connect Resistors in Data line without external pcb, these Pins are bend away from the shield.
 *  On the Screw terminal Shield these Pins are connected to the other end of the Resistors.
 *  !!Important check they are not connected to the Arduino, high risk of short-circuit!!
*/
#define CONNECTPIN1 5 // To connect Resistor without external pcb... resistor is bridged from DATAPIN1
#define CONNECTPIN2 7 // To connect Resistor without external pcb... resistor is bridged from DATAPIN2

EthernetUDP udp;
uint8_t artnetHeader[ART_DMX_START];
uint16_t packetSize;

Adafruit_NeoPixel leds1 = Adafruit_NeoPixel(NUMLEDS, DATAPIN1, NEO_GRB + NEO_KHZ800);

// Arduino Selection change IPs, MACs and Universe to you needs
 
// 1th Arduino Nano
#if ARDUINO_NR == 1
  #define STARTUNIVERSE 0
  #define SECONDUNIVERSE STARTUNIVERSE+1
  IPAddress ip(192, 168, 0, 100);
  byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x00, 0xEC};

// 2nd Arduino Nano
#elif ARDUINO_NR == 2
  #define STARTUNIVERSE 2
  #define SECONDUNIVERSE STARTUNIVERSE+1
  IPAddress ip(192, 168, 0, 102);
  byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x02, 0xEC};

// 3th Arduino Nano
#elif ARDUINO_NR == 3
  #define STARTUNIVERSE 4
  #define SECONDUNIVERSE STARTUNIVERSE+1
  IPAddress ip(192, 168, 0, 104);
  byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x04, 0xEC};

// 4th Arduino Nano
#elif ARDUINO_NR == 4
  #define STARTUNIVERSE 6
  #define SECONDUNIVERSE STARTUNIVERSE+1
  IPAddress ip(192, 168, 0, 106);
  byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x06, 0xEC};
 
// 5th Arduino Nano
#elif ARDUINO_NR == 5
  #define STARTUNIVERSE 8
  #define SECONDUNIVERSE STARTUNIVERSE+1
  IPAddress ip(192, 168, 0, 108);
  byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x08, 0xEC};
#endif

void setup()
{
  #ifdef FPS_COUNTER
  Serial.begin(9600);   // For tests only
  #endif
  
  pinMode(CONNECTPIN1, OUTPUT);   // Just for safety, remember the resistor and pcb stuff
  pinMode(CONNECTPIN2, OUTPUT);   // Just for safety, remember the resistor and pcb stuff
  digitalWrite(CONNECTPIN1, LOW); // Just for safety, remember the resistor and pcb stuff
  digitalWrite(CONNECTPIN2, LOW); // Just for safety, remember the resistor and pcb stuff
  
  Ethernet.begin(mac,ip);
  udp.begin(ART_NET_PORT);
  leds1.begin();
  
  initTest(); // Light test on Startup
}

void loop()
{
  packetSize = udp.parsePacket();

  if (packetSize <= MAX_BUFFER_ARTNET && packetSize > 0)
  {
      udp.read(artnetHeader, ART_DMX_START);

      // Check that packetID is "Art-Net" else ignore
      if ( artnetHeader[0] == 'A' && artnetHeader[1] == 'r' && artnetHeader[2] == 't' && artnetHeader[3] == '-' 
            && artnetHeader[4] == 'N' && artnetHeader[5] == 'e' && artnetHeader[6] == 't') {
        if ((artnetHeader[8] | artnetHeader[9]) << 8 == ART_DMX)
        {
          udp.read(leds1.getPixels(),artnetHeader[17] | artnetHeader[16] << 8); // Size of universe = artnetHeader[17] | artnetHeader[16] << 8
          
          switch (artnetHeader[14] | artnetHeader[15] << 8) { // Universe
            case (STARTUNIVERSE):
              leds1.setPin(DATAPIN1);
              break;
            case (SECONDUNIVERSE):
              leds1.setPin(DATAPIN2);
              break;
            default: 
              break;
            }     

          leds1.show();
        };
      }
  }

  // FPS-Counter for statistics only
  #ifdef FPS_COUNTER
    fps(5);
  #endif
}


// LED Testing Pattern
void initTest()
{
  for (int i = 0 ; i < NUMLEDS ; i++)
    leds1.setPixelColor(i, 127, 0, 0);
  leds1.setPin(DATAPIN1);
  leds1.show();
  leds1.setPin(DATAPIN2);
  leds1.show();
  delay(500);
  for (int i = 0 ; i < NUMLEDS ; i++)
    leds1.setPixelColor(i, 0, 127, 0);
  leds1.setPin(DATAPIN1);
  leds1.show();
  leds1.setPin(DATAPIN2);
  leds1.show();
  delay(500);
  for (int i = 0 ; i < NUMLEDS ; i++)
    leds1.setPixelColor(i, 0, 0, 127);
  leds1.setPin(DATAPIN1);
  leds1.show();
  leds1.setPin(DATAPIN2);
  leds1.show();
  delay(500);
  for (int i = 0 ; i < NUMLEDS ; i++)
    leds1.setPixelColor(i, 0, 0, 0);
  leds1.setPin(DATAPIN1);
  leds1.show();
  leds1.setPin(DATAPIN2);
  leds1.show();
}


// Frames per second counter for statistics only
#ifdef FPS_COUNTER
static inline void fps(const int seconds){
  // Create static variables so that the code and variables can all be declared inside a function
  static unsigned long lastMillis;
  static unsigned long frameCount;
  static unsigned int framesPerSecond;

  // It is best if we declare millis() only once
  unsigned long now = millis();
  frameCount ++;
  if (now - lastMillis >= seconds * 1000) {
    framesPerSecond = frameCount / seconds;
    Serial.print("fps: ");
    Serial.println(framesPerSecond);
    Serial.print("Free Ram: ");
    Serial.println(freeRam());
    frameCount = 0;
    lastMillis = now;
  }
}

// Free ram calculation for statistics only
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#endif
