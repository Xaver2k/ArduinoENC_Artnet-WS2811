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

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif

#define NEO_KHZ800 0x0000 // 800 KHz datastream
#ifndef __AVR_ATtiny85__
#define NEO_KHZ400 0x0100 // 400 KHz datastream
#endif


// Arduino Nano with Deek-Robot and ENC28J60 Shield

#include <SPI.h> // Nesseccary for Ethernet Library

// Ethernet Librarys
#include <UIPEthernet.h>
#include <UIPUdp.h>

// Artnet Specific
#define ART_NET_PORT 6454       // UDP specific
#define ART_DMX 0x5000          // Opcode Artnet DMX Paket
#define MAX_BUFFER_ARTNET 530   // Artnet Buffer Size
//#define ART_NET_ID "Art-Net\0"  // Artnet Paket ID
#define ART_DMX_START 18        // Artnet Header and LED Offset

// Led Data
#define NUMLEDS 171 // Number of LEDs per Universe/Output
#define NUMBYTES 513 // Number of Leds multiplied by Pixelsize RGB=3
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
uint16_t packetSize;
uint8_t artnetPackage[ART_DMX_START];

boolean
#ifdef NEO_KHZ400  // If 400 KHz NeoPixel support enabled...
    is800KHz = true;      // ...true if 800 KHz pixels
#endif
uint8_t
   pixels[NUMBYTES],
   outputPin;
   
//uint32_t endTime;   

    

// Arduino Selection change IPs, MACs and Universe to you needs
 
// 1th Arduino Nano
#if ARDUINO_NR == 1
  #define STARTUNIVERSE 0
  #define SECONDUNIVERSE STARTUNIVERSE+1
  #define IPADDRESS (192, 168, 0, 100)
  #define MACADDRESS (0x04, 0xE9, 0xE5, 0x00, 0x00, 0xEC)
  IPAddress ip(192, 168, 0, 100);
  byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x00, 0xEC};

// 2nd Arduino Nano
#elif ARDUINO_NR == 2
  #define STARTUNIVERSE 2
  #define SECONDUNIVERSE STARTUNIVERSE+1
  #define IPADDRESS (192, 168, 0, 102)
  #define MACADDRESS (0x04, 0xE9, 0xE5, 0x00, 0x02, 0xEC)
  //IPAddress ip(192, 168, 0, 102);
  //byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x02, 0xEC};

// 3th Arduino Nano
#elif ARDUINO_NR == 3
  #define STARTUNIVERSE 4
  #define SECONDUNIVERSE STARTUNIVERSE+1
  #define IPADDRESS (192, 168, 0, 104)
  #define MACADDRESS (0x04, 0xE9, 0xE5, 0x00, 0x04, 0xEC)
  //IPAddress ip(192, 168, 0, 104);
  //byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x04, 0xEC};

// 4th Arduino Nano
#elif ARDUINO_NR == 4
  #define STARTUNIVERSE 6
  #define SECONDUNIVERSE STARTUNIVERSE+1
  #define IPADDRESS (192, 168, 0, 106)
  #define MACADDRESS (0x04, 0xE9, 0xE5, 0x00, 0x06, 0xEC)
  //IPAddress ip(192, 168, 0, 106);
  //byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x06, 0xEC};
 
// 5th Arduino Nano
#elif ARDUINO_NR == 5
  #define STARTUNIVERSE 8
  #define SECONDUNIVERSE STARTUNIVERSE+1
  #define IPADDRESS (192, 168, 0, 108)
  #define MACADDRESS (0x04, 0xE9, 0xE5, 0x00, 0x08, 0xEC)
  //IPAddress ip(192, 168, 0, 108);
  //byte mac[] = {0x04, 0xE9, 0xE5, 0x00, 0x08, 0xEC};
#endif


void setup()
{
#ifdef FPS_COUNTER
  Serial.begin(9600);   // For tests only
#endif
  
  //Ethernet.begin(MACADDRESS, IPADDRESS);
  Ethernet.begin(mac,ip);
  udp.begin(ART_NET_PORT);

  // Initialize LED Output Pin
  pinMode(DATAPIN1, OUTPUT);
  digitalWrite(DATAPIN1, LOW);
  pinMode(DATAPIN2, OUTPUT);
  digitalWrite(DATAPIN2, LOW);
  pinMode(CONNECTPIN1, OUTPUT);   // Just for safety, remember the resistor and pcb stuff
  digitalWrite(CONNECTPIN1, LOW); // Just for safety, remember the resistor and pcb stuff
  pinMode(CONNECTPIN2, OUTPUT);   // Just for safety, remember the resistor and pcb stuff
  digitalWrite(CONNECTPIN2, LOW); // Just for safety, remember the resistor and pcb stuff

//  initAdaLeds(NUMLEDS);
  
//  initTest(); // Light test on Startup
}

void loop()
{
  packetSize = udp.parsePacket();
  //Serial.println(packetSize);
  
  if (packetSize <= MAX_BUFFER_ARTNET && packetSize > 0)
  {
    Serial.println("test2");
    // read artnet header from packet
    udp.read(artnetPackage, ART_DMX_START);
    Serial.println("test1");

    // Check that packetID is "Art-Net" else ignore
    if ( artnetPackage[0] == 'A' && artnetPackage[1] == 'r' && artnetPackage[2] == 't' && artnetPackage[3] == '-' 
          && artnetPackage[4] == 'N' && artnetPackage[5] == 'e' && artnetPackage[6] == 't' 
          && (artnetPackage[8] | artnetPackage[9]) << 8 == ART_DMX) 
    {
      udp.read(pixels , artnetPackage[17] | artnetPackage[16] << 8);
      switch (artnetPackage[14] | artnetPackage[15] << 8) { // Universe
        case (STARTUNIVERSE):
          setPin(DATAPIN1);
          break;
        case (SECONDUNIVERSE):
          setPin(DATAPIN2);
          break;
        default: 
          break;
        };
      Serial.println("test0000");
      for(int i = 0; i < sizeof(pixels); i++)
      {
        Serial.println(pixels[i]);
      }
      ledsShow();
    };
  };

  // FPS-Counter for statistics only
  #ifdef FPS_COUNTER
//    fps(5);
  #endif
}

//inline bool canShow(void) { return (micros() - endTime) >= 50L; };

#ifdef __AVR__
  volatile uint8_t
    *port;         // Output PORT register
  uint8_t
    pinMask;       // Output PORT bitmask
#endif
    
void initAdaLeds(uint16_t n)
{
//  endTime = 0;
  // updateLength n
  //if(pixels) free(pixels); // Free existing data (if any)

  // Allocate new data -- note: ALL PIXELS ARE CLEARED
  //numBytes = n * 3; // RGB Pixels need 3 Bytes
  //if((pixels = (uint8_t *)malloc(NUMBYTES))) {
  //  memset(pixels, 0, NUMBYTES);
  //}
}

void setPin(uint8_t p) {
  outputPin = p;
#ifdef __AVR__
  port    = portOutputRegister(digitalPinToPort(p));
  pinMask = digitalPinToBitMask(p);
#endif
}

void ledsShow()
{
//  if(!pixels) return;

//  while(!canShow());
 
  noInterrupts(); // Need 100% focus on instruction timing

  #ifdef __AVR__
// AVR MCUs -- ATmega & ATtiny (no XMEGA) ---------------------------------

  volatile uint16_t
    i   = NUMBYTES; // Loop counter
  volatile uint8_t
   *ptr = pixels,   // Pointer to next byte
    b   = *ptr++,   // Current byte value
    hi,             // PORT w/output bit set high
    lo;             // PORT w/output bit set low

  // Hand-tuned assembly code issues data to the LED drivers at a specific
  // rate.  There's separate code for different CPU speeds (8, 12, 16 MHz)
  // for both the WS2811 (400 KHz) and WS2812 (800 KHz) drivers.  The
  // datastream timing for the LED drivers allows a little wiggle room each
  // way (listed in the datasheets), so the conditions for compiling each
  // case are set up for a range of frequencies rather than just the exact
  // 8, 12 or 16 MHz values, permitting use with some close-but-not-spot-on
  // devices (e.g. 16.5 MHz DigiSpark).  The ranges were arrived at based
  // on the datasheet figures and have not been extensively tested outside
  // the canonical 8/12/16 MHz speeds; there's no guarantee these will work
  // close to the extremes (or possibly they could be pushed further).
  // Keep in mind only one CPU speed case actually gets compiled; the
  // resulting program isn't as massive as it might look from source here.

// 8 MHz(ish) AVR ---------------------------------------------------------
#if (F_CPU >= 7400000UL) && (F_CPU <= 9500000UL)

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif

    volatile uint8_t n1, n2 = 0;  // First, next bits out

    // Squeezing an 800 KHz stream out of an 8 MHz chip requires code
    // specific to each PORT register.

    // 10 instruction clocks per bit: HHxxxxxLLL
    // OUT instructions:              ^ ^    ^   (T=0,2,7)

    // PORTD OUTPUT ----------------------------------------------------

#if defined(PORTD)
 #if defined(PORTB) || defined(PORTC) || defined(PORTF)
    if(port == &PORTD) {
 #endif // defined(PORTB/C/F)

      hi = PORTD |  pinMask;
      lo = PORTD & ~pinMask;
      n1 = lo;
      if(b & 0x80) n1 = hi;

      // Dirty trick: RJMPs proceeding to the next instruction are used
      // to delay two clock cycles in one instruction word (rather than
      // using two NOPs).  This was necessary in order to squeeze the
      // loop down to exactly 64 words -- the maximum possible for a
      // relative branch.

      asm volatile(
       "headD:"                   "\n\t" // Clk  Pseudocode
        // Bit 7:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
        "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 6"        "\n\t" // 1-2  if(b & 0x40)
         "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp .+0"                "\n\t" // 2    nop nop
        // Bit 6:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
        "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 5"        "\n\t" // 1-2  if(b & 0x20)
         "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp .+0"                "\n\t" // 2    nop nop
        // Bit 5:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
        "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 4"        "\n\t" // 1-2  if(b & 0x10)
         "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp .+0"                "\n\t" // 2    nop nop
        // Bit 4:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
        "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 3"        "\n\t" // 1-2  if(b & 0x08)
         "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp .+0"                "\n\t" // 2    nop nop
        // Bit 3:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
        "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 2"        "\n\t" // 1-2  if(b & 0x04)
         "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp .+0"                "\n\t" // 2    nop nop
        // Bit 2:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
        "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 1"        "\n\t" // 1-2  if(b & 0x02)
         "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "rjmp .+0"                "\n\t" // 2    nop nop
        // Bit 1:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n2]   , %[lo]"    "\n\t" // 1    n2   = lo
        "out  %[port] , %[n1]"    "\n\t" // 1    PORT = n1
        "rjmp .+0"                "\n\t" // 2    nop nop
        "sbrc %[byte] , 0"        "\n\t" // 1-2  if(b & 0x01)
         "mov %[n2]   , %[hi]"    "\n\t" // 0-1   n2 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "sbiw %[count], 1"        "\n\t" // 2    i-- (don't act on Z flag yet)
        // Bit 0:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi
        "mov  %[n1]   , %[lo]"    "\n\t" // 1    n1   = lo
        "out  %[port] , %[n2]"    "\n\t" // 1    PORT = n2
        "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++
        "sbrc %[byte] , 7"        "\n\t" // 1-2  if(b & 0x80)
         "mov %[n1]   , %[hi]"    "\n\t" // 0-1   n1 = hi
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo
        "brne headD"              "\n"   // 2    while(i) (Z flag set above)
      : [byte]  "+r" (b),
        [n1]    "+r" (n1),
        [n2]    "+r" (n2),
        [count] "+w" (i)
      : [port]   "I" (_SFR_IO_ADDR(PORTD)),
        [ptr]    "e" (ptr),
        [hi]     "r" (hi),
        [lo]     "r" (lo));

 #if defined(PORTB) || defined(PORTC) || defined(PORTF)
    } else // other PORT(s)
 #endif // defined(PORTB/C/F)
#endif // defined(PORTD)

    // PORTB OUTPUT ----------------------------------------------------

#if defined(PORTB)
 #if defined(PORTD) || defined(PORTC) || defined(PORTF)
    if(port == &PORTB) {
 #endif // defined(PORTD/C/F)

      // Same as above, just switched to PORTB and stripped of comments.
      hi = PORTB |  pinMask;
      lo = PORTB & ~pinMask;
      n1 = lo;
      if(b & 0x80) n1 = hi;

      asm volatile(
       "headB:"                   "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 6"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 5"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 4"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 3"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 2"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 1"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 0"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "sbiw %[count], 1"        "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "ld   %[byte] , %a[ptr]+" "\n\t"
        "sbrc %[byte] , 7"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "brne headB"              "\n"
      : [byte] "+r" (b), [n1] "+r" (n1), [n2] "+r" (n2), [count] "+w" (i)
      : [port] "I" (_SFR_IO_ADDR(PORTB)), [ptr] "e" (ptr), [hi] "r" (hi),
        [lo] "r" (lo));

 #if defined(PORTD) || defined(PORTC) || defined(PORTF)
    }
 #endif
 #if defined(PORTC) || defined(PORTF)
    else
 #endif // defined(PORTC/F)
#endif // defined(PORTB)

    // PORTC OUTPUT ----------------------------------------------------

#if defined(PORTC)
 #if defined(PORTD) || defined(PORTB) || defined(PORTF)
    if(port == &PORTC) {
 #endif // defined(PORTD/B/F)

      // Same as above, just switched to PORTC and stripped of comments.
      hi = PORTC |  pinMask;
      lo = PORTC & ~pinMask;
      n1 = lo;
      if(b & 0x80) n1 = hi;

      asm volatile(
       "headC:"                   "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 6"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 5"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 4"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 3"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 2"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 1"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 0"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "sbiw %[count], 1"        "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "ld   %[byte] , %a[ptr]+" "\n\t"
        "sbrc %[byte] , 7"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "brne headC"              "\n"
      : [byte] "+r" (b), [n1] "+r" (n1), [n2] "+r" (n2), [count] "+w" (i)
      : [port] "I" (_SFR_IO_ADDR(PORTC)), [ptr] "e" (ptr), [hi] "r" (hi),
        [lo] "r" (lo));

 #if defined(PORTD) || defined(PORTB) || defined(PORTF)
    }
 #endif // defined(PORTD/B/F)
 #if defined(PORTF)
    else
 #endif
#endif // defined(PORTC)

    // PORTF OUTPUT ----------------------------------------------------

#if defined(PORTF)
 #if defined(PORTD) || defined(PORTB) || defined(PORTC)
    if(port == &PORTF) {
 #endif // defined(PORTD/B/C)

      hi = PORTF |  pinMask;
      lo = PORTF & ~pinMask;
      n1 = lo;
      if(b & 0x80) n1 = hi;

      asm volatile(
       "headF:"                   "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 6"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 5"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 4"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 3"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 2"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 1"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n2]   , %[lo]"    "\n\t"
        "out  %[port] , %[n1]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "sbrc %[byte] , 0"        "\n\t"
         "mov %[n2]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "sbiw %[count], 1"        "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "mov  %[n1]   , %[lo]"    "\n\t"
        "out  %[port] , %[n2]"    "\n\t"
        "ld   %[byte] , %a[ptr]+" "\n\t"
        "sbrc %[byte] , 7"        "\n\t"
         "mov %[n1]   , %[hi]"    "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "brne headF"              "\n"
      : [byte] "+r" (b), [n1] "+r" (n1), [n2] "+r" (n2), [count] "+w" (i)
      : [port] "I" (_SFR_IO_ADDR(PORTF)), [ptr] "e" (ptr), [hi] "r" (hi),
        [lo] "r" (lo));

 #if defined(PORTD) || defined(PORTB) || defined(PORTC)
    }
 #endif // defined(PORTD/B/C)
#endif // defined(PORTF)

#ifdef NEO_KHZ400
  } else { // end 800 KHz, do 400 KHz

    // Timing is more relaxed; unrolling the inner loop for each bit is
    // not necessary.  Still using the peculiar RJMPs as 2X NOPs, not out
    // of need but just to trim the code size down a little.
    // This 400-KHz-datastream-on-8-MHz-CPU code is not quite identical
    // to the 800-on-16 code later -- the hi/lo timing between WS2811 and
    // WS2812 is not simply a 2:1 scale!

    // 20 inst. clocks per bit: HHHHxxxxxxLLLLLLLLLL
    // ST instructions:         ^   ^     ^          (T=0,4,10)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head20:"                  "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port], %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
      "sbrc %[byte] , 7"        "\n\t" // 1-2  if(b & 128)
       "mov  %[next], %[hi]"    "\n\t" // 0-1   next = hi    (T =  4)
      "st   %a[port], %[next]"  "\n\t" // 2    PORT = next   (T =  6)
      "mov  %[next] , %[lo]"    "\n\t" // 1    next = lo     (T =  7)
      "dec  %[bit]"             "\n\t" // 1    bit--         (T =  8)
      "breq nextbyte20"         "\n\t" // 1-2  if(bit == 0)
      "rol  %[byte]"            "\n\t" // 1    b <<= 1       (T = 10)
      "st   %a[port], %[lo]"    "\n\t" // 2    PORT = lo     (T = 12)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 14)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 16)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 18)
      "rjmp head20"             "\n\t" // 2    -> head20 (next bit out)
     "nextbyte20:"              "\n\t" //                    (T = 10)
      "st   %a[port], %[lo]"    "\n\t" // 2    PORT = lo     (T = 12)
      "nop"                     "\n\t" // 1    nop           (T = 13)
      "ldi  %[bit]  , 8"        "\n\t" // 1    bit = 8       (T = 14)
      "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 16)
      "sbiw %[count], 1"        "\n\t" // 2    i--           (T = 18)
      "brne head20"             "\n"   // 2    if(i != 0) -> (next byte)
      : [port]  "+e" (port),
        [byte]  "+r" (b),
        [bit]   "+r" (bit),
        [next]  "+r" (next),
        [count] "+w" (i)
      : [hi]    "r" (hi),
        [lo]    "r" (lo),
        [ptr]   "e" (ptr));
  }
#endif // NEO_KHZ400

// 12 MHz(ish) AVR --------------------------------------------------------
#elif (F_CPU >= 11100000UL) && (F_CPU <= 14300000UL)

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif

    // In the 12 MHz case, an optimized 800 KHz datastream (no dead time
    // between bytes) requires a PORT-specific loop similar to the 8 MHz
    // code (but a little more relaxed in this case).

    // 15 instruction clocks per bit: HHHHxxxxxxLLLLL
    // OUT instructions:              ^   ^     ^     (T=0,4,10)

    volatile uint8_t next;

    // PORTD OUTPUT ----------------------------------------------------

#if defined(PORTD)
 #if defined(PORTB) || defined(PORTC) || defined(PORTF)
    if(port == &PORTD) {
 #endif // defined(PORTB/C/F)

      hi   = PORTD |  pinMask;
      lo   = PORTD & ~pinMask;
      next = lo;
      if(b & 0x80) next = hi;

      // Don't "optimize" the OUT calls into the bitTime subroutine;
      // we're exploiting the RCALL and RET as 3- and 4-cycle NOPs!
      asm volatile(
       "headD:"                   "\n\t" //        (T =  0)
        "out   %[port], %[hi]"    "\n\t" //        (T =  1)
        "rcall bitTimeD"          "\n\t" // Bit 7  (T = 15)
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" // Bit 6
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" // Bit 5
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" // Bit 4
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" // Bit 3
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" // Bit 2
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeD"          "\n\t" // Bit 1
        // Bit 0:
        "out  %[port] , %[hi]"    "\n\t" // 1    PORT = hi    (T =  1)
        "rjmp .+0"                "\n\t" // 2    nop nop      (T =  3)
        "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++   (T =  5)
        "out  %[port] , %[next]"  "\n\t" // 1    PORT = next  (T =  6)
        "mov  %[next] , %[lo]"    "\n\t" // 1    next = lo    (T =  7)
        "sbrc %[byte] , 7"        "\n\t" // 1-2  if(b & 0x80) (T =  8)
         "mov %[next] , %[hi]"    "\n\t" // 0-1    next = hi  (T =  9)
        "nop"                     "\n\t" // 1                 (T = 10)
        "out  %[port] , %[lo]"    "\n\t" // 1    PORT = lo    (T = 11)
        "sbiw %[count], 1"        "\n\t" // 2    i--          (T = 13)
        "brne headD"              "\n\t" // 2    if(i != 0) -> (next byte)
         "rjmp doneD"             "\n\t"
        "bitTimeD:"               "\n\t" //      nop nop nop     (T =  4)
         "out  %[port], %[next]"  "\n\t" // 1    PORT = next     (T =  5)
         "mov  %[next], %[lo]"    "\n\t" // 1    next = lo       (T =  6)
         "rol  %[byte]"           "\n\t" // 1    b <<= 1         (T =  7)
         "sbrc %[byte], 7"        "\n\t" // 1-2  if(b & 0x80)    (T =  8)
          "mov %[next], %[hi]"    "\n\t" // 0-1   next = hi      (T =  9)
         "nop"                    "\n\t" // 1                    (T = 10)
         "out  %[port], %[lo]"    "\n\t" // 1    PORT = lo       (T = 11)
         "ret"                    "\n\t" // 4    nop nop nop nop (T = 15)
         "doneD:"                 "\n"
        : [byte]  "+r" (b),
          [next]  "+r" (next),
          [count] "+w" (i)
        : [port]   "I" (_SFR_IO_ADDR(PORTD)),
          [ptr]    "e" (ptr),
          [hi]     "r" (hi),
          [lo]     "r" (lo));

 #if defined(PORTB) || defined(PORTC) || defined(PORTF)
    } else // other PORT(s)
 #endif // defined(PORTB/C/F)
#endif // defined(PORTD)

    // PORTB OUTPUT ----------------------------------------------------

#if defined(PORTB)
 #if defined(PORTD) || defined(PORTC) || defined(PORTF)
    if(port == &PORTB) {
 #endif // defined(PORTD/C/F)

      hi   = PORTB |  pinMask;
      lo   = PORTB & ~pinMask;
      next = lo;
      if(b & 0x80) next = hi;

      // Same as above, just set for PORTB & stripped of comments
      asm volatile(
       "headB:"                   "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeB"          "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "ld   %[byte] , %a[ptr]+" "\n\t"
        "out  %[port] , %[next]"  "\n\t"
        "mov  %[next] , %[lo]"    "\n\t"
        "sbrc %[byte] , 7"        "\n\t"
         "mov %[next] , %[hi]"    "\n\t"
        "nop"                     "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "sbiw %[count], 1"        "\n\t"
        "brne headB"              "\n\t"
         "rjmp doneB"             "\n\t"
        "bitTimeB:"               "\n\t"
         "out  %[port], %[next]"  "\n\t"
         "mov  %[next], %[lo]"    "\n\t"
         "rol  %[byte]"           "\n\t"
         "sbrc %[byte], 7"        "\n\t"
          "mov %[next], %[hi]"    "\n\t"
         "nop"                    "\n\t"
         "out  %[port], %[lo]"    "\n\t"
         "ret"                    "\n\t"
         "doneB:"                 "\n"
        : [byte] "+r" (b), [next] "+r" (next), [count] "+w" (i)
        : [port] "I" (_SFR_IO_ADDR(PORTB)), [ptr] "e" (ptr), [hi] "r" (hi),
          [lo] "r" (lo));

 #if defined(PORTD) || defined(PORTC) || defined(PORTF)
    }
 #endif
 #if defined(PORTC) || defined(PORTF)
    else
 #endif // defined(PORTC/F)
#endif // defined(PORTB)

    // PORTC OUTPUT ----------------------------------------------------

#if defined(PORTC)
 #if defined(PORTD) || defined(PORTB) || defined(PORTF)
    if(port == &PORTC) {
 #endif // defined(PORTD/B/F)

      hi   = PORTC |  pinMask;
      lo   = PORTC & ~pinMask;
      next = lo;
      if(b & 0x80) next = hi;

      // Same as above, just set for PORTC & stripped of comments
      asm volatile(
       "headC:"                   "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "ld   %[byte] , %a[ptr]+" "\n\t"
        "out  %[port] , %[next]"  "\n\t"
        "mov  %[next] , %[lo]"    "\n\t"
        "sbrc %[byte] , 7"        "\n\t"
         "mov %[next] , %[hi]"    "\n\t"
        "nop"                     "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "sbiw %[count], 1"        "\n\t"
        "brne headC"              "\n\t"
         "rjmp doneC"             "\n\t"
        "bitTimeC:"               "\n\t"
         "out  %[port], %[next]"  "\n\t"
         "mov  %[next], %[lo]"    "\n\t"
         "rol  %[byte]"           "\n\t"
         "sbrc %[byte], 7"        "\n\t"
          "mov %[next], %[hi]"    "\n\t"
         "nop"                    "\n\t"
         "out  %[port], %[lo]"    "\n\t"
         "ret"                    "\n\t"
         "doneC:"                 "\n"
        : [byte] "+r" (b), [next] "+r" (next), [count] "+w" (i)
        : [port] "I" (_SFR_IO_ADDR(PORTC)), [ptr] "e" (ptr), [hi] "r" (hi),
          [lo] "r" (lo));

 #if defined(PORTD) || defined(PORTB) || defined(PORTF)
    }
 #endif // defined(PORTD/B/F)
 #if defined(PORTF)
    else
 #endif
#endif // defined(PORTC)

    // PORTF OUTPUT ----------------------------------------------------

#if defined(PORTF)
 #if defined(PORTD) || defined(PORTB) || defined(PORTC)
    if(port == &PORTF) {
 #endif // defined(PORTD/B/C)

      hi   = PORTF |  pinMask;
      lo   = PORTF & ~pinMask;
      next = lo;
      if(b & 0x80) next = hi;

      // Same as above, just set for PORTF & stripped of comments
      asm volatile(
       "headF:"                   "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out   %[port], %[hi]"    "\n\t"
        "rcall bitTimeC"          "\n\t"
        "out  %[port] , %[hi]"    "\n\t"
        "rjmp .+0"                "\n\t"
        "ld   %[byte] , %a[ptr]+" "\n\t"
        "out  %[port] , %[next]"  "\n\t"
        "mov  %[next] , %[lo]"    "\n\t"
        "sbrc %[byte] , 7"        "\n\t"
         "mov %[next] , %[hi]"    "\n\t"
        "nop"                     "\n\t"
        "out  %[port] , %[lo]"    "\n\t"
        "sbiw %[count], 1"        "\n\t"
        "brne headF"              "\n\t"
         "rjmp doneC"             "\n\t"
        "bitTimeC:"               "\n\t"
         "out  %[port], %[next]"  "\n\t"
         "mov  %[next], %[lo]"    "\n\t"
         "rol  %[byte]"           "\n\t"
         "sbrc %[byte], 7"        "\n\t"
          "mov %[next], %[hi]"    "\n\t"
         "nop"                    "\n\t"
         "out  %[port], %[lo]"    "\n\t"
         "ret"                    "\n\t"
         "doneC:"                 "\n"
        : [byte] "+r" (b), [next] "+r" (next), [count] "+w" (i)
        : [port] "I" (_SFR_IO_ADDR(PORTF)), [ptr] "e" (ptr), [hi] "r" (hi),
          [lo] "r" (lo));

 #if defined(PORTD) || defined(PORTB) || defined(PORTC)
    }
 #endif // defined(PORTD/B/C)
#endif // defined(PORTF)

#ifdef NEO_KHZ400
  } else { // 400 KHz

    // 30 instruction clocks per bit: HHHHHHxxxxxxxxxLLLLLLLLLLLLLLL
    // ST instructions:               ^     ^        ^    (T=0,6,15)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head30:"                  "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port], %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
      "sbrc %[byte] , 7"        "\n\t" // 1-2  if(b & 128)
       "mov  %[next], %[hi]"    "\n\t" // 0-1   next = hi    (T =  4)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T =  6)
      "st   %a[port], %[next]"  "\n\t" // 2    PORT = next   (T =  8)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 10)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 12)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 14)
      "nop"                     "\n\t" // 1    nop           (T = 15)
      "st   %a[port], %[lo]"    "\n\t" // 2    PORT = lo     (T = 17)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 19)
      "dec  %[bit]"             "\n\t" // 1    bit--         (T = 20)
      "breq nextbyte30"         "\n\t" // 1-2  if(bit == 0)
      "rol  %[byte]"            "\n\t" // 1    b <<= 1       (T = 22)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 24)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 26)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 28)
      "rjmp head30"             "\n\t" // 2    -> head30 (next bit out)
     "nextbyte30:"              "\n\t" //                    (T = 22)
      "nop"                     "\n\t" // 1    nop           (T = 23)
      "ldi  %[bit]  , 8"        "\n\t" // 1    bit = 8       (T = 24)
      "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 26)
      "sbiw %[count], 1"        "\n\t" // 2    i--           (T = 28)
      "brne head30"             "\n"   // 1-2  if(i != 0) -> (next byte)
      : [port]  "+e" (port),
        [byte]  "+r" (b),
        [bit]   "+r" (bit),
        [next]  "+r" (next),
        [count] "+w" (i)
      : [hi]     "r" (hi),
        [lo]     "r" (lo),
        [ptr]    "e" (ptr));
  }
#endif // NEO_KHZ400

// 16 MHz(ish) AVR --------------------------------------------------------
#elif (F_CPU >= 15400000UL) && (F_CPU <= 19000000L)

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif

    // WS2811 and WS2812 have different hi/lo duty cycles; this is
    // similar but NOT an exact copy of the prior 400-on-8 code.

    // 20 inst. clocks per bit: HHHHHxxxxxxxxLLLLLLL
    // ST instructions:         ^   ^        ^       (T=0,5,13)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head20:"                   "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port],  %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
      "sbrc %[byte],  7"         "\n\t" // 1-2  if(b & 128)
       "mov  %[next], %[hi]"     "\n\t" // 0-1   next = hi    (T =  4)
      "dec  %[bit]"              "\n\t" // 1    bit--         (T =  5)
      "st   %a[port],  %[next]"  "\n\t" // 2    PORT = next   (T =  7)
      "mov  %[next] ,  %[lo]"    "\n\t" // 1    next = lo     (T =  8)
      "breq nextbyte20"          "\n\t" // 1-2  if(bit == 0) (from dec above)
      "rol  %[byte]"             "\n\t" // 1    b <<= 1       (T = 10)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 12)
      "nop"                      "\n\t" // 1    nop           (T = 13)
      "st   %a[port],  %[lo]"    "\n\t" // 2    PORT = lo     (T = 15)
      "nop"                      "\n\t" // 1    nop           (T = 16)
      "rjmp .+0"                 "\n\t" // 2    nop nop       (T = 18)
      "rjmp head20"              "\n\t" // 2    -> head20 (next bit out)
     "nextbyte20:"               "\n\t" //                    (T = 10)
      "ldi  %[bit]  ,  8"        "\n\t" // 1    bit = 8       (T = 11)
      "ld   %[byte] ,  %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 13)
      "st   %a[port], %[lo]"     "\n\t" // 2    PORT = lo     (T = 15)
      "nop"                      "\n\t" // 1    nop           (T = 16)
      "sbiw %[count], 1"         "\n\t" // 2    i--           (T = 18)
       "brne head20"             "\n"   // 2    if(i != 0) -> (next byte)
      : [port]  "+e" (port),
        [byte]  "+r" (b),
        [bit]   "+r" (bit),
        [next]  "+r" (next),
        [count] "+w" (i)
      : [ptr]    "e" (ptr),
        [hi]     "r" (hi),
        [lo]     "r" (lo));

#ifdef NEO_KHZ400
  } else { // 400 KHz

    // The 400 KHz clock on 16 MHz MCU is the most 'relaxed' version.

    // 40 inst. clocks per bit: HHHHHHHHxxxxxxxxxxxxLLLLLLLLLLLLLLLLLLLL
    // ST instructions:         ^       ^           ^         (T=0,8,20)

    volatile uint8_t next, bit;

    hi   = *port |  pinMask;
    lo   = *port & ~pinMask;
    next = lo;
    bit  = 8;

    asm volatile(
     "head40:"                  "\n\t" // Clk  Pseudocode    (T =  0)
      "st   %a[port], %[hi]"    "\n\t" // 2    PORT = hi     (T =  2)
      "sbrc %[byte] , 7"        "\n\t" // 1-2  if(b & 128)
       "mov  %[next] , %[hi]"   "\n\t" // 0-1   next = hi    (T =  4)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T =  6)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T =  8)
      "st   %a[port], %[next]"  "\n\t" // 2    PORT = next   (T = 10)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 12)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 14)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 16)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 18)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 20)
      "st   %a[port], %[lo]"    "\n\t" // 2    PORT = lo     (T = 22)
      "nop"                     "\n\t" // 1    nop           (T = 23)
      "mov  %[next] , %[lo]"    "\n\t" // 1    next = lo     (T = 24)
      "dec  %[bit]"             "\n\t" // 1    bit--         (T = 25)
      "breq nextbyte40"         "\n\t" // 1-2  if(bit == 0)
      "rol  %[byte]"            "\n\t" // 1    b <<= 1       (T = 27)
      "nop"                     "\n\t" // 1    nop           (T = 28)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 30)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 32)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 34)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 36)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 38)
      "rjmp head40"             "\n\t" // 2    -> head40 (next bit out)
     "nextbyte40:"              "\n\t" //                    (T = 27)
      "ldi  %[bit]  , 8"        "\n\t" // 1    bit = 8       (T = 28)
      "ld   %[byte] , %a[ptr]+" "\n\t" // 2    b = *ptr++    (T = 30)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 32)
      "st   %a[port], %[lo]"    "\n\t" // 2    PORT = lo     (T = 34)
      "rjmp .+0"                "\n\t" // 2    nop nop       (T = 36)
      "sbiw %[count], 1"        "\n\t" // 2    i--           (T = 38)
      "brne head40"             "\n"   // 1-2  if(i != 0) -> (next byte)
      : [port]  "+e" (port),
        [byte]  "+r" (b),
        [bit]   "+r" (bit),
        [next]  "+r" (next),
        [count] "+w" (i)
      : [ptr]    "e" (ptr),
        [hi]     "r" (hi),
        [lo]     "r" (lo));
  }
#endif // NEO_KHZ400

#else
 #error "CPU SPEED NOT SUPPORTED"
#endif // end F_CPU ifdefs on __AVR__

// END AVR ----------------------------------------------------------------


#elif defined(__arm__)

// ARM MCUs -- Teensy 3.0, 3.1, LC, Arduino Due ---------------------------

#if defined(__MK20DX128__) || defined(__MK20DX256__) // Teensy 3.0 & 3.1
#define CYCLES_800_T0H  (F_CPU / 4000000)
#define CYCLES_800_T1H  (F_CPU / 1250000)
#define CYCLES_800      (F_CPU /  800000)
#define CYCLES_400_T0H  (F_CPU / 2000000)
#define CYCLES_400_T1H  (F_CPU /  833333)
#define CYCLES_400      (F_CPU /  400000)

  uint8_t          *p   = pixels,
                   *end = p + NUMBYTES, pix, mask;
  volatile uint8_t *set = portSetRegister(outputPin),
                   *clr = portClearRegister(outputPin);
  uint32_t          cyc;

  ARM_DEMCR    |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif
    cyc = ARM_DWT_CYCCNT + CYCLES_800;
    while(p < end) {
      pix = *p++;
      for(mask = 0x80; mask; mask >>= 1) {
        while(ARM_DWT_CYCCNT - cyc < CYCLES_800);
        cyc  = ARM_DWT_CYCCNT;
        *set = 1;
        if(pix & mask) {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_800_T1H);
        } else {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_800_T0H);
        }
        *clr = 1;
      }
    }
    while(ARM_DWT_CYCCNT - cyc < CYCLES_800);
#ifdef NEO_KHZ400
  } else { // 400 kHz bitstream
    cyc = ARM_DWT_CYCCNT + CYCLES_400;
    while(p < end) {
      pix = *p++;
      for(mask = 0x80; mask; mask >>= 1) {
        while(ARM_DWT_CYCCNT - cyc < CYCLES_400);
        cyc  = ARM_DWT_CYCCNT;
        *set = 1;
        if(pix & mask) {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_400_T1H);
        } else {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_400_T0H);
        }
        *clr = 1;
      }
    }
    while(ARM_DWT_CYCCNT - cyc < CYCLES_400);
  }
#endif // NEO_KHZ400

#elif defined(__MKL26Z64__) // Teensy-LC

#if F_CPU == 48000000
  uint8_t          *p   = pixels,
       pix, count, dly,
                   bitmask = digitalPinToBitMask(outputPin);
  volatile uint8_t *reg = portSetRegister(outputPin);
  uint32_t         num = NUMBYTES;
  asm volatile(
  "L%=_begin:"        "\n\t"
  "ldrb %[pix], [%[p], #0]"   "\n\t"
  "lsl  %[pix], #24"      "\n\t"
  "movs %[count], #7"     "\n\t"
  "L%=_loop:"       "\n\t"
  "lsl  %[pix], #1"     "\n\t"
  "bcs  L%=_loop_one"     "\n\t"
  "L%=_loop_zero:"
  "strb %[bitmask], [%[reg], #0]" "\n\t"
  "movs %[dly], #4"     "\n\t"
  "L%=_loop_delay_T0H:"     "\n\t"
  "sub  %[dly], #1"     "\n\t"
  "bne  L%=_loop_delay_T0H"   "\n\t"
  "strb %[bitmask], [%[reg], #4]" "\n\t"
  "movs %[dly], #13"      "\n\t"
  "L%=_loop_delay_T0L:"     "\n\t"
  "sub  %[dly], #1"     "\n\t"
  "bne  L%=_loop_delay_T0L"   "\n\t"
  "b  L%=_next"     "\n\t"
  "L%=_loop_one:"
  "strb %[bitmask], [%[reg], #0]" "\n\t"
  "movs %[dly], #13"      "\n\t"
  "L%=_loop_delay_T1H:"     "\n\t"
  "sub  %[dly], #1"     "\n\t"
  "bne  L%=_loop_delay_T1H"   "\n\t"
  "strb %[bitmask], [%[reg], #4]" "\n\t"
  "movs %[dly], #4"     "\n\t"
  "L%=_loop_delay_T1L:"     "\n\t"
  "sub  %[dly], #1"     "\n\t"
  "bne  L%=_loop_delay_T1L"   "\n\t"
  "nop"         "\n\t"
  "L%=_next:"       "\n\t"
  "sub  %[count], #1"     "\n\t"
  "bne  L%=_loop"     "\n\t"
  "lsl  %[pix], #1"     "\n\t"
  "bcs  L%=_last_one"     "\n\t"
  "L%=_last_zero:"
  "strb %[bitmask], [%[reg], #0]" "\n\t"
  "movs %[dly], #4"     "\n\t"
  "L%=_last_delay_T0H:"     "\n\t"
  "sub  %[dly], #1"     "\n\t"
  "bne  L%=_last_delay_T0H"   "\n\t"
  "strb %[bitmask], [%[reg], #4]" "\n\t"
  "movs %[dly], #10"      "\n\t"
  "L%=_last_delay_T0L:"     "\n\t"
  "sub  %[dly], #1"     "\n\t"
  "bne  L%=_last_delay_T0L"   "\n\t"
  "b  L%=_repeat"     "\n\t"
  "L%=_last_one:"
  "strb %[bitmask], [%[reg], #0]" "\n\t"
  "movs %[dly], #13"      "\n\t"
  "L%=_last_delay_T1H:"     "\n\t"
  "sub  %[dly], #1"     "\n\t"
  "bne  L%=_last_delay_T1H"   "\n\t"
  "strb %[bitmask], [%[reg], #4]" "\n\t"
  "movs %[dly], #1"     "\n\t"
  "L%=_last_delay_T1L:"     "\n\t"
  "sub  %[dly], #1"     "\n\t"
  "bne  L%=_last_delay_T1L"   "\n\t"
  "nop"         "\n\t"
  "L%=_repeat:"       "\n\t"
  "add  %[p], #1"     "\n\t"
  "sub  %[num], #1"     "\n\t"
  "bne  L%=_begin"      "\n\t"
  "L%=_done:"       "\n\t"
  : [p] "+r" (p),
    [pix] "=&r" (pix),
    [count] "=&r" (count),
    [dly] "=&r" (dly),
    [num] "+r" (num)
  : [bitmask] "r" (bitmask),
    [reg] "r" (reg)
  );
#else
#error "Sorry, only 48 MHz is supported, please set Tools > CPU Speed to 48 MHz"
#endif // F_CPU == 48000000

#elif defined(__SAMD21G18A__) // Arduino Zero

  // Tried this with a timer/counter, couldn't quite get adequate
  // resolution.  So yay, you get a load of goofball NOPs...

  uint8_t  *ptr, *end, p, bitMask, portNum;
  uint32_t  pinMask;

  portNum =  g_APinDescription[outputPin].ulPort;
  pinMask =  1ul << g_APinDescription[outputPin].ulPin;
  ptr     =  pixels;
  end     =  ptr + NUMBYTES;
  p       = *ptr++;
  bitMask =  0x80;

  volatile uint32_t *set = &(PORT->Group[portNum].OUTSET.reg),
                    *clr = &(PORT->Group[portNum].OUTCLR.reg);

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif
    for(;;) {
      *set = pinMask;
      asm("nop; nop; nop; nop; nop; nop; nop; nop;");
      if(p & bitMask) {
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop;");
        *clr = pinMask;
      } else {
        *clr = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop;");
      }
      if(bitMask >>= 1) {
        asm("nop; nop; nop; nop; nop; nop; nop; nop; nop;");
      } else {
        if(ptr >= end) break;
        p       = *ptr++;
        bitMask = 0x80;
      }
    }
#ifdef NEO_KHZ400
  } else { // 400 KHz bitstream
    for(;;) {
      *set = pinMask;
      asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
      if(p & bitMask) {
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop;");
        *clr = pinMask;
      } else {
        *clr = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop;");
      }
      asm("nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop;");
      if(bitMask >>= 1) {
        asm("nop; nop; nop; nop; nop; nop; nop;");
      } else {
        if(ptr >= end) break;
        p       = *ptr++;
        bitMask = 0x80;
      }
    }
  }
#endif

#elif defined (ARDUINO_STM32_FEATHER) // FEATHER WICED (120MHz)

  // Tried this with a timer/counter, couldn't quite get adequate
  // resolution.  So yay, you get a load of goofball NOPs...

  uint8_t  *ptr, *end, p, bitMask;
  uint32_t  pinMask;

  pinMask =  BIT(PIN_MAP[outputPin].gpio_bit);
  ptr     =  pixels;
  end     =  ptr + NUMBYTES;
  p       = *ptr++;
  bitMask =  0x80;

  volatile uint16_t *set = &(PIN_MAP[outputPin].gpio_device->regs->BSRRL);
  volatile uint16_t *clr = &(PIN_MAP[outputPin].gpio_device->regs->BSRRH);

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif
    for(;;) {
      if(p & bitMask) { // ONE
        // High 800ns
        *set = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop;");
        // Low 450ns
        *clr = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop;");
      } else { // ZERO
        // High 400ns
        *set = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop;");
        // Low 850ns
        *clr = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop;");
      }
      if(bitMask >>= 1) {
        // Move on to the next pixel
        asm("nop;");
      } else {
        if(ptr >= end) break;
        p       = *ptr++;
        bitMask = 0x80;
      }
    }
#ifdef NEO_KHZ400
  } else { // 400 KHz bitstream
    // ToDo!
  }
#endif

#else // Other ARM architecture -- Presumed Arduino Due

  #define SCALE      VARIANT_MCK / 2UL / 1000000UL
  #define INST       (2UL * F_CPU / VARIANT_MCK)
  #define TIME_800_0 ((int)(0.40 * SCALE + 0.5) - (5 * INST))
  #define TIME_800_1 ((int)(0.80 * SCALE + 0.5) - (5 * INST))
  #define PERIOD_800 ((int)(1.25 * SCALE + 0.5) - (5 * INST))
  #define TIME_400_0 ((int)(0.50 * SCALE + 0.5) - (5 * INST))
  #define TIME_400_1 ((int)(1.20 * SCALE + 0.5) - (5 * INST))
  #define PERIOD_400 ((int)(2.50 * SCALE + 0.5) - (5 * INST))

  int             pinMask, time0, time1, period, t;
  Pio            *port;
  volatile WoReg *portSet, *portClear, *timeValue, *timeReset;
  uint8_t        *p, *end, pix, mask;

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)TC3_IRQn);
  TC_Configure(TC1, 0,
    TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_Start(TC1, 0);

  pinMask   = g_APinDescription[outputPin].ulPin; // Don't 'optimize' these into
  port      = g_APinDescription[outputPin].pPort; // declarations above.  Want to
  portSet   = &(port->PIO_SODR);            // burn a few cycles after
  portClear = &(port->PIO_CODR);            // starting timer to minimize
  timeValue = &(TC1->TC_CHANNEL[0].TC_CV);  // the initial 'while'.
  timeReset = &(TC1->TC_CHANNEL[0].TC_CCR);
  p         =  pixels;
  end       =  p + NUMBYTES;
  pix       = *p++;
  mask      = 0x80;

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif
    time0  = TIME_800_0;
    time1  = TIME_800_1;
    period = PERIOD_800;
#ifdef NEO_KHZ400
  } else { // 400 KHz bitstream
    time0  = TIME_400_0;
    time1  = TIME_400_1;
    period = PERIOD_400;
  }
#endif

  for(t = time0;; t = time0) {
    if(pix & mask) t = time1;
    while(*timeValue < period);
    *portSet   = pinMask;
    *timeReset = TC_CCR_CLKEN | TC_CCR_SWTRG;
    while(*timeValue < t);
    *portClear = pinMask;
    if(!(mask >>= 1)) {   // This 'inside-out' loop logic utilizes
      if(p >= end) break; // idle time to minimize inter-byte delays.
      pix = *p++;
      mask = 0x80;
    }
  }
  while(*timeValue < period); // Wait for last bit
  TC_Stop(TC1, 0);

#endif // end Due

// END ARM ----------------------------------------------------------------


#elif defined(ESP8266)

// ESP8266 ----------------------------------------------------------------

  // ESP8266 show() is external to enforce ICACHE_RAM_ATTR execution
  espShow(outputPin, pixels, NUMBYTES, is800KHz);

#elif defined(__ARDUINO_ARC__)

// Arduino 101  -----------------------------------------------------------

#define NOPx7 { __builtin_arc_nop(); \
  __builtin_arc_nop(); __builtin_arc_nop(); \
  __builtin_arc_nop(); __builtin_arc_nop(); \
  __builtin_arc_nop(); __builtin_arc_nop(); }

  PinDescription *pindesc = &g_APinDescription[outputPin];
  register uint32_t loop = 8 * NUMBYTES; // one loop to handle all bytes and all bits
  register uint8_t *p = pixels;
  register uint32_t currByte = (uint32_t) (*p);
  register uint32_t currBit = 0x80 & currByte;
  register uint32_t bitCounter = 0;
  register uint32_t first = 1;

  // The loop is unusual. Very first iteration puts all the way LOW to the wire -
  // constant LOW does not affect NEOPIXEL, so there is no visible effect displayed.
  // During that very first iteration CPU caches instructions in the loop.
  // Because of the caching process, "CPU slows down". NEOPIXEL pulse is very time sensitive
  // that's why we let the CPU cache first and we start regular pulse from 2nd iteration
  if (pindesc->ulGPIOType == SS_GPIO) {
    register uint32_t reg = pindesc->ulGPIOBase + SS_GPIO_SWPORTA_DR;
    uint32_t reg_val = __builtin_arc_lr((volatile uint32_t)reg);
    register uint32_t reg_bit_high = reg_val | (1 << pindesc->ulGPIOId);
    register uint32_t reg_bit_low  = reg_val & ~(1 << pindesc->ulGPIOId);

    loop += 1; // include first, special iteration
    while(loop--) {
      if(!first) {
        currByte <<= 1;
        bitCounter++;
      }

      // 1 is >550ns high and >450ns low; 0 is 200..500ns high and >450ns low
      __builtin_arc_sr(first ? reg_bit_low : reg_bit_high, (volatile uint32_t)reg);
      if(currBit) { // ~400ns HIGH (740ns overall)
        NOPx7
        NOPx7
      }
      // ~340ns HIGH
      NOPx7
     __builtin_arc_nop();

      // 820ns LOW; per spec, max allowed low here is 5000ns */
      __builtin_arc_sr(reg_bit_low, (volatile uint32_t)reg);
      NOPx7
      NOPx7

      if(bitCounter >= 8) {
        bitCounter = 0;
        currByte = (uint32_t) (*++p);
      }

      currBit = 0x80 & currByte;
      first = 0;
    }
  } else if(pindesc->ulGPIOType == SOC_GPIO) {
    register uint32_t reg = pindesc->ulGPIOBase + SOC_GPIO_SWPORTA_DR;
    uint32_t reg_val = MMIO_REG_VAL(reg);
    register uint32_t reg_bit_high = reg_val | (1 << pindesc->ulGPIOId);
    register uint32_t reg_bit_low  = reg_val & ~(1 << pindesc->ulGPIOId);

    loop += 1; // include first, special iteration
    while(loop--) {
      if(!first) {
        currByte <<= 1;
        bitCounter++;
      }
      MMIO_REG_VAL(reg) = first ? reg_bit_low : reg_bit_high;
      if(currBit) { // ~430ns HIGH (740ns overall)
        NOPx7
        NOPx7
        __builtin_arc_nop();
      }
      // ~310ns HIGH
      NOPx7

      // 850ns LOW; per spec, max allowed low here is 5000ns */
      MMIO_REG_VAL(reg) = reg_bit_low;
      NOPx7
      NOPx7

      if(bitCounter >= 8) {
        bitCounter = 0;
        currByte = (uint32_t) (*++p);
      }

      currBit = 0x80 & currByte;
      first = 0;
    }
  }

#endif


// END ARCHITECTURE SELECT ------------------------------------------------


  interrupts();
//  endTime = micros(); // Save EOD time for latch on next call
}







/*
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
*/

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
