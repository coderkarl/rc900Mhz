// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX

#include <SPI.h>
#include <RH_RF95.h>

//for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

//////////////////////PPM CONFIGURATION///////////////////////////////
#define NUM_CHANNELS 8
#define FRAME_LENGTH 20000
#define DEFAULT_PULSE_LENGTH 1500
#define PULSE_LOW_LENGTH 400
// NOTE: the output pin must be Arduino pin 5 on a Feather32U4 (or Leonardo)
#define sigPin 5  //set PPM signal output pin on the arduino

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
volatile uint16_t ppm[NUM_CHANNELS];
//////////////////////////////////////////////////////////////////

#define PACKET_TIMEOUT 300
long timeNewPacket;

void setup()
{
  timeNewPacket = millis();
  
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  /*while (!Serial) {
    delay(1);
  }*/
  delay(100);

  Serial.println("Feather LoRa RX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  //////////////////////PPM SETUP///////////////////////////////
  //initiallize default ppm values
  // 0 - Right Horz, turn
  // 1 - Right Vert, fwd/rev
  // 2 - Left Vert, throttle, auto mode
  // 3 - Left Horz, blade deadman switch

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, HIGH);

  cli();
  for(int i = 0; i < NUM_CHANNELS; ++i){
    ppm[i]= DEFAULT_PULSE_LENGTH;
  }

  // Stop the timer/counter
  TCCR3A = TCCR3B = 0;
  // Set the low pulse length.  This will never change.
  OCR3A = PULSE_LOW_LENGTH;
  // Time until first falling edge, can be any value greater than PULSE_LOW_LENGTH.
  ICR3 = PULSE_LOW_LENGTH + 1;
  // Clear the count
  TCNT3 = 0;
  // Fast PWM; TOP = ICR3; set OC3A on compare match, clear on TOP; prescale = /8 (1MHz on Feather32u4)
  TCCR3A = _BV(COM3A1) | _BV(COM3A0) | _BV(WGM31);
  TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31);
  // Clear any pending timer overflow interrupt flag
  TIFR3 = _BV(TOV3);
  // Interrupt on overflow (at TOP, immediately after falling edge)
  TIMSK3 = _BV(TOIE3);
  sei();
  //////////////////////////////////////////////////////////////
}

void loop()
{
  if(timeSince(timeNewPacket) > PACKET_TIMEOUT)
  {
    set_default_ppm();
  }
  
  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      //digitalWrite(LED, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      if(buf[0] == 0xCC && buf[1] == 0xAA)
      {
        for(int k=0; k<4; ++k)
        {
          int16_t pulse_val = buf[2*k+2]*255 + buf[2*k+3]; //1000 to 2000
          
          if(pulse_val < 990 || pulse_val > 2010)
          {
            pulse_val = 1500;
          }
          else if(pulse_val < 1001)
          {
            pulse_val = 1001;
          }
          else if(pulse_val > 1999)
          {
            pulse_val = 1999;
          }
          cli();
          ppm[k] = pulse_val; //Consider hard coding to 1500 and see if it removes hiccups
          sei();
          Serial.print(pulse_val);
          Serial.print(",");
        }
        timeNewPacket = millis();
      }
      
      Serial.println();
      //Serial.print("Got: ");
      //Serial.println((char*)buf);
      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);

      // Send a reply
      /*
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      //Serial.println("Sent a reply");
      */
      
      //digitalWrite(LED, LOW);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}

uint16_t timeSince(uint16_t startTime)
{
  return (uint16_t)(millis() - startTime);
}

void set_default_ppm()
{
  cli();
  for(int i = 0; i < NUM_CHANNELS; ++i){
    ppm[i]= DEFAULT_PULSE_LENGTH;
  }
  sei();
}

// The Overflow interrupt occurs when the counter reaches the TOP value in ICR3.
// The hardware automatically emits a falling edge at this time.
// The hardware will automatically emit a rising edge later based on the value in OCR3A.
// The ISR just needs to store the time of the next falling edge in ICR3.
ISR(TIMER3_OVF_vect){
    static byte cur_chan_num = 0;
    static unsigned int calc_rest = 0;  // Microseconds since start of frame
  
    if(cur_chan_num >= NUM_CHANNELS){
      // If all channels have been output, then idle for the remainder of the frame
      ICR3 = FRAME_LENGTH - calc_rest;
      cur_chan_num = 0;
      calc_rest = 0;
    }
    else{
      // Set the time for the next falling edge.
      ICR3 = ppm[cur_chan_num];
      calc_rest += ppm[cur_chan_num];
      cur_chan_num++;
    }     
}
