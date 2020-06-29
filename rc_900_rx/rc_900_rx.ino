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
// Based off of https://github.com/DzikuVx/ppm_encoder
#define CHANNEL_NUMBER 8  //set the number of channels
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define FRAME_LENGTH 20000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 400  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 10  //set PPM signal output pin on the arduino

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[CHANNEL_NUMBER];
//////////////////////////////////////////////////////////////////

void setup()
{
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
  for(int i=0; i<CHANNEL_NUMBER; i++){
    ppm[i]= CHANNEL_DEFAULT_VALUE;
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR3A = 0; // set entire TCCR1 register to 0
  TCCR3B = 0;
  
  OCR3A = 100;  // compare match register, change this
  TCCR3B |= (1 << WGM32);  // turn on CTC mode
  TCCR3B |= (1 << CS31);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK3 |= (1 << OCIE3A); // enable timer compare interrupt
  sei();
  //////////////////////////////////////////////////////////////
}

void loop()
{
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
          ppm[k] = pulse_val; //Consider hard coding to 1500 and see if it removes hiccups
          Serial.print(pulse_val);
          Serial.print(",");
        }
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

ISR(TIMER3_COMPA_vect){
  static boolean state = true;
  
  TCNT3 = 0;
  
  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR3A = PULSE_LENGTH;
    state = false;
  } else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;// 
      OCR3A = (FRAME_LENGTH - calc_rest);
      calc_rest = 0;
    }
    else{
      OCR3A = (ppm[cur_chan_numb] - PULSE_LENGTH);
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
