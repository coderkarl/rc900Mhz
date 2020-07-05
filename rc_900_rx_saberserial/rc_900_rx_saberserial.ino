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

///////////////////////////////////////////////////////////////
#include <USBSabertooth_NB.h>
// Sabertooth Serial directly from radio receiver
USBSabertoothSerial saberSerial(Serial1);
USBSabertooth ST(saberSerial, 128); // Address 128, and use saberSerial as the serial port.

int16_t steer_pwm = 1500;
int16_t speed_pwm = 1500;
int16_t blade_pwm = 1500;
int16_t auto_pwm = 1500;

int16_t scaled_speed_power = 0;
int16_t scaled_steer_power = 0;

long timeCMD;
#define CMD_PERIOD 100

// Read "PPM" data directly from radio packets
//RC CHANNELS
#define RC_STEER 0
#define RC_SPEED 1
#define RC_BLADE 3
#define RC_AUTO 2

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
#define NUM_CHANNELS 8
#define DEFAULT_PULSE_LENGTH 1500
uint16_t ppm[NUM_CHANNELS];
//////////////////////////////////////////////////////////////////

#define PACKET_TIMEOUT 300
long timeNewPacket;

void setup()
{
  timeNewPacket = millis();
  
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial1.begin(9600);
  ST.drive(scaled_speed_power);
  ST.turn(scaled_steer_power);
  timeCMD = millis();

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
}

void loop()
{
  if(timeSince(timeNewPacket) > PACKET_TIMEOUT)
  {
    set_default_ppm();
  }

  if(millis() - timeCMD > CMD_PERIOD)
  {
    timeCMD = millis();
    steer_pwm = ppm[RC_STEER];
    speed_pwm = ppm[RC_SPEED];
    blade_pwm = ppm[RC_BLADE];
    auto_pwm = ppm[RC_AUTO];
    
    if(auto_pwm < 1700)
    {
      if(abs(speed_pwm - 1500) <= 500 && abs(steer_pwm-1500) <= 500 )
      {
        // power -2047 to 2047
        scaled_speed_power = ((float)(speed_pwm - 1500))*1.5;
        scaled_steer_power = ((float)(steer_pwm - 1500))*1.5;
      }
      else
      {
        scaled_speed_power = 0;
        scaled_steer_power = 0;
      }
      ST.drive(scaled_speed_power);
      ST.turn(-scaled_steer_power);
    }
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
          ppm[k] = pulse_val; //Consider hard coding to 1500 and see if it removes hiccups
          
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
  for(int i = 0; i < NUM_CHANNELS; ++i){
    ppm[i]= DEFAULT_PULSE_LENGTH;
  }
}
