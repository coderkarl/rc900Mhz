// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>

//for feather32u4 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define RC_PACKET_SIZE 10

#define SHOW_SERIAL 0

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  if(SHOW_SERIAL)
  {
    Serial.begin(115200);
    while (!Serial) {
      delay(1);
    }
  }

  delay(100);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    if(SHOW_SERIAL)
    {
      Serial.println("LoRa radio init failed");
      Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    }
    while (1);
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    if(SHOW_SERIAL)
    {
      Serial.println("setFrequency failed");
    }
    while (1);
  }
  if(SHOW_SERIAL)
  {
    Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  }
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission
bool send_joy_data = true;

void loop()
{
  delay(10);
  //Serial.println("Transmitting..."); // Send a message to rf95_server
  analogRead(A0);
  int16_t joy_x_right = 3000 - min(analogRead(A0)/1024.0*1000.0+1000, 2000);
  delay(10);
  analogRead(A1);
  int16_t joy_y_right = min(analogRead(A1)/1024.0*1000.0+1000, 2000);
  if(abs(joy_x_right - 1500) > 150)
  {
    joy_y_right = 1500 + (joy_y_right - 1500)/3;
  }
  else
  {
    joy_y_right = 1500 + (joy_y_right - 1500)/2;
  }
  delay(10);
  analogRead(A2);
  int16_t joy_x_left = 3000 - min(analogRead(A2)/1024.0*1000.0+1000, 2000);
  delay(10);
  analogRead(A3);
  int16_t joy_y_left = min(analogRead(A3)/1024.0*1000.0+1000, 2000);
  int16_t joysig[] = {joy_y_right, joy_x_right, joy_x_left, joy_y_left};
  int n = 0;
  bool all_neutral = true;

  char radiopacket[RC_PACKET_SIZE];

  /*n = sprintf(radiopacket, "%d,%d,%d,%d", joy_y_right, joy_x_right, joy_x_left, joy_y_left);
  Serial.print("Sending "); Serial.println(radiopacket);
  for(int k=n; k<RC_PACKET_SIZE; ++k)
  {
    radiopacket[k] = 0;
  }*/

  radiopacket[0] = 0xCC;
  radiopacket[1] = 0xAA;
  for(int k=0;k<4;++k)
  {
    radiopacket[2*k+2] = (joysig[k] >> 8) & 0xFF;
    radiopacket[2*k+3] = (joysig[k]) & 0xFF;
    all_neutral = all_neutral && (abs(joysig[k] - 1500) < 20);
    if(SHOW_SERIAL)
    {
      Serial.print(joysig[k]);
      Serial.print(",");
    }
  }
  if(SHOW_SERIAL){Serial.println();}

  if(!all_neutral)
  {
    send_joy_data = true;
  }

  if(send_joy_data)
  {
    //Serial.println("Sending...");
    delay(5);
    rf95.send((uint8_t *)radiopacket, RC_PACKET_SIZE);
  
    //Serial.println("Waiting for packet to complete..."); 
    delay(5);
    rf95.waitPacketSent();
    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
  }
  if(all_neutral)
  {
    send_joy_data = false;
  }

  /*Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }*/

}
