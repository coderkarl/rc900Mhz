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

#include <PS2X_lib.h>  //for v1.6
PS2X ps2x; // create PS2 Controller Class
int error = 0; 
byte type = 0;
byte vibrate = 0;
bool set_neutral = true;
#define INIT_TRIM_RX 20
#define INIT_TRIM_RY -20
int16_t trim_rx = INIT_TRIM_RX;
int16_t trim_ry = INIT_TRIM_RY;

unsigned blade_db_count = 0;
unsigned auto_db_count = 0;
unsigned blade_off_count = 0;
unsigned auto_off_count = 0;
bool blade_active = false;
bool auto_active = false;
bool blade_press = false;
bool auto_press = false;

unsigned x_db_count = 0;

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  if(SHOW_SERIAL)
  {
    Serial.begin(57600);
    while (!Serial) {
      delay(1);
    }
  }  

  delay(100);
  //error = ps2x.config_gamepad(SCK,MOSI,10,MISO, true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(13,11,10,12, true, true);
  if(error == 0 && SHOW_SERIAL){
    Serial.println("Found Controller, configured successful");
  }
  type = ps2x.readType(); 
  if(SHOW_SERIAL)
  {
    switch(type) {
       case 0:
        Serial.println("Unknown Controller type");
       break;
       case 1:
        Serial.println("DualShock Controller Found");
       break;
    }
  }

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
  
  delay(1000);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission
bool send_joy_data = true;

void loop()
{
  delay(50);
  char radiopacket[RC_PACKET_SIZE];
  bool all_neutral = true;
  int16_t joysig[] = {1500, 1500, 1500, 1500};
  
  if(error != 1 )
  {
    ps2x.read_gamepad(false, vibrate);
    if(SHOW_SERIAL)
    {
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC); 
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC); 
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC);
    }

    if (ps2x.NewButtonState())  //will be TRUE if any button changes state (on to off, or off to on)
    {
      if(ps2x.Button(PSB_PAD_UP)){trim_ry += 2;}
      if(ps2x.Button(PSB_PAD_DOWN)){trim_ry -= 2;}
      if(ps2x.Button(PSB_PAD_RIGHT)){trim_rx += 2;}
      if(ps2x.Button(PSB_PAD_LEFT)){trim_rx -= 2;}

      if(ps2x.Button(PSB_SELECT)) {
        trim_rx = INIT_TRIM_RX;
        trim_ry = INIT_TRIM_RY;
      }
    }
    
    int16_t joy_x_right = ps2x.Analog(PSS_RX)/255.0*1000.0+1000 + trim_rx; // (0 to 255) -> (1000 to 2000
    int16_t joy_y_right = 3000 - (ps2x.Analog(PSS_RY)/255.0*1000.0+1000) + trim_ry; // (0 to 255) -> (1000 to 2000
    if(abs(joy_y_right - 1500) > 150)
    {
      joy_x_right = 1500 + (joy_x_right - 1500)/3;
    }
    else
    {
      joy_x_right = 1500 + (joy_x_right - 1500)/2;
    }
    //int16_t joy_x_left = ps2x.Analog(PSS_LX)/255.0*1000.0+1000; // (0 to 255) -> (1000 to 2000
    //int16_t joy_y_left = 3000 - (ps2x.Analog(PSS_LY)/255.0*1000.0+1000); // (0 to 255) -> (1000 to 2000
    joysig[0] = joy_x_right; //steer
    joysig[1] = joy_y_right; //speed
    if(!auto_press && ps2x.Button(PSB_R2))
    {
      ++auto_db_count;
      if(auto_db_count > 1)
      {
        auto_press = true;
        auto_off_count = 0;
      }
    }
    else if(auto_press && !ps2x.Button(PSB_R2))
    {
      ++auto_off_count;
      if(auto_off_count > 1)
      {
        auto_press = false;
        auto_active = !auto_active;
        Serial.print("AUTO: "); Serial.println(auto_active);
        auto_db_count = 0;
      }
    }
    
    if(!blade_press && ps2x.Button(PSB_R1))
    {
      ++blade_db_count;
      if(blade_db_count > 1)
      {
        blade_press = true;
        blade_off_count = 0;
      }
    }
    else if(blade_press && !ps2x.Button(PSB_R1))
    {
      ++blade_off_count;
      if(blade_off_count > 1)
      {
        blade_press = false;
        blade_active = !blade_active;
        Serial.print("BLADE: "); Serial.println(blade_active);
        blade_db_count = 0;
      }
    }

    if(ps2x.Button(PSB_CROSS))
    {
      ++x_db_count;
      if(x_db_count > 1)
      {
        blade_active = false;
        auto_active = false;
        Serial.println("BLADE AND AUTO OFF");
      }
    }
    else
    {
      x_db_count = 0;
    }
    
    joysig[2] = auto_active ? 1900 : 1500;
    joysig[3] = blade_active ? 1010 : 1500;
    //joysig[2] = joy_y_left; //auto
    //joysig[3] = joy_x_left; // blade
  }
  else
  {
    if(SHOW_SERIAL){Serial.println("PS2 Error");}
  }

  radiopacket[0] = 0xCC;
  radiopacket[1] = 0xAA;
  for(int k=0;k<4;++k)
  {
    radiopacket[2*k+2] = (joysig[k] >> 8) & 0xFF;
    radiopacket[2*k+3] = (joysig[k]) & 0xFF;
    all_neutral = all_neutral && (abs(joysig[k] - 1500) < 40);
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
