/*
  CM904Slave.h - Library for using OpenCM9.04 board as slave device.
  Requires ROBOTIS library OpenCM9.04 VERSIOIN 0.0.3
  Created by Sergi Martínez, March 9, 2018.
  Released into the public domain.
*/
#ifndef SlaveCM904_h
#define SlaveCM904_h

#include "Arduino.h"
#include <Dynamixel.h>

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

Dynamixel Dxl(DXL_BUS_SERIAL1);

class SlaveCM904
{
  public:
    //SlaveCM904(int baud);
    void Begin(int baud);
    byte *GetMessage();
    void CheckID();
    void Blink();
    void Ping();
  private:
    byte ModelNumber = 1;
    byte FirmwareVersion = 1;
    byte ID = 4;
    byte BaudRate = 1;
    byte ReturnDelayTime;
    byte StatusReturnLevel;
    byte DXLBaudRate;
};

void SlaveCM904::Begin(int baud)
{
  Serial.begin(115200);
  Dxl.begin(baud);
}

byte *SlaveCM904::GetMessage()
{
  bool finish = false, start=false;
  int i=0,lenght=10;
  byte message[24];
  if (Dxl.available())
  {
    while(!finish){
      message[i]=Dxl.readRaw();
      if(message[i]==0xFF && start==false){
        start=true;
      }
      if(i==3){
        lenght=message[i];
      }
      if(start){
        i++;
      }
      if(i == lenght+4){
        start=false;
        i=0;
        lenght=10;
        finish=true;
      }
    }
    /*for(int j=0; j<=5; j++){
      Serial.print(message[j],HEX);
    }
    Serial.print("\n");*/
  }
  delayMicroseconds(800);
  return message;
}

void SlaveCM904::Ping()
{
  int rep[]={ 255, 255, 4, 2, 0, 249};
  for(int j=0; j<=5; j++){
    Dxl.writeRaw(rep[j]);
  }
  delayMicroseconds(800);
}

void SlaveCM904::Blink()
{
  int led_pin = 14;
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);  // set to as HIGH LED is turn-off
  Serial.println("led_off");
  delay(1000);                   // Wait for 0.1 second
  digitalWrite(led_pin, HIGH);   // set to as LOW LED is turn-on
  Serial.println("led_on");
  delay(1000);                   // Wait for 0.1 second
}
#endif
