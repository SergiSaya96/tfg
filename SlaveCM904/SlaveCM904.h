/*
  CM904Slave.h - Library for using OpenCM9.04 board as a slave device.
  Requires ROBOTIS library OpenCM9.04 VERSIOIN 0.0.3
  Created by Sergi Martínez, March 9, 2018.
  Released into the public domain.
*/

#ifndef SlaveCM904_h
#define SlaveCM904_h

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Arduino.h"
#include <Dynamixel.h>


#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define MODEL_CM_904 400
#define MODEL_AX_12 12

#define FIRMWARE_S18 1

#define BAUD_9600 0
#define BAUD_57000 1
#define BAUD_115200 2
#define BAUD_1M 3
/*#define BAUD_2M 4
#define BAUD_3M 5
#define BAUD_4M 6
#define BAUD_4_5 7
#define BAUD_10_5 8*/



#define DXL_BAUD_RATE_9600 0
#define DXL_BAUD_RATE_57600 1
#define DXL_BAUD_RATE_115200 2
#define DXL_BAUD_RATE_1Mbps 3

#define LED_OFF 0
#define LED_ON 1

enum INSTRUCTION
{
  NONE,
  APING,
  READ,
  WRITE,
  REG_WRITE,
  ACTION,
  ARESET,
  SYNC_WRITE=0x83
};

enum DECODE_STEP
{
  DE_HEADER1,
  DE_HEADER2,
  DE_ID,
  DE_LENGTH,
  DE_INSTRUCTION,
  DE_DATA,
  DE_CHECKSUM
};

Dynamixel Dxl(DXL_BUS_SERIAL1);

class SlaveCM904
{
  public:
    void Begin();
    void Set(byte MN, byte FV, byte ID, byte BR, byte RDT, byte SRL, byte DBR, byte led);
    virtual bool GetMessage();
    void GenerateCheckSum();
    virtual bool CheckID();
    void Blink();
    virtual void Ping();
  protected:
    int DecodeIndex;
    byte mID;
    byte mCount;
    byte mLength;
    byte mInstruction;
    byte mData[ 256 ];
    byte mChecksum;

    void ProcessMessage( byte instruction, byte* data, int len );
  private:
    byte ModelNumberL ;
    byte ModelNumberH ;
    byte FirmwareVersion ;
    byte ID ;
    byte BaudRate ;
    byte ReturnDelayTime ;
    byte StatusReturnLevel ;
    byte DXLBaudRate;
    byte LED;
};

//------------------------------------------------------------------------------
// SlaveCM904::Begin - Initialize the object getting values from the EEPROM
//------------------------------------------------------------------------------
//falta cargar automaticament els valors de la emprom
void SlaveCM904::Begin()
{
  Dxl.begin(DXL_BAUD_RATE_1Mbps);
  switch (BaudRate) {
    case BAUD_9600:
      Serial.begin(9600);
      break;
    case BAUD_57000:
      Serial.begin(57000);
      break;
    case BAUD_115200:
      Serial.begin(115200);
      break;
    case BAUD_1M:
      Serial.begin(1000000);
      break;
  }
}


//------------------------------------------------------------------------------
// SlaveCM904::Begin - Initialize the object from given values
//------------------------------------------------------------------------------
//falta assignar els valors a la emprom
void SlaveCM904::Set(byte MN, byte FV, byte iden, byte BR, byte RDT, byte SRL, byte DBR, byte led)
{
  ModelNumberL = MN;
  FirmwareVersion = FV;
  ID=iden;
  BaudRate = BR;
  switch (BaudRate) {
    case BAUD_9600:
      Serial.begin(9600);
      break;
    case BAUD_57000:
      Serial.begin(57000);
      break;
    case BAUD_115200:
      Serial.begin(115200);
      break;
    case BAUD_1M:
      Serial.begin(1000000);
      break;
  }
  ReturnDelayTime = RDT;
  StatusReturnLevel = SRL;
  DXLBaudRate = DBR;
  Dxl.begin(DXLBaudRate);
  LED = led;
}



bool SlaveCM904::GetMessage()
{
  bool finish = false;
  bool ID_OK = false;
  byte input=0;
  DecodeIndex = DE_HEADER1;
  if (Dxl.available())
  {
    while (!finish)
    {
      input = Dxl.readRaw();
      switch ( DecodeIndex )
      {
        case DE_HEADER1:
          if ( input == 0xFF )
          {
          DecodeIndex = DE_HEADER2;
          Serial.println("DE_HEADER1");
          }
          break;
        case DE_HEADER2:
          if ( input == 0xFF )
          {
            DecodeIndex = DE_ID;
            Serial.println("DE_HEADER2");
          }
          else DecodeIndex = DE_HEADER1;
          break;
        case DE_ID:
          DecodeIndex = DE_HEADER1;
          if ( input != 0xFF )    // we are not allowed 3 0xff's in a row, ie. id != 0xff
          {
            mID = input;
            DecodeIndex = DE_LENGTH;
            Serial.print("ID:");
            Serial.println(input);
          }
          break;
        case DE_LENGTH:
          mLength = input;
          DecodeIndex = DE_INSTRUCTION;
          Serial.print("DE_LENGTH");
          Serial.println(mLength);
          break;
        case DE_INSTRUCTION:
          mInstruction = input;
          mCount = 0;
          DecodeIndex = DE_DATA;
          Serial.print("DE_INSTRUCTION");
          Serial.println(mInstruction);
          if ( mLength == 2 ) DecodeIndex = DE_CHECKSUM;
          break;
        case DE_DATA:
          mData[ mCount++ ] = input;
          if ( mCount >= mLength - 2 )
          {
            DecodeIndex = DE_CHECKSUM;
          }
          break;
        case DE_CHECKSUM:
          {
            Serial.print("DE_CHECKSUM");
            DecodeIndex = DE_HEADER1;
            GenerateCheckSum();
            if (mChecksum  ==  input )
            {
              if (ID_OK = CheckID() )
              {
                Serial.print(mID);
                Serial.print(mLength);
                Serial.print(mInstruction);
                for (int i = 0; i<=mCount; i++)
                {
                  Serial.print(mData[i]);
                }
                Serial.println(mChecksum);
                ProcessMessage( mInstruction, mData, mLength - 2 );
              }
            }
          }
          finish = true;
          break;
      }
    }
  }
  delayMicroseconds(800);
  return ID_OK;
}

void SlaveCM904::ProcessMessage( byte instruction, byte* data, int len )
{
  switch ( instruction )
  {
    case APING:
      {
        Ping();
      }
      break;
    /*case READ:
      {
        byte index = data[ 0 ];
        byte datalen = data[ 1 ];
        preProcessRegisterRead(index, datalen);
        transmitMessage( 0, &REG[ index ], datalen );
      }
      break;
    case WRITE:
      {
        byte index = data[ 0 ];
        byte length = len - 1;

        transmitMessage( 0, 0, 0 ); // Start sending response as quick as possible
        writeValues(index, length, data + 1 );
        postProcessRegisterWrite(index, length);
      }
      break;
    case REG_WRITE:
      {
      	regWrites[regWriteCount].RegWriteAddress = data[ 0 ];
        regWrites[regWriteCount].RegWriteLength = len - 1;

        for ( int i = 0; i < regWrites[regWriteCount].RegWriteLength; ++i )
        {
          RegWriteData[ i ] = data[ i + 1 ];
        }
        regWriteCount++;
      }
      break;
    case ACTION:
      {
        if ( regWriteCount > 0 )
        {
        	for( int i = 0; i < regWriteCount; ++i )
        	{
	          writeValues( regWrites[i].RegWriteAddress, regWrites[i].RegWriteLength, RegWriteData );
	          postProcessRegisterWrite(regWrites[i].RegWriteAddress, regWrites[i].RegWriteLength);
        	}
        	regWriteCount = 0;
        }
      }
      break;
    case RESET:
      processReset();
      break;
    case SYNC_WRITE:
      {
        byte index = data[ 0 ];
        byte datalen = data[ 1 ];
        processSyncWrite(index, datalen, data + 2, len - 3);
      }
      break;*/
  }
}

void SlaveCM904::GenerateCheckSum()
{
  byte paramsum=0;
  for (int i = 0; i<=mCount; i++)
  {
    paramsum = paramsum +  mData[i];
  }
  mChecksum = ~(mID + mLength + mInstruction + paramsum);
}

bool SlaveCM904::CheckID()
{
  bool ID_OK=false;
  if(mID == ID) ID_OK = true;
  return ID_OK;
}

void SlaveCM904::Ping()
{
  int rep[] = { 255, 255, ID, 2, 0, 249};
  for (int j = 0; j <= 5; j++) {
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
