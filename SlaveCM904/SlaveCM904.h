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


//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

#define MODEL_CM_904L 0x90
#define MODEL_CM_904H 0x01
#define MODEL_AX_12L 12
#define MODEL_AX_12H 0

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



#define DXL_BAUD_RATE_9600 207
#define DXL_BAUD_RATE_57600 34
#define DXL_BAUD_RATE_115200 16
#define DXL_BAUD_RATE_1Mbps 1

#define LED_OFF 0
#define LED_ON 1

enum INSTRUCTION
{
  NONE,
  APING,
  READ,
  WRITE,
  //-------------
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

enum EEPPROM_REG
{
  EEPROM_MODEL_NUMBER_L,
  EEPROM_MODEL_NUMBER_H,
  EEPROM_FIRMWARE_VERSION=0x06,
  EEPROM_ID,
  EEPROM_BUD_RATE,
  EEPROM_RETURN_DELAY_TIME,
  EEPROM_STATUS_RETURN_LEVEL,
  EEPROM_DXL_BAUD_RATE=0x12,
  EEPROM_LED,
  EEPROM_ERR_OFF_L,
  EEPROM_ERR_OFF_H,
  EEPROM_ERR_GAIN_L,
  EEPROM_ERR_GAIN_H
};

enum RAM_REG
{
  RAM_ADL=0x18,
  RAM_ADH,
  RAM_CAL_OFF_PORCESS,
  RAM_CAL_GAIN1_L,
  RAM_CAL_GAIN1_H,
  RAM_CAL_GAIN2_L,
  RAM_CAL_GAIN2_H,
};

Dynamixel Dxl(DXL_BUS_SERIAL1);

class SlaveCM904
{
  public:
    byte REGISTER[48];
    //byte ADL=0;
    //byte ADH=0;
    void Begin();
    void Set(byte MNL, byte MNH, byte FV, byte ID, byte BR, byte RDT, byte SRL, byte DBR, byte led);
    bool GetMessage();
    void TransmitMessage( byte err, byte index, byte len );
    void GenerateCheckSum();
    void postProcessRegisterWrite(); //funció per actualitzar despres de l'escritra a la eeprom
    bool CheckID();
    void Indicator(int status);
    void Blink();
    void Ping(byte err);
  protected:
    int DecodeIndex;
    byte mID;
    byte mCount;
    byte mLength;
    byte mInstruction;
    byte mParam[ 256 ];
    byte mChecksum;
    void ProcessMessage( byte instruction, byte* data, int len );
  /*private:
    byte ModelNumberL ;
    byte ModelNumberH ;
    byte FirmwareVersion ;
    byte ID ;
    byte BaudRate ;
    byte ReturnDelayTime ;
    byte StatusReturnLevel ;
    byte DXLBaudRate;
    byte LED;
    byte OFFL;
    byte OFFH;
    byte ERRL;
    byte ERRH;*/
};

//------------------------------------------------------------------------------
// SlaveCM904::Begin - Initialize the object getting values from the EEPROM
//------------------------------------------------------------------------------
//falta cargar automaticament els valors de la emprom
void SlaveCM904::Begin()
{

}


//------------------------------------------------------------------------------
// SlaveCM904::Begin - Initialize the object from given values
//------------------------------------------------------------------------------
//falta assignar els valors a la emprom 
void SlaveCM904::Set(byte MNL, byte MNH, byte FV, byte iden, byte BR, byte RDT, byte SRL, byte DBR, byte led)
{
  REGISTER[EEPROM_MODEL_NUMBER_L] = MNL;
  REGISTER[EEPROM_MODEL_NUMBER_H] = MNH;
  REGISTER[EEPROM_FIRMWARE_VERSION] = FV;
  REGISTER[EEPROM_ID]=iden;
  REGISTER[EEPROM_BUD_RATE] = BR;
  switch (REGISTER[EEPROM_BUD_RATE]) {
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
  REGISTER[EEPROM_RETURN_DELAY_TIME] = RDT;
  REGISTER[EEPROM_STATUS_RETURN_LEVEL] = SRL;
  REGISTER[EEPROM_DXL_BAUD_RATE] = DBR;
  switch (REGISTER[EEPROM_DXL_BAUD_RATE]) {
    case DXL_BAUD_RATE_9600:
      Dxl.begin(0);
      break;
    case DXL_BAUD_RATE_57600:
      Dxl.begin(1);
      break;
    case DXL_BAUD_RATE_115200:
      Dxl.begin(2);
      break;
    case DXL_BAUD_RATE_1Mbps:
      Dxl.begin(3);
      break;
  }
  
  REGISTER[EEPROM_LED] = led;
  if( led == LED_ON ) Indicator(LED_ON);
  if( led == LED_OFF ) Indicator(LED_OFF);
}



bool SlaveCM904::GetMessage()
{
  bool finish = false;
  bool ID_OK = false;
  byte input=0;
  DecodeIndex = DE_HEADER1;
  if (Dxl.available())
  {
    Serial.println("if");
    while (!finish)
    {
      Serial.println("While");
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
            Serial.print("ID: ");
            Serial.println(input);
          }
          break;
        case DE_LENGTH:
          mLength = input;
          DecodeIndex = DE_INSTRUCTION;
          Serial.print("DE_LENGTH: ");
          Serial.println(mLength);
          break;
        case DE_INSTRUCTION:
          mInstruction = input;
          mCount = 0;
          mParam[0] = 0;
          DecodeIndex = DE_DATA;
          Serial.print("DE_INSTRUCTION: ");
          Serial.println(mInstruction);
          if ( mLength == 2 ) DecodeIndex = DE_CHECKSUM;
          break;
        case DE_DATA:
          Serial.print("DE_PARAM: ");
          Serial.println(input);
          Serial.print("DE_Count: ");
          Serial.println(mCount);
          mParam[ mCount++ ] = input;
          if ( mCount >= mLength - 2 )
          {
            DecodeIndex = DE_CHECKSUM;
          }
          break;
        case DE_CHECKSUM:
          {
            Serial.println("DE_CHECKSUM");
            DecodeIndex = DE_HEADER1;
            GenerateCheckSum();
            Serial.print("Recieved: ");
            Serial.print(input);
            Serial.print(" Generated: ");
            Serial.println(mChecksum);
            if (mChecksum  ==  input )
            {
              Serial.println("ChecksumOK");
              if (ID_OK = CheckID() )
              {
                Serial.print("ID:");
                Serial.print(mID);
                Serial.print(" Length:");
                Serial.print(mLength);
                Serial.print(" Ims:");
                Serial.print(mInstruction);
                Serial.print(" Param:");
                for (int i = 0; i<=(mCount-1); i++)
                {
                  Serial.print(mParam[i]);
                  Serial.print("-");
                }
                Serial.print(" CheS:");
                Serial.println(mChecksum);
                ProcessMessage( mInstruction, mParam, mLength - 2 );
              }
            }
            else
            {
              if (ID_OK = CheckID() )
              {
                Ping(8);
              }
            }
          }
          finish = true;
          break;
      }
    }
  delayMicroseconds(800);
  }
  return ID_OK;
}

void SlaveCM904::ProcessMessage( byte instruction, byte* param, int len )
{
  switch ( instruction )
  {
    case NONE:
      {;}
      break;
    case APING:
      {
        Ping(0);
      }
      break;
    case READ:
      {
        byte index = param[ 0 ];
        byte datalen = param[ 1 ];
        TransmitMessage( 0, index, datalen );
      }
      break;
    /*case WRITE:
      {
        byte index = param[ 0 ];
        byte length = len - 1;
        transmitMessage( 0, 0, 0 ); // Start sending response as quick as possible
        writeValues(index, length, param + 1 );  //Write values on EEPROM & update REGISTER
        postProcessRegisterWrite(); //execute possible changes
      }
      break;*/
  }
}

void SlaveCM904::TransmitMessage( byte err, byte index, byte len )
{
  byte txmsg[ 1024 ];

  txmsg[ 0 ] = txmsg[ 1 ] = 0xFF;
  txmsg[ 2 ] = REGISTER[ EEPROM_ID ];
  txmsg[ 3 ] = len + 2;
  txmsg[ 4 ] = err;

  for (int i = 0; i < len; ++i )
  {
    txmsg[ 5 + i ] = REGISTER[ index + i ];
  }

  byte checksum = 0;
  for (int i = 2; i < 5 + len; ++i )
  {
    checksum += txmsg[ i ];
  }
  checksum = ~checksum;
  txmsg[ 5 + len ] = checksum;

  for (int j = 0; j <= 5+len; j++) {
    Dxl.writeRaw(txmsg[j]);
  }
  delayMicroseconds(800);
}

void SlaveCM904::Indicator(int status)
{
  int led_pin = 14;
  pinMode(led_pin, OUTPUT);
  if(status == LED_ON) digitalWrite(led_pin, LOW);  // set to as HIGH LED is turn-off
  if(status == LED_OFF) digitalWrite(led_pin, HIGH);   // set to as LOW LED is turn-on
}

void SlaveCM904::GenerateCheckSum()
{
  byte paramsum=0;
  for (int i = 0; i<=mCount; i++)
  {
    paramsum = paramsum +  mParam[i];
  }
  mChecksum = ~(mID + mLength + mInstruction + paramsum);
}

bool SlaveCM904::CheckID()
{
  bool ID_OK=false;
  if(mID == REGISTER[EEPROM_ID]) ID_OK = true;
  return ID_OK;
}

void SlaveCM904::Ping(byte error)
{
  int rep[] = { 255, 255, REGISTER[EEPROM_ID], 2, error, ~(REGISTER[EEPROM_ID]+2)};
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
