/*
  CM904Slave.h - Library for using OpenCM9.04 board as a slave device.
  Requires ROBOTIS library OpenCM9.04 VERSIOIN 0.0.3
  Created by Sergi Martínez, March 9, 2018.
  Released into the public domain.
*/

/*
-dac gain process
-save in Newtons
*/

#ifndef SlaveCM904_h
#define SlaveCM904_h

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "Arduino.h"
#include <Dynamixel.h>
#include <SlowSoftI2CMaster.h>
#include <Wire.h>


//------------------------------------------------------------------------------
// Defines
//------------------------------------------------------------------------------
#define I2C_EEPROM 80
#define I2C_DAC_VREF 9
#define I2C_DAC_VP 76
#define I2C_DAC_VN 78

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

#define MODEL_CM_904L 0x90
#define MODEL_CM_904H 0x01
#define MODEL_AX_12L 12
#define MODEL_AX_12H 0

#define FIRMWARE_S18 100

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
  EEPROM_MODEL_NUMBER_L,  //Y -> 0x00
  EEPROM_MODEL_NUMBER_H,  //Y
  EEPROM_FIRMWARE_VERSION,  //Y
  EEPROM_ID,  //Y
  EEPROM_DXL_BAUD_RATE,  //Y
  EEPROM_RETURN_DELAY_TIME,
  EEPROM_STATUS_RETURN_LEVEL=0x10,
  EEPROM_BAUD_RATE,  //Y
  EEPROM_ERR_OFF_L,  //Y
  EEPROM_ERR_OFF_H,  //Y
  EEPROM_ERR_GAIN_L,
  EEPROM_ERR_GAIN_H
};

enum RAM_REG
{
  RAM_LED=0x18,  //Y
  RAM_GAUGE_L,  //Y
  RAM_GAUGE_H,  //Y -> 0x1A
  RAM_SUPPLY_L,  //Y
  RAM_SUPPLY_H,  //Y
  RAM_START_CAL_OFF_PROCESS,  //Y -> 0x1D
  RAM_CAL_OFF_PROCESS_DONE, //Y -> 0x1E
  RAM_CAL_GAIN_PROCESS,
};

Dynamixel Dxl(DXL_BUS_SERIAL1);

class SlaveCM904
{
  public:
    byte REGISTER[48];
    void Setup();
    void Begin();
    virtual void Set(byte MNL, byte MNH, byte FV, byte iden, byte DBR, byte RDT, byte SRL, byte BR, byte EOL, byte EOH, byte EGH, byte EGL);
    bool GetMessage();
    virtual void TransmitMessage( byte err, byte index, byte len );
    void GenerateCheckSum();
    virtual void PostProcess(); //funció per actualitzar despres de l'escritura a la eeprom
    bool CheckID();
    void Peripherals();
    void Indicator(int status);
    void CalibrationOffset();
    void CompensateOffset();
    void CalibrationGain();
    void Blink();
    void Ping(byte err);
    void i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length );
    void i2c_eeprom_write_page( int deviceaddress, unsigned int eeaddresspage, byte* data, byte length );
  protected:
    int DecodeIndex;
    byte mID;
    byte mCount;
    byte mLength;
    byte mInstruction;
    byte mParam[ 256 ];
    byte mChecksum;
    void ProcessMessage( byte instruction, byte* data, int len );
    void WriteValues( byte index, byte len, byte* data );
    void ReturnTimeMicroseconds(uint32_t usec);
};

void SlaveCM904::Setup()
{
  Wire.begin(); //Start i2c
  analogReadResolution(12);  //Set ADC's resolution
  REGISTER[RAM_LED] = 0;
  REGISTER[RAM_START_CAL_OFF_PROCESS] = 0;
  REGISTER[RAM_CAL_OFF_PROCESS_DONE] = 0;
  REGISTER[RAM_CAL_GAIN_PROCESS] = 0;

}

//------------------------------------------------------------------------------
// SlaveCM904::Begin - Initialize the object getting values from the EEPROM
//------------------------------------------------------------------------------
void SlaveCM904::Begin()
{
  Serial.println("Reading EEPROM...");
  byte buffer[32];
  i2c_eeprom_read_buffer( I2C_EEPROM, byte(EEPROM_MODEL_NUMBER_L),(byte*)buffer, 6 );
  REGISTER[EEPROM_MODEL_NUMBER_L] = buffer[0];
  REGISTER[EEPROM_MODEL_NUMBER_H] = buffer[1];  
  REGISTER[EEPROM_FIRMWARE_VERSION] = buffer[2];
  REGISTER[EEPROM_ID]=buffer[3];
  REGISTER[EEPROM_DXL_BAUD_RATE] = buffer[4];
  REGISTER[EEPROM_RETURN_DELAY_TIME] = buffer[5];
  for ( int i = 0; i < 6; ++i )
  {
    Serial.println(buffer[i]);
  }
  delay(5);
  i2c_eeprom_read_buffer( I2C_EEPROM, byte(EEPROM_STATUS_RETURN_LEVEL),(byte*)buffer, 6 );
  REGISTER[EEPROM_STATUS_RETURN_LEVEL] = buffer[0];
  REGISTER[EEPROM_BAUD_RATE] = buffer[1];
  REGISTER[EEPROM_ERR_OFF_L] = buffer[2];
  REGISTER[EEPROM_ERR_OFF_H] = buffer[3];
  REGISTER[EEPROM_ERR_GAIN_L] = buffer[4];
  REGISTER[EEPROM_ERR_GAIN_H] = buffer[5];
  for ( int i = 0; i < 6; ++i )
  {
    Serial.println(buffer[i]);
  }
  
  switch (REGISTER[EEPROM_BAUD_RATE]) {
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

  Peripherals();

}

//------------------------------------------------------------------------------
// SlaveCM904::GetMessage - Decodes de instruction packect
//------------------------------------------------------------------------------
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
      //Serial.print("GET MESSAGE WAITING at ");
      //Serial.println(REGISTER[EEPROM_DXL_BAUD_RATE]);
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
            DecodeIndex = DE_HEADER1;
            GenerateCheckSum();
            Serial.println("DE_CHECKSUM");
            /*Serial.print("Recieved: ");
            Serial.print(input);
            Serial.print(" Generated: ");
            Serial.println(mChecksum);*/
            if (mChecksum  ==  input )
            {
              //Serial.println("ChecksumOK");
            
              
              
              if (ID_OK = CheckID() )
              {
                /*Serial.print("ID:");
                Serial.print(mID,HEX);
                Serial.print(" Length:");
                Serial.print(mLength,HEX);
                Serial.print(" Ims:");
                Serial.print(mInstruction,HEX);
                Serial.print(" Param:");
                for (int i = 0; i<=(mCount-1); i++)
                {
                  Serial.print(mParam[i],HEX);
                  Serial.print("-");
                }
                Serial.print(" CheS:");
                Serial.println(mChecksum,HEX);*/
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
  }
  return ID_OK;
}

//------------------------------------------------------------------------------
// SlaveCM904::ProcessMessage - Executes de corresponding action
//------------------------------------------------------------------------------
void SlaveCM904::ProcessMessage( byte instruction, byte* param, int len )
{
  int time;
  time=((REGISTER[EEPROM_RETURN_DELAY_TIME])*2)-100;
  if(time>0)
    ReturnTimeMicroseconds(time);

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
    case WRITE:
      {
        byte index = param[ 0 ];
        byte length = len - 1;
        TransmitMessage( 0, 0, 0 ); // Start sending response as quick as possible
        WriteValues(index, length, param + 1 );
        PostProcess(); //execute possible changes
      }
      break;
  }
}

//------------------------------------------------------------------------------
// SlaveCM904::TransmitMessage - Sends the corresponding Status packet
//------------------------------------------------------------------------------
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
  //delayMicroseconds(800);
}

//------------------------------------------------------------------------------
// SlaveCM904::WriteValues - Write values on EEPROM & update REGISTER
//------------------------------------------------------------------------------
void SlaveCM904::WriteValues( byte index, byte len, byte* data )
{
  /*Serial.print("Writing in ");
  Serial.print(index);
  Serial.print("  ");
  Serial.print(len);
  Serial.print(" bits Value:");
  Serial.println(data[0]);*/
  for ( int i = 0; i < len; ++i )
  {
    REGISTER[ index + i ] = data[ i ];
  }
  if(index<byte(0x18)){
    i2c_eeprom_write_page( I2C_EEPROM, index, (byte*) data, len );
  }
}

//------------------------------------------------------------------------------
// SlaveCM904::WriteValues - Executes possible changes
//------------------------------------------------------------------------------
void SlaveCM904::PostProcess()
{
  switch (REGISTER[EEPROM_BAUD_RATE]) {
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

  Peripherals();
} 

//------------------------------------------------------------------------------
// SlaveCM904::Peripherals - Writes values at the peripherals
//------------------------------------------------------------------------------
void SlaveCM904::Peripherals()
{
  //Serial.print("PERIPHERALS!!  ");
  //Serial.println(REGISTER[RAM_START_CAL_OFF_PROCESS]);
  if( REGISTER[RAM_LED] == LED_ON ) Indicator(LED_ON);
  if( REGISTER[RAM_LED] == LED_OFF ) Indicator(LED_OFF);
  if( REGISTER[byte(RAM_START_CAL_OFF_PROCESS)] == 1 ) CalibrationOffset();
  else CompensateOffset();
  if( REGISTER[RAM_CAL_GAIN_PROCESS] == 1) CalibrationGain();
}

void SlaveCM904::Indicator(int status)
{
  int led_pin = 14;
  pinMode(led_pin, OUTPUT);
  if(status == LED_ON) digitalWrite(led_pin, LOW);  // set to as HIGH LED is turn-off
  if(status == LED_OFF) digitalWrite(led_pin, HIGH);   // set to as LOW LED is turn-on
}

void SlaveCM904::CalibrationOffset()
{
  //Serial.println("Ofsset cal started ");
  unsigned int val = 0xAF0;
  REGISTER[RAM_CAL_OFF_PROCESS_DONE] = 0;
  while(analogRead(A3)<2048)
  {
    /*Serial.print(analogRead(A3));
    Serial.print("  ");
    Serial.println(val,HEX);*/
    Wire.beginTransmission(I2C_DAC_VREF); // transmit to Vref DAC
    Wire.write(val >> 8);          // sends instruction byte
    Wire.write(val & byte(0x00ff));      
    Wire.endTransmission();     // stop transmitting
    val++;
    delayMicroseconds(50);
  }
  delay(5);
  REGISTER[RAM_START_CAL_OFF_PROCESS] = 0;
  REGISTER[RAM_CAL_OFF_PROCESS_DONE] = 5;
  REGISTER[EEPROM_ERR_OFF_H] = val>>8;
  REGISTER[EEPROM_ERR_OFF_L] = val;
  //Serial.println(val,HEX);
  Wire.beginTransmission(I2C_EEPROM);  //EEPROM adress
  Wire.write(byte(0x00)); //EEPROM reg Low
  Wire.write(EEPROM_ERR_OFF_L);  //EEPROM reg Low
  Wire.write(val);  //data
  Wire.write(val>>8);
  Wire.endTransmission(); //finish transmision

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
  delay(5);
}

void SlaveCM904::CompensateOffset()
{
  //Serial.println("Compensating offset ");
  Wire.beginTransmission(I2C_DAC_VREF); // transmit to Vref DAC
  Wire.write(REGISTER[EEPROM_ERR_OFF_H]);          // sends instruction byte
  Wire.write(REGISTER[EEPROM_ERR_OFF_L]);      
  Wire.endTransmission();     // stop transmitting
}

void SlaveCM904::CalibrationGain()
{
  //Serial.println("Gain cal started ");
  Wire.beginTransmission(I2C_DAC_VP); 
  Wire.write(byte(0x30));
  Wire.write(byte(0x5f));
  Wire.write(byte(0xff));
  Wire.endTransmission();     // stop transmitting
  delay(5);
  Wire.beginTransmission(I2C_DAC_VN); 
  Wire.write(byte(0x30));
  Wire.write(byte(0x5f));
  Wire.write(byte(0xff));
  Wire.endTransmission();     // stop transmitting
  delay(5);
}

void SlaveCM904::GenerateCheckSum()
{
  byte paramsum=0;
  for (int i = 0; i<mCount; i++)
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
  //Serial.println("PING");
  int time;
  time=((REGISTER[EEPROM_RETURN_DELAY_TIME])*2)-100;
  if(time>0)
    ReturnTimeMicroseconds(time);
  int rep[] = { 255, 255, REGISTER[EEPROM_ID], 2, error, ~(REGISTER[EEPROM_ID]+2)};
  for (int j = 0; j <= 5; j++) {
    Dxl.writeRaw(rep[j]);
  }
}

void SlaveCM904::Blink()
{
  int led_pin = 14;
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);  // set to as HIGH LED is turn-off
  //Serial.println("led_off");
  delay(1000);                   // Wait for 0.1 second
  digitalWrite(led_pin, HIGH);   // set to as LOW LED is turn-on
  //Serial.println("led_on");
  delay(1000);                   // Wait for 0.1 second
}

void SlaveCM904::i2c_eeprom_write_page( int deviceaddress, unsigned int eeaddresspage, byte* data, byte length ) {
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddresspage >> 8)); // MSB
    Wire.write((int)(eeaddresspage & 0xFF)); // LSB
    byte c;
    for ( c = 0; c < length; c++)
        Wire.write(data[c]);
    Wire.endTransmission();
}

void SlaveCM904::i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length ) {
    Wire.beginTransmission(deviceaddress);
    Wire.write((int)(eeaddress >> 8)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.requestFrom(deviceaddress,length);
    int c = 0;
    for ( c = 0; c < length; c++ )
        if (Wire.available()) buffer[c] = Wire.read();
}

void SlaveCM904::ReturnTimeMicroseconds(uint32_t usec)
{
  uint32_t tTime;
  tTime = micros();
  while( (micros()-tTime) < usec );
}

//------------------------------------------------------------------------------
// SlaveCM904::Begin - Initialize the object from given values when EEPROM empty
//------------------------------------------------------------------------------
void SlaveCM904::Set(byte MNL, byte MNH, byte FV, byte iden, byte DBR, byte RDT, byte SRL, byte BR, byte EOL, byte EOH, byte EGH, byte EGL)
{
  REGISTER[EEPROM_MODEL_NUMBER_L] = MNL;
  REGISTER[EEPROM_MODEL_NUMBER_H] = MNH;
  REGISTER[EEPROM_FIRMWARE_VERSION] = FV;
  REGISTER[EEPROM_ID]=iden;
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
  REGISTER[EEPROM_RETURN_DELAY_TIME] = RDT;
  REGISTER[EEPROM_STATUS_RETURN_LEVEL] = SRL;
  REGISTER[EEPROM_BAUD_RATE] = BR;
  switch (REGISTER[EEPROM_BAUD_RATE]) {
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
  REGISTER[EEPROM_ERR_OFF_L] = EOL;
  REGISTER[EEPROM_ERR_OFF_H] = EOH;
  REGISTER[EEPROM_ERR_GAIN_L] = EGL;
  REGISTER[EEPROM_ERR_GAIN_H] = EGH;

  
  Peripherals();
  

  Serial.println("Writing EEPROM...");
  delay(5);
  Wire.beginTransmission(I2C_EEPROM);  //EEPROM adress
  Wire.write(byte(0x00)); //EEPROM reg Low
  Wire.write(byte(EEPROM_MODEL_NUMBER_L));  //EEPROM reg Low
  Wire.write(byte(MNL));  //data
  Wire.write(byte(MNH));
  Wire.write(byte(FV));
  Wire.write(byte(iden));
  Wire.write(byte(DBR));
  Wire.write(byte(RDT));
  Wire.endTransmission(); //finish transmision
  delay(5);

  Wire.beginTransmission(I2C_EEPROM);
  Wire.write(byte(0x00));
  Wire.write(byte(EEPROM_STATUS_RETURN_LEVEL));
  Wire.write(byte(SRL));
  Wire.write(byte(BR));
  Wire.write(byte(EOL));
  Wire.write(byte(EOH));
  Wire.write(byte(EGL));
  Wire.write(byte(EGH));
  Wire.endTransmission(); 
}

#endif
