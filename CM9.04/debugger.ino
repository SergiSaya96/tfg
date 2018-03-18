#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

Dynamixel Dxl(DXL_BUS_SERIAL1);

int counter;
byte aByte=0;
uint8 aUint8;
bool start=false;
int i=0,lenght=10;

void blinkOnce()
{
  digitalWrite(BOARD_LED_PIN, LOW);
  delay_us(100);
  digitalWrite(BOARD_LED_PIN, HIGH);
}

void setup()
{  
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  pinMode(BOARD_LED_PIN, OUTPUT);
  counter=0;
}


void loop() 
{ 
  if (Dxl.available())
  {
    aByte=Dxl.readRaw();
    blinkOnce();
    SerialUSB.print("0x");
    SerialUSB.print(aByte,HEX);
    SerialUSB.print(";");
    if(aByte==0xFF && start==false){
      start=true;
    }
    if(start){
      i++;
    }
    if(i==4){
      lenght=aByte;
    }
    if(i == lenght+4){
      SerialUSB.print("\t");
      SerialUSB.print(micros());      
      SerialUSB.print("\n");
      start=false;
      i=0;
      lenght=10;
    }
  } 
}
