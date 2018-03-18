//openCM IDE

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

Dynamixel Dxl(DXL_BUS_SERIAL1);
byte ID_CM=4;
byte aByte=0;
bool start=false;
int i=0,lenght=10;
double off=0;

byte message[24];

uint8 rep[]={
  255, 255, 4, 2, 0, 249j};

void blinkOnce()
{
  digitalWrite(BOARD_LED_PIN, LOW);
  delay_us(20);
  digitalWrite(BOARD_LED_PIN, HIGH);
}
void Ping(){
  for(int j=0; j<=5; j++){
    Dxl.writeRaw(rep[j]);
  }
  //Dxl.writeWord(ID_CM, 0, 0);
}
boolean CheckID(byte m[24]){
  bool match=false;
  if(m[2]==ID_CM)
    match=true;
  return match;
}

void setup()
{  
  // Dynamixel 2.0 Protocol -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  pinMode(BOARD_LED_PIN, OUTPUT);
}


void loop() 
{ 
  if (Dxl.available())
  {
    //blinkOnce();
    message[i]=Dxl.readRaw();
    if(message[i]==0xFF && start==false){
      start=true;
      off=micros();
    }
    SerialUSB.print("0x");
    SerialUSB.print(message[i],HEX);
    SerialUSB.print(";");
    if(i==3){
      lenght=message[i];
    }
    if(start){
      i++;
    }
    if(i == lenght+4){
      SerialUSB.print("\n"); 
      start=false;
      i=0;
      lenght=10;
      if(CheckID(message)){
        //SerialUSB.println("MMMMMMMMMMMMMMMM");
        delay_us(800);
        Ping();
      }
    }
  } 
}



