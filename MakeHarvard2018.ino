int FrontA_AIN1 = 13;
int FrontA_AIN2 = 2;
int FrontB_BIN1 = 12;
int FrontB_BIN2 = 4;

int RearA_AIN1 = 6;
int RearA_AIN2 = 7;
int RearB_BIN1 = 8;
int RearB_BIN2 = 9;

int FrontA_PWM = 10;
int FrontB_PWM = 3;

int RearA_PWM = 5;
int RearB_PWM = 11;

int incomingByte = 0;
int speed = 0;

// Arduino addressing is using 7bit
// All the addresses were calculated in 8bit
// And were right shifted when actually used

#include <Wire.h>

#define MAX_STRIPS 8
#define MAX_SENSORS 1

#define NUM_SENSORS MAX_STRIPS*MAX_SENSORS // reserve addresses for 8 strips with 6 sensors on each
#define PRECISION 0

#define FREESCALE_ADDRESS 0xC0
#define SENSOR_ALL_ON 0x0C
#define SENSOR_ALL_OFF 0x0D


float a0[NUM_SENSORS];
float b1[NUM_SENSORS];
float b2[NUM_SENSORS];
float c12[NUM_SENSORS];

byte addressArray[NUM_SENSORS];
byte addressLength;

float pressureHistory[NUM_SENSORS];
boolean flagHistoryExists=false;

boolean flagShowAddress=false;
boolean flagShowPressure=true;
boolean flagShowTemperature=false;

void initialize() {
    // s 0C
  Wire.beginTransmission(SENSOR_ALL_ON>>1);
  Wire.endTransmission();
  
  // s C0 12 01
  Wire.beginTransmission(0xC0>>1);
  Wire.write(0x12);
  Wire.write(0x01);
  Wire.endTransmission();
  
  // s 0D
  Wire.requestFrom(SENSOR_ALL_ON>>1, 1);
  
  delay(5);
}

void readCoeffs(byte addressSensor, byte num) {
  
  // Select sensor
  Wire.beginTransmission(addressSensor>>1);
  Wire.endTransmission();
  
  // Request coefficients
  Wire.beginTransmission(FREESCALE_ADDRESS>>1);
  Wire.write(0x04);
  Wire.endTransmission();
  Wire.requestFrom(FREESCALE_ADDRESS>>1, 8);
  int16_t a0coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  int16_t b1coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  int16_t b2coeff = (( (uint16_t) Wire.read() << 8) | Wire.read());
  int16_t c12coeff = (( (uint16_t) (Wire.read() << 8) | Wire.read())) >> 2;
  // Turn sensor off
  Wire.requestFrom(addressSensor>>1, 1);
  
  a0[num] = (float)a0coeff / 8;
  b1[num] = (float)b1coeff / 8192;
  b2[num] = (float)b2coeff / 16384;
  c12[num] = (float)c12coeff;
  c12[num] /= 4194304.0;
}


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  for (int i = 2; i <=9; i++)
  {
    pinMode(i, OUTPUT);
  }
  checkAddresses(); // check how many sensors are connected
 
  // for each found sensor, read the coefficients ..
  for(int i=0;i<addressLength;i++) {
    readCoeffs(addressArray[i],i);
  }
}

void readData(byte addressSensor, float* oTemp, float* oPressure)
{
  // Select sensor
  Wire.beginTransmission(addressSensor>>1);
  Wire.endTransmission();

  // Request P/T data
  Wire.beginTransmission(FREESCALE_ADDRESS>>1);
  Wire.write((byte)0x00);
  Wire.endTransmission();

  Wire.requestFrom(FREESCALE_ADDRESS>>1, 4);
  uint16_t pressure = (( (uint16_t) Wire.read() << 8) | Wire.read()) >> 6;
  uint16_t temp = (( (uint16_t) Wire.read() << 8) | Wire.read()) >> 6;
  
  // Turn sensor off
  Wire.requestFrom(addressSensor>>1, 1);

  // ------ Ignore the calibrations for the moment
  
  //float pressureComp = a0[addressSensor] + (b1[addressSensor] + c12[addressSensor] * temp) * pressure + b2[addressSensor] * temp;

  // Calculate temp & pressure
  //*oPressure = ((65.0F / 1023.0F) * pressureComp) + 50.05F; // kPa
  //*oTemp = ((float) temp - 498.0F) / -5.35F + 25.0F; // C
  
  // ------ 

  *oPressure = pressure;
  Serial.print("Pressure is: ");
  Serial.println(pressure);
  //*oTemp = temp;
}

void checkAddresses()
{
  addressLength=0;
  int temp_add=0;
  // check every strip
  for (int strip_n=0;strip_n<MAX_STRIPS;strip_n++){
    // check every sensor
    for (int sensor_n=0;sensor_n<MAX_SENSORS;sensor_n++){
      temp_add=(strip_n<<4)+sensor_n*2; // calculate the address

      // check if the Attiny responds with its address
      // this also switches ON the Chip Select line on the desired sensor
      Wire.beginTransmission(temp_add>>1); // take into account that its 7bit !
      // if response finishes with an ACK then continue
      if (Wire.endTransmission()==0) {
        // check if there is a sensor on this line
        Wire.beginTransmission(FREESCALE_ADDRESS>>1);
        // if there is an ACK then there is a sensor
        if (Wire.endTransmission()==0){
          addressArray[addressLength]=temp_add;
          addressLength++;
          Serial.println("Added sensor");
        }
        // Turn off the chip select line
        Wire.requestFrom(temp_add>>1, 1);
      }
    }
    //Serial.println(']');
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  float oTemp=0;
  float oPressure=0;
  float p_current=0;
  float p_history=0;
  float delta_up=0;
  float delta_down=0;
  
  initialize();
 
  //Serial.print('[');
  for(int i=0;i<addressLength;i++)
  {
    //Serial.println("Is i ever > 1");
    if (i>0){
          Serial.print(',');
    }
    readData(addressArray[i], &oTemp, &oPressure);

    // the calculations of the wrapping
    // that are used to calculate the minus signs
    if (flagHistoryExists){
      p_current=oPressure;
      p_history=pressureHistory[i];
      delta_up=p_current-p_history;
      delta_down=p_history-(p_current-1024);
      if (delta_up<delta_down){
        oPressure=p_history+delta_up;
      }else{
        oPressure=p_history-delta_down;
      }
    }
    pressureHistory[i]=oPressure;
    
    // ------------------------------
    // Start output to the serial port
    
   // Serial.print('[');
    
   // Serial.print('[');
   
    // Print out sensor ID value if the flag was set
//    if (flagShowAddress){
//      Serial.print(addressArray[i],HEX);
//    }

//    // Print out Pressure values if the flag was set
//    if (flagShowPressure){
//      if (flagShowAddress){
//        Serial.print(',');
//      }
      //Serial.print("Pressure is::: ");
      //Serial.println(oPressure,PRECISION);
//    }
//
//    // Print out Temperature values if the flag was set
//    if (flagShowTemperature){
//      if (flagShowPressure){
//        Serial.print(',');
//      }
//    Serial.print(oTemp,PRECISION);
//    }
//    Serial.print(']');
//  }
//  Serial.println(']');
// 
//    // End output to the serial port
//    // ------------------------------
    flagHistoryExists=true;
  }
  speed = map(oPressure, 300, -28, 0, 255);
  //Serial.print("Speed is: ");
  //Serial.println(speed);
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
    if (incomingByte == 'f')
     {
        forward();
     }
    if (incomingByte == 'b')
     {
        backward();
     }
    if (incomingByte == 's')
    {
      stop();
    }  
    if (incomingByte == 'r')
    {
      right();  
    }
    if (incomingByte == 'l')
    {
      left();
    }
  }
}

void forward()
{
  analogWrite(FrontA_PWM, speed);
  analogWrite(FrontB_PWM, speed);

  analogWrite(RearA_PWM, speed);
  analogWrite(RearB_PWM, speed);
  
  analogWrite(FrontA_AIN1, LOW);
  digitalWrite(FrontA_AIN2, HIGH);
  
  digitalWrite(FrontB_BIN1, LOW);
  digitalWrite(FrontB_BIN2, HIGH);

  digitalWrite(RearA_AIN1, LOW);
  digitalWrite(RearA_AIN2, HIGH);

  digitalWrite(RearB_BIN1, LOW);
  digitalWrite(RearB_BIN2, HIGH);
}

void backward()
{

  analogWrite(FrontA_PWM, speed);
  analogWrite(FrontB_PWM, speed);

  analogWrite(RearA_PWM, speed);
  analogWrite(RearB_PWM, speed);
  
  digitalWrite(FrontA_AIN1, HIGH);
  digitalWrite(FrontA_AIN2, LOW);
  
  digitalWrite(FrontB_BIN1, HIGH);
  digitalWrite(FrontB_BIN2, LOW);

  digitalWrite(RearA_AIN1, HIGH);
  digitalWrite(RearA_AIN2, LOW);

  digitalWrite(RearB_BIN1, HIGH);
  digitalWrite(RearB_BIN2, LOW);
}

void stop()
{
  digitalWrite(FrontA_AIN1, HIGH);
  digitalWrite(FrontA_AIN2, HIGH);
  
  digitalWrite(FrontB_BIN1, HIGH);
  digitalWrite(FrontB_BIN2, HIGH);
  
  digitalWrite(RearA_AIN1, HIGH);
  digitalWrite(RearA_AIN2, HIGH);
  
  digitalWrite(RearB_BIN1, HIGH);
  digitalWrite(RearB_BIN2, HIGH);
}


void right()
{
  //digitalWrite(Rear_AIN1, LOW);
  //digitalWrite(Rear_AIN2, HIGH);
}

void left()
{
  //digitalWrite(Rear_AIN1, HIGH);
  //digitalWrite(Rear_AIN2, LOW);
}




