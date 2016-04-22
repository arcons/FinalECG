/*

 This example code is in the public domain.
 
 */
 
 /*
 Name: WWECGRIGHT
 MAC: 74DAEAA8A790
 IMME: 0
 PIN: Default 
 */ 

#include <Wire.h>
#define LED RED_LED
#define Addr 0x48

boolean start = false;
char recMessage[2];
//ASCII for go
char goMessage[2] = {'g', 'o'};
byte adcData[2];
byte outputData[6];
unsigned long timeStamp;
uint16_t voltage;
uint16_t temp;

void setup()  
{

  // Open serial communications and wait for port to open:
  pinMode(LED, OUTPUT);  
  Serial.begin(9600);
  delay(100);
  Wire.begin();
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select configuration register
  Wire.write(0x01);
  // AINP = AIN0 and AINN = AIN1, +/- 2.048V
  Wire.write(0x80);
  // Continuous conversion mode, 250 SPS
  Wire.write(0xC3);
  //Allow the LED to output
  Wire.endTransmission();
  //Check for reset
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
  //ads.startComparator_SingleEnded(0, 1000);
  delay(300);
}

void loop() // run over and over
{
  if(start)
  {
    
    //Set the LED High
    digitalWrite(LED, HIGH);
//    //Serial.println("Start I2C Transmission");
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    //Serial.println("Write Select Data Register");
    Wire.write(0x00);
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 2 bytes of data
    Wire.requestFrom(Addr, 2);

    // Read 2 bytes of data
    // raw_adc msb, raw_adc lsb
     if(Wire.available() == 2)
      {
        outputData[0] = Wire.read();//MSB
        outputData[1] = Wire.read();//LSB
        
        timeStamp = millis();
        outputData[2] = (timeStamp  >> 24) & 0x000000FF;
        outputData[3] = (timeStamp  >> 16) & 0x000000FF;
        outputData[6] = (timeStamp  >> 8) & 0x000000FF;
        outputData[5] = timeStamp & 0x000000FF;
      }
     if(Serial.available()>=1)
        {
        Serial.write(outputData, 6);  
        digitalWrite(LED, LOW);
        Serial.read();
        }
//      }  
//    //Develop a recognized disconnect method
//    if(!Serial.available())
//    {
//      start=false;
//    }
  }
  
  else
  {
//Serial.println("Output LED");
    digitalWrite(LED, LOW);
    delay(500);
    digitalWrite(LED, HIGH);
  }
  
  if(Serial.available()>=2 && !start)
  {
    Serial.readBytes(recMessage, 2);
    //Check if the go message was received
    if(recMessage[0] == goMessage[0]  && recMessage[1] == goMessage[1]) 
    {
      //Start the run
      start = true;
      digitalWrite(LED,HIGH);
      delay(3000);
      digitalWrite(LED,LOW);
      
    }
  }
}


