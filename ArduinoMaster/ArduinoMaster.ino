#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(11, 10);
 Adafruit_MCP4725 dacRight;
 Adafruit_MCP4725 dacLeft;
 bool connectionStatus=false;
 char goMessage[2] = {'g', 'o'};
 byte rightInput[10];
 byte leftInput[10];
 uint16_t rightVoltage;
 uint16_t leftVoltage;
 uint16_t rightMSB;
 uint16_t rightLSB;
 uint16_t counter=0;

 
void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  
  Serial.begin(9600);
  //Serial2 is Left RX1 of BOARD
  Serial2.begin(9600);
//  BTSerial.begin(9600);
  //Serial3 is Right RX2 of BOARD
  Serial3.begin(9600);
  
  delay(100);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(100);     
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(100);     
  //Start the write address(Jumper from A0 to VDD)
  dacRight.begin(0x62);
  dacLeft.begin(0x63);
  Serial2.print("go");
//  BTSerial.print("go");
  Serial3.print("go");
  Serial2.write(0x00);
//  BTSerial.print(0);
  Serial3.write(0);
  Serial.println("Starting transmission");
}

void loop() {
  // put your main code here, to run repeatedly:
    //Serial2.print("go");
    //Input should be no larger than twenty bytes since thats however many that can sent via BLE
    memset(rightInput,0,sizeof(rightInput)); 
    memset(leftInput,0,sizeof(rightInput)); 
//    //output the hex values
//    Serial.print(Serial.read(), HEX);
   if(Serial2.available()>=2 && Serial3.available()>=2)
//   if(BTSerial.available()>=2 && Serial3.available()>=2) 
   {
//      digitalWrite(13, HIGH);  
//      Serial2.readBytes(rightInput,2);
        leftInput[0]=Serial2.read();
        leftInput[1]=Serial2.read();
//        rightInput[0]=BTSerial.read();
//        rightInput[1]=BTSerial.read();
//      Serial3.readBytes(leftInput,6);
        rightInput[0]=Serial3.read();
        rightInput[1]=Serial3.read();
        Serial2.write(0x00);
        Serial3.write(0x00);
        digitalWrite(13, HIGH);
//        BTSerial.print(0);
          digitalWrite(13, LOW);
       
//    if(rightInput[0] == 0xFF && rightInput[3] == 0xFF)
//    {
//      rightMSB = ((rightInput[0] << 8) & 0xFF00) | (rightInput[1] & 0x00FF);
      
//      rightLSB = ((rightInput[2] << 8) & 0xFF00) | (rightInput[3] & 0x00FF);
//      rightVoltage=(rightMSB*256)+rightLSB;
//      float voltage = rightMSB * .0000625;
      rightVoltage = (rightInput[0]*256) + rightInput[1];
      leftVoltage = (leftInput[0]*256) + leftInput[1];
//      Serial.println(voltage);
//      leftVoltage=leftVoltage/16;
      Serial.print(leftVoltage);
      Serial.print("\t");
      Serial.println(rightVoltage);
//      rightVoltage=rightVoltage/16;
      if(rightMSB < 60000)
      {
//      dacRight.setVoltage(rightVoltage, false);
//      Serial.print(rightInput[0], HEX);
//      Serial.print("\t");
//      Serial.println(rightInput[1], HEX);
//      Serial.flush();
//     Serial.print("\t");
//      Serial.println(leftVoltage);
      }
   }
}
