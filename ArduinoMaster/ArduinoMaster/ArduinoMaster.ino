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
 uint16_t lastLeftValue=0;
 uint16_t lastRightValue=0;
 int16_t voltageLeftDiffer=0;
 int16_t voltageRightDiffer=0;
 uint16_t counter=0;
 float tstOutput;
 byte joeyInput[4];
 
void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  //Karson is Serial
  Serial.begin(9600);
  //Joey is Serial1
  Serial1.begin(9600);
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
    //Input should be no larger than twenty bytes since thats however many that can sent via BLE
    memset(rightInput,0,sizeof(rightInput)); 
    memset(leftInput,0,sizeof(rightInput)); 
//    //output the hex values
//   if(Serial2.available()>=2 && Serial3.available()>=2)
//   if(BTSerial.available()>=2 && Serial3.available()>=2)
   if(Serial3.available()>=2) 
   {
        leftInput[0]=Serial2.read();
        leftInput[1]=Serial2.read();
        rightInput[0]=Serial3.read();
        rightInput[1]=Serial3.read();
        Serial2.write(0x00);
        Serial3.write(0x00);
        digitalWrite(13, HIGH);
        digitalWrite(13, LOW);
      
      rightVoltage = (rightInput[0]*256) + rightInput[1];
      leftVoltage = (leftInput[0]*256) + leftInput[1];

      //find a difference between the previous value 
      voltageRightDiffer = rightVoltage-lastRightValue;
      voltageLeftDiffer = leftVoltage-lastLeftValue;

      //set the last value to the current value
      lastLeftValue = leftVoltage;
      lastRightValue = rightVoltage;

      //add the difference between the two
      rightVoltage=rightVoltage+voltageRightDiffer;
      leftVoltage=leftVoltage+voltageLeftDiffer;
            //add the difference between the two
      //differenc
//      rightVoltage=rightVoltage/8;
      rightVoltage=(rightVoltage/16)+voltageRightDiffer;
//      leftVoltage=leftVoltage/8;
//      leftVoltage=leftVoltage+voltageLeftDiffer;
      leftVoltage=(leftVoltage/16)+voltageLeftDiffer;
      rightVoltage=rightVoltage/2;
      leftVoltage=leftVoltage/2;
//        dacLeft.setVoltage(leftVoltage,false);
//      digitalWrite(13,HIGH);
//      Serial.print(leftVoltage);
////      Serial.print("\t");
////      Serial.print(voltageRightDiffer);
//      Serial.print("\t");
//      Serial.println(rightVoltage);
      if(leftVoltage<0)
      {
        leftVoltage=0;
      }
      else if(leftVoltage>4096)
      {
        leftVoltage=4095;
      }
      if(rightVoltage<0)
      {
        rightVoltage=0;
      }
      else if(rightVoltage>4096)
      {
        rightVoltage=4095;
      }
//
      dacRight.setVoltage(rightVoltage, false);
      dacLeft.setVoltage(leftVoltage, false);
//      tstOutput=rightVoltage*.0008;
//      Serial.println(tstOutput);
//      digitalWrite(13,LOW);
      
//      dacLeft.setVoltage(pgm_read_word(&(DACLookup_FullSine_9Bit[counter])), false);
//      counter++;
//        if(counter==512)
//        {
//          counter=0;
//        }
//      Serial.print(rightInput[0], HEX);
//      Serial.print("\t");
//      Serial.println(rightInput[1], HEX);
//      Serial.flush();
//     Serial.print("\t");
//      Serial.println(leftVoltage);
//      }
//        Serial1.readBytes(joeyInput, 4);
//        Serial.write(joeyInput,4);
   }
//   else
//   {
//  delay(100);
//  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(100);              // wait for a second
//  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
////  delay(100);     
//  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
////  delay(100);              // wait for a second
//  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
//  delay(100);  
//   }
      if(Serial1.available())
      {
      Serial1.readBytes(joeyInput, 4);
      Serial.write(joeyInput,4);
      }
}
