#include <Wire.h>
#include <Adafruit_MCP4725.h>

#define PACKET_SIZE 6
#define SIZE_DATA_BUFFER 5

typedef struct BufferItem {
	long timestamp;
	uint16_t voltage;
} BufferItem;


/**
 * Functional Prototypes
 */
void deleteBufferItem(BufferItem syncBuffer[], uint16_t index);
void insert(BufferItem syncBuffer[], long timestamp, uint16_t voltage);
void pushBack(BufferItem* syncBuffer, uint16_t index);
bool isIndexInBounds(uint16_t index);
void sort(BufferItem syncBuffer[]);
uint16_t getCount(BufferItem syncBuffer[]);


/**
 * Global Variables
 */
Adafruit_MCP4725 dacRight;
Adafruit_MCP4725 dacLeft;
bool connectionStatus=false;
char goMessage[2] = {'g', 'o'};
byte rightInput[10];
byte leftInput[10];
uint16_t voltage;
uint16_t lastLeftValue=0;
uint16_t lastRightValue=0;
int16_t voltageDiffer=0;
uint16_t counter=0;
float tstOutput=0;
byte joeyInput[4];
bool firstRun = true;
BufferItem rightBuffer[SIZE_DATA_BUFFER];
BufferItem leftBuffer[SIZE_DATA_BUFFER];
int16_t timestamp=0;
int16_t timestampDiff=0;
int16_t leftCount=0;
int16_t rightCount=0;

void setup() {

	// put your setup code here, to run once:
	pinMode(13, OUTPUT);
	//Karson is Serial RX3
	Serial.begin(9600);
	//Joey is Serial1
	Serial1.begin(57600);
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
	Serial3.write(0x00);
	Serial.println("Starting transmission");
}

void loop() {
	int a;
	
	//Input should be no larger than twenty bytes since thats however many that can sent via BLE
	memset(rightInput,0,sizeof(rightInput)); 
	memset(leftInput,0,sizeof(leftInput));

	while (Serial2.available() >= PACKET_SIZE && 
		getCount(leftBuffer) < SIZE_DATA_BUFFER) {
		
		// Read packets
		for(a=0; a<PACKET_SIZE; a++) {
			leftInput[a] = Serial2.read();
		}
		
		// Send ACK
		Serial2.write(0x00);
		
		voltage = (leftInput[0] * 256) + leftInput[1];

		//find a difference between the previous value 
		voltageDiffer = voltage - lastRightValue;

		//set the last value to the current value
		lastRightValue = voltage;
		
		//add the difference between the two
		voltage = voltage + voltageDiffer;
		
		//add the difference between the two
		voltage = (voltage / 16) + voltageDiffer;
		
		voltage = voltage / 2;
		
		if (voltage < 0) {
			voltage = 0;
		}
		else if (voltage > 4096) {
			voltage = 4095;
		}
		
		// Packet Timestamp
		timestamp = 0;
		timestamp += leftInput[2] << 24;
		timestamp += leftInput[3] << 16;
		timestamp += leftInput[4] << 8;
		timestamp += leftInput[5];
		
		insert(leftBuffer, timestamp, voltage);
	}
	
	while (Serial3.available() >= PACKET_SIZE && 
		getCount(leftBuffer) < SIZE_DATA_BUFFER) {
		
		// Read packets
		for(a=0; a<PACKET_SIZE; a++) {
			rightInput[a] = Serial3.read();
		}
		
		// Send ACK
		Serial3.write(0x00);
		
		voltage = (rightInput[0] * 256) + rightInput[1];

		//find a difference between the previous value 
		voltageDiffer = voltage - lastRightValue;

		//set the last value to the current value
		lastRightValue = voltage;

		//add the difference between the two
		voltage = voltage + voltageDiffer;
		
		//add the difference between the two
		voltage = (voltage / 16) + voltageDiffer;
		
		voltage = voltage / 2;
		
		if (voltage < 0) {
			voltage = 0;
		}
		else if (voltage > 4096) {
			voltage = 4095;
		}
		
		// Packet Timestamp
		timestamp = 0;
		timestamp += rightInput[2] << 24;
		timestamp += rightInput[3] << 16;
		timestamp += rightInput[4] << 8;
		timestamp += rightInput[5];
		
		insert(rightBuffer, timestamp, voltage);
	}
	
	leftCount = getCount(leftBuffer);
	rightCount = getCount(rightBuffer);
	while (leftCount >= 6 && rightCount >= 6) {
		dacRight.setVoltage(rightBuffer[rightCount - 1].voltage, false);
		dacLeft.setVoltage(leftBuffer[leftCount - 1].voltage, false);
		deleteBufferItem(rightBuffer, --rightCount);
		deleteBufferItem(leftBuffer, --leftCount);
	}
	digitalWrite(13, LOW);
	
	if (Serial1.available()) {
		Serial1.readBytes(joeyInput, 4);
		Serial.write(joeyInput, 4);
	}
}

void deleteBufferItem(BufferItem syncBuffer[], uint16_t index) {
	if (syncBuffer == NULL ||
		!isIndexInBounds(index)) {
		return;
	}
	syncBuffer[index].timestamp = -1;
	sort(syncBuffer);
}

void sort(BufferItem syncBuffer[]) {
	int a;
  int b;
	long tTimestamp;
	uint16_t tVoltage;
	if (syncBuffer == NULL) {
		return;
	}
	
	// TODO: Use a better algorithm
	for(a=0; a<=SIZE_DATA_BUFFER; a++) {
		for(b=a+1; b<=SIZE_DATA_BUFFER; b++) {
			if (syncBuffer[a].timestamp < syncBuffer[b].timestamp) {
				tTimestamp = syncBuffer[a].timestamp;
				tVoltage = syncBuffer[a].voltage;
				
				syncBuffer[a].timestamp = syncBuffer[b].timestamp;
				syncBuffer[a].voltage = syncBuffer[b].voltage;
				
				syncBuffer[b].timestamp = tTimestamp;
				syncBuffer[b].voltage = tVoltage;
			}
		}
	}
}

uint16_t getCount(BufferItem syncBuffer[]) {
	int count = 0;
	int a;
	if (syncBuffer == NULL) {
		return 0;
	}
	for(a=0; a<SIZE_DATA_BUFFER; a++) {
		if (syncBuffer[a].timestamp == -1) {
			break;
		}
		count++;
	}
	return count;
}

void insert(BufferItem syncBuffer[], long timestamp, uint16_t voltage) {
	int a;
	for(a=0; a<SIZE_DATA_BUFFER; a++) {
		if (syncBuffer[a].timestamp < timestamp) {
			pushBack(syncBuffer, a);
			syncBuffer[a].timestamp = timestamp;
			syncBuffer[a].voltage = voltage;
			break;
		}
	}
}

void pushBack(BufferItem syncBuffer[], uint16_t index) {
	if (syncBuffer == NULL || 
		!isIndexInBounds(index) ||
		index - 1 < SIZE_DATA_BUFFER ||
		syncBuffer[index].timestamp == -1) {
		return;
	}
	pushBack(syncBuffer, index);
	syncBuffer[index + 1].timestamp = syncBuffer[index].timestamp;
	syncBuffer[index + 1].voltage = syncBuffer[index].voltage;
}

bool isIndexInBounds(uint16_t index) {
	return index >= 0 && index < SIZE_DATA_BUFFER;
}
