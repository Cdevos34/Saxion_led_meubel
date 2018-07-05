const int incomingMessagesSize = 10;

unsigned int intervalTime = 500;
unsigned long previousTime = 0;

const int sensorDataSize = incomingMessagesSize - 2;
byte sensorData[sensorDataSize];


void setup(){
//Uiteindelijk moet de serial op serial1 werken omdat de serial3 poorten bezet zijn door de leddrivers
	Serial.begin(19200);
	Serial1.begin(19200);
	Serial.println("Ready for takeoff");
}

void loop(){
	
	readoutSerial();
}

void readoutSerial(){
	byte incomingMessage[incomingMessagesSize];
	if(Serial1.available() > 0){
		
		Serial1.readBytesUntil('$', incomingMessage, incomingMessagesSize);
	}

	if(incomingMessage[0] == '*'){
		// incoming message seems to conform to the protocol -> we want to save that
		for(int i=0; i<sensorDataSize; i++){
		    sensorData[i] = incomingMessage[i+1];
		    
		}
		printOutSensorData();
	}
}

void printOutSensorData(){

	for(int i=0; i<sensorDataSize; i++){
		    Serial.print(sensorData[i]);
		    Serial.print(" ");
		}
		Serial.println();
}