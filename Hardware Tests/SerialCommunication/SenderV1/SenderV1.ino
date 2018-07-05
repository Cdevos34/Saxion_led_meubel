const int payLoadLength = 10;
uint8_t payLoad[] = {
	
	'*', 	//Start of message
	0,		// 0 or 1 -- someone is sitting down on the outmost left spot
	0,		// 0 or 1 -- someone is sitting down on the middle left spot
	0,		// 0 or 1 -- someone is sitting down on the middle right spot
	0,		// 0 or 1 -- someone is sitting down on the outmost right spot
	0,		// 0 to 3 -- Rangefinder detection level outmost left sensor
	1,		// 0 to 3 -- Rangefinder detection level middle left sensor
	2,		// 0 to 3 -- Rangefinder detection level middle right sensor
	3,		// 0 to 3 -- Rangefinder detection level outmost right sensor
	'$'		//End of message
};

unsigned int messageInterval = 200;
unsigned long previousTime = 0;

void setup(){
	
	Serial1.begin(19200);
}

void loop(){


	unsigned long currentTime = millis();
	if((unsigned long)(currentTime - previousTime) >= messageInterval){
		//If the intervaltime has passed
		sendPayload();
		//Save the current time for the next interval
		previousTime = currentTime;
	}
}

void sendPayload(){

	for(int i=0; i<payLoadLength; i++){
	   Serial1.write(payLoad[i]);
	}
}
