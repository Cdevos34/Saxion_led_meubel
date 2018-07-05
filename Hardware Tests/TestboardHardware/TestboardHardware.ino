//#include "AssdetectorButton.h"
#include "Assdetector.h"
#include "RangeFinder.h"

/*****[Collection of Assdetectors]*****/
AssDetector buttox1(19,21,2);
AssDetector buttox2(19,22,3);
AssDetector buttox3(19,23,4);
AssDetector buttox4(19,20,5);

/*****[Collection of legDetectors]*****/
RangeFinder legDetector1(14, 50, 6);
RangeFinder legDetector2(15, 50, 7);
RangeFinder legDetector3(16, 50, 8);
RangeFinder legDetector4(17, 50, 9);

/*****[Payload that needs to be sent to the ledcontroller]*****/
const int payLoadLength = 10;
uint8_t payLoad[payLoadLength] = {
	
	'*', 	//Start of message
	0,		// 0 or 1 -- someone is sitting down on the outmost left spot
	0,		// 0 or 1 -- someone is sitting down on the middle left spot
	0,		// 0 or 1 -- someone is sitting down on the middle right spot
	0,		// 0 or 1 -- someone is sitting down on the outmost right spot
	0,		// 0 to 3 -- Rangefinder detection level outmost left sensor
	0,		// 0 to 3 -- Rangefinder detection level middle left sensor
	0,		// 0 to 3 -- Rangefinder detection level middle right sensor
	0,		// 0 to 3 -- Rangefinder detection level outmost right sensor
	'$'		//End of message
};

/*****[Timer Variables]*****/
unsigned int messageInterval = 100;
unsigned long previousTimeMessage = 0;

void setup(){

	Serial.begin(19200);
	Serial1.begin(19200);


	//only for hard value threshold
	buttox1.CalculateTreshold();
	buttox2.CalculateTreshold();
	buttox3.CalculateTreshold();
	buttox4.CalculateTreshold();

	//Serial.print("ready");
}

void loop(){
	unsigned long currentTime = millis();

	legDetector1.Update();
	legDetector2.Update();
	legDetector3.Update();
	legDetector4.Update();

	//use for hard value threshold
	buttox1.UpdateHard();
	buttox2.UpdateHard();
	buttox3.UpdateHard();
	buttox4.UpdateHard();

	composePayload();
	
	if((unsigned long)(currentTime - previousTimeMessage) >= messageInterval){
		//If the intervaltime has passed
		sendPayload();
		//Save the current time for the next interval
		previousTimeMessage = currentTime;
	}


	/*****[Debug part]*****/
	Serial.print(buttox1.DebugValues());
	Serial.print("\t");
	Serial.print(buttox2.DebugValues());
	Serial.print("\t");
	Serial.print(buttox3.DebugValues());
	Serial.print("\t");
	Serial.println(buttox4.DebugValues());
}

void  composePayload(){

	payLoad[1] = buttox1.ReturnState();
	payLoad[2] = buttox2.ReturnState();
	payLoad[3] = buttox3.ReturnState();
	payLoad[4] = buttox4.ReturnState();
	payLoad[5] = legDetector1.ReturnState();
	payLoad[6] = legDetector2.ReturnState();
	payLoad[7] = legDetector3.ReturnState();
	payLoad[8] = legDetector4.ReturnState();
	
	//buttox.ReturnState();	
	//Debug for Rangefinder Values
	//Serial.println();
}

void sendPayload(){

	for(int i=0; i<payLoadLength; i++){
	   Serial1.write(payLoad[i]);

	   /*****[Debug]*****/
	   //Serial.print(payLoad[i]);
	  // Serial.print(" ");
	}
	//Serial.println();	
}