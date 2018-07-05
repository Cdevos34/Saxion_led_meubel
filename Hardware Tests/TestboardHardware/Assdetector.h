#include "CapacitiveSensor.h"
#include "SimpleKalmanFilter.h"

class AssDetector{

	int debugLedPin;
	int ledState;

	int currentCoupling;
	int prevCoupling;

	int assDetected;
	int thresholdHard;
	int thresholdChange;

	unsigned long interval;
	unsigned long previousTime;
	
	CapacitiveSensor cs;
	SimpleKalmanFilter kalmanFilter;

	public:
	AssDetector( int sendPin_, int receivePin_, int debugLedPin_): cs(sendPin_,receivePin_), kalmanFilter(10000,10000,0.05){
		//kalmanFilter(2000,2000,0.05)
		debugLedPin = debugLedPin_;
		
		currentCoupling = 0;
		prevCoupling = 0;

		ledState = LOW;
		assDetected = LOW;	

		//interval =  600000;

		thresholdChange = 2000;
		//turn the led off
		pinMode(debugLedPin, OUTPUT);
		digitalWrite(debugLedPin, ledState);
		
		//Turn off autocalibration
		cs.set_CS_AutocaL_Millis(0xFFFFFFFF);

	}

	void UpdateHard(){
		//Use either this one to detect the asses not both!
		//make a snapshot of the time
		//unsigned long currentTime = millis();

		//calculate the current coupling value filtering using kalman
		currentCoupling = kalmanFilter.updateEstimate(cs.capacitiveSensor(20));

		if(currentCoupling > thresholdHard){
			//detection
			assDetected = HIGH;
			ledState = HIGH;
		}

		else if(currentCoupling < thresholdHard){

			assDetected = LOW;
			ledState = LOW;

			//if((unsigned long)(currentTime - previousTime) >= interval){
				//If the interval time has passed

				//Update the threshold hard
			//	thresholdHard = (0.4*currentCoupling)+currentCoupling;
			//	previousTime = currentTime;
				//Save the current time for the next interval
			//}
		}

		digitalWrite(debugLedPin, ledState);
	}

	void UpdateChange(){
		//Or use this one to detect the asses not both!

		//calculate the current coupling value filtering using kalman
		currentCoupling = kalmanFilter.updateEstimate(cs.capacitiveSensor(20));

		int change = currentCoupling - prevCoupling;

		if 		(change > thresholdChange){

			assDetected = HIGH;
			ledState = HIGH;
		}
		else if(change < -thresholdChange){

			assDetected = LOW;
			ledState = LOW;
		}

		prevCoupling = currentCoupling;
		digitalWrite(debugLedPin, ledState);
	}

	int ReturnState(){

		//return currentCoupling;
		return assDetected;
	}

	int DebugValues(){

		return currentCoupling;
	}

	void CalculateTreshold(){
		//threshold should be an increase of 20%
		//fill the kalmanFilter with data

		digitalWrite(debugLedPin, HIGH);
		for(int i=0; i<50; i++){
			currentCoupling = kalmanFilter.updateEstimate(cs.capacitiveSensor(20));
			delay(100);  
		}
		
		//calculate the threshold
		thresholdHard = (0.7*currentCoupling)+currentCoupling;
		//thresholdHard = (currentCoupling + 2000);
		digitalWrite(debugLedPin, LOW);
	}

};