class AssDetectorButton{

	int sensorPin;
	int debugLedPin;

	int ledState;
	int sensorState;
	int lastSensorState;
	int reading;

	unsigned long lastDebounceTime;
	unsigned long debounceDelay;

	public:
	AssDetectorButton( int sensorPin_, int debugLedPin_){

		sensorPin = sensorPin_;
		debugLedPin = debugLedPin_;

		ledState = HIGH;
		lastSensorState = HIGH;

		lastDebounceTime = 0;
		debounceDelay = 50;

		//turn the led off
		pinMode(sensorPin, INPUT);
		pinMode(debugLedPin, OUTPUT);
		digitalWrite(debugLedPin, ledState);
	}

	void Update(){
		//read the sensorState
		reading = digitalRead(sensorPin);
		//Serial.println(sensorPin);
		//check to see if you just pressed the button
		if( reading != lastSensorState){
			//record the time
			lastDebounceTime = millis();
		}
	
		if ((millis() - lastDebounceTime) > debounceDelay) {
			//if the button has been pressed longer then the delay 
			//(so a human pressed the button and its not bouncing)
	
			//if the buttonState has changed
			if(reading != sensorState) {
				sensorState = reading;
	
				if(sensorState == HIGH){
					ledState = !ledState;
				}
			}
		}
	
		digitalWrite(debugLedPin, ledState);
	
		lastSensorState = reading;
	}

	int ReturnState(){

		return ledState;
	}
};