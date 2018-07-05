class RangeFinder{
	//declare variables
	int sensorPin;
	int debugLedPin;

	int reading;
	int sampleSize;

	int medianValue;
	int state;

	SimpleKalmanFilter kalmanFilter;

	public:
	RangeFinder( int sensorPin_, int sampleSize_, int debugLedPin_ ):kalmanFilter(10000,10000,0.05) {
		//set variables and one time operations
		sensorPin = sensorPin_;
		debugLedPin = debugLedPin_;
		sampleSize = sampleSize_;

		pinMode(debugLedPin, OUTPUT);
		digitalWrite(debugLedPin, LOW);
	}

	void Update(){
		//read the sensorState "SampleSize" times and store it in an array
		int distanceCm[sampleSize];
		
		for(int i=0; i<sampleSize; i++){
		    reading = analogRead(sensorPin);
		    distanceCm[i] = (reading/2)/0.3937;
		    //distanceCm[i] = reading;
		}

		medianValue = ReturnMedian(distanceCm, sampleSize);

		//set the debug led and the states
		if		(medianValue < 60)	{
			//people are very close to the sensor
			state = 3;
			digitalWrite(debugLedPin, HIGH);
		}
		else if (medianValue > 60 && medianValue < 200)	{
			//people are further then 60 cm roughly
			state = 2;
			digitalWrite(debugLedPin, HIGH);
		}
		else if (medianValue > 200 && medianValue < 400)	{
			state = 1;
			digitalWrite(debugLedPin, HIGH);
		}
		else if (medianValue > 400){

			state = 0;
			digitalWrite(debugLedPin, LOW);
		}	
	}

	int ReturnState(){

		/*****[Debug]*****/
		//Serial.print(sensorPin);
		//Serial.print(" = ");
		//Serial.print(medianValue);
		//Serial.print(" = ");
		//Serial.print(state);
		//Serial.print("    ");

		return state;
	}

	private:
	int ReturnMedian(int values[], int valueAmount){
		//sort the values from small to big using bubble sort (don't use too much values!)
		//then return the median --> the middle reading

		//calculate the amount of passed needed
		int sortAmounts = valueAmount-1;
		//keep track of the amount of passes done to skip the already sorted places
		int count = 0;
		for(int i=0; i<sortAmounts; i++){
			//loop through the array skipping though the places already sorted by substracting the counts and skipping the last value
			for(int i=0; i<valueAmount-count-1; i++){
		    	//compare the current value to the next value
		    	if(values[i] > values[i+1]){
		    		//if the value is bigger, move it up the chain by swapping it
		    		//save the second value temporary
		    		int saveIt = values[i+1];
		    		//overwrite the second value with the first value
		    		values[i+1] = values[i];
		    		//overwrite the first value with the saved value
		    		values[i] = saveIt;
		    	}
		    }
		    //we've just done a pass, so lets record that fact
		    count ++;
		}	

		//return the median
		return values[valueAmount/2+1];
	}
};
