#include <CapacitiveSensor.h>

/*
 * CapitiveSense Library Demo Sketch
 * Paul Badger 2008
 * Uses a high value resistor e.g. 10M between send pin and receive pin
 * Resistor effects sensitivity, experiment with values, 50K - 50M. Larger resistor values yield larger sensor values.
 * Receive pin is the sensor pin - try different amounts of foil/metal on this pin
 */
const int arraySize = 10;
const int ledPin = 6;
int sensorValues[arraySize];

CapacitiveSensor   cs_4_2 = CapacitiveSensor(4,2);        // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired

void setup()                    
{
   cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
   Serial.begin(9600);

   pinMode(ledPin, OUTPUT);

}

void loop()                    
{
    long start = millis();
    int total1 =  cs_4_2.capacitiveSensor(30);

    //Serial.print(millis() - start);        // check on performance in milliseconds
    //Serial.print("              ");
    //Serial.println(total1);                  // print sensor output 1 - this is also the value you can use to use this in other projects

    for(int i=arraySize-1; i<0; i--){
        
        sensorValues[i] = sensorValues[i-1];
    }
    sensorValues[0]=total1;
    //analogWrite(6, map(constrain(total1, 500, 2000),500,2000,0,255));

    int average = 0;
    for(int i=0; i<arraySize; i++){

    	average += sensorValues[i];
    }
    average = average/10;
    Serial.println(average);
    //delay(10);                             // arbitrary delay to limit data to serial port

    if 	(average > 100){

    	digitalWrite(ledPin, HIGH);
    }
    else{

    	digitalWrite(ledPin, LOW);
    }

    delay(50);
}


int calibrateBaseLine(long interval){

	int baseLine = 0;
	long startTime = millis();

	int rawValue = 0;
	int maxValue = 0;
	int minValue = 0;

	while(millis < (startTime +interval)){
		//read Sensor
		if (rawValue > maxValue){
			
			maxValue = rawValue;
		}
	}



	return baseLine;
}