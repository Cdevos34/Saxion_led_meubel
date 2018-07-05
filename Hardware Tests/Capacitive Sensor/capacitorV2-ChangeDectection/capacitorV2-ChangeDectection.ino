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
int prevAverage;
bool assDetected = false;

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

    for(int i=arraySize-1; i<0; i--){
        
        sensorValues[i] = sensorValues[i-1];
    }
    sensorValues[0]=total1;

    int average = 0;
    for(int i=0; i<arraySize; i++){

    	average += sensorValues[i];
    }
    average = average/10;
    Serial.print(average);

    int change = calibrateIncrease(average, prevAverage);
    prevAverage = average;
    
    Serial.print("                  ");
    Serial.print(change);
    Serial.print("                  ");

    if 	    (change > 50){

        assDetected = true;
        digitalWrite(ledPin, HIGH);
    }
    else if (change < -30){

        assDetected = false;   
        digitalWrite(ledPin, LOW);
    }

    Serial.println(assDetected);
    delay(10);
}


int calibrateIncrease(int newNumber, int oldNumber){

    int change = newNumber - oldNumber;
    return change;
}
//Trigger op de hellings bepalen. Bij een groei van X positief gaat er iemand zitten. Bij een groei van X negatief gaat er iemand opstaan
// (nieuw-oud)/oud *100 = percentage if nieuw - oud is absoluut