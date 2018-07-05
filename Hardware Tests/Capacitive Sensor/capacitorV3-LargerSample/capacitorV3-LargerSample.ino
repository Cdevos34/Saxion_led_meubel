#include "CapacitiveSensor.h"

/*
 * CapitiveSense Library Demo Sketch
 * Paul Badger 2008
 * Uses a high value resistor e.g. 10M between send pin and receive pin
 * Resistor effects sensitivity, experiment with values, 50K - 50M. Larger resistor values yield larger sensor values.
 * Receive pin is the sensor pin - try different amounts of foil/metal on this pin
 */
const int arraySize = 10;
const int ledPin = 2;
int sensorValues[arraySize];
int prevCoupling;
bool assDetected = false;

CapacitiveSensor cs_4_2 = CapacitiveSensor(19,20);        // 10M resistor between pins 4 & 2, pin 2 is sensor pin, add a wire and or foil if desired

void setup()                    
{
   //cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1 - just as an example
   Serial.begin(19200);

   pinMode(ledPin, OUTPUT);

}

void loop()                    
{
    //long start = millis();
    int coupling =  cs_4_2.capacitiveSensor(80);

    Serial.println(coupling);

    int change = calibrateIncrease(coupling, prevCoupling);
    prevCoupling = coupling;
    
    Serial.print("                  ");
    Serial.print(change);
    Serial.print("                  ");

    if 	    (change > 4000){

        assDetected = true;
        digitalWrite(ledPin, HIGH);
        delay(1000);
        //CHANGE!!
    }
    else if (change < -4000){

        assDetected = false;   
        digitalWrite(ledPin, LOW);
        delay(1000);
        //CHANGE!!!
    }

    Serial.println(assDetected);
    delay(10);
    
    delay(100);
}


int calibrateIncrease(int newNumber, int oldNumber){

    int change = newNumber - oldNumber;
    return change;
}
//Trigger op de hellings bepalen. Bij een groei van X positief gaat er iemand zitten. Bij een groei van X negatief gaat er iemand opstaan
// (nieuw-oud)/oud *100 = percentage if nieuw - oud is absoluut