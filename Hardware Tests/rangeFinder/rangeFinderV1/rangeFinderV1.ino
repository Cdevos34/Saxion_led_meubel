const int rangeFinderPin = A9;
int range = 0;

void setup(){

	Serial.begin(9600);
}

void loop(){
	
	range = analogRead(rangeFinderPin);
	//Serial.print("AnalogReading: ");
	Serial.println(range);
	delay(20);
} 