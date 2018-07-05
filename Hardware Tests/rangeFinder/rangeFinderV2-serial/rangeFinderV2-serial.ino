int incomingByte = 0;

void setup(){

	Serial.begin(9600);
	Serial3.begin(9600);
}

void loop(){
	if(Serial3.available() > 0){

		incomingByte = Serial3.read();
		Serial.println(incomingByte);
	}
}