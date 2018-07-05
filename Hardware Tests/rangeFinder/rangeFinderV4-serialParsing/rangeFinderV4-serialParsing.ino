void setup(){

	Serial.begin(9600);
	Serial3.begin(9600);
}

void loop(){
	int buffer[5];
	int i = 0;
	while(Serial3.available() > 0){
			i++;
		    buffer[i] = Serial3.read();
		    Serial.println(buffer[i],HEX);
	}
	//Serial.println("");
}