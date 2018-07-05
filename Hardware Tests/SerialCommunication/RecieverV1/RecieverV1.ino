const int incomingMessagesSize = 10;


unsigned int messageInterval = 500;
unsigned long previousTime = 0;
int count = 0;


void setup(){
//Uiteindelijk moet de serial op serial1 werken omdat de serial3 poorten bezet zijn door de leddrivers
	Serial.begin(115200);
	Serial1.begin(115200);
	Serial.println("Ready for takeoff");
}

void loop(){
	count = 0;
	while(Serial1.available() > 0){

		int incomingMessages[incomingMessagesSize];
		incomingMessages[count] = Serial1.read();

		Serial.print(count);
		Serial.print("    ");
		Serial.print(incomingMessages[count]);
		count++;

		//		if(incomingMessages[9] == 36){
			//Serial.println("GOT DOLLAH");		
		//			for(int i=0; i<incomingMessagesSize; i++){
			 
		    	//Serial.print(incomingMessages[i]);
		    //	Serial.print(',');
		//			}
			//Serial.println();
		//			count = 0;
		//		}
	}	
}