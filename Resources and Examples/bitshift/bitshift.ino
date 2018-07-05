int red = 170;
int green = 236;
int blue = 103;

void setup(){

	Serial.begin(9600);	
	

	seperateColors(combineColors(red, green, blue));
}

void loop(){
	


}

uint32_t combineColors( uint32_t red, uint32_t green, uint32_t blue ){
	//Function that bitshifts three 8-bit values into one 32-bit value so you can send one value instead of 3
	//Create a 32-bit unsigned integer (it has room voor 32 bits and cannot go below 0)
	uint32_t oneBigColor = 0;

	oneBigColor = oneBigColor |(red << 16);
	oneBigColor = oneBigColor |(green << 8);
	oneBigColor = oneBigColor |(blue);
	//shift green into the thing 
	Serial.println(oneBigColor);
	
	return oneBigColor;
}

void seperateColors(uint32_t combinedColor){

	uint8_t extractedRed = combinedColor >> 16;
	uint8_t extractedGreen = combinedColor >> 8;
	uint8_t extractedBlue = combinedColor;

	Serial.println(extractedRed);
	Serial.println(extractedBlue);
	Serial.println(extractedGreen);
}