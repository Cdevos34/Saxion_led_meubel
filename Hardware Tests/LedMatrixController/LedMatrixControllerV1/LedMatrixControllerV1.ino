#include <OctoWS2811.h>

const int  LEDAMOUNT = 209;
//Gaat uiteindelijk naar 908 oid

DMAMEM int displayMemory[LEDAMOUNT*6];
int drawingMemory[LEDAMOUNT*6];

const int config = WS2811_GRB | WS2811_800kHz;

OctoWS2811 leds(LEDAMOUNT, displayMemory, drawingMemory, config);

void setup(){
	//Initialize the library
	leds.begin();
	//Update the led's Turning them all to black/off
	leds.show();	
}

void loop(){
	
	for(int i=0; i<LEDAMOUNT; i++){
	    
	    leds.setPixel(i,255,0,0);
      leds.show(); 
  delay(10);
	}
  
	for(int i=0; i<LEDAMOUNT; i++){
	    
	    leds.setPixel(i,0,255,0);
     leds.show(); 
  delay(10);
	}
 
	for(int i=0; i<LEDAMOUNT; i++){
	    
	    leds.setPixel(i,0,0,255);
     leds.show(); 
 delay(10);
	}
}
