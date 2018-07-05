#include <OctoWS2811.h>

const int  LEDAMOUNT = 203;
const int LEDSHOR =  29;
const int LEDSVERT = 7;
//Gaat uiteindelijk naar 908 oid

//Array to store all the colors
uint32_t currentColors[LEDSHOR][LEDSVERT];

DMAMEM int displayMemory[LEDAMOUNT*6];
int drawingMemory[LEDAMOUNT*6];
const int config = WS2811_GRB | WS2811_800kHz;
OctoWS2811 leds(LEDAMOUNT, displayMemory, drawingMemory, config);

//Global variables for the framerate
unsigned int frameRate = 10;
unsigned int interval= (10000/frameRate);
unsigned long previousMillis = 0;
bool ledsApplied = true;

//animation variables
int count = 0;
int switcher = 1;

void setup()
{
  //Initialize the library
  leds.begin();
  //Update the led's Turning them all to black/off
  leds.show(); 
  Serial.begin(9600);
}

void applyColorsToStrip(){//uint32_t colorArray[LEDSHOR][LEDSVERT], int arrayLength){

  Serial.println("start apply colors");
  
  for(int y=0; y<LEDSVERT; y++){
      for(int x=0; x<LEDSHOR; x++){
          //fetch the colors    
          uint8_t red = currentColors[x][y] >> 16;
          uint8_t green = currentColors[x][y] >> 8;
          uint8_t blue = currentColors[x][y];
          
          leds.setPixel((LEDSHOR * y) + x,red, green, blue);

      }
  }
  leds.show();
}

void putColorsInArray(int led, uint8_t red, uint8_t green, uint8_t blue)
{
    currentColors[led%LEDSHOR][led/LEDSHOR] = ((uint32_t)red << 16)| ((uint32_t)green << 8) |((uint32_t) blue);
}

void putColorsInArray(int x, int y, uint8_t red, uint8_t green, uint8_t blue)
{
    currentColors[x][y] = ((uint32_t)red << 16)| ((uint32_t)green << 8) |((uint32_t) blue);
}

uint8_t returnColor(uint32_t combinedColor, char choice){
  //returns an rgb color value based on the choice given.
  //Only accepts r g b

  if    ( choice == 'r'){
    return combinedColor >>16;
  }
  else if( choice == 'g'){
    return combinedColor >>8;
  }
  else if( choice == 'b'){
    return combinedColor;
  }
  else {
    return 0;
  }
}

void everyOtherLedFlash(int fps, int r, int g, int b)
{
  //if the previous state is applied continue to the next animation step
  if(ledsApplied)
  {
    for(int count= 0; count < LEDAMOUNT;count++)
    {    
      if(count%2==0 && switcher == 1)
      {
        putColorsInArray(count ,r, g, b);    
      }
      else if(count%2!=0 && switcher == 1)
      {
        putColorsInArray(count,0,0,0);
      }
      
      if(count%2==0 && switcher == 0)
      {
        putColorsInArray(count,0,0,0);
      }
      else if(count%2!=0 && switcher == 0)
      {
        putColorsInArray(count,r, g, b);
      }
    }
    
    if(switcher == 1)
    {
      switcher = 0;
    }
    else if (switcher == 0)
    {
      switcher = 1;
    }
  } 
  applyInIntervalTime(fps);
}

void applyInIntervalTime(int fps)
{
  //Make sure ledsApplied is always false before checking the framerate
  if(ledsApplied) ledsApplied = false;
  //Apply the Color Array According to a framerate
  unsigned long currentMillis = millis();
  if((unsigned long)(currentMillis - previousMillis) >= (1000/fps)){
    //If the interval time has passed do this:
    applyColorsToStrip();
    //Save the current time for the next interval
    previousMillis = currentMillis;
    ledsApplied = true;
  }
}

void loop()
{
  everyOtherLedFlash(10, 255, 0, 255);
}

