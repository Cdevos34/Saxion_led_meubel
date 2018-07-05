#include <OctoWS2811.h>

const int LEDSHOR =  29;
const int LEDSVERT = 7;
const int  LEDAMOUNT = LEDSHOR*LEDSVERT;
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
bool directionSwitch = false;
bool extreme = false;
int swipeExtreme = 20;

//animation variables
int count = 0;

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

void knightRider(int fps, int swipeWidth, int r, int g, int b)
{
  
  //if the previous state is applied continue to the next animation step
  if(ledsApplied)
  {
/*----------------------------------------------- 
wanneer directionSwitch false is beweegt de swipe
van links naar rechts
------------------------------------------------*/
    if (directionSwitch == false)
    {
      //loop om het aantal ledjes dat naast de count aan moet gaan te bepalen (swipeWidth)
      for(int i=0; i<swipeWidth;i++)
      {
        //loop om het effect op meerdere rijen (Y as) toe te passen.
        for(int p=0; p<LEDSVERT; p++)
        {
         //leds worden aangezet (de count led + het aantal dat je invult bij SwipeWidth)
         putColorsInArray(count+i,p, r, g, b); 
        } 
      }
      //turn of previous led
      // zolanf de count geen 0 is zet het ledje links van count uit
      if(count != 0)
      {
        //loop om het uitzetten van de leds op meerdere rijen (Y as) toe te passen.
        for ( int p = 0; p<LEDSVERT; p++)
        {
         //leds worden uitgezet
         putColorsInArray(count-1,p,0,0,0); 
        }
      }
      //count gaat gaat met 1 omhoog aan het einde van elke loop om de swipe van links naar rechts te bewegen
      count ++;
      //wanneer count+swipeWidth hoger of gelijk is aan de hoeveelheid horiontale LEDS zet directionSwitch op true
      if(count+swipeWidth-1 >= LEDSHOR)
      {
       // count ++;
        directionSwitch = true;
        extreme = true;
      }
    }

    
/*----------------------------------------------- 
wanneer directionSwitch true is beweegt de swipe
van rechts naar links
------------------------------------------------*/
    if(directionSwitch == true)
    {
       //aan het eind van de loop count-1
      count --;
      //loop om het aantal ledjes dat naast de count aan moet gaan te bepalen (swipeWidth)
      for(int i=0; i<swipeWidth;i++)
      {
        //loop om het effect op meerdere rijen (Y as) toe te passen.
        for (int p=0; p<LEDSVERT; p++)
        {
          //leds worden aangezet
          putColorsInArray(count+i,p, r, g, b); 
        }
      } 
      
      //turn of previous led
      //zolang de count+swipeWidth niet gelijk is aan het totaal aantal horizontale LEDS
      if (count+swipeWidth != LEDSHOR)
      { 
        //pas het uitzetten van de ledjes toe op meerder rijen (Y as)
        for(int i=0; i<LEDSVERT; i++)
        {
          // zet ledjes count+swipeWidht uit
          putColorsInArray(count+swipeWidth,i,0,0,0);
        }
      }

      //wanneer count gelijk is aan 0 zet directionSwitch weer op false
      if(count == 0)
      {
        directionSwitch = false;
        extreme = true;
      }
    }
   
  }

  //apply interval
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

    Serial.println(count);
    
    //Save the current time for the next interval
    previousMillis = currentMillis;
    ledsApplied = true;
  }
}

/*------------------------------------------
  Cleargrid functie zet alle leds uit.
------------------------------------------*/
void clearGrid()
{
  for (int i = 0; i<LEDAMOUNT; i++)
  {
    putColorsInArray(i,0,0,0);
  }
  applyColorsToStrip();
}

/*------------------------------------------
  fillGrid functie vult grid met aangegeven kleur (RGB)
------------------------------------------*/
void fillGrid(int r, int g, int b)
{
  for (int i = 0; i<LEDAMOUNT; i++)
  {
    putColorsInArray(i,r,g,b);
  }
  applyColorsToStrip();
}

void knightRiderExtreme(int fps, int r, int g, int b)
{
  //wanneer aan het eind van de knightrider functie extrem op true word gezet en ledsApplied ook true is tel 1 bij swipeExtreme op.
  // zet vervolgens extreme weer op false zodat swipeExtreme ++ alleen weer word uitgevoerd nadat de nightrider functie weer volledig doorlopen is.
  if(extreme == true && ledsApplied)
  {
    swipeExtreme ++;
    extreme = false;
  }

  //knightRider functie word aangeroepen
  knightRider(fps, swipeExtreme, r, g, b);
  
  //Wanneer swipeExtreme gelijkt is aan het aantal horizontale LEDS zet extreme op false zet alle lEDS uit met clearGrid en reset swipeExtreme weer naar 1
  if(swipeExtreme == LEDSHOR-1)
  {
    extreme = false;
    clearGrid();
    swipeExtreme = 1;
    
  }
}

void loop()
{ 
  knightRiderExtreme(10, 255, 0, 0);
  //clearGrid();
  //fillGrid(255,0,255);
}

