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

//animation variables
int count = 0;
int switcher = 1;
bool rood = false;
bool groen = false;
bool blauw = false;
bool pattern = false;



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

void putColorsInArray(int x, int y, uint32_t color)
{
    currentColors[x][y] = color;
}

void putColorsInArray(int led, uint32_t color)
{
    currentColors[led%LEDSHOR][led/LEDSHOR] = color;
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

/*-------------------------------------------------
function with X and Y parameters
--------------------------------------------------*/

void putRandomColorsInArray(int x, int y)
{
  putColorsInArray(x, y, generateRandomColors());
}

/*-------------------------------------------------
function with strip length (i) parameter
--------------------------------------------------*/

void putRandomColorsInArray(int i)
{
  putColorsInArray(i, generateRandomColors());
}

/*-------------------------------------------------
generate random colors with at least R, G, or B set to 0
--------------------------------------------------*/

uint32_t generateRandomColors()
{
  //set bool rood to true or false based on 50/50 chance
  if(random(2) % 2 == 0)
  {
    rood = true;
  }else
  {
    rood = false; 
  }
  //set bool blauw to true or false based on 50/50 chance
  if(random(2) % 2 == 0)
  {
    blauw = true;
  }else
  {
    blauw = false; 
  }
  //When bool rood and blauw are both true make sure green is false
  if (rood == true && blauw == true)
  {
    groen = false;
  }
  //When bool rood and blauw are both false make sure green is set to true
  if (rood == false && blauw == false)
  {
    groen = true;
  }

  //Random generated number for RGB colors
  int rRandom=0;
  int gRandom=0;
  int bRandom=0;

  //Generates a random color value if bool rood is true 
  if(rood == true)
  {
    rRandom=random(50,255);
  }
  //Generates a random color value if bool groen is true 
  if(groen == true)
  {
    gRandom=random(50,255);
  }
  //Generates a random color value if bool blauw is true 
  if(blauw == true)
  {
    bRandom=random(50,255); 
  }

  //Bitshift and return color value
  return ((uint32_t)rRandom << 16)| ((uint32_t)gRandom << 8) |((uint32_t) bRandom);
}

/*-------------------------------------------------
fill both sides of the grid horizantally
--------------------------------------------------*/

void fillBothSidesHor(int fps,int r, int g, int b)
{
  //if the previous state is applied continue to the next animation step
  if(ledsApplied)
  {  
      putColorsInArray(count%LEDSHOR, count/LEDSHOR, r, g, b);
      int a = LEDAMOUNT-count-1;
      putColorsInArray(a%LEDSHOR, a/LEDSHOR,r, g, b);
      count++;
  }
  
  if(count > LEDAMOUNT/2 + 1)
  {
    clearGrid();
    count = 0;
  }
  
  //apply interval
  applyInIntervalTime(fps);
}

/*-------------------------------------------------
fill both sides of the grid vertically 
--------------------------------------------------*/

void fillBothSidesVert(int fps,int r, int g, int b)
{
  //if the previous state is applied continue to the next animation step
  if(ledsApplied)
  {  
      putColorsInArray(count/LEDSVERT, count%LEDSVERT, r, g, b);
      int a = LEDAMOUNT-count-1;
      putColorsInArray(a/LEDSVERT, a%LEDSVERT,r, g, b);
      count++;
  }

  if(count == LEDAMOUNT/2 + 1)
  {
    clearGrid();
    count = 0;
  }
  
  //apply interval
  applyInIntervalTime(fps);
}

/*-------------------------------------------------
Fill both sides of the grid horizontally with a rondom generated color
--------------------------------------------------*/

void fillBothSidesHorDisco(int fps)
{
  //if the previous state is applied continue to the next animation step
  if(ledsApplied)
  {  
      putRandomColorsInArray(count);   //(count%LEDSHOR, count/LEDSHOR);
      int a = LEDAMOUNT-count-1;
      putRandomColorsInArray(a);   //(a%LEDSHOR, a/LEDSHOR);
      count++;
  }

  if(count > LEDAMOUNT/2 + 1)
  {
    clearGrid();
    count = 0;
  }
  
  //apply interval
  applyInIntervalTime(fps);
}

/*-------------------------------------------------
fill both sides of the grid vertically with random generated colors
--------------------------------------------------*/

void fillBothSidesVertDisco(int fps)
{
  //if the previous state is applied continue to the next animation step
  if(ledsApplied)
  {  
      putRandomColorsInArray(count/LEDSVERT, count%LEDSVERT);
      int a = LEDAMOUNT-count-1;
      putRandomColorsInArray(a/LEDSVERT, a%LEDSVERT);
      count++;
      //putRandomColorsInArray(20);
  }

  if(count > LEDAMOUNT/2 + 1)
  {
    clearGrid();
    count = 0;
  }
  
  //apply interval
  applyInIntervalTime(fps);
}

/*-------------------------------------------------
Turn off all leds
--------------------------------------------------*/

void clearGrid()
{
  //loop throught the total amount of leds available
  for (int i = 0; i<LEDAMOUNT; i++)
  {
    //turn off/set leds to black
    putColorsInArray(i,0,0,0);
  }
  //apply once
  applyColorsToStrip();
}

/*-------------------------------------------------
Turn off all leds
--------------------------------------------------*/

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

/*-------------------------------------------------
light up a Pattern with random colors
--------------------------------------------------*/

void fillPatternDisco(int fps)
{
 if(ledsApplied)
 {
   // Pattern set to false = loop can start
   if(pattern == false)
   {
    for(int x = 0; x < LEDSHOR; x++)
    {
      for(int y = 1; y <LEDSVERT-1;y++)
      {
        if(x == count || 
           x == count+6 ||
           y == 1 && x < count+6 && x > count || 
           y == LEDSVERT-2 && x < count+6 && x > count )
        {
          //turn on random leds
          //and set pattern to true so colors are set once
          putRandomColorsInArray(x, y);
          //pattern = true; 
        }else
        {
          putColorsInArray(count-1,y,0,0,0);
          for(int i=2; i<5; i++)
          {
            putColorsInArray(count+5,i,0,0,0);
          }
        } 
      }
    }
   } 
   count++;
   if(count == LEDSHOR-5)
   {
    count = 0;
    clearGrid();
   }
 }
 applyInIntervalTime(fps); 
}
  

void loop()
{
  //fillBothSidesHor(10, 255,255,0);
  //fillBothSidesVert(10, 255,0,0);
  //fillBothSidesVertDisco(10);
  //fillBothSidesHorDisco(10);
  //putRandomColorsInArray(25, 25);
  fillPatternDisco(10);
}

