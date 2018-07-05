#include <OctoWS2811.h>

const int LEDSHOR =  16;
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
bool once = true;
bool directionSwitch = true;
uint8_t redColorSwitch = 0;
uint8_t  green = 0;
uint8_t blue = 0;
int c = 0;



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
           (y == 1 && x < count+6 && x > count) || 
           (y == LEDSVERT-2 && x < count+6 && x > count) )
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

/***********
 * * color wave
 **********/

void colorWave( int fps, int r, int g, int b, bool rUp,bool gUp,bool bUp)
{         
  if(ledsApplied)
  {
    if(directionSwitch == true){
    
      if(once == true)
      { 
        for(int y = 0; y<LEDSVERT; y++)
        {
            putColorsInArray(0,y,r,g,b);
        }
        once = false;
      }
      
  /*------------------------------------------------------- 
      loop to create the colorWave /\/\/\/\/\
  --------------------------------------------------------*/
  
      for(int x = 0; x<LEDSHOR; x++)
      {
        for(int y = 0; y<LEDSVERT; y++)
        {
          /*if(rUp == true) 
          {
            rood = rood + count; 
          }else
          {
            rood = rood-count;
          }
  
          if(gUp == true)
          {
            groen = groen + count;
          }else
          {
            groen = groen - count;
          }
  
          if(bUp == true)
          {
            blauw = blauw + count;
          }else
          {
            blauw = blauw - count;
          }
          */
          if(x != 0)
          {
           //Returns color from led on the left(x-1 in this case)
           redColorSwitch = returnColor(currentColors[x-1][y],'r');
           //Turn on the leds with returned color 'rood' + 1 and repeat.
           
            if(redColorSwitch+2 > 255)
            {
              c = 255;
             //directionSwitch = true;
             }
             else
             {
               c = redColorSwitch+2;
             }
            
            putColorsInArray(x,y, c, green, blue); 
            
          }
          else
          {
            
              if(r+count > 255)
              {
                c = 255;
              }
              else
              {
                c = r+count;
              }
              
            putColorsInArray(x,y, c, green, blue);
            
          } 
          
          if(c == 255)//(red+1 == 255) &&
          {
            directionSwitch = false;
          }
        }
      }
      count++; 
    }

/*-----------------------------------------------------
      Change direction of wave   
-----------------------------------------------------*/
    
    else
    {
      for(int x = LEDSHOR-1; x>-1; x++)
      {
        for(int y = LEDSVERT-1; y>-1; y++)
        {
          /*if(rUp == true) 
          {
            rood = rood + count; 
          }else
          {
            rood = rood-count;
          }
  
          if(gUp == true)
          {
            groen = groen + count;
          }else
          {
            groen = groen - count;
          }
  
          if(bUp == true)
          {
            blauw = blauw + count;
          }else
          {
            blauw = blauw - count;
          }
          */
          if(x != LEDSHOR-1)
          {
           //Returns color from led on the left(x-1 in this case)
           redColorSwitch = returnColor(currentColors[x+1][y],'r');
           //Turn on the leds with returned color 'rood' + 1 and repeat.
             
              if(redColorSwitch-2 > 0)
              {
                c = 0;
                //directionSwitch = true;
              }
              else
              {
                c = redColorSwitch-2;
              }
              
            putColorsInArray(x,y, c, green, blue); 
            
          }
          else
          {
              if(r-count < 0)
              {
                c = 0;
              }
              else
              {
                c = r-count;
              }
              
            putColorsInArray(x,y, c, green, blue);
            
          } 
          
          if(c == 0)
          {
            directionSwitch = true;
            once = true;
          }
        }
      }
      count--;
    }
  }
  
  applyInIntervalTime(fps); 
}

/***********
 * change own color slowly
 * 
 ************/

bool rSwitch = true;
bool gSwitch = true;
bool bSwitch = true;
        
void changeOwnColor( int fps, int r, int g, int b, bool rUp,bool gUp,bool bUp)
{
 if(ledsApplied)
 {
    if(once){
      for(int led = 0; led<LEDAMOUNT-1; led++)
      {
        putColorsInArray(led,r,g,b);
      }
      once = false;
    }
  
    for(int led = 0; led<LEDAMOUNT-1; led++)
        uint8_t redColorSwitch = returnColor(currentColors[led],'r');
        uint8_t greenColorSwitch = returnColor(currentColors[led],'g');
        uint8_t blueColorSwitch = returnColor(currentColors[led],'b');

        //red
        if(stateSwitch(rSwitch,rUp))
        {
          redColorSwitch += 1;
          //hier switchet de state naar false
          if(redColorSwitch == 255)
          {
            rSwitch = false;
          }
        }
        else
        {
           redColorSwitch -= 1;
           //hier switcht die naar true
           if(redColorSwitch == 0)
           {
              rSwitch = true;
           }
        }
        
        //green
        if(stateSwitch(gSwitch,gUp))
        {
          greenColorSwitch += 1;
          if(greenColorSwitch == 255)
          {
            gSwitch = false;
          }
        }
        else
        {
           greenColorSwitch -= 1;
           if(greenColorSwitch == 0)
           {
              gSwitch = true;
           }
        }

        //blue
        if(stateSwitch(bSwitch,bUp))
        {
          blueColorSwitch += 1;
          if(blueColorSwitch == 255)
          {
            bSwitch = false;
          }
        }
        else
        {
           blueColorSwitch -= 1;
           if(blueColorSwitch == 0)
           {
              bSwitch = true;
           }
        }
        
        putColorsInArray(led,redColorSwitch, greenColorSwitch, blueColorSwitch);// greenColorSwitch, blueColorSwitch); 
    } 
  applyInIntervalTime(fps); 
}

/*
 * state switch for both on true or different = false;
 */
bool stateSwitch(bool x, bool y)
{
  if(
    (x == true && y == true) ||
    (x == false && y == false))
  {
      return true;
  }
  else if(
    (x == true && y == false) ||
    (x == false && y == true))
  {
      return false;
  }
}

void loop()
{
  //fillBothSidesHor(10, 255,255,0);
  //fillBothSidesVert(10, 255,0,0);
  //fillBothSidesVertDisco(10);
  //fillBothSidesHorDisco(10);
  //putRandomColorsInArray(25, 25);
  //fillPatternDisco(10);
  colorWave(25,1, 0, 0, true, false, false);
  
  //changeOwnColor(50,5, 200, 0, true, false, true);
}


