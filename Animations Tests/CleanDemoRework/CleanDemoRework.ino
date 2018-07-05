/*-------------------------------------------
    global variables & dependencies 
------------------------------------------*/

#include <OctoWS2811.h> // include library

//Communnication variables
const byte numChars = 8;  //array length
byte splitData[numChars]; // an array to store the received data
char prevData[numChars];  // comparer for changes in data
boolean newData = false;  //checks if the la test input is handled correctly

//led control variables
const int LEDSHORKLEIN =  16; // the amount of horizontal LEDs in the small bak
const int LEDSHORMIDDEL = 25; // the amount of horizontal LEDs in the medium bak
const int LEDSHORGROOT =  29; // the amount of horizontal LEDs in the large bak
const int LEDSVERT = 7; // vertical amount of LEDS in the bakken
const int arrayLength = (LEDSHORKLEIN*LEDSVERT)+(LEDSHORMIDDEL*LEDSVERT)+(LEDSHORGROOT*LEDSVERT*3); // the total amount of LEDs in the string

uint32_t currentColors[arrayLength]; // array for colors

//octoshield variables
const int  LEDAMOUNT = LEDSHORGROOT*LEDSVERT; // Amount of LEDS per connection Octoshield
DMAMEM int displayMemory[LEDAMOUNT*6];
int drawingMemory[LEDAMOUNT*6];
const int config = WS2811_GRB | WS2811_800kHz;
OctoWS2811 leds(LEDAMOUNT, displayMemory, drawingMemory, config);

//Global variables for the framerate
unsigned int frameRate = 10; // standard framerate
unsigned int interval= (10000/frameRate); // Interval time (based on fps)
unsigned long previousMillis = 0;
bool ledsApplied = false;

//animation variables
bool redOn = false; // variable for Random color generator
bool greenOn = false; // variable for Random color generator
bool blueOn = false; // variable for Random color generator
bool directionChosen = false; //bool to determine the animation direction
uint32_t randomColor; // variable for storing a random color for full animation cycle
uint32_t randomFillColor = 25500000255; // Initializing color
uint32_t raindropColor; // color for the sit animtion
int count = 0; // general animation counter
int trigger = 0; // delay trigger fillTopToBottom (wegwerken indien mogelijk)
int loopAmount = 0; // varibale for amount of times to cycle through single idle animation
int idleSwitcher = 0; // trigger for switching idle animations
int init = false; // initializing animation
int sitDownAnimationLength = 27; // height of the sitdown animation grid is 27.
int dropPosition = 0; // start position for the sit animation
int previousColor = 0; // exclude previous color from generating next color
int chosenDirection = 2; //The direction chosen based on the input
int animationLength = 63;//total horizontal width of the bakken

//states for stateSwitcher
bool idleState = false;
bool chairState = false;
bool walkState = false;

bool someoneIsSittingBool = false;
bool motionDetectedBool = false;

/*---------------------------------------------
 Setup for library
-----------------------------------------------*/

void setup()
{
 //Initialize the library
 leds.begin();
 //Update the led's Turning them all to black/off
 leds.show();

 //setting the baudrate
 Serial.begin(19200);
 //Serial.println("start arduino");

 Serial1.begin(19200);
}

/*---------------------------------------------
expecting data in serial consisting of 00000000$ 
where 0 is the single char datapoint
-----------------------------------------------*/

void loop()
{
  receiveOnlyUpdatedData();
  animationLoop();
}

//--------- Detection and trigger functions ----------//

/*-------------------------------------
Only Receive Updated incoming data 
--------------------------------------*/

void receiveOnlyUpdatedData()
{
  static byte index = 0;
  char startMarker = '*';
  char endMarker = '$';
  char divider = ',';
  byte serialInput;
 
  while (Serial1.available() > 0 && newData == false) {
     serialInput = Serial1.read();
    
     if (serialInput != endMarker && serialInput != divider && serialInput != startMarker)
     {       
         splitData[index] = serialInput;
         index++;
     }
     else if(serialInput == endMarker){
         //splitData[index] = '\0'; // terminate the string
         index = 0;
         newData = true;
         
         applyChangedData();

         newData = false;
     }
  }
} 

/*-------------------------------------
Print incoming data to the Serial monitor
--------------------------------------*/

void printAllData()
{
   for(int i=0; i<numChars; i++){
       Serial.print(splitData[i]);
   }         
   Serial.println(); 
}

/*-------------------------------------------
  Animation Loop
  if someone is sitting run sit animation
  if someone is walking by run walk by animation
  else run the idle animation
 ------------------------------------------*/

void animationLoop()
{ 
  //Check if someone is sitting
  if(someoneIsSittingBool)
  {
    //set previous states to false
    // add transition animation here
    if(idleState)
    {
      count = 0;
      idleState = false;
    }

    if(walkState)
    {
      count = 0;
      walkState = false;
    }
    
    if(chairState != true)
    {
      count = 0;
      chairState = true;
      transitionAnimation();
    }

    //determine activated chairs to animate per separate chairs
    for(int i=0; i<4;i++)
    {
      if(splitData[i] == 1)
      {
        chairAnimation(i, 1);  
      }
      else if(splitData[i] == 0)
      {
        chairAnimation(i, 0);
      }
    }

    // single chair animation
    applyChairAnimation(10);

  }

  //---------------------------------------------//

  // check if someone is walking by
  else if(motionDetectedBool)
  {
    // set any previous state to false 
    // add transition animation here
    if(idleState)
    {
      count = 0;
      idleState = false;
      clearGrid();
    }

    if(chairState)
    {
      chairState = false;
    }

    if(walkState != true)
    {
      count = 0;
      walkState = true;
      transitionAnimation();
    }
    
    // run walk by animation
    walkByAnimation();
  }

  //---------------------------------------------//

  else // if nobody is walking by or sitting
  {
    // set any previous state to false 
    // add transition animation here
    if(chairState)
    {
      chairState = false;
    } 

    if(walkState)
    {
      walkState = false;
    }

    if(idleState != true)
    {
      count = 0;
      idleState = true;
      transitionAnimation();
    }

    // Run the idle animation
    idleAnimation();
  }
}

/*---------------------------------------
  Animation per chair here
  parameter (i) runs from 1 to 4 for every chair
----------------------------------------*/

void chairAnimation(int i, int downOrUp)
{
  sitDownAnimation(i, downOrUp);
}


/*---------------------------------------
  Single chair animation here
----------------------------------------*/

void applyChairAnimation(int fps)
{
  if(ledsApplied)
  {
    count++;

    if(count > sitDownAnimationLength+10) // wait for an extra 10 fps with ending the animation
    {
      count = 0;
      //fillGridWhite(); // fill all with white at the end.
      clearGrid();  //clear all when the animation is finished
      motionDetectedBool = motionDetected();
      someoneIsSittingBool = someoneIsSitting();
    }
  }
  
  applyInIntervalTime(fps); //hardcoded 10fps fixen in chairanimatie
}

/*-------------------------------------------
    Put animation based on motion sensors 
    (walkByDetection) here.
------------------------------------------*/

void walkByAnimation()
{
  fillHorizontalOverAllLeds(12);
}

/*-------------------------------------------
    Put your idle animation here 
    meaning no seats or motion is detected
------------------------------------------*/

void idleAnimation()
{
  idleAnimationCycle(5);
}

/*-------------------------------------
idle animation cycler
--------------------------------------*/

void idleAnimationCycle(int loopAmount)
{ 
  if (idleSwitcher <= loopAmount)
  {
    //Serial.println("Idle1");
    fillTopToBottom(20);
  }
  else if (idleSwitcher > loopAmount && idleSwitcher <= loopAmount*2)
  {
    //Serial.println("Idle2");
    fillHorAll(20);
  }
  else if (idleSwitcher > loopAmount*2 && idleSwitcher <= loopAmount*3)
  {
    //Serial.println("Idle3");
    fill(75);
  }
  else if(idleSwitcher >= loopAmount*3)
  {
    idleSwitcher = 0;
  }

}

//------------------ Utility Functions ------------------//

/*-------------------------------------
apply colors to strip
--------------------------------------*/

void applyColorsToStrip()
{
  //uint32_t colorArray[LEDSHOR][LEDSVERT], int arrayLength){
  //  Serial.println("start apply colors");
  for(int i=0; i < arrayLength;i++)
  {
    uint8_t red =     currentColors[i] >> 16;
    uint8_t green =   currentColors[i] >> 8;
    uint8_t blue =    currentColors[i];
    int index = i;
    if(i > 489) // vanaf 489 (dus ledje 490) springen naar volgende bak
    {
      index = i + 119;
    }
    else if(i > 111) // vanaf 111 (dus ledje 112) springen naar volgende bak
    {
      index = i + 91;
    }
    leds.setPixel(index,red, green, blue);   
  }
  leds.show();
}

/*---------------------------------------------
use PutColorsInArray to light up specified LEDS
led = position of led seeing all the leds as a single string
int x = x position on the grid
int y = y position on the grid
int bak = specify the bak 
-----------------------------------------------*/

void putColorsInArray(int led, uint8_t red, uint8_t green, uint8_t blue)
{
    currentColors[led] = ((uint32_t)red << 16)| ((uint32_t)green << 8) |((uint32_t) blue);
}

void putColorsInArray(int x, int y, int bak, uint8_t red, uint8_t green, uint8_t blue)
{
    currentColors[(getBakWidth(bak)*y)+x+getBakStartpoint(bak)] = ((uint32_t)red << 16)| ((uint32_t)green << 8) |((uint32_t) blue);
}

void putColorsInArray(int x, int y, int bak, uint32_t color)
{
    currentColors[(getBakWidth(bak)*y)+x+getBakStartpoint(bak)] = color;
}

void putColorsInArray(int led, uint32_t color)
{
    currentColors[led] = color;
}

//------------- Read above + RandomColors -------------------//

void putRandomColorsInArray(int x, int y, int bak)
{
  putColorsInArray(x, y, bak, generateRandomColors());
}

void putRandomColorsInArray(int i)
{
  putColorsInArray(i, generateRandomColors());
}

/*---------------------------------------------------
  Randomize RGB and make it so colors dont repeat 
----------------------------------------------------*/

void randomizeRGB()
{
  previousColor = 0;

  //set bool for turning on R, G or B LED to true or false based on 50/50 chance
  if(random(2) % 2 == 0)
  {
    redOn = true;
    previousColor += 1;
  }
  else
  {
    redOn = false; 
  }

  if(random(2) % 2 == 0)
  {
    blueOn = true;
    previousColor += 2;
  }
  else
  {
    blueOn = false; 
  }

  //---------- Making sure there is no white and repeating colors-----------//

  if (redOn == true && blueOn == true)
  {
    //When bool redOn and blueOn are both true make sure greenOn is false
    greenOn = false;
  } 
  else if((redOn == true) || (blueOn == true))
  {
      if(random(2) % 2 == 0)
      {
        greenOn = true;
        previousColor += 4;
      }
      else
      {
        greenOn = false; 
      }
  } 
  else if (redOn == false && blueOn == false)
  {
    //When bool redOn and blueOn are both false make sure greenOn is set to true
    greenOn = true;
    previousColor += 4;
  }
}

/*-------------------------------------------------
Turn off all leds
--------------------------------------------------*/

void clearGrid()
{
  //loop throught the total amount of leds available
  for (int i = 0; i<arrayLength; i++)
  {
    //turn off/set leds to black
    putColorsInArray(i,0,0,0);
  }
  //apply once
  applyColorsToStrip();
}

/*-------------------------------------------------
 clear a Specific LED bak
--------------------------------------------------*/

void clearBak(int bak)
{
  for (int i = 0; i<(getBakWidth(bak)*LEDSVERT); i++)
  {
    putColorsInArray(i+getBakStartpoint(bak),0,0,0);
  }
}

/*-------------------------------------------------
Interval function used to set a animation framerate
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

/*-----------------------------------------
changing data 
----------------------------------------*/

void applyChangedData()
{ 
  for(int i=0; i < numChars; i++)
  {
    if(splitData[i] != prevData[i])
    {
      //Serial.print("changed data: ");
      //Serial.print(i); 
      //Serial.print(" is now :");
      //Serial.println(splitData[i]);

      prevData[i] = splitData[i];
    }
  }
}

/*-------------------------------------------------
 Show the changed data in the serial monitor
--------------------------------------------------*/

void showNewData() {
  if (newData == true) {
     //Serial.println("This just in ... ");
     for(int i=0 ; i < numChars; i++)
     {
        Serial.println(splitData[i]);
     }
  }
}

//---------------- Animation functions ------------------//

/*--------------------------------------
  Walk by animation
 -------------------------------------*/

void fillHorizontalOverAllLeds(int fps)
{
  //if someone walks by, ergo one of the distance sensors is activated

  if(ledsApplied == true)
  {
    if(!directionChosen)
    {
      chosenDirection = getAnimationDirection();
      directionChosen = true;
      //Serial.print("direction = ");
      //Serial.println(chosenDirection);
      randomColor = generateRandomColors();
      count = 0;
    }

    if(directionChosen)
    {
      fillSidetoSide(chosenDirection);
    }
  }
  
  applyInIntervalTime(fps);
}

/*-------------------------------------------
    FillSideToSide
------------------------------------------*/

void fillSidetoSide(int i)
{
  if(i == 0)
  {
    // for each bak, if within the range of the offset, fill the leds horizontaly.
    fillSidetoSideToArray(count);
  }

  if(i == 1)
  {
    // for each bak, if within the range of the offset, fill the leds horizontaly, backwards.
    int countBackwards = animationLength-count;
    fillSidetoSideToArray(countBackwards);
  }

  if(i != 0 && i != 1)
  {

    //fill both inwards option
    /*
    fillSidetoSideToArray(count);
    int countBackwards = animationLength-count;
    fillSidetoSideToArray(countBackwards);
    */

    //fill both outwards
    fillSidetoSideToArray(count+animationLength/2);
    int countBackwards = animationLength/2-count;
    fillSidetoSideToArray(countBackwards);
  }
 
  if(animationComplete( i, count, animationLength))
  {
    count = 0;
    clearGrid();
    directionChosen = false;
    motionDetectedBool = motionDetected();
    someoneIsSittingBool = someoneIsSitting();
  }

  count ++;
}

/*-------------------------------------------
    FillSideToSideArray
------------------------------------------*/

void fillSidetoSideToArray(int c)
{
  for(int b=0; b<5; b++)
  {
    if(isBakInHorOffset(b,c))
    {
      for(int y=0; y<7; y++)
      {
        putColorsInArray(c-getBakHorOffset(b),y,b,getColorsFromSensors());
      }
    }
  }
}


/*---------------------------------------------
transition animation
-----------------------------------------------*/

void transitionAnimation()
{
  //create state transition animation here
  for (int i = 0; i<arrayLength; i++)
  {
    putColorsInArray(i,255,255,255);
  }
  //apply once
  applyColorsToStrip();
  clearGrid();
}

/*---------------------------------------------
transition animation
-----------------------------------------------*/

void fillTopToBottom(int fps)
{
  if(ledsApplied)
  {
    for(int i = 0; i < 5; i++)
      {
        for(int x = 0; x < getBakWidth(i); x++)
        {
          putColorsInArray(x,count, i, randomColor);
          //putColorsInArray(x, count-1, i, r,g,b);
        }
      }

      count++;


      if(count == 7)
      {
        count = 0;
        trigger++;
        motionDetectedBool = motionDetected();

      }

      if(trigger == 5)
      {
        randomColor = generateRandomColors();
        trigger = 0;
        idleSwitcher++;
        motionDetectedBool = motionDetected();
        someoneIsSittingBool = someoneIsSitting();
      }

    }

  applyInIntervalTime(fps);

}

/*---------------------------------------------
Fill every bak horizontally
-- part of Idle animation cycle --
-----------------------------------------------*/

void fillHorAll(int fps)
{
  if (ledsApplied)
  {
    for (int y = 0; y< LEDSVERT; y++)
    {
      for (int bak = 0; bak < 5; bak++)
      {
        putColorsInArray(count, y, bak, randomColor);
      }
    }
    count++;
    motionDetectedBool = motionDetected();
    if(count > 28)
    {
      count = 0;
      randomColor = generateRandomColors();
      idleSwitcher++;

      motionDetectedBool = motionDetected();
      someoneIsSittingBool = someoneIsSitting();
    }
  }

  applyInIntervalTime(fps);
}

/*--------------------------------------
    Sit down animation
---------------------------------------*/

void sitDownAnimation(int chair, int downOrUp)
{
  if(ledsApplied)
  {
    //fill entire area
    for(int i=0; i<16; i++)
    {
      upwardRain(i+(chair*16), downOrUp);
    }
  }
}

void upwardRain(int row, int downOrUp)
{
  for(int b=0; b<5; b++)
  {
    if(isBakInHorOffset(b,row))
    {
      if(isBakInVertOffset(b,sitDownAnimationLength-count))
      {
        if(downOrUp == 1)
        {
          putColorsInArray(row-getBakHorOffset(b),sitDownAnimationLength-count-getBakVertOffset(b),b,getColorsFromSensors());
        }
        else
        {
         putColorsInArray(row-getBakHorOffset(b),sitDownAnimationLength-count-getBakVertOffset(b),b,makeBitshiftedColor(50,50,50)); 
        }
          
      }
    }
  }
}

void clearVert(int b)
{
  for(int x=0; x<16; x++)
  {
    for(int y=0; y<sitDownAnimationLength; y++)
    {
      for(int bak=0; bak<5; bak++)
      {
        if(isBakInVertOffset(bak,y))
        {
          if(isBakInHorOffset(bak,x+(b*16)))
          {
            putColorsInArray(x+(b*16)-getBakHorOffset(bak),y-getBakVertOffset(bak),bak,0,0,0);
          }
        }
      }
    }
  }
  applyColorsToStrip();
}

/*---------------------------------------------
Fill every bak horizontally line by line
-- part of Idle animation cycle --
-----------------------------------------------*/

void fill(int fps)
{

  if(ledsApplied)
  {
      // Fill the entire string of leds
      putColorsInArray(count,randomFillColor); 

    count++;
    motionDetectedBool = motionDetected();
  }

  if(count == arrayLength+1)
  {
    //clearGrid(); // clearGrid after completion option
    randomFillColor = generateRandomColors();
    count = 0;
    idleSwitcher++;

    motionDetectedBool = motionDetected();
    someoneIsSittingBool = someoneIsSitting();
  }
  
  //apply interval
  applyInIntervalTime(fps);
}

//---------------- Return Functions -----------------//

/*----------------------------------------------
 incoming data checks 
 ---------------------------------------------*/

bool someoneIsSitting()
{
  if(splitData[0] == 1 ||
   splitData[1] == 1 ||
   splitData[2] == 1 ||
   splitData[3] == 1)
  {
    return true;
  }
  else 
  {
    return false;
  }
}

bool motionDetected()
{
  if(splitData[4] != 0 ||
  splitData[5] != 0 ||
  splitData[6] != 0 ||
  splitData[7] != 0)
  {
    return true;
  }
  else 
  {
    return false;
  }
}

bool motionDetectedLeft()
{
  if((splitData[4] != 0
  || splitData[5] != 0) && 
  (splitData[6] == 0
  && splitData[7] == 0))
  {
    return true;
  }
  else 
  {
    return false;
  }
}

bool motionDetectedRight()
{
  if((splitData[4] == 0
  && splitData[5] == 0) && 
  (splitData[6] != 0
  || splitData[7] != 0))
  {
    return true;
  }
  else 
  {
    return false;
  }
}

bool animationComplete( int i, int count, int arrayLength)
{
  if(((i != 0 && i != 1) && count > animationLength/2+2) ||
    ((i == 0 || i == 1) && count > animationLength))
  {
    return true;
  }
  else 
  {
    return false;
  }
}

/*-------------------------------------------
    Get Horizontal and vertical offset
---------------------------------------------*/

bool isBakInHorOffset(int b, int x)
{
  return (x >= getBakHorOffset(b) && x < getBakWidth(b) + getBakHorOffset(b)); 
}

bool isBakInVertOffset(int b, int y)
{
    return (y >= getBakVertOffset(b) && y < 7 + getBakVertOffset(b));
}

/*-------------------------------------------
    function to determine the direction of
    the fill for the walk by animation based
    on which sensors are triggered
------------------------------------------*/

int getAnimationDirection()
{
  //richting bepalen
  //als 1 of 2 van de linker triggered zijn. else 1/2 van de rechter, else alle andere combi's
  if(motionDetectedLeft())
  {
    return 0;
  }
  else if(motionDetectedRight())
  {
    return 1;
  }
  // Motion on the left and right detected 
  return 2;
}
/*-------------------------------------
Function that returns colors based on the 
input of all four distance sensors
--------------------------------------*/

uint32_t getColorsFromSensors()
{
  int redIntensity = 0;
  int greenIntensity = 0;
  int blueIntensity = 0;

  if(motionDetected() == false)
  {
    return randomColor;   // return a previously generated color
  }

  //dit verder randomizen met waardes tussen de 0-85?
  for(int i=4; i<8; i++)
  {
    if(splitData[i] == 1)
    {
      redIntensity = redIntensity + 63;
    }
    else if(splitData[i] == 2)
    {
      greenIntensity = greenIntensity + 63;
    }
    else if(splitData[i] == 3)
    {
      blueIntensity = blueIntensity + 63;
    }
  }

  return ((uint32_t)redIntensity << 16)| ((uint32_t)greenIntensity << 8) |((uint32_t) blueIntensity); //fill color with above values
}

/*-------------------------------------
Determine the starting point of every bak
--------------------------------------*/

int getBakStartpoint(int bak)
{    
    int startingLed = 0;
    if(bak == 1) startingLed = 112;
    if(bak == 2) startingLed = 315;
    if(bak == 3) startingLed = 490;
    if(bak == 4) startingLed = 693;
    return startingLed;
}

/*-------------------------------------
Determine the width of every bak
--------------------------------------*/

int getBakWidth(int bak)
{    
    int horizontalLedAmount = LEDSHORGROOT;
    if(bak == 0) horizontalLedAmount = LEDSHORKLEIN;
    if(bak == 2) horizontalLedAmount = LEDSHORMIDDEL;
    return horizontalLedAmount;
}

/*-------------------------------------
Determine the horizontal offset of the bakken
--------------------------------------*/

int getBakHorOffset(int bak)
{
  if(bak == 0) return 1;
  if(bak == 1) return 20;
  if(bak == 2) return 15;
  if(bak == 3) return 0;
  if(bak == 4) return 30;
}

/*-------------------------------------
Determine the vertical offset of the bakken
--------------------------------------*/

int getBakVertOffset(int bak)
{
  if(bak == 0 || bak == 1) return 0;
  if(bak == 2) return 10;
  if(bak == 3 || bak == 4) return 20;
}

/*-------------------------------------
Return the color from a specified LED
--------------------------------------*/

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
generate semi-random colors with either R, G, or B set to 0 randomly
Function also checks for the previously generated color to
make sure you wont get the same color twice.
--------------------------------------------------*/

uint32_t generateRandomColors()
{

  // if the last color variable happens to 
  // be the same as the generated color repeat
  int colorToExclude = previousColor;
  while(previousColor == colorToExclude){
    randomizeRGB();
  }
  Serial.println(previousColor);

  //Random generated number for RGB colors
  int rRandom=0;
  int gRandom=0;
  int bRandom=0;

  //Generates a random color value if bool redOn is true 
  if(redOn == true)
  {
    rRandom=random(50,255);
  }
  //Generates a random color value if bool greenOn is true 
  if(greenOn == true)
  {
    gRandom=random(50,255);
  }
  //Generates a random color value if bool blueOn is true 
  if(blueOn == true)
  {
    bRandom=random(50,255); 
  }

  //Bitshift and return color value
  return ((uint32_t)rRandom << 16)| ((uint32_t)gRandom << 8) |((uint32_t) bRandom);
}

uint32_t makeBitshiftedColor(int r,int g, int b)
{

  return ((uint32_t)r << 16)| ((uint32_t)g << 8) |((uint32_t) b);
}

