#include <OctoWS2811.h>

//Communnication variables
const byte numChars = 8;  //array length
byte splitData[numChars]; // an array to store the received data
char prevData[numChars];  // comparer for changes in data
boolean newData = false;  //checks if the latest input is handled correctly

//led control variables
const int LEDSHORKLEIN =  16;
const int LEDSHORMIDDEL = 25;
const int LEDSHORGROOT =  29;
const int LEDSVERT = 7;
const int arrayLength = (LEDSHORKLEIN*LEDSVERT)+(LEDSHORMIDDEL*LEDSVERT)+(LEDSHORGROOT*LEDSVERT*3);

uint32_t currentColors[arrayLength];


//octoshield variables
const int  LEDAMOUNT = LEDSHORGROOT*LEDSVERT;
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
int switcher = 0;
bool rood = false;
bool groen = false;
bool blauw = false;
bool directionSwitch = false;
bool directionSwitchers[4];
int extreme = false;
int counters[4];
int rGain = 0;
int gGain = 0;
int bGain =0;
int32_t RandomKleur;
int trigger = 0;
int i = 0;
int init = false;
int count2 = 0;


//states
bool idleState = false;
bool chairState = false;
bool walkState = false;


void setup() {
 //Initialize the library
 leds.begin();
 //Update the led's Turning them all to black/off
 leds.show(); 
  
 Serial.begin(19200);
 Serial.println("*0,0,0,0,0,0,0,0$");
 //Serial.write("*0,0,0,0,0,0,0,0$");

 //receiver serial
 Serial1.begin(19200);
}

// expecting data in serial consisting of *0,0,0,0,0,0,0,0$ where 0 is the single char datapoint

void loop()
{
  receiveOnlyUpdatedData();
  animationLoop();
}

void animationLoop()
{
/*
 * if (0-3 is 1)
 *  doe stoel animatie met kleuren van 4-7
 * else if( 4-7 is niet 0)
 *  doe langs loop animatie
 * else 
 *  doe idle animatie
 */
  if(splitData[0] == 49 ||
     splitData[1] == 49 ||
     splitData[2] == 49 ||
     splitData[3] == 49)
  {

    if(idleState){
      idleState = false;
      flash();
      count = 0;
    }

    if(walkState){
      count = 0;
      walkState = false;
    } 
    
    if(chairState != true){
      flash();
      chairState = true;
    }

    //animate chairs
    for(int i=0; i<4;i++)
    {
      if(splitData[i] == 49)
      {
        chairAnimation(i);  
      }
    }
    applyInIntervalTime(10);     //hardcoded 10fps fixen in chairanimatie
  }
  else if(splitData[4] != 48 ||
  splitData[5] != 48 ||
  splitData[6] != 48 ||
  splitData[7] != 48)
  {
    if(idleState){
      idleState = false;
      clearGrid();
      count = 0;
    } 
    if(chairState) chairState = false;
    if(walkState != true){
      walkState = true;
      flash();
      count = 0;
    } 
    walkByAnimation();
  }
  else
  {
    if(chairState) chairState = false;
    if(walkState) walkState = false;
    if(idleState != true){
    flash();
    count = 0;
     idleState = true;
   }
    idleAnimation();
  }
}

void stateSwitcher(bool a,bool b,bool c)
{
    if(a) a = false;
    if(b) b = false;
    if(c != true) c = true;
}

void stateSwitcherWithClear(bool a,bool b,bool c)
{
    if(a) a = false;
    if(b) b = false;
    if(c != true) c = true;
    clearGrid();
}

void chairAnimation(int i)
{
  Serial.println("chairAnimation1");
   Serial.println(i);
  //knightRider(10, 4, i, 0,0,255);
  //newAnimation(10, i, 255,0,255);
  //everyOtherLedFlash(10);
  //Serial.print("chair animation: ");
  //Serial.println(i);  
  //printAllData();
}

void applyChairAnimation()
{
  Serial.println("chairAnimation2");
  //Serial.println("chairAnimation2");
  //everyOtherLedFlash(10);
  //Serial.println("chair animation applier");  
  //printAllData();
}

void walkByAnimation()
{
  //fill(100);
  fillHalfSensors(20, 200,0,0);
  Serial.println("walk by animation");
  //fadeColorInSteps(getColorsFromSensors(),generateRandomColors(),2,2);
  //printAllData();
}

void idleAnimation()
{
  //Serial.println("idle animation");
  //fill(100);
  horWipe(20);
  //fillHorAllTotal(20);
  //knightRiderHor(10);
  //fillHalfIdle(20, 200,0,0 ,0,240,240);
  //fillHorAll(20);
  //fillTopToBottom(10);
  //FillWave(10, 255, 255, 0);
  //everyOtherLedFlash(10);
  //fillBothSidesHorAll(10,50,255,100, 100,35,55);
  //colorFadeHor(10, 0,255,0, 255,0,0);
  //printAllData();
  //fill(50,50,50,50);
}

uint32_t getColorsFromSensors()
{
  int redIntensity = 0;
  int greenIntensity = 0;
  int blueIntensity = 0;

  if( splitData[4] == 48 &&
      splitData[5] == 48 &&
      splitData[6] == 48 &&
      splitData[7] == 48)
  {
    return generateRandomColors();   //hier random kleur pakken?
  }

  //dit verder randomizen met waardes tussen de 0-85?
  for(int i=4; i<8; i++)
  {
    if(splitData[i] == 49)
    {
      redIntensity = redIntensity + 63;
    }
    else if(splitData[i] == 50)
    {
      greenIntensity = greenIntensity + 63;
    }
    else if(splitData[i] == 51)
    {
      blueIntensity = blueIntensity + 63;
    }
  }

  return ((uint32_t)redIntensity << 16)| ((uint32_t)greenIntensity << 8) |((uint32_t) blueIntensity); //fill color with above values
}


void receiveOnlyUpdatedData() {
  static byte index = 0;
  char startMarker = '*';
  char endMarker = '$';
  char divider = ',';
  byte serialInput;
 
  while (Serial.available() > 0 && newData == false) {
     serialInput = Serial.read();
    
     if (serialInput != endMarker && serialInput != divider && serialInput != startMarker)
     {       
         splitData[index] = serialInput;
         index++;
     }
     else if(serialInput == endMarker){
         //splitData[index] = '\0'; // terminate the string
         index = 0;
         newData = true;
         
         //showNewData(); 
         applyChangedData();
         //printAllData();

         newData = false;
     }
  }
}

void printAllData()
{
   for(int i=0; i<numChars; i++){
       Serial.print(splitData[i]);
   }         
   Serial.println();
}

void applyColorsToStrip(){//uint32_t colorArray[LEDSHOR][LEDSVERT], int arrayLength){

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

  /*
  for(int y=0; y<LEDSVERT; y++){
      for(int x=0; x<LEDSHOR; x++){
          //fetch the colors    
          uint8_t red = currentColors[x][y] >> 16;
          uint8_t green = currentColors[x][y] >> 8;
          uint8_t blue = currentColors[x][y];

          
          
          leds.setPixel((LEDSHOR * y) + x,red, green, blue);          
      }
  }*/
  leds.show();
}

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

int getBakStartpoint(int b)
{    
    int e = 0;
    if(b == 1) e = 112;
    if(b == 2) e = 315;
    if(b == 3) e = 490;
    if(b == 4) e = 693;
    return e;
}

int getBakWidth(int w)
{    
    int e = LEDSHORGROOT;
    if(w == 0) e = LEDSHORKLEIN;
    if(w == 2) e = LEDSHORMIDDEL;
    return e;
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

void putRandomColorsInArray(int x, int y, int bak)
{
  putColorsInArray(x, y, bak, generateRandomColors());
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

void fillGrid(int r, int g, int b)
{
  //loop throught the total amount of leds available
  for (int i = 0; i<arrayLength; i++)
  {
    //turn off/set leds to black
    putColorsInArray(i,r,g,b);
  }
  //apply once
  applyColorsToStrip();
}

void clearBak(int bak)
{
  for (int i = 0; i<(getBakWidth(bak)*LEDSVERT); i++)
  {
    putColorsInArray(i+getBakStartpoint(bak),0,0,0);
  }
}

/*-------------------------------------------------
interval
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
      Serial.print("changed data: ");
      Serial.print(i); 
      Serial.print(" is now :");
      Serial.println(splitData[i]);

      if(i < 4 && splitData[i] == 48)
      {
        clearBak(i);
      }

      prevData[i] = splitData[i];
    }
  }
}

void showNewData() {
   if (newData == true) {
     Serial.println("This just in ... ");
     for(int i=0 ; i < numChars; i++)
     {
       Serial.println(splitData[i]);
     }
   }
}

/*--------------------------------------
      animations
      Horizontaal
---------------------------------------*/

void fillBothSidesHor(int fps, int bak, int r, int g, int b)
{

  //if the previous state is applied continue to the next animation step
  if(ledsApplied == true)
  {  
      putColorsInArray(count+getBakStartpoint(bak),r,g,b);
      int a = getBakWidth(bak)*LEDSVERT-count-1;
      putColorsInArray(a+getBakStartpoint(bak),r, g, b);            //(a);

      /*
      putColorsInArray(count%LEDSHORGROOT, count/LEDSHORGROOT, w, bak, r, g, b);    //(count);   
      int a = LEDSHORGROOT*LEDSVERT-count;
      putColorsInArray(a%LEDSHORGROOT, a/LEDSHORGROOT,w,bak,r, g, b);            //(a);
      */

      count++;
      //Serial.println(count);
  }

  if(count > getBakWidth(bak)*LEDSVERT/2+1)
  {
    //clearGrid();
    //count = 0;
  }
  
  //apply interval
  applyInIntervalTime(fps);
}

/*--------------------------------------
      animations
      Horizontaal
      For all 5 bakken
---------------------------------------*/

void fillBothSidesHorAll(int fps, int r, int g, int b, int rFade, int gFade, int bFade)
{

  for (int i = 0; i < 5; i++)
  {

      if(counters[i] > getBakWidth(1)*LEDSVERT/2+1)
      {
        //clearGrid();
        //counters[i] = 0;
      }

      //if the previous state is applied continue to the next animation step
      if(ledsApplied == true)
      {
          putColorsInArray(counters[i]+getBakStartpoint(i),r,g,b);
          int a = getBakWidth(i)*LEDSVERT-counters[i]-1;
          putColorsInArray(a+getBakStartpoint(i),rFade, gFade, bFade);            //(a);

          
          //putColorsInArray(count%LEDSHORGROOT, count/LEDSHORGROOT, w, bak, r, g, b);    //(count);   
          //int a = LEDSHORGROOT*LEDSVERT-count;
          //putColorsInArray(a%LEDSHORGROOT, a/LEDSHORGROOT,w,bak,r, g, b);            //(a);
          

          if(ledsApplied == true)
          {

            if (counters[i] > getBakWidth(i)*LEDSVERT/2-2)
            {
              //gFade = rFade/2; //middle fade
            } else
            {
              counters[i] = counters[i]+1;
            }

          }
          //if(counters[0] > getBakWidth(0)*LEDSVERT/2+1)
          //{
           // counters[0] = getBakWidth(0)*LEDSVERT/2+1;
          //}

          //Serial.println(count);
      }
    }
  //count++;

  //apply interval
  applyInIntervalTime(fps);
}

/*--------------------------------------
      Verticaal
---------------------------------------


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
}*/

void fill(int fps)
{
  if(ledsApplied)
  {  
    if(count%3 == 0)
    {
      //putRandomColorsInArray(count);
      putColorsInArray(count,getColorsFromSensors());
    }
      
    
    count++;
  }

  if(count == arrayLength+1)
  {
    clearGrid();
    count = 0;

  }
  
  //apply interval
  applyInIntervalTime(fps);
}

void flash()
{
  //loop throught the total amount of leds available
  for (int i = 0; i<arrayLength; i++)
  {
    //turn off/set leds to black
    putColorsInArray(i,255,255,255);
  }
  //apply once
  applyColorsToStrip();
  clearGrid();
}

void everyOtherLedFlash(int fps)
{
  //if the previous state is applied continue to the next animation step
  if(ledsApplied)
  {
    for(int count= 0; count < arrayLength-1;count++)
    {    
      if(count%2==0 && switcher == 1)
      {
        putColorsInArray(count, getColorsFromSensors());    
      }
      else if(count%2==1 && switcher == 1)
      {
        putColorsInArray(count,0,0,0);
      }
      
      if(count%2==0 && switcher == 0)
      {
        putColorsInArray(count,0,0,0);
      }
      else if(count%2==1 && switcher == 0)
      {
        putColorsInArray(count, getColorsFromSensors());
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

/*-----------------------------------------
  horizontal color fade
-----------------------------------------*/

void colorFadeHor(int fps, int r, int g, int b, int rFade, int gFade, int bFade)
{

  for (int i = 0; i < 5; i++)
  {

      if(counters[i] > getBakWidth(i)*LEDSVERT/2+1)
      {
        clearGrid();
        counters[i] = 0;
      }

      //if the previous state is applied continue to the next animation step
      if(ledsApplied == true)
      {
          putColorsInArray(counters[i]+getBakStartpoint(i),r,g,b);
          int a = getBakWidth(i)*LEDSVERT-counters[i]-1;
          putColorsInArray(a+getBakStartpoint(i),rFade, gFade, bFade);          //(a);

          /*
          putColorsInArray(count%LEDSHORGROOT, count/LEDSHORGROOT, w, bak, r, g, b);    //(count);   
          int a = LEDSHORGROOT*LEDSVERT-count;
          putColorsInArray(a%LEDSHORGROOT, a/LEDSHORGROOT,w,bak,r, g, b);            //(a);
          */

          if(ledsApplied == true)
          {
            counters[i] = counters[i] + 1;
          }

      }
    }

    //if(ledsApplied == true){
    //  counters[i] = counters[i] + 1;
    //}
  //count++;

  //apply interval
  applyInIntervalTime(fps);
}



void knightRider(int fps, int swipeWidth, int bak, int r, int g, int b)
{
  

  //if the previous state is applied continue to the next animation step
  if(ledsApplied)
  {

/*----------------------------------------------- 
wanneer directionSwitch false is beweegt de swipe
van links naar rechts
------------------------------------------------*/
Serial.println(counters[bak]);

    if (directionSwitchers[bak] == false)
    {
      //loop om het aantal ledjes dat naast de count aan moet gaan te bepalen (swipeWidth)
      for(int i=0; i<swipeWidth;i++)
      {
        //loop om het effect op meerdere rijen (Y as) toe te passen.
        for(int y=0; y<LEDSVERT; y++)
        {
         //leds worden aangezet (de count led + het aantal dat je invult bij SwipeWidth)
         putColorsInArray(counters[bak]+i,y,bak, getColorsFromSensors()); 
        } 
      }
      
      //turn of previous led
      // zolanf de count geen 0 is zet het ledje links van count uit
      if(counters[bak] != 0)
      {
        //loop om het uitzetten van de leds op meerdere rijen (Y as) toe te passen.
        for ( int y = 0; y<LEDSVERT; y++)
        {
         //leds worden uitgezet
         putColorsInArray(counters[bak]-1,y,bak,0,0,0); 
        }
      }
      //count gaat gaat met 1 omhoog aan het einde van elke loop om de swipe van links naar rechts te bewegen
      counters[bak] = counters[bak]+1;


      //wanneer count+swipeWidth hoger of gelijk is aan de hoeveelheid horiontale LEDS zet directionSwitch op true
      if(counters[bak]+swipeWidth-1 >= getBakWidth(bak))
      {
       // count ++;
        directionSwitchers[bak] = true;
        //extreme = true;
      }
      
    }

    
/*----------------------------------------------- 
wanneer directionSwitch true is beweegt de swipe
van rechts naar links
------------------------------------------------*/
   
    if(directionSwitchers[bak] == true)
    {
       //aan het eind van de loop count-1
      counters[bak] = counters[bak]-1;
      //loop om het aantal ledjes dat naast de count aan moet gaan te bepalen (swipeWidth)
      for(int i=0; i<swipeWidth;i++)
      {
        //loop om het effect op meerdere rijen (Y as) toe te passen.
        for (int p=0; p<LEDSVERT; p++)
        {
          //leds worden aangezet
          putColorsInArray(counters[bak]+i,p,bak, getColorsFromSensors()); 
        }
      } 
      
      //turn of previous led
      //zolang de count+swipeWidth niet gelijk is aan het totaal aantal horizontale LEDS
      if (counters[bak]+swipeWidth != getBakWidth(bak))
      { 
        //pas het uitzetten van de ledjes toe op meerder rijen (Y as)
        for(int i=0; i<LEDSVERT; i++)
        {
          // zet ledjes count+swipeWidht uit
          putColorsInArray(counters[bak]+swipeWidth,i,bak,0,0,0);
        }
      }

      //wanneer count gelijk is aan 0 zet directionSwitch weer op false
      if(counters[bak] == 0)
      {
        directionSwitchers[bak] = false;
        //extreme = true;
      }
    }
   
  }

  //apply interval
  //applyInIntervalTime(fps);
}

void newAnimation(int fps, int bak, int r, int g, int b)
{
  if (ledsApplied)
  {
    if(directionSwitch == false)
    {
      for (int i = 0; i < 5; i++)
      {
        Serial.print("hello");
        for(int x = 0; x < getBakWidth(bak); x++)
        {
          putColorsInArray(x,count, i, r,g,b);
          //putColorsInArray(x, count-1, bak, 0,0,0);
        }
      }

      count++;

      if(count == LEDSVERT)
      {
        //directionSwitch = true;
        //Serial.println(directionSwitch);
      }
    }

    /*if(directionSwitch == true)
    {
      for (int i = 5; i > 0; i--)
      {
        for(int x = 0; x > getBakWidth(i); x--)
        {
          putColorsInArray(x,count, i, r,g,b);
          putColorsInArray(x, count+1, i, 0,0,0);
        } 
      }
      count--;
      if(count == LEDSVERT)
      {
        directionSwitch = true;
      }
    }*/

  }
}


void fillTopToBottom(int fps)
{
  if(ledsApplied)
  {
    for(int i = 0; i < 5; i++)
      {
        for(int x = 0; x < getBakWidth(i); x++)
        {
          putColorsInArray(x,count, i, RandomKleur);
          //putColorsInArray(x, count-1, i, r,g,b);
        }
      }

      count++;

      Serial.println(RandomKleur);
      /*
      rGain = rGain+12;
      if(rGain > 255 && count == 0)
      {
        rGain = 3;
      }*/

      //gGain = gGain+30;
      //bGain = bGain+40;


      Serial.println(count);
      Serial.println(rGain);

      if(count == 7)
      {
        Serial.println("doe dan");
        count = 0;
        trigger++;
      }

      if(trigger == 5)
      {
        RandomKleur = generateRandomColors();
        trigger = 0;
      }

  }
  applyInIntervalTime(fps);

}


void FillWave(int fps, int r, int g, int b)
{
  if(ledsApplied)
  {
  
      for(int y = 0; y < 7; y++)
      {
        for(int i = 0; i < 5; i++)
        {
          putColorsInArray(count, y, i, r, g, b);
        }
        //putRandomColorsInArray(count);
      }
      
    
    count++;
  }

  if(count > 29)
  {
    clearGrid();
    count = 0;

  }
  
  //apply interval
  applyInIntervalTime(fps);
}


void fillHalfSensors(int fps, int r, int g, int b)//, int rFade, int gFade, int bFade)
{
  if(ledsApplied)
  {
    for(int i = 0; i < 5; i++)
      {
        for(int x = 0; x < getBakWidth(i); x++)
        {
          if(count > LEDSVERT/2)
          {
            Serial.println("doe dan");
            putColorsInArray(x,count, i, getColorsFromSensors());//rFade, gFade, bFade);
          }
          else
          {
            putColorsInArray(x,count, i, r, g, b);
          }
        }
      }

      count++;
      Serial.println(count);


      if(count == LEDSVERT)
      {
        count = 0;
      }

  }
  applyInIntervalTime(fps);

}


void fillHalfIdle(int fps, int r, int g, int b, int rFade, int gFade, int bFade)
{
  if(ledsApplied)
  {
    for(int i = 0; i < 5; i++)
      {
        for(int x = 0; x < getBakWidth(i); x++)
        {
          if(count > LEDSVERT/2)
          {

            putColorsInArray(x,count, i, rFade, gFade, bFade);
          }
          else
          {
            putColorsInArray(x,count, i, r, g, b);
          }
        }
      }

      count++;
      Serial.println(count);


      if(count == LEDSVERT)
      {
        count = 0;
      }

  }
  applyInIntervalTime(fps);

}

void fillHorAll(int fps)
{
  if (ledsApplied)
  {
    for (int y = 0; y< LEDSVERT; y++)
    {
      for (int bak = 0; bak < 5; bak++)
      {
        putColorsInArray(count, y, bak, RandomKleur);
      }
    }
    count++;
    if(count > 28)
    {
      count = 0;
      RandomKleur = generateRandomColors();
    }
  }

  applyInIntervalTime(fps);
}

void fillHorAllTotal(int fps)
{

  if( init == false)
    {
      RandomKleur = 255,255,255;
      init = true;
      Serial.println(init);
    }

  //RandomKleur = generateRandomColors(); // regenboogkots hor
  if (ledsApplied)
  {
    for (int y = 0; y< LEDSVERT; y++)
    {
      
      putColorsInArray(count, y, 0+i, RandomKleur);
        
        Serial.println(i);

    }

    count++;

    if (count == 16 && i == 0 ||
        count == 29 && i == 1 ||
        count == 25 && i == 2 ||
        count == 29 && i == 3 )
    {
      count = 0;
      i++;
    }

    if(count == 29 && i == 4)
    {
      i = 0;
      count = 0;
      RandomKleur = generateRandomColors();
    }

  }

  applyInIntervalTime(fps);
}

void knightRiderHor(int fps)
{

  if(ledsApplied)
  {
    //fillGrid(rBack, gBack, bBack);
    for (int y = 0; y< LEDSVERT; y++)
    {
      putColorsInArray(count, y, i, RandomKleur);
      if(count != 0)
      {
        putColorsInArray(count-1, y, i, 0,0,0);
      }

    }

    count++;

    if (count == 16+1 && i == 0 ||
        count == 29+1 && i == 1 ||
        count == 25+1 && i == 2 ||
        count == 29+1 && i == 3 )
    {
      count = 0;
      i++;
    } else if (count == 29 && i == 4)
    {
      i = 0;
      count = 0;
      RandomKleur = generateRandomColors();
    }

    if (count == 0)
    {
      for(int bak = 0; bak<5; bak++)
      {
        clearBak(bak);
      }
    }

  }

  applyInIntervalTime(fps);

}

void horWipe(int fps)
{

  if( init == false)
    {
      RandomKleur = 255,255,255;
      init = true;
      Serial.println(init);
    }

  //RandomKleur = generateRandomColors(); // regenboogkots hor
  if (ledsApplied)
  {
    for (int y = 0; y< LEDSVERT; y++)
    {
      
      putColorsInArray(count, y, i, RandomKleur);
      putColorsInArray(count2, y, 2, RandomKleur);
        
        Serial.println(i);

    }

    count++;
    if(count < 17)
    {
      count2++;
    }

    if (count == 7 && i == 0)
    {
      count = 0;
      i = 2;
    }

    if(count == 29 && i == 4)
    {
      i = 0;
      count = 0;
      RandomKleur = generateRandomColors();
    }

  }

  applyInIntervalTime(fps);
}

/************************************************
 * Color fading functions. Call this function with a decreased fps equal to fps/steps.
 ************************************************/
uint32_t fadeColorInSteps(uint32_t sourceColor, uint32_t targetColor, int getStep, int steps)
{
  int red =   fadeColorInSteps(returnColor(sourceColor,'r'), returnColor(targetColor,'r'), getStep, steps);
  int green = fadeColorInSteps(returnColor(sourceColor,'g'), returnColor(targetColor,'g'), getStep, steps);
  int blue =  fadeColorInSteps(returnColor(sourceColor,'b'), returnColor(targetColor,'b'), getStep, steps);
  return ((uint32_t)red << 16)| ((uint32_t)green << 8) |((uint32_t) blue);
}
int fadeColorInSteps(int sourceColor, int targetColor, int getStep, int steps)
{
  int color = 0;
  if(getStep > steps) getStep = steps;
  if(targetColor > sourceColor)
  {
    color = (int)sourceColor + (((targetColor - sourceColor)/steps)*getStep);
  }
  else
  {
    color = (int)sourceColor - (((targetColor - sourceColor)/steps)*getStep);    
  }
  return color;
}