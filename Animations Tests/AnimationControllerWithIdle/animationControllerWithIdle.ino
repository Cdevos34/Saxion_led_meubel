#include <OctoWS2811.h>

//Communnication variables
const byte numChars = 8;  //array length
byte splitData[numChars]; // an array to store the received data
//char prevData[numChars];  // comparer for changes in data
boolean newData = false;  //checks if the latest input is handled correctly

//led control variables
const int LEDSHOR =  160;
const int LEDSVERT = 7;
uint32_t currentColors[LEDSHOR][LEDSVERT];
const int  LEDAMOUNT = LEDSHOR*LEDSVERT;

//octoshield variables
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
bool rood = false;
bool groen = false;
bool blauw = false;

void setup() {
 //Initialize the library
 leds.begin();
 //Update the led's Turning them all to black/off
 leds.show(); 
  
 Serial.begin(19200);
 Serial.println("<Arduino is ready>");
 //Serial.write("*0,0,0,0,0,0,0,0$");

 //receiver serial
 //Serial1.begin(19200);
}

// expecting data in serial consisting of *0,0,0,0,0,0,0,0$ where 0 is the single char datapoint

void loop()
{
  receiveOnlyUpdatedData();
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
    //animate chairs
    for(int i=0; i<4;i++)
    {
      if(splitData[i] == 49)
      {
        chairAnimation(i);  
      }
    }
    applyChairAnimation();
  }
  else if(splitData[4] != 48 ||
  splitData[5] != 48 ||
  splitData[6] != 48 ||
  splitData[7] != 48)
  {
    walkByAnimation();
  }
  else
  {
    idleAnimation();
  }
}

void chairAnimation(int i)
{
  Serial.print("chair animation: ");
  Serial.println(i);  
  printAllData();
}

void applyChairAnimation()
{
  Serial.println("chair animation applier");  
  printAllData();
}

void walkByAnimation()
{
  Serial.println("walk by animation");
  printAllData();
}

void idleAnimation()
{
  Serial.println("idle animation");
  printAllData();
  fillBothSidesHor(10, 255,255,0);
}

uint32_t getColorsFromSensors()
{
  int redIntensity = 0;
  int greenIntensity = 0;
  int blueIntensity = 0;

  if( splitData[4] == '0' &&
      splitData[5] == '0' &&
      splitData[6] == '0' &&
      splitData[7] == '0')
  {
    return generateRandomColors();   //hier random kleur pakken?
  }

  //dit verder randomizen met waardes tussen de 0-85?
  for(int i=4; i<8; i++)
  {
    if(splitData[i] == '1')
    {
      redIntensity = redIntensity + 64;
    }
    else if(splitData[i] == '2')
    {
      greenIntensity = greenIntensity + 64;
    }
    else if(splitData[i] == '3')
    {
      blueIntensity = blueIntensity + 64;
    }
  }

  return ((uint32_t)redIntensity << 16)| ((uint32_t)greenIntensity << 8) |((uint32_t) blueIntensity); //fill color with above values
}


void receiveOnlyUpdatedData() {
  static byte index = 0;
  char startMarker = '*';
  byte endMarker = 36;
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
         //applyChangedData();
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

/*
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
*/

/*-------------------------------
Animations
------------------------------*/

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


