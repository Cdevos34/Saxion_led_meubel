#include <OctoWS2811.h>

const int LEDSHOR =  16;
const int LEDSVERT = 7;
const int  LEDAMOUNT = LEDSHOR*LEDSVERT;


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
bool seatOneOn = false;

//animation variables
int count = 0;
//--------
const byte numChars = 8;  //array length
char splitData[numChars]; // an array to store the received data
char prevData[numChars];  // comparer for changes in data
boolean newData = false;  //checks if the latest input is handled correctly

void setup() {
 Serial.begin(9600);
 
 Serial.println("<Arduino is ready>");
}

// expecting data in serial consisting of *0,0,0,0,0,0,0,0$ where 0 is the single char datapoint

void loop() {
    //receiveOnlyUpdatedData();
    //Serial.println("<Arduino is ready>");
    fillBothSidesVert(10,255,0,0);
      //if(seatOneOn == true){
       // Serial.println ("test");
        //fillBothSidesVert(10,255,0,0);
      //}
}

void receiveOnlyUpdatedData() {
 static byte index = 0;
 char startMarker = '*';
 char endMarker = '$';
 char divider = ',';
 char serialInput;

 
 while (Serial.available() > 0 && newData == false) {
   serialInput = Serial.read();
  
   if (serialInput != endMarker && serialInput != divider && serialInput != startMarker)
   {       
       splitData[index] = serialInput;
       index++;
   }
   else if(serialInput == endMarker){
       splitData[index] = '\0'; // terminate the string
       index = 0;
       newData = true;
       
       //showNewData(); 
       applyChangedData();
       
       newData = false;
   }
  }
}

void applyChangedData()
{ 
  for(int i=0; i < 4; i++)
  {
    if(splitData[i] != prevData[i])
    {
       
      //0-3 is ass
      if(i<4){
      fatAssDetector(i);
      }
      else{

      }
      //4-7 is afstand
      if(i>3){}


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


// i==2 is een check voor welke sensor er getriggered word
// && splitData[i] == 1 betekend dat sensor getriggered is
// i 0 tm 3 zijn bankjes dus 0 of 1 
// i 4 tot 7 zijn afstand sensoren dus 0 is geen meting 1 t/m 4 zijn verschillende afstanden

void fatAssDetector(int i)
{
 if(i==3 && splitData[i] == '1')
      {
        seatOneOn = true;
        Serial.println(seatOneOn);
          //doe een zit animatie voor dit bankje
          //fillBothSidesVert(10, 255,0,255);
      } 
      /*else 
      {
        seatOneOn = false;
        Serial.println(seatOneOn);
      }*/

}

/*void (int i)
{
 if(i==4 && splitData[i] == 1)
      {
        fatLegDetector = true;
          //doe een zit animatie voor dit bankje
          fillBothSidesVert(10, 255,0,255);
      } else 
      {
        seatOneOn = false;
      }

}*/

uint32_t haalKleurenUitAfstandsSensoren()
{

  return 0;
}

/*--------------------------------------------
FPS setup
----------------------------------------------*/

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

void applyColorsToStrip(){//uint32_t colorArray[LEDSHOR][LEDSVERT], int arrayLength){

  Serial.println("start apply colors");
  
  for(int y=0; y<LEDSVERT; y++){
      for(int x=0; x<LEDSHOR; x++){
          //fetch the colors    
          uint8_t red = currentColors[x][y] >> 16;
          uint8_t green = currentColors[x][y] >> 8;
          uint8_t blue = currentColors[x][y];
          Serial.println("turn on");
          
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


/*--------------------------------------------
Animation variables 
----------------------------------------------*/


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
  Serial.println("leds");
  applyInIntervalTime(fps);
}

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

/*----------------------------------------
bank en sensor triggers
----------------------------------------*/

void loop(){

  if(ledsApplied)
  {
    for(int i = 0; i<4; i ++)
    {
      if(i != 0)
      {
        FatAssAnimation(i)
      }
    }
      else if(array[4] != '0' || array[5] != '0' || array[6] != '0' || array[7] != '0')
      {
          KnightRiderExtreme()
      }
      else if(array[4] != '0' || array[5] != '0' || array[6] != '0' || array[7] != '0')
  } 
}
