const byte numChars = 8;  //array length
byte splitData[numChars]; // an array to store the received data
char prevData[numChars];  // comparer for changes in data
boolean newData = false;  //checks if the latest input is handled correctly

void setup() {
 Serial.begin(19200);
 Serial.println("<Arduino is ready>");

 Serial1.begin(19200);
}

// expecting data in serial consisting of *0,0,0,0,0,0,0,0$ where 0 is the single char datapoint

void loop() {
    receiveOnlyUpdatedData();
}

void receiveOnlyUpdatedData() {
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
       
       //showNewData(); 
      // applyChangedData();
       
       for(int i=0; i<numChars; i++){
           Serial.print(splitData[i]);
       }
       Serial.println();
       newData = false;
   }
  }
}

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
