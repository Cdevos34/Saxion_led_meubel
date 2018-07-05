//Todo: Update it so it returns an 32bit int with the rgb values instead of needing 3 global variables

void convertColor(float hue_, float saturation_, float lightness_){
  //This function converts hsl to rgb
  //It needs three global variables; int red; int green; int blue;
  //to return and store the matching rgb values

  //Convert hue, sat and lightness to numbers between 0 and 1
  float hueFloat = hue_/360;
  float saturationFloat = saturation_/100;
  float lightnessFloat = lightness_/100;
  float temp1 = 0;
  float temp2 = 0;

  //Create some temporaryValues for easy calculating
  if(lightnessFloat < 0.5){
  	temp1 = lightnessFloat*(1+saturationFloat);
  }
  else{
  	temp1 = lightnessFloat+saturationFloat - lightnessFloat*saturationFloat;
  }

  //Create temp2
  temp2 = 2*lightnessFloat - temp1;

  //Create some temporary values for each color channel
  //These Values need to be between 0 and 1(if they're not, Fix it!)
  
  float tempR = hueFloat + 0.333;
  if(tempR >1){
  	tempR -=1;
  }
  else if(tempR < 0){
  	tempR +=1;
  }

  float tempG = hueFloat;

  float tempB = hueFloat - 0.333;
  if(tempB >1){
  	tempB -=1;
  }
  else if(tempB <0){
  	tempB +=1;
  }

  //Using the temp values of each channel, we make some tests to select the correct formula
  //RED CHANNEL
  float redFloat = 0;

  if((6*tempR) < 1){
  	redFloat = temp2 + (temp1 - temp2)*6*tempR;
  }
  else if(2*tempR < 1){
  	redFloat = temp1;
  }
  else if(3*tempR < 2){
  	redFloat = temp2 + (temp1 - temp2)*(0.666 -tempR)*6;
  }
  else{
  	redFloat = temp2;
  }
  //GREEN CHANNEL
  float greenFloat = 0;

  if((6*tempG) < 1){
  	greenFloat = temp2 + (temp1 - temp2)*6*tempG;
  }
  else if(2*tempG < 1){
  	greenFloat = temp1;
  }
  else if(3*tempG < 2){
  	greenFloat = temp2 + (temp1 - temp2)*(0.666 -tempG)*6;
  }
  else{
  	greenFloat = temp2;
  }

  //BLUE CHANNEL
  float blueFloat = 0;

  if((6*tempB) < 1){
  	blueFloat = temp2 + (temp1 - temp2)*6*tempB;
  }
  else if(2*tempB < 1){
  	blueFloat = temp1;
  }
  else if(3*tempB < 2){
  	blueFloat = temp2 + (temp1 - temp2)*(0.666 -tempB)*6;
  }
  else{
  	blueFloat = temp2;
  }

  red = redFloat * 255;
  green = greenFloat * 255;
  blue = blueFloat * 255; 
}