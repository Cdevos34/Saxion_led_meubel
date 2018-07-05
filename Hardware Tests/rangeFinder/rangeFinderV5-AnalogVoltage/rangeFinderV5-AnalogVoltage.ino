const int sensorPin = 23;
int distance = 0;
const int sampleSize = 50;
int distanceCm[sampleSize];

void setup(){

	Serial.begin(9600);
}

void loop(){

	for(int i=0; i<sampleSize; i++){
		distance = analogRead(sensorPin);
		distanceCm[i] = (distance/2)/0.3937;
		//delay();
	}
	
	Serial.println(returnMedian(distanceCm, sampleSize));
	//Serial.println(distanceCm);
	delay(100);
}

int returnAverage(int values[], int valueAmount){

	int average = 0;
	long total = 0;
	for(int i=0; i<valueAmount; i++){
		total = total + values[i];    
	}
	average = total/valueAmount;

	return average;
}

int returnMedian(int values[], int valueAmount){
	//sort values from small to big using bubble sort (dont use too much values!)
	
	//Calculate the amount of passes that needs to be done e.g. one less then the amount of numbers
	int sortAmounts = valueAmount-1;
	//Keep track of the amount of passes done. The program can skip the array places already sorted to save time
	int count = 0;
	for(int i=0; i<sortAmounts; i++){
		//loop through the array skipping though the places already sorted by substracting the counts and skipping the last value
		for(int i=0; i<valueAmount-count-1; i++){
		    //compare the current value to the next value
		    if(values[i] > values[i+1]){
		    	//if the value is bigger, move it up the chain by swapping it
		    	//save the second value temporary
		    	int saveIt = values[i+1];
		    	//overwrite the second value with the first value
		    	values[i+1] = values[i];
		    	//overwrite the first value with the saved value
		    	values[i] = saveIt;
		    }
		}
		//we've just done a pass, so lets record that fact
		count ++;
	}
	return values[valueAmount/2 + 1];
}