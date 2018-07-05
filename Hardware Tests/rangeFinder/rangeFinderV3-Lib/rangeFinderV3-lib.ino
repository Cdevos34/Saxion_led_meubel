
void setup()
{
  Serial.begin(9600);
  Serial3.begin(9600);
}

void loop()
{
  unsigned long start;
  
  Serial.println("Reading...");
  
#ifdef MAXBOTIX_WITH_SOFTWARE_SERIAL
  // TX
  start = millis();
  Serial.print("TX: ");
  Serial.print(rangeSensorTX.getRange());
  Serial.print("cm - ");
  Serial.print(millis() - start);
  Serial.println("ms");
#endif

	delay(5000);
}
