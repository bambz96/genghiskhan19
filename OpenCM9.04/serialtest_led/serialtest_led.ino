unsigned long time;
unsigned long readTime; 
int led_pin = 14;
int value; 

void setup()
{
pinMode(led_pin, OUTPUT);
Serial.begin(57600);
}

void loop()
{
  if(Serial.available()>0)
  {
    time = millis();
    value=Serial.read();
    readTime = time-millis();
    
    if  (value == 1)           
    {
    digitalWrite(led_pin, HIGH);
    Serial.print("Read time =");
    Serial.print(readTime);
    Serial.println("ms");
    }
    if(value == 0)         
    { 
    digitalWrite(led_pin, LOW);
    Serial.print("Read time =");
    Serial.print(readTime);
    Serial.println("ms");
    }
  }
}
