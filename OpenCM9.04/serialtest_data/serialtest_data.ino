unsigned long time;
unsigned long readTime; 
int led_pin = LED_BUILTIN; // 13 for Uno/Mega2560, 14 for OpenCM

const int WAITING = 0;
const int RECEIVING = 1;

int state = WAITING;

int value;

String str;

int idx;
float x;
float y;
float z;
float theta;

void setup()
{
  pinMode(led_pin, OUTPUT);
  Serial.begin(57600);
  while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB
    }
    Serial.println("RDY");
}

void loop()
{
  readData();
}

void readData() {
  if(Serial.available()>0)
  {
//    str = Serial.readStringUntil('\n');
//      value = Serial.read();
    idx = Serial.parseInt();
    x = Serial.parseFloat();
    y = Serial.parseFloat();
    z = Serial.parseFloat();
    theta = Serial.parseFloat();
//    Serial.print("A ");
    Serial.print(idx); Serial.print(' ');
//    Serial.print(x); Serial.print(' ');
//    Serial.print(y); Serial.print(' ');
//    Serial.print(z); Serial.print(' ');
//    Serial.print(theta, 5);
//    Serial.print(str);
    Serial.println();
  }
}
