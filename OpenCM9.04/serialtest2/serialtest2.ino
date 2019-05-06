unsigned long time;
unsigned long readTime; 
int led_pin = LED_BUILTIN; // 13 for Uno/Mega2560, 14 for OpenCM

const int WAITING = 0;
const int DONE = 1;

int state = WAITING;

int value;

long count = 0;
double sum = 0;

String str;

float a3 = 0.5;
float a2 = 0.1234;
float a1 = 12.456;
float a0 = -6.145;

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
  sum = poly(millis()/1000);
  count++;
  if (millis() > 1000*5 && state == WAITING) {
    Serial.println(count);
    Serial.println(count/millis());
    Serial.println(sum);
    state = DONE;
  }
}

void readData() {
  if(Serial.available()>0)
  {
//    str = Serial.readStringUntil('\n');
//      value = Serial.read();
//    idx = Serial.parseInt();
//    x = Serial.parseFloat();
//    y = Serial.parseFloat();
//    z = Serial.parseFloat();
//    theta = Serial.parseFloat();
//    Serial.print("A ");
//    Serial.print(idx); Serial.print(' ');
//    Serial.print(x); Serial.print(' ');
//    Serial.print(y); Serial.print(' ');
//    Serial.print(z); Serial.print(' ');
//    Serial.print(theta, 5);
//    Serial.print(str);
    Serial.println();
  }
}

float poly(float t) {
  return a3*t*t*t + a2*t*t + a1*t + a0;
}
