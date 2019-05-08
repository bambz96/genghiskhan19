#define FLOAT_TO_INT 1000
// states
#define WAITING 0
#define RECEIVING_X 1
#define RECEIVING_Y 2
#define RECEIVING_Z 3
#define RECEIVING_TH 4
#define PLOTTING 5 // send all paths (t, x, y, z) back to Matlab
#define SIMULATION 6 // simulate measurement/control
#define POSITION_CONTROL 7

int led_pin = LED_BUILTIN; // 13 for Uno/Mega2560, 14 for OpenCM

int state = WAITING;

int value;
int count = 0;

// stores cubic polynomial coefficients and duration tf
// int instead of float to halve needed bytes (4 -> 2)
struct Cubic {
  int coef[4]; // FLOAT_TO_INT times larger than actual value
  int tf; // milliseconds
};

// 54 blocks, so this gives ~10 polys per block
// a single path is 6-9 polys
//struct Cubic polys[500];
struct Cubic xpoly[100];
struct Cubic ypoly[100];
struct Cubic zpoly[100];
struct Cubic thpoly[100];

// todo
// plotting does correct length, maybe tells matlab how many samples are being sent
// save t0 = millis() when beginning any path, then use dt = millis() - t0 to get path
// SIMULATION state - fake read, fake control, fake current position sent back to matlab which compares with intended path?
// matlab
// handle multiple paths being returned

void setup()
{
  pinMode(led_pin, OUTPUT);
  Serial.begin(57600);
  while (!Serial) {} // wait for serial port to connect. Needed for native USB
}

void loop() {
  if (state == WAITING) {
    if(Serial.available()>0) {
      value = Serial.parseInt();
      Serial.read(); // clear rest of input buffer (i.e. trailing \n)
      Serial.println(value);
      state = RECEIVING_X;
    } else {
//      Serial.println('A'); // hello?
//      delay(300);
    }
  } else if (state == RECEIVING_X) {
    readData(xpoly);
    if (count >= value) {
      count = 0;
      state = RECEIVING_Y;
    }
  } else if (state == RECEIVING_Y) {
    readData(ypoly);
    if (count >= value) {
      count = 0;
      state = RECEIVING_Z;
    }
  } else if (state == RECEIVING_Z) {
    readData(zpoly);
    if (count >= value) {
      count = 0;
      state = RECEIVING_TH;
    }
  } else if (state == RECEIVING_TH) {
    readData(thpoly);
    if (count >= value) {
      count = value; // should assert count == value
      state = POSITION_CONTROL;
    }
  } else if (state == PLOTTING) {
    // evaluate and send paths back so matlab can plot in 3D the trajectory for validation
    int verify = 1; // number of paths to verify
    sendNPoly(verify, xpoly);
    sendNPoly(verify, ypoly);
    sendNPoly(verify, zpoly);
    sendNPoly(verify, thpoly);
    // reset count of most recent read in rows
    count = 0;
    // clear polys arrays?
    state = WAITING;
  } else if (state == POSITION_CONTROL) {
    unsigned int t0 = millis();
    unsigned int dt = 0;
    unsigned int tf = 3;
    while (dt < tf) {
      // get task space coord
      float x = evaluate(&xpoly[0], dt/1000.0);
      float y = evaluate(&ypoly[0], dt/1000.0);
      float z = evaluate(&zpoly[0], dt/1000.0);

      // get joint space with IK


      // write joint space to servos

      dt = millis() - t0;
    }
  }
}

void readData(struct Cubic *poly) {
  // "a3 a2 a1 a0 tf" has been sent, parse it in, create a new Cubic struct
  // and add it to the array provided
  if(Serial.available()>0)
  {
    float a3 = Serial.parseFloat();
    float a2 = Serial.parseFloat();
    float a1 = Serial.parseFloat();
    float a0 = Serial.parseFloat();
    float tf = Serial.parseFloat();
    Serial.read(); // clear rest of input buffer (i.e. trailing \n
    // reply with read values
    Serial.print(a3); Serial.print(' ');
    Serial.print(a2); Serial.print(' ');
    Serial.print(a1); Serial.print(' ');
    Serial.print(a0); Serial.print(' ');
    Serial.print(tf); Serial.print(' ');
    Serial.println();
    // create Cubic struct and save to given array of polynomials
    struct Cubic cubic;
    cubic.coef[0] = int(a0*FLOAT_TO_INT);
    cubic.coef[1] = int(a1*FLOAT_TO_INT);
    cubic.coef[2] = int(a2*FLOAT_TO_INT);
    cubic.coef[3] = int(a3*FLOAT_TO_INT);
    cubic.tf = tf*1000; // convert s to ms
    poly[count] = cubic;
    count++;
  }
}

float poly(float t, float a0, float a1, float a2, float a3) {
  // evaluate the given cubic polynomial
  return a3*t*t*t + a2*t*t + a1*t + a0;
}

void sendNPoly(int n, struct Cubic cubic[100]) {
  // send the first n polynomial path segments
  float t0 = 0.0;
  for (int i=0; i<n; i++) {
    for (int j=0; j<100; j++) {
      float t = j/100.0 * cubic[i].tf/1000.0; // convert to real duration
      sendPolyAtTime(t, t0, &cubic[i]);
    }
    t0 += cubic[i].tf/1000.0;
  }
}

void sendPolyAtTime(float t, float t0, struct Cubic *cubic) {
  // send ti and x(ti)
  float a0 = float(cubic->coef[0])/FLOAT_TO_INT;
  float a1 = float(cubic->coef[1])/FLOAT_TO_INT;
  float a2 = float(cubic->coef[2])/FLOAT_TO_INT;
  float a3 = float(cubic->coef[3])/FLOAT_TO_INT;
  float x = poly(t, a0, a1, a2, a3);
  Serial.print(t+t0, 5); Serial.print(' ');
  Serial.print(x, 5); Serial.print(' ');
  Serial.println();
}

float evaluate(struct Cubic *cubic, float t) {
   float a0 = float(cubic->coef[0])/FLOAT_TO_INT;
    float a1 = float(cubic->coef[1])/FLOAT_TO_INT;
    float a2 = float(cubic->coef[2])/FLOAT_TO_INT;
    float a3 = float(cubic->coef[3])/FLOAT_TO_INT;
    return poly(t, a0, a1, a2, a3);
}
