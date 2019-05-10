#define FLOAT_TO_INT 100000 // experiment storying cubics with int coefficients for storage, probably not worth the hassle and potential loss in accuracy
#define MAX_CUBICS 10
// states
#define WAITING 0			// listen for communication from Matlab over serial, which send N, the number of path segments coming
#define RECEIVING_X 1		// receive polynomial coefficients for all N cubic path segments x(t)
#define RECEIVING_Y 2		//     ... for y(t)
#define RECEIVING_Z 3		//     ... for z(t)
#define RECEIVING_TH 4		//     ... for theta(t)
#define PLOTTING 5			// send all paths (t, x, y, z) back to Matlab
#define SIMULATION 6		// simulate measurement/control
#define POSITION_CONTROL 7	// position control, no feedback
#define FINISHED 8			// do nothing

int led_pin = LED_BUILTIN; // 13 for Uno/Mega2560, 14 for OpenCM

int state = WAITING;

int nPolys;		// number of polynomials sent by Matlab and stored for operation
int count = 0;	// used to count up to nPolys whilst receiving coefficients from Matlab, and to hold current cubic path segment while operating

// stores cubic polynomial coefficients and duration tf
// int instead of float to halve needed bytes (4 -> 2)
struct Cubic {
  int coef[4]; // FLOAT_TO_INT times larger than actual value
  unsigned int tf; // milliseconds
};

struct Cubic xpoly[MAX_CUBICS];
struct Cubic ypoly[MAX_CUBICS];
struct Cubic zpoly[MAX_CUBICS];
struct Cubic thpoly[MAX_CUBICS];

// task space coordinates
typedef struct {
  float x; 
  float y;
  float z;
  float theta; 
} X_t;

// joint space coordinates
typedef struct {
  float q1;
  float q2;
  float q3;
  float q4;
  float q5;
} Q_t;

X_t X;

Q_t Q = {0, 0, 0, 0, 0};

float L1 = 0.2;
float L2 = 0.2;
float L3 = 0.2;
float L4 = 0.138;

float piOverTwo = M_PI/2;

void setup()
{
  pinMode(led_pin, OUTPUT);
  Serial.begin(57600);
  while (!Serial) {} // wait for serial port to connect. Needed for native USB

while (1) {
  if (state == WAITING) {
    if(Serial.available()>0) {
      nPolys = Serial.parseInt();
      Serial.read(); // clear rest of input buffer (i.e. trailing \n)
      Serial.println(nPolys);
      state = RECEIVING_X;
    }
  } else if (state == RECEIVING_X) {
    readData(xpoly);
    if (count >= nPolys) {
      count = 0;
      state = RECEIVING_Y;
    }
  } else if (state == RECEIVING_Y) {
    readData(ypoly);
    if (count >= nPolys) {
      count = 0;
      state = RECEIVING_Z;
    }
  } else if (state == RECEIVING_Z) {
    readData(zpoly);
    if (count >= nPolys) {
      count = 0;
      state = RECEIVING_TH;
    }
  } else if (state == RECEIVING_TH) {
    readData(thpoly);
    if (count >= nPolys) {
      count = 0;
      state = POSITION_CONTROL;
    }
  } else if (state == PLOTTING) {
    // evaluate and send paths back so matlab can plot in 3D the trajectory for validation
    int verify = nPolys; // number of paths to verify
    sendNPoly(verify, xpoly);
    sendNPoly(verify, ypoly);
    sendNPoly(verify, zpoly);
    sendNPoly(verify, thpoly);
    // clear polys arrays?
    state = WAITING;
  } else if (state == POSITION_CONTROL) {
	// delay before starting trajectory
    delay(1000);

  	count = 0;
  	
  	while (count < nPolys) {
  		// delay before each new segment
  		delay(1000);
  		unsigned int t0 = millis();
  		unsigned int dt = 0;
  		// duration of current polynomial, note xpoly/ypoly/zpoly/thpoly should all agree on tf value
  		unsigned int tf = xpoly[count].tf;
  		
  		// complete current path
  		while (dt < tf) {
  		  // get task space coordinates and assign to X
  		  float x = evaluate(&xpoly[count], dt/1000.0);
  		  float y = evaluate(&ypoly[count], dt/1000.0);
  		  float z = evaluate(&zpoly[count], dt/1000.0);
  		  X.x = x;
  		  X.y = y;
  		  X.z = z;
  		  X.theta = 0;
  
  		  // get joint space Q with IK, using X
  		  inverse_kinematics(&Q, &X);
  
  		  // write joint space Q to servos
  		  // writeQ(&Q,&groupSyncWrite430, &groupSyncWrite320,  packetHandler);
  
  		  dt = millis() - t0;
  		}
  		// current path finished
  		count++;
  	}
      
  	// all paths done, reset and listen for new paths
  	// todo empty all xpoly/ypoly/zpoly/thpoly instead of overwriting
      state = WAITING;
      count = 0;
  	nPolys = 0;
  	
  } else if (state == FINISHED) {
    // rest
  }
}
}

void loop() {
  
}

void inverse_kinematics(Q_t *Q, X_t *X) {
  Q->q1 =  atan2(X->y,X->x);
  
  float L_b = sqrt(sq(X->x) +sq(X->y) + sq(X->z+L4-L1));
  //make sure to check the home position ...
  Q->q3 = cosine_rule(L_b, L2, L3) - piOverTwo;

  float rad = atan2(X->z+L4-L1, sqrt(sq(X->x) + sq(X->y)));
  Q->q2 = -(piOverTwo-rad-cosine_rule(L3,L_b,L2));

  Q->q4 = -(Q->q2 + Q->q3);

  Q->q5 = Q->q1 - X->theta;
}

float cosine_rule(float a, float b, float c) {
  // a is opposite the required angle
  return acos((sq(b) + sq(c) - sq(a))/(2*b*c));
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
    Serial.print(a3,5); Serial.print(' ');
    Serial.print(a2,5); Serial.print(' ');
    Serial.print(a1,5); Serial.print(' ');
    Serial.print(a0,5); Serial.print(' ');
    Serial.print(tf,5); Serial.print(' ');
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

void sendNPoly(int n, struct Cubic cubic[MAX_CUBICS]) {
  // send the first n polynomial path segments
  float t0 = 0.0;
  for (int i=0; i<n; i++) {
    // calculate trajectory for ith segment and send
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
  // evaluate the given cubic at time t
  float a0 = float(cubic->coef[0])/FLOAT_TO_INT;
  float a1 = float(cubic->coef[1])/FLOAT_TO_INT;
  float a2 = float(cubic->coef[2])/FLOAT_TO_INT;
  float a3 = float(cubic->coef[3])/FLOAT_TO_INT;
  return poly(t, a0, a1, a2, a3);
}