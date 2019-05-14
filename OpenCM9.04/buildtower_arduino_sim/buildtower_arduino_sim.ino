#define MAX_CUBICS 25
#define PLOTTED_PATH_RES 50 // nbr of samples in paths generated for plotting in Matlab, does not affect real operation
// states
#define WAITING 0			// listen for communication from Matlab over serial, which send N, the number of path segments coming
#define RECEIVING_X 1		// receive polynomial coefficients for all N cubic path segments x(t)
#define RECEIVING_Y 2		//     ... for y(t)
#define RECEIVING_Z 3		//     ... for z(t)
#define RECEIVING_TH 4		//     ... for theta(t)
#define RECEIVING_GRIP 5    //     ... for theta(t)
#define PLOTTING 6			// send all paths (t, x, y, z) back to Matlab
#define SIMULATION 7		// simulate measurement/control
#define POSITION_CONTROL 8	// position control
#define VELOCITY_CONTROL 9  // velocity control
#define PASSIVE_READ 10  // turn torques off and read Q -> FK -> print x/y/z/theta
#define FINISHED 11			// do nothing

int led_pin = LED_BUILTIN; // 13 for Uno/Mega2560, 14 for OpenCM

int state = WAITING;

int nPolys;		// number of polynomials sent by Matlab and stored for operation
int count = 0;	// used to count up to nPolys whilst receiving coefficients from Matlab, and to hold current cubic path segment while operating

// stores cubic polynomial coefficients and duration tf
// int instead of float to halve needed bytes (4 -> 2)
struct Cubic {
  float coef[4];
  unsigned int tf; // milliseconds
};

// only MAX_CUBICS cubic path segments can be stored at a time due to SRAM constraints
struct Cubic xpoly[MAX_CUBICS];
struct Cubic ypoly[MAX_CUBICS];
struct Cubic zpoly[MAX_CUBICS];
struct Cubic thpoly[MAX_CUBICS];
struct Cubic grippoly[MAX_CUBICS];

// task space coordinates
typedef struct {
  float x;
  float y;
  float z;
  float theta;
  float grip;
} X_t;

// joint space coordinates
typedef struct {
  float q1;
  float q2;
  float q3;
  float q4;
  float q5;
  float q6;
} Q_t;

X_t Xref; //reference position
X_t X; //robot position
X_t Xprev ={0.2, 0, 0.3, 0, 0}; //previous robot position, initial is home
Q_t Q = {0, 0, 0, 0, 0}; //robot joint angles
Q_t Qc = {0, 0, 0, 0, 0}; //controller joint angles

float L1 = 0.2;
float L2 = 0.2;
float L3 = 0.2;
float L4 = 0.1;

float piOverTwo = M_PI / 2;

void setup()
{
  pinMode(led_pin, OUTPUT);
  Serial.begin(57600);
  while (!Serial) {} // wait for serial port to connect. Needed for native USB
  

while (1) {
  if (state == WAITING) {
    if(Serial.available()>0) {
      String command = Serial.readStringUntil('\n');
      if (command == "N") {
        // receiving N polynomials
        Serial.println(command);
        while (Serial.available()==0) {} // wait for reply
        nPolys = Serial.parseInt();
        Serial.read(); // clear rest of input buffer (i.e. trailing \n)
        Serial.println(nPolys); // confirm N polys before Matlab will send
        state = RECEIVING_X;
      } else if (command == "P") {
        // instruction to plot current stored trajectory
        delay(100); // delay 100ms to make sure Matlab is ready to receive
        Serial.println(nPolys); // tell Matlab number of paths
        // if N=0, do nothing (stay WAITING), otherwise...
        if (nPolys != 0) {
          delay(100);
          Serial.println(PLOTTED_PATH_RES); // tell Matlab path resolution used
          delay(100);
          state = PLOTTING; // begin sending path data
        }
      } else if (command == "PC") {
        // position control
        Serial.println("PC");
        state = POSITION_CONTROL;
      } else if (command == "VC") {
        // velocity control
        Serial.println("VC");
        state = VELOCITY_CONTROL;
      } else if (command == "R") {
        // passive read, no torques enabled
        state = PASSIVE_READ;
      }
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
      state = RECEIVING_GRIP;
    }
  } else if (state == RECEIVING_GRIP) {
    readData(grippoly);
    if (count >= nPolys) {
      count = 0;
      state = WAITING;
    }
  } else if (state == PLOTTING) {
    // evaluate and send paths back so matlab can plot in 3D the trajectory for validation
    int verify = nPolys; // number of paths to verify
    sendNPoly(verify, xpoly);
    sendNPoly(verify, ypoly);
    sendNPoly(verify, zpoly);
    sendNPoly(verify, thpoly);
    sendNPoly(verify, grippoly); 
    // reset count of most recent read in rows
    count = 0;
    // clear polys arrays?
    state = WAITING;
  } else if (state == POSITION_CONTROL) {
	// delay before starting trajectory
    delay(2000);

	count = 0;
	
	while (count < nPolys) {
		// delay before each new segment
		delay(500);
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
      float theta = evaluate(&thpoly[count],dt/1000.0); 
      float grip = evaluate(&grippoly[count],dt/1000.0); 
		  X.x = x;
		  X.y = y;
		  X.z = z;
		  X.theta = theta;
      X.grip = grip; 

		  // get joint space Q with IK, using X
		  inverse_kinematics(&Q, &X);

		  // write joint space Q to servos

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
  } else if (state == VELOCITY_CONTROL) {
    
	} else if (state == PASSIVE_READ) {
    Serial.println("random test data");
    if(Serial.available()>0) {
      Serial.read(); // doesn't matter what's sent, just end PASSIVE_READ
      state = WAITING;
    }
  } else if (state == FINISHED) {
    // rest
  }
}
}

X_t feedback(X_t Xprev, X_t Xref, X_t X){
  X_t Xe;
  float K = 1;
  Xe.x = Xref.x + K*(Xprev.x - X.x);
  Xe.y = Xref.y + K*(Xprev.y - X.y);
  Xe.z = Xref.z + K*(Xprev.z - X.z);
  Xe.theta = Xref.theta + K*(Xprev.theta - X.theta);
  Xe.grip = Xref.grip;
  return Xe;
}

void forward_kinematics(X_t *X, Q_t Q) {
  X->x = cos(Q.q1) * (L3 * cos(Q.q2 + Q.q3) - L2 * sin(Q.q2));
  X->y = sin(Q.q1) * (L3 * cos(Q.q2 + Q.q3) - L2 * sin(Q.q2));
  X->z = L3 * sin(Q.q2 + Q.q3) + L2 * cos(Q.q2) - L4 + L1;
  X->theta = Q.q1 - Q.q5;
}

void inverse_kinematics(Q_t *Q, X_t *X) {
  Q->q1 =  atan2(X->y, X->x);

  float L_b = sqrt(sq(X->x) + sq(X->y) + sq(X->z + L4 - L1));
  //make sure to check the home position ...
  Q->q3 = cosine_rule(L_b, L2, L3) - piOverTwo;

  float rad = atan2(X->z + L4 - L1, sqrt(sq(X->x) + sq(X->y)));
  Q->q2 = -(piOverTwo - rad - cosine_rule(L3, L_b, L2));

  Q->q4 = -(Q->q2 + Q->q3);

  Q->q5 = Q->q1 - X->theta;

  Q->q6 = X->grip; 
}

float cosine_rule(float a, float b, float c) {
  // a is opposite the required angle
  return acos((sq(b) + sq(c) - sq(a)) / (2 * b * c));
}

void readData(struct Cubic *poly) {
  // "a3 a2 a1 a0 tf" has been sent, parse it in, create a new Cubic struct
  // and add it to the array provided
  if (Serial.available() > 0)
  {
    float a3 = Serial.parseFloat();
    float a2 = Serial.parseFloat();
    float a1 = Serial.parseFloat();
    float a0 = Serial.parseFloat();
    float tf = Serial.parseFloat();
    Serial.read(); // clear rest of input buffer (i.e. trailing \n
    // reply with read values
    Serial.print(a3, 5); Serial.print(' ');
    Serial.print(a2, 5); Serial.print(' ');
    Serial.print(a1, 5); Serial.print(' ');
    Serial.print(a0, 5); Serial.print(' ');
    Serial.print(tf, 5); Serial.print(' ');
    Serial.println();
    // create Cubic struct and save to given array of polynomials
    struct Cubic cubic;
    cubic.coef[0] = a0;
    cubic.coef[1] = a1;
    cubic.coef[2] = a2;
    cubic.coef[3] = a3;
    cubic.tf = tf * 1000; // convert s to ms
    poly[count] = cubic;
    count++;
  }
}

float poly(float t, float a0, float a1, float a2, float a3) {
  // evaluate the given cubic polynomial at time t
  return a3*t*t*t + a2*t*t + a1*t + a0;
}

void sendNPoly(int n, struct Cubic cubic[MAX_CUBICS]) {
  // send the first n polynomial path segments
  float t0 = 0.0;
  for (int i=0; i<n; i++) {
    for (int j=0; j<PLOTTED_PATH_RES; j++) {
      float t = j/float(PLOTTED_PATH_RES) * cubic[i].tf/1000.0; // convert to real duration
      sendPolyAtTime(t, t0, &cubic[i]);
    }
    t0 += cubic[i].tf/1000.0;
  }
}

void sendPolyAtTime(float t, float t0, struct Cubic *cubic) {
  // send ti and x(ti)
  float x = evaluate(cubic, t);
//  float a0 = cubic->coef[0];
//  float a1 = cubic->coef[1];
//  float a2 = cubic->coef[2];
//  float a3 = cubic->coef[3];
//  float x = poly(t, a0, a1, a2, a3);
  Serial.print(t+t0, 5); Serial.print(' ');
  Serial.print(x, 5); Serial.print(' ');
  Serial.println();
}

float evaluate(struct Cubic *cubic, float t) {
	// evaluate the given cubic at time t
	float a0 = cubic->coef[0];
	float a1 = cubic->coef[1];
	float a2 = cubic->coef[2];
	float a3 = cubic->coef[3];
	return poly(t, a0, a1, a2, a3);
}
