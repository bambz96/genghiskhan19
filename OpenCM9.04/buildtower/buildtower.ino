#define MAX_CUBICS 25
// states
#define WAITING 0			// listen for communication from Matlab over serial, which send N, the number of path segments coming
#define RECEIVING_X 1		// receive polynomial coefficients for all N cubic path segments x(t)
#define RECEIVING_Y 2		//     ... for y(t)
#define RECEIVING_Z 3		//     ... for z(t)
#define RECEIVING_TH 4		//     ... for theta(t)
#define RECEIVING_GRIP 5    //     ... for theta(t)
#define PLOTTING 6			// send all paths (t, x, y, z) back to Matlab
#define SIMULATION 7		// simulate measurement/control
#define POSITION_CONTROL 8	// position control, no feedback
#define PASSIVE_READ 9  // turn torques off and read Q -> FK -> print x/y/z/theta
#define FINISHED 10			// do nothing

///////////////////////////////////////////////////////////////////////////////////////

#include <DynamixelSDK.h>

// Control table 430
#define ADDRESS_TORQUE_ENABLE_430           64
#define ADDRESS_OPERATING_MODE_430          11
#define ADDRESS_GOAL_POSITION_430           116
#define ADDRESS_GOAL_VELOCITY_430           104
#define ADDRESS_PRESENT_POSITION_430        132

// Control table 320
#define ADDRESS_TORQUE_ENABLE_320           24
#define ADDRESS_OPERATING_MODE_320          11
#define ADDRESS_GOAL_POSITION_320           30
#define ADDRESS_GOAL_VELOCITY_320           32
#define ADDRESS_PRESENT_POSITION_320        37

// Data Byte Length
#define LENGTH_GOAL_POSITION_430            4
#define LENGTH_GOAL_VELOCITY_430            4
#define LENGTH_PRESENT_POSITION_430         4

#define LENGTH_GOAL_POSITION_320            2
#define LENGTH_GOAL_VELOCITY_320            2
#define LENGTH_PRESENT_POSITION_320         2

#define ANGLE_CONVERSION_CONSTANT_430       0.001535889741755 //rads per unit
#define ANGLE_CONVERSION_CONSTANT_320       0.005061454830784 //rads per unit

#define DXL1_OFFSET                         5.5 //motor unit offset
#define DXL2_OFFSET                         -23.5
#define DXL3_OFFSET                         -31.5
#define DXL4_OFFSET                         5.5
#define DXL5_OFFSET                         0
#define DXL6_OFFSET                         0

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define DXL3_ID                         3                   // Dynamixel#3 ID: 3
#define DXL4_ID                         4                   // Dynamixel#4 ID: 4
#define DXL5_ID                         5                   // Dynamixel#5 ID: 5
#define DXL6_ID                         6                   // Dynamixel#6 ID: 6

#define BAUDRATE                        57600
#define DEVICENAME                      "1"                 // Check which port is being used on your controller
// DEVICENAME "1" -> Serial1
// DEVICENAME "2" -> Serial2
// DEVICENAME "3" -> Serial3(OpenCM 485 EXP)

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define VELOCITY_MODE                   1                   // Value for switching to velocity mode
#define POSITION_MODE_320               2
#define POSITION_MODE_430               3
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b
uint8_t dxl_error = 0;                          // Dynamixel error
unsigned long loops = 0;

int dxl_comm_result;             // Communication result
bool dxl_addparam_result;                // addParam result

///////////////////////////////////////////////////////////////////////////////////////

int led_pin = LED_BUILTIN; // 13 for Uno/Mega2560, 14 for OpenCM

int state = PASSIVE_READ;

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
Q_t Q = {0, 0, 0, 0, 0}; //robot joint angles
Q_t Qc = {0, 0, 0, 0, 0}; //controller joint angles

typedef struct {
  float Q[];
  float sum=0;
  int window =5;
  int num_readings=0;
  int index=0;
} SMA_t;

SMA_t Qsma[5]; //array of simple moving average parameters

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
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite430(portHandler, packetHandler, ADDRESS_GOAL_POSITION_430, LENGTH_GOAL_POSITION_430);
  dynamixel::GroupSyncWrite groupSyncWrite320(portHandler, packetHandler, ADDRESS_GOAL_POSITION_320, LENGTH_GOAL_POSITION_320);

  // Initialize GroupSyncRead instance for Present Position
  dynamixel::GroupSyncRead groupSyncRead430(portHandler, packetHandler, ADDRESS_PRESENT_POSITION_430, LENGTH_PRESENT_POSITION_430);
  dynamixel::GroupSyncRead groupSyncRead320(portHandler, packetHandler, ADDRESS_PRESENT_POSITION_320, LENGTH_PRESENT_POSITION_320);


  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  // Open port
  if (portHandler->openPort())
  {
    Serial.print("Succeeded to open the port!\n");
  }
  else
  {
    Serial.print("Failed to open the port!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate!\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    Serial.print("Press any key to terminate...\n");
    return;
  }

  // Disable Torques
  //  disableTorque430(DXL1_ID, portHandler, packetHandler);
  //  disableTorque430(DXL2_ID, portHandler, packetHandler);
  //  disableTorque430(DXL3_ID, portHandler, packetHandler);
  //  disableTorque320(DXL4_ID, portHandler, packetHandler);
  //  disableTorque320(DXL5_ID, portHandler, packetHandler);
  //  disableTorque320(DXL6_ID, portHandler, packetHandler);

  // Set to position mode;
  positionMode430(DXL1_ID, portHandler, packetHandler);
  positionMode430(DXL2_ID, portHandler, packetHandler);
  positionMode430(DXL3_ID, portHandler, packetHandler);
  positionMode320(DXL4_ID, portHandler, packetHandler);
  positionMode320(DXL5_ID, portHandler, packetHandler);
  positionMode320(DXL6_ID, portHandler, packetHandler);

  // Add parameter storage for Dynamixel#1 present position value
  dxl_addparam_result = groupSyncRead430.addParam(DXL1_ID);
  dxl_addparam_result = groupSyncRead430.addParam(DXL2_ID);
  dxl_addparam_result = groupSyncRead430.addParam(DXL3_ID);
  dxl_addparam_result = groupSyncRead320.addParam(DXL4_ID);
  dxl_addparam_result = groupSyncRead320.addParam(DXL5_ID);
  dxl_addparam_result = groupSyncRead320.addParam(DXL6_ID);

  // uncomment these to test writing the pose Q, note Q is initialised above
  // Q_t Q = {10*PI/180,10*PI/180,10*PI/180,20*PI/180,10*PI/180};
  // writeQ(&Q,&groupSyncWrite430, &groupSyncWrite320,  packetHandler);

  enableTorque430(DXL1_ID, portHandler, packetHandler);
  enableTorque430(DXL2_ID, portHandler, packetHandler);
  enableTorque430(DXL3_ID, portHandler, packetHandler);
  enableTorque320(DXL4_ID, portHandler, packetHandler);
  enableTorque320(DXL5_ID, portHandler, packetHandler);
  enableTorque320(DXL6_ID, portHandler, packetHandler);

  while (1) {
    if (state == WAITING) {
      if (Serial.available() > 0) {
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
        state = RECEIVING_GRIP;
      }
    } else if (state == RECEIVING_GRIP) {
      readData(grippoly);
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
        //delay(500);
        unsigned int t0 = millis();
        unsigned int dt = 0;
        // duration of current polynomial, note xpoly/ypoly/zpoly/thpoly should all agree on tf value
        unsigned int tf = xpoly[count].tf;


        // complete current path
        while (dt < tf) {
          //find current joint angles
          readQ(&Q, &groupSyncRead430, &groupSyncRead320,  packetHandler, Qsma);
          //find actual task space
          forward_kinematics(&X, Q);

          // get task space coordinates and assign to X
          float x = evaluate(&xpoly[count], dt / 1000.0);
          float y = evaluate(&ypoly[count], dt / 1000.0);
          float z = evaluate(&zpoly[count], dt / 1000.0);
          float theta = evaluate(&thpoly[count], dt / 1000.0);
          float grip = evaluate(&grippoly[count], dt / 1000.0);
          Xref.x = x;
          Xref.y = y;
          Xref.z = z;
          Xref.theta = theta;
          Xref.grip = grip;



          //Calculate feedback from measured task space, reference task space and previous measured task space
          X_t Xerr, Xtotalerr, Xdeltaerr, Xpreverr; //error for pid control

          Xerr = xSubtraction(Xref, X);
          Xtotalerr = xAddition(Xtotalerr, Xerr);
          Xdeltaerr = xSubtraction(Xerr, Xpreverr);


          X_t Xc = pid(Xerr, Xtotalerr, Xdeltaerr, dt / 1000.0);
          // get joint space control, Qc with IK, using feedback
          inverse_kinematics(&Qc, &Xref);

          // write joint space Qc to servos
          writeQ(&Qc, &groupSyncWrite430, &groupSyncWrite320,  packetHandler);

          Xpreverr = Xerr;

          dt = millis() - t0;

          delay(33);
        }
        // current path finished
        count++;
      }

      // all paths done, reset and listen for new paths
      // todo empty all xpoly/ypoly/zpoly/thpoly instead of overwriting
      state = WAITING;
      count = 0;
      nPolys = 0;
      // Disable Torques
      //      disableTorque430(DXL1_ID, portHandler, packetHandler);
      //      disableTorque430(DXL2_ID, portHandler, packetHandler);
      //      disableTorque430(DXL3_ID, portHandler, packetHandler);
      //      disableTorque320(DXL4_ID, portHandler, packetHandler);
      //      disableTorque320(DXL5_ID, portHandler, packetHandler);
      //      disableTorque320(DXL6_ID, portHandler, packetHandler);

    } else if (state == PASSIVE_READ) {
      // Disable Torques
      disableTorque430(DXL1_ID, portHandler, packetHandler);
      disableTorque430(DXL2_ID, portHandler, packetHandler);
      disableTorque430(DXL3_ID, portHandler, packetHandler);
      disableTorque320(DXL4_ID, portHandler, packetHandler);
      disableTorque320(DXL5_ID, portHandler, packetHandler);
      disableTorque320(DXL6_ID, portHandler, packetHandler);

//      readQ(&Q, &groupSyncRead430, &groupSyncRead320,  packetHandler, Qsma);
//      forward_kinematics(&X, Q);
//      Serial.print(Q.q1 * 180 / PI); Serial.print(' ');
//      Serial.print(Q.q2 * 180 / PI); Serial.print(' ');
//      Serial.print(Q.q3 * 180 / PI); Serial.print(' ');
//      Serial.print(Q.q4 * 180 / PI); Serial.print(' ');
//      Serial.print(Q.q5 * 180 / PI); Serial.print(' ');
//      Serial.print(1000 * X.x); Serial.print(' ');
//      Serial.print(1000 * X.y); Serial.print(' ');
//      Serial.print(1000 * X.z); Serial.print(' ');
//      Serial.print(X.theta, 4); Serial.print(' ');
//      Serial.println();
        Serial.print("Window size: "); Serial.println(Qsma[0].window);
        readQ1(&Q, &groupSyncRead430, packetHandler, &Qsma[0]);
        Serial.print(Q.q1 * 180 / PI);
        Serial.print("Readings: "); Serial.println(Qsma[0].num_readings); 
        delay(500);
    } else if (state == FINISHED) {
      // rest
    }
  }
}

void loop() {

}

X_t pid(X_t Xerr, X_t Xdeltaerr, X_t Xtotalerr, float dt) {
  /*Xref = reference, X = measured*/
  float Kp = 0.01, Ki = 0.01, Kd = 0.01;
  X_t Xc;
  Xc.x = Kp * Xerr.x + Ki * dt * Xtotalerr.x + (Kd / dt) * Xdeltaerr.x;
  Xc.y = Kp * Xerr.y + Ki * dt * Xtotalerr.y + (Kd / dt) * Xdeltaerr.y;
  Xc.z = Kp * Xerr.z + Ki * dt * Xtotalerr.z + (Kd / dt) * Xdeltaerr.z;
  Xc.theta = Kp * Xerr.theta + Ki * dt * Xtotalerr.theta + (Kd / dt) * Xdeltaerr.theta;
  Xc.grip = Xerr.grip;
  return Xc;
}

X_t xAddition(X_t x1, X_t x2) {
  X_t x;
  x.x = x1.x + x2.x;
  x.y = x1.y + x2.y;
  x.z = x1.z + x2.z;
  x.theta = x1.theta + x2.theta;
  x.grip = x1.grip;
  //x.grip = x1.grip + x2.grip;
  return x;
}

X_t xSubtraction(X_t x1, X_t x2) {
  X_t x;
  x.x = x1.x - x2.x;
  x.y = x1.y - x2.y;
  x.z = x1.z - x2.z;
  x.theta = x1.theta - x2.theta;
  x.grip = x1.grip;
  //x.grip = x1.grip + x2.grip;
  return x;
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
  return a3 * t * t * t + a2 * t * t + a1 * t + a0;
}

float polyVelocity(float t, float a1, float a2, float a3) {
  // evaluate the given cubic polynomial at time t
  return 3 * a3 * t * t  + 2 * a2 * t + a1 ;
}

float evaluateVelocity(struct Cubic *cubic, float t) {
  // evaluate the given cubic at time t
  float a0 = cubic->coef[0];
  float a1 = cubic->coef[1];
  float a2 = cubic->coef[2];
  float a3 = cubic->coef[3];
  return polyVelocity(t, a1, a2, a3);
}

void sendNPoly(int n, struct Cubic cubic[MAX_CUBICS]) {
  // send the first n polynomial path segments
  float t0 = 0.0;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < 100; j++) {
      float t = j / 100.0 * cubic[i].tf / 1000.0; // convert to real duration
      sendPolyAtTime(t, t0, &cubic[i]);
    }
    t0 += cubic[i].tf / 1000.0;
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
  Serial.print(t + t0, 5); Serial.print(' ');
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

//////////////////////////////////////////////////////////////////////////////////////////////

void checkComms(int dxl_comm_result, dynamixel::PacketHandler *packetHandler ) {
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
}

int enableTorque430(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_TORQUE_ENABLE_430, TORQUE_ENABLE, &dxl_error);
}

int enableTorque320(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_TORQUE_ENABLE_320, TORQUE_ENABLE, &dxl_error);
}

int disableTorque430(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_TORQUE_ENABLE_430, TORQUE_DISABLE, &dxl_error);
}

int disableTorque320(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_TORQUE_ENABLE_320, TORQUE_DISABLE, &dxl_error);
}

int positionMode430(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_OPERATING_MODE_430, POSITION_MODE_430, &dxl_error);
}

int positionMode320(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_OPERATING_MODE_320, POSITION_MODE_320, &dxl_error);
}

void readQ1(Q_t *Q, dynamixel::GroupSyncRead *groupSyncRead430, dynamixel::PacketHandler *packetHandler, SMA_t *Qsma) {
  dxl_comm_result = groupSyncRead430->txRxPacket();
  Q->q1 = -PI + ANGLE_CONVERSION_CONSTANT_430 * sma(groupSyncRead430->getData(DXL1_ID, ADDRESS_PRESENT_POSITION_430, LENGTH_PRESENT_POSITION_430) + DXL1_OFFSET, Qsma);
 }

void readQ(Q_t *Q, dynamixel::GroupSyncRead *groupSyncRead430, dynamixel::GroupSyncRead *groupSyncRead320, dynamixel::PacketHandler *packetHandler, SMA_t Qsma[5]) {
  dxl_comm_result = groupSyncRead430->txRxPacket();
  dxl_comm_result = groupSyncRead320->txRxPacket();
  Q->q1 = -PI + ANGLE_CONVERSION_CONSTANT_430 * sma(groupSyncRead430->getData(DXL1_ID, ADDRESS_PRESENT_POSITION_430, LENGTH_PRESENT_POSITION_430) + DXL1_OFFSET, &Qsma[0]);
  Q->q2 = -(PI + ANGLE_CONVERSION_CONSTANT_430 * sma(groupSyncRead430->getData(DXL2_ID, ADDRESS_PRESENT_POSITION_430, LENGTH_PRESENT_POSITION_430) + DXL2_OFFSET, &Qsma[1]));
  Q->q3 = -(PI + ANGLE_CONVERSION_CONSTANT_430 * sma(groupSyncRead430->getData(DXL3_ID, ADDRESS_PRESENT_POSITION_430, LENGTH_PRESENT_POSITION_430) + DXL3_OFFSET, &Qsma[2]));
  Q->q4 = -(5 * PI / 6) + ANGLE_CONVERSION_CONSTANT_320 * sma(groupSyncRead320->getData(DXL4_ID, ADDRESS_PRESENT_POSITION_320, LENGTH_PRESENT_POSITION_320) + DXL4_OFFSET, &Qsma[3]);
  Q->q5 = -(5 * PI / 6) + ANGLE_CONVERSION_CONSTANT_320 * sma(groupSyncRead320->getData(DXL5_ID, ADDRESS_PRESENT_POSITION_320, LENGTH_PRESENT_POSITION_320) + DXL5_OFFSET, &Qsma[4]);
  Q->q6 = -(5 * PI / 6) + ANGLE_CONVERSION_CONSTANT_320 * (groupSyncRead320->getData(DXL6_ID, ADDRESS_PRESENT_POSITION_320, LENGTH_PRESENT_POSITION_320) + DXL6_OFFSET);
}

//q = value to be filtered, Q = previous values, window = window_size, num_readings = number of readings so far, index = next index
float sma(float q, SMA_t *Qsma){
  if (Qsma->num_readings < Qsma->window ){
    ++Qsma->num_readings; //increase number of readings
    Qsma->sum += q; //increase avg
  } else {
    Qsma->sum += q - Qsma->Q[Qsma->index]; //sum is adding current reading and subtracting previous reading
  }
  Qsma->Q[Qsma->index] = q;
  if (++Qsma->index >= Qsma->window) Qsma->index =0;
  return (Qsma->sum + Qsma->num_readings /2) / Qsma->num_readings;
}

void writeQ(Q_t *Q, dynamixel::GroupSyncWrite *groupSyncWrite430, dynamixel::GroupSyncWrite *groupSyncWrite320,  dynamixel::PacketHandler *packetHandler) {
  
  
  int q1 = convertToPositionCommand430(Q->q1, false) + DXL1_OFFSET;
  int q2 = convertToPositionCommand430(Q->q2, true) + DXL2_OFFSET;
  int q3 = convertToPositionCommand430(Q->q3, true) + DXL3_OFFSET;
  int q4 = convertToPositionCommand320(Q->q4, false) + DXL4_OFFSET;
  int q5 = convertToPositionCommand320(Q->q5, false) + DXL5_OFFSET;
  int q6 = convertToPositionCommand320(Q->q6, false) + DXL6_OFFSET;

  uint8_t q1_ba[4];
  uint8_t q2_ba[4];
  uint8_t q3_ba[4];
  uint8_t q4_ba[4];
  uint8_t q5_ba[4];
  uint8_t q6_ba[4];

  convertToByteArray(q1_ba, q1);
  convertToByteArray(q2_ba, q2);
  convertToByteArray(q3_ba, q3);
  convertToByteArray(q4_ba, q4);
  convertToByteArray(q5_ba, q5);
  convertToByteArray(q6_ba, q6);

  dxl_addparam_result = groupSyncWrite430->addParam(DXL1_ID, q1_ba);
  dxl_addparam_result = groupSyncWrite430->addParam(DXL2_ID, q2_ba);
  dxl_addparam_result = groupSyncWrite430->addParam(DXL3_ID, q3_ba);
  dxl_addparam_result = groupSyncWrite320->addParam(DXL4_ID, q4_ba);
  dxl_addparam_result = groupSyncWrite320->addParam(DXL5_ID, q5_ba);
  dxl_addparam_result = groupSyncWrite320->addParam(DXL6_ID, q6_ba);


  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite430->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);
  dxl_comm_result = groupSyncWrite320->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

  // Clear syncwrite parameter storage
  groupSyncWrite430->clearParam();
  groupSyncWrite320->clearParam();

}

int convertToPositionCommand430(float q, boolean flip) {
  if (flip) {
    return (-q + PI) / ANGLE_CONVERSION_CONSTANT_430;
  }
  else {
    return (q + PI) / ANGLE_CONVERSION_CONSTANT_430;
  }
}

int convertToPositionCommand320(float q, boolean flip) {
  if (flip) {
    return (-q + 150 * PI / 180) / ANGLE_CONVERSION_CONSTANT_320;
  }
  else {
    return (q + 150 * PI / 180) / ANGLE_CONVERSION_CONSTANT_320;
  }
}

void convertToByteArray(uint8_t *a, int val) {
  a[0] = DXL_LOBYTE(DXL_LOWORD(val));
  a[1] = DXL_HIBYTE(DXL_LOWORD(val));
  a[2] = DXL_LOBYTE(DXL_HIWORD(val));
  a[3] = DXL_HIBYTE(DXL_HIWORD(val));
}
