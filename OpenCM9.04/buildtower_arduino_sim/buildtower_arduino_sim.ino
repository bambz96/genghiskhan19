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

///////////////////////////////////////////////////////////////////////////////////////

//#include <DynamixelSDK.h>
//
//// Control table 430
//#define ADDRESS_TORQUE_ENABLE_430           64
//#define ADDRESS_OPERATING_MODE_430          11
//#define ADDRESS_GOAL_POSITION_430           116
//#define ADDRESS_GOAL_VELOCITY_430           104
//#define ADDRESS_PRESENT_POSITION_430        132
//
//// Control table 320
//#define ADDRESS_TORQUE_ENABLE_320           24
//#define ADDRESS_OPERATING_MODE_320          11
//#define ADDRESS_GOAL_POSITION_320           30
//#define ADDRESS_GOAL_VELOCITY_320           32
//#define ADDRESS_PRESENT_POSITION_320        37
//
//// Data Byte Length
//#define LENGTH_GOAL_POSITION_430            4
//#define LENGTH_GOAL_VELOCITY_430            4
//#define LENGTH_PRESENT_POSITION_430         4
//
//#define LENGTH_GOAL_POSITION_320            2
//#define LENGTH_GOAL_VELOCITY_320            2
//#define LENGTH_PRESENT_POSITION_320         2
//
//#define ANGLE_CONVERSION_CONSTANT_430       0.001535889741755 //rads per unit
//#define ANGLE_CONVERSION_CONSTANT_320       0.005061454830784 //rads per unit
//
//#define Q1_SCALE                            1.020078546
//#define Q2_SCALE                            1.0
//#define Q3_SCALE                            1.0
//#define Q4_SCALE                            1.0
//#define Q5_SCALE                            1.0
//#define Q6_SCALE                            1.0
//
//#define Q1_OFFSET                           2.65
//#define Q2_OFFSET                           -3.8236666//-3.870333333
//#define Q3_OFFSET                           -0.77973//-0.9364
//#define Q4_OFFSET                           -4.583636364
//#define Q5_OFFSET                           -1.39625
//
//
//#define Q6_OFFSET                           0
//          
////#define DXL1_OFFSET                         5.5 //motor unit offset
////#define DXL2_OFFSET                         -23.5
////#define DXL3_OFFSET                         -31.5
////#define DXL4_OFFSET                         5.5
////#define DXL5_OFFSET                         0
////#define DXL6_OFFSET                         0
//
//// Protocol version
//#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel
//
//// Default setting
//#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
//#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
//#define DXL3_ID                         3                   // Dynamixel#3 ID: 3
//#define DXL4_ID                         4                   // Dynamixel#4 ID: 4
//#define DXL5_ID                         5                   // Dynamixel#5 ID: 5
//#define DXL6_ID                         6                   // Dynamixel#6 ID: 6
//
//#define BAUDRATE                        57600
//#define DEVICENAME                      "1"                 // Check which port is being used on your controller
//// DEVICENAME "1" -> Serial1
//// DEVICENAME "2" -> Serial2
//// DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
//
//#define TORQUE_ENABLE                   1                   // Value for enabling the torque
//#define TORQUE_DISABLE                  0                   // Value for disabling the torque
//#define VELOCITY_MODE                   1                   // Value for switching to velocity mode
//#define POSITION_MODE_320               2
//#define POSITION_MODE_430               3
//#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
//#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
//#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
//
//#define ESC_ASCII_VALUE                 0x1b
//uint8_t dxl_error = 0;                          // Dynamixel error
//unsigned long loops = 0;
//
//int dxl_comm_result;             // Communication result
//bool dxl_addparam_result;                // addParam result

///////////////////////////////////////////////////////////////////////////////////////

int led_pin = LED_BUILTIN; // 13 for Uno/Mega2560, 14 for OpenCM

int state = WAITING;

boolean debugging = true; // will print additional info during operation, set off by Matlab if desired

int nPolys;		// number of polynomials sent by Matlab and stored for operation
int count = 0;	// used to count up to nPolys whilst receiving coefficients from Matlab, and to hold current cubic path segment while operating

// stores cubic polynomial coefficients and duration tf
// int instead of float to halve needed bytes (4 -> 2)
struct Cubic {
  float coef[4];
  unsigned int t0; // milliseconds
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

float L1 = 0.206;
float L2 = 0.2;
float L3 = 0.2;
float L4 = 0.1;

float piOverTwo = M_PI/2;

void setup()
{
  pinMode(led_pin, OUTPUT);
  Serial.begin(57600);
  while (!Serial) {} // wait for serial port to connect. Needed for native USB
//  // Initialize PortHandler instance
//  // Set the port path
//  // Get methods and members of PortHandlerLinux or PortHandlerWindows
//  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
//
//  // Initialize PacketHandler instance
//  // Set the protocol version
//  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
//  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
//
//  // Initialize GroupSyncWrite instance
//  dynamixel::GroupSyncWrite groupSyncWrite430(portHandler, packetHandler, ADDRESS_GOAL_POSITION_430, LENGTH_GOAL_POSITION_430);
//  dynamixel::GroupSyncWrite groupSyncWrite320(portHandler, packetHandler, ADDRESS_GOAL_POSITION_320, LENGTH_GOAL_POSITION_320);
//
//  // Initialize GroupSyncRead instance for Present Position
//  dynamixel::GroupSyncRead groupSyncRead430(portHandler, packetHandler, ADDRESS_PRESENT_POSITION_430, LENGTH_PRESENT_POSITION_430);
//  dynamixel::GroupSyncRead groupSyncRead320(portHandler, packetHandler, ADDRESS_PRESENT_POSITION_320, LENGTH_PRESENT_POSITION_320);
//
//
//  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//  bool dxl_addparam_result = false;                // addParam result
//  bool dxl_getdata_result = false;                 // GetParam result
//
//  // Open port
//  if (portHandler->openPort())
//  {
//    Serial.print("Succeeded to open the port!\n");
//  }
//  else
//  {
//    Serial.print("Failed to open the port!\n");
//    Serial.print("Press any key to terminate...\n");
//    return;
//  }
//
//  // Set port baudrate
//  if (portHandler->setBaudRate(BAUDRATE))
//  {
//    Serial.print("Succeeded to change the baudrate!\n");
//  }
//  else
//  {
//    Serial.print("Failed to change the baudrate!\n");
//    Serial.print("Press any key to terminate...\n");
//    return;
//  }
//
//  // Disable Torques
//  disableTorque430(DXL1_ID, portHandler, packetHandler);
//  disableTorque430(DXL2_ID, portHandler, packetHandler);
//  disableTorque430(DXL3_ID, portHandler, packetHandler);
//  disableTorque320(DXL4_ID, portHandler, packetHandler);
//  disableTorque320(DXL5_ID, portHandler, packetHandler);
//  disableTorque320(DXL6_ID, portHandler, packetHandler);
//
//  // Set to position mode;
//  positionMode430(DXL1_ID, portHandler, packetHandler);
//  positionMode430(DXL2_ID, portHandler, packetHandler);
//  positionMode430(DXL3_ID, portHandler, packetHandler);
//  positionMode320(DXL4_ID, portHandler, packetHandler);
//  positionMode320(DXL5_ID, portHandler, packetHandler);
//  positionMode320(DXL6_ID, portHandler, packetHandler);
//
//  // Set velocity limits
//  velocityLimit430(DXL1_ID, portHandler, packetHandler);
//  velocityLimit430(DXL2_ID, portHandler, packetHandler);
//  velocityLimit430(DXL3_ID, portHandler, packetHandler);
//
//  // Add parameter storage for Dynamixel#1 present position value
//  dxl_addparam_result = groupSyncRead430.addParam(DXL1_ID);
//  dxl_addparam_result = groupSyncRead430.addParam(DXL2_ID);
//  dxl_addparam_result = groupSyncRead430.addParam(DXL3_ID);
//  dxl_addparam_result = groupSyncRead320.addParam(DXL4_ID);
//  dxl_addparam_result = groupSyncRead320.addParam(DXL5_ID);
//  dxl_addparam_result = groupSyncRead320.addParam(DXL6_ID);

  

  // uncomment these to test writing the pose Q, note Q is initialised above
  // Q_t Q = {10*PI/180,10*PI/180,10*PI/180,20*PI/180,10*PI/180};
  // writeQ(&Q,&groupSyncWrite430, &groupSyncWrite320,  packetHandler);

while (1) {
  if (state == WAITING) {
    if(Serial.available()>0) {
      String command = Serial.readStringUntil('\n');
      if (command == "N") {
        // receiving N polynomials
        // first clear
        // todo empty all xpoly/ypoly/zpoly/thpoly instead of overwriting
        count = 0;
        nPolys = 0;
      
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
      } else if (command == "D") {
        Serial.println("D");
        debugging = !debugging;
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
      state = WAITING;
  } else if (state == POSITION_CONTROL) {
      // Enable Torques
//      enableTorque430(DXL1_ID, portHandler, packetHandler);
//      enableTorque430(DXL2_ID, portHandler, packetHandler);
//      enableTorque430(DXL3_ID, portHandler, packetHandler);
//      enableTorque320(DXL4_ID, portHandler, packetHandler);
//      enableTorque320(DXL5_ID, portHandler, packetHandler);
//      enableTorque320(DXL6_ID, portHandler, packetHandler);
  
      // delay before starting trajectory
      delay(500);

      unsigned int tstart = millis();

      count = 0;
//      readQ(&Q, &groupSyncRead430, &groupSyncRead320,  packetHandler);
      forward_kinematics(&Xprev, Q);
      while (count < nPolys) {
        // delay before each new segment
        //delay(500);
        unsigned int dt = millis() - tstart; // should be very close to 0 on the first poly (count=0)
        // duration of current polynomial, note xpoly/ypoly/zpoly/thpoly should all agree on tf value
        unsigned int t0 = xpoly[count].t0;
        unsigned int tf = xpoly[count].tf;


        // complete current path
        while (dt < tf) {
          
          //find current joint angles
//          readQ(&Q, &groupSyncRead430, &groupSyncRead320,  packetHandler);
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
          X_t ex = feedback(Xprev, Xref, X);

          // get joint space control, Qc with IK, using feedback
          inverse_kinematics(&Qc, &ex);

          // write joint space Qc to servos
//          writeQ(&Qc, &groupSyncWrite430, &groupSyncWrite320,  packetHandler);

          Xprev = Xref;

          if (debugging) {
            float xa = x + random(10)/100;
            float ya = y + random(10)/100;
            float za = z + random(10)/100;
            Serial.print(millis()); Serial.print(' ');
            Serial.print(x, 5); Serial.print(' ');
            Serial.print(y, 5); Serial.print(' ');
            Serial.print(z, 5); Serial.print(' ');
            Serial.print(xa, 5); Serial.print(' ');
            Serial.print(ya, 5); Serial.print(' ');
            Serial.print(za, 5); Serial.print(' ');
            Serial.println();
          }

          dt = millis() - tstart;
        }
        // current path finished
        count++;
      }

      // all paths done, return to WAITING
      Serial.println("DONE");
      state = WAITING;
    } else if (state == VELOCITY_CONTROL) {
    
      // all paths done, return to WAITING
      Serial.println("DONE");
      state = WAITING;
    } else if (state == PASSIVE_READ) {
//      readQ(&Q, &groupSyncRead430, &groupSyncRead320,  packetHandler);
      forward_kinematics(&X, Q);
      Serial.print(Q.q1*180/PI); Serial.print(' ');
      Serial.print(Q.q2*180/PI); Serial.print(' ');
      Serial.print(Q.q3*180/PI); Serial.print(' ');
      Serial.print(Q.q4*180/PI); Serial.print(' ');
      Serial.print(Q.q5*180/PI); Serial.print(' ');
      Serial.print(1000*X.x); Serial.print(' ');
      Serial.print(1000*X.y); Serial.print(' ');
      Serial.print(1000*X.z); Serial.print(' ');
      Serial.print(X.theta,4); Serial.print(' ');
      Serial.println();
      if(Serial.available()>0) {
        Serial.read(); // doesn't matter what's sent, just end PASSIVE_READ
        state = WAITING;
        }
    } else if (state == FINISHED) {
      // rest
    }
  }
}

void loop() {
  // unused because scope
}
