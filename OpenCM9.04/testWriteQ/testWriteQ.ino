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
#define ANGLE_CONVERSION_CONSTANT_320       0.005061454830784 //degrees per unit



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



typedef struct {
  float q1;
  float q2;
  float q3;
  float q4;
  float q5;
} Q_t;

void setup() {
  Serial.begin(57600);
  while(!Serial);

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

  // Initialize Groupsyncread instance for Present Position
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
  disableTorque430(DXL1_ID, portHandler, packetHandler);
  disableTorque430(DXL2_ID, portHandler, packetHandler);
  disableTorque430(DXL3_ID, portHandler, packetHandler);
  disableTorque320(DXL4_ID, portHandler, packetHandler);
  disableTorque320(DXL5_ID, portHandler, packetHandler);
  disableTorque320(DXL6_ID, portHandler, packetHandler);

  // Set to position mode;
  positionMode430(DXL1_ID, portHandler, packetHandler); 
  positionMode430(DXL2_ID, portHandler, packetHandler); 
  positionMode430(DXL3_ID, portHandler, packetHandler); 
  positionMode320(DXL4_ID, portHandler, packetHandler); 
  positionMode320(DXL5_ID, portHandler, packetHandler); 
  positionMode320(DXL6_ID, portHandler, packetHandler); 

  // Enable Torques
  enableTorque430(DXL1_ID, portHandler, packetHandler);
  enableTorque430(DXL2_ID, portHandler, packetHandler);
  enableTorque430(DXL3_ID, portHandler, packetHandler);
  enableTorque320(DXL4_ID, portHandler, packetHandler);
  enableTorque320(DXL5_ID, portHandler, packetHandler);
  //enableTorque320(DXL6_ID, portHandler, packetHandler);

  // Add parameter storage for Dynamixel#1 present position value
  dxl_addparam_result = groupSyncRead430.addParam(DXL1_ID);
  dxl_addparam_result = groupSyncRead430.addParam(DXL2_ID);
  dxl_addparam_result = groupSyncRead430.addParam(DXL3_ID);
  dxl_addparam_result = groupSyncRead320.addParam(DXL4_ID);
  dxl_addparam_result = groupSyncRead320.addParam(DXL5_ID);
  dxl_addparam_result = groupSyncRead320.addParam(DXL6_ID);

  Q_t Q = {10*PI/180,10*PI/180,10*PI/180,20*PI/180,10*PI/180};

  writeQ(&Q,&groupSyncWrite430, &groupSyncWrite320,  packetHandler);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}

void checkComms(int dxl_comm_result,dynamixel::PacketHandler *packetHandler ){
    if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
}

int enableTorque430(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_TORQUE_ENABLE_430, TORQUE_ENABLE, &dxl_error);
}

int enableTorque320(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_TORQUE_ENABLE_320, TORQUE_ENABLE, &dxl_error);
}

int disableTorque430(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_TORQUE_ENABLE_430, TORQUE_DISABLE, &dxl_error);
}

int disableTorque320(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_TORQUE_ENABLE_320, TORQUE_DISABLE, &dxl_error);
}

int positionMode430(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_OPERATING_MODE_430, POSITION_MODE_430, &dxl_error);
}

int positionMode320(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_OPERATING_MODE_320, POSITION_MODE_320, &dxl_error);
}

void writeQ(Q_t *Q, dynamixel::GroupSyncWrite *groupSyncWrite430, dynamixel::GroupSyncWrite *groupSyncWrite320,  dynamixel::PacketHandler *packetHandler){
  
  
  int q1 = convertToPositionCommand430(Q->q1,false); 
  int q2 = convertToPositionCommand430(Q->q2,true); 
  int q3 = convertToPositionCommand430(Q->q3,true);  
  int q4 = convertToPositionCommand320(Q->q4,false); 
  int q5 = convertToPositionCommand320(Q->q5,false); 

  uint8_t q1_ba[4];
  uint8_t q2_ba[4];
  uint8_t q3_ba[4];
  uint8_t q4_ba[4];
  uint8_t q5_ba[4];

  convertToByteArray(q1_ba,q1); 
  convertToByteArray(q2_ba,q2); 
  convertToByteArray(q3_ba,q3); 
  convertToByteArray(q4_ba,q4); 
  convertToByteArray(q5_ba,q5); 

  dxl_addparam_result = groupSyncWrite430->addParam(DXL1_ID, q1_ba);
  dxl_addparam_result = groupSyncWrite430->addParam(DXL2_ID, q2_ba);
  dxl_addparam_result = groupSyncWrite430->addParam(DXL3_ID, q3_ba);
  dxl_addparam_result = groupSyncWrite320->addParam(DXL4_ID, q4_ba);
  dxl_addparam_result = groupSyncWrite320->addParam(DXL5_ID, q5_ba);
  
  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite430->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);
  dxl_comm_result = groupSyncWrite320->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

  // Clear syncwrite parameter storage
  groupSyncWrite430->clearParam();
  groupSyncWrite320->clearParam();

}

int convertToPositionCommand430(float q, boolean flip){
  if(flip){
    return (-q+PI)/ANGLE_CONVERSION_CONSTANT_430;
    }
  else{
    return (q+PI)/ANGLE_CONVERSION_CONSTANT_430;
  }
}

int convertToPositionCommand320(float q, boolean flip){
  if(flip){
    return (-q+150*PI/180)/ANGLE_CONVERSION_CONSTANT_320;
    }
  else{
    return (q+150*PI/180)/ANGLE_CONVERSION_CONSTANT_320;
  }
}

void convertToByteArray(uint8_t *a, int val){
  a[0] = DXL_LOBYTE(DXL_LOWORD(val));
  a[1] = DXL_HIBYTE(DXL_LOWORD(val));
  a[2] = DXL_LOBYTE(DXL_HIWORD(val));
  a[3] = DXL_HIBYTE(DXL_HIWORD(val));
}
