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

int velocityLimit430(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
  return packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDRESS_PROFILE_VELOCITY_430, VELOCITY_LIMIT_430, &dxl_error);
}

int accelerationLimit430(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
  return packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDRESS_PROFILE_ACCELERATION_430, ACCELERATION_LIMIT_430, &dxl_error);
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

int velocityMode430(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_OPERATING_MODE_430, VELOCITY_MODE, &dxl_error);
}

int PWMMode430(int DXL_ID, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {
  return packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDRESS_OPERATING_MODE_430, PWM_MODE, &dxl_error);
}

int setIntegralVelocity430(int DXL_ID, int KVI, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
 return packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDRESS_INTEGRAL_VELOCITY_430, KVI, &dxl_error);
}

int setProportionalVelocity430(int DXL_ID, int KVP, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
 return packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDRESS_PROPORTIONAL_VELOCITY_430, KVP, &dxl_error);
}

int setProportionalPosition430(int DXL_ID, int KPP, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
 return packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDRESS_PROPORTIONAL_POSITION_430, KPP, &dxl_error);
}

int setIntegralPosition430(int DXL_ID, int KPI, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
 return packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDRESS_INTEGRAL_POSITION_430, KPI, &dxl_error);
}

int setDerivativePosition430(int DXL_ID, int KPD, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
 return packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDRESS_DERIVATIVE_POSITION_430, KPD, &dxl_error);
}

int setGoalVelocity430(int DXL_ID, int goalVelocity, dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler){
 return packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDRESS_GOAL_VELOCITY_430, goalVelocity, &dxl_error);
}

void readPWM(Q430_t *PWM430, dynamixel::GroupSyncRead *groupSyncReadPWM430, dynamixel::PacketHandler *packetHandler) {
  dxl_comm_result = groupSyncReadPWM430->txRxPacket();

  int pwm1_encoder = groupSyncReadPWM430->getData(DXL1_ID, ADDRESS_PRESENT_PWM_430, LENGTH_PRESENT_PWM_430);
  int pwm2_encoder = groupSyncReadPWM430->getData(DXL2_ID, ADDRESS_PRESENT_PWM_430, LENGTH_PRESENT_PWM_430);
  int pwm3_encoder = groupSyncReadPWM430->getData(DXL3_ID, ADDRESS_PRESENT_PWM_430, LENGTH_PRESENT_PWM_430);

  PWM430->q1 = pwm1_encoder*PWM_TO_PERCENTAGE;
  PWM430->q2 = pwm2_encoder*PWM_TO_PERCENTAGE;
  PWM430->q3 = pwm3_encoder*PWM_TO_PERCENTAGE;
}

void readLoad(Q430_t *I430, dynamixel::GroupSyncRead *groupSyncReadLoad430, dynamixel::PacketHandler *packetHandler) {
  dxl_comm_result = groupSyncReadLoad430->txRxPacket();

  int i1_encoder = groupSyncReadLoad430->getData(DXL1_ID, ADDRESS_PRESENT_LOAD_430, LENGTH_PRESENT_LOAD_430);
  int i2_encoder = groupSyncReadLoad430->getData(DXL2_ID, ADDRESS_PRESENT_LOAD_430, LENGTH_PRESENT_LOAD_430);
  int i3_encoder = groupSyncReadLoad430->getData(DXL3_ID, ADDRESS_PRESENT_LOAD_430, LENGTH_PRESENT_LOAD_430);

  I430->q1 = i1_encoder*LOAD_TO_PERCENTAGE;
  I430->q2 = i2_encoder*LOAD_TO_PERCENTAGE;
  I430->q3 = i3_encoder*LOAD_TO_PERCENTAGE;
}



void readQd(Q430_t *Qd430, dynamixel::GroupSyncRead *groupSyncReadVelocity430, dynamixel::PacketHandler *packetHandler) {
  dxl_comm_result = groupSyncReadVelocity430->txRxPacket();

  int qd1_encoder = groupSyncReadVelocity430->getData(DXL1_ID, ADDRESS_PRESENT_VELOCITY_430, LENGTH_PRESENT_VELOCITY_430);
  int qd2_encoder = groupSyncReadVelocity430->getData(DXL2_ID, ADDRESS_PRESENT_VELOCITY_430, LENGTH_PRESENT_VELOCITY_430);
  int qd3_encoder = groupSyncReadVelocity430->getData(DXL3_ID, ADDRESS_PRESENT_VELOCITY_430, LENGTH_PRESENT_VELOCITY_430);

  Qd430->q1 = convertToRadiansPerSecond(qd1_encoder, false);
  Qd430->q2 = convertToRadiansPerSecond(qd2_encoder, true);
  Qd430->q3 = convertToRadiansPerSecond(qd3_encoder, true);

}
void readQ(Q430_t *Q430, Q320_t *Q320, dynamixel::GroupSyncRead *groupSyncRead430, dynamixel::GroupSyncRead *groupSyncRead320, dynamixel::PacketHandler *packetHandler) {
  dxl_comm_result = groupSyncRead430->txRxPacket();
  dxl_comm_result = groupSyncRead320->txRxPacket();

  int q1_encoder = groupSyncRead430->getData(DXL1_ID, ADDRESS_PRESENT_POSITION_430, LENGTH_PRESENT_POSITION_430);
  int q2_encoder = groupSyncRead430->getData(DXL2_ID, ADDRESS_PRESENT_POSITION_430, LENGTH_PRESENT_POSITION_430);
  int q3_encoder = groupSyncRead430->getData(DXL3_ID, ADDRESS_PRESENT_POSITION_430, LENGTH_PRESENT_POSITION_430);
  int q4_encoder = groupSyncRead320->getData(DXL4_ID, ADDRESS_PRESENT_POSITION_320, LENGTH_PRESENT_POSITION_320);
  int q5_encoder = groupSyncRead320->getData(DXL5_ID, ADDRESS_PRESENT_POSITION_320, LENGTH_PRESENT_POSITION_320);
  int q6_encoder = groupSyncRead320->getData(DXL6_ID, ADDRESS_PRESENT_POSITION_320, LENGTH_PRESENT_POSITION_320);

  Q430->q1 = convertToRadians430(q1_encoder, false, Q1_SCALE, Q1_OFFSET);
  Q430->q2 = convertToRadians430(q2_encoder, true,  Q2_SCALE, Q2_OFFSET);
  Q430->q3 = convertToRadians430(q3_encoder, true,  Q3_SCALE, Q3_OFFSET);
  Q320->q4 = convertToRadians320(q4_encoder, false, Q4_SCALE, Q4_OFFSET);
  Q320->q5 = convertToRadians320(q5_encoder, false, Q5_SCALE, Q5_OFFSET);
  Q320->q6 = convertToRadians320(q6_encoder, false, Q6_SCALE, Q6_OFFSET);
  //  Q->q1 = -Q1_OFFSET*PI/180 - PI + Q1_SCALE*ANGLE_CONVERSION_CONSTANT_430*(groupSyncRead430->getData(DXL1_ID, ADDRESS_PRESENT_POSITION_430, LENGTH_PRESENT_POSITION_430));
  //  Q->q2 = Q2_OFFSET*PI/180 + PI - ANGLE_CONVERSION_CONSTANT_430*(groupSyncRead430->getData(DXL2_ID, ADDRESS_PRESENT_POSITION_430, LENGTH_PRESENT_POSITION_430));
  //  Q->q3 = Q3_OFFSET*PI/180 + PI - ANGLE_CONVERSION_CONSTANT_430*(groupSyncRead430->getData(DXL3_ID, ADDRESS_PRESENT_POSITION_430, LENGTH_PRESENT_POSITION_430));
  //  Q->q4 = -(5*PI/6) + ANGLE_CONVERSION_CONSTANT_320 * (groupSyncRead320->getData(DXL4_ID, ADDRESS_PRESENT_POSITION_320, LENGTH_PRESENT_POSITION_320));
  //  Q->q5 = -(5*PI/6) + ANGLE_CONVERSION_CONSTANT_320 * (groupSyncRead320->getData(DXL5_ID, ADDRESS_PRESENT_POSITION_320, LENGTH_PRESENT_POSITION_320));
  //  Q->q6 = -(5*PI/6) + ANGLE_CONVERSION_CONSTANT_320 * (groupSyncRead320->getData(DXL6_ID, ADDRESS_PRESENT_POSITION_320, LENGTH_PRESENT_POSITION_320));
}

float convertToRadiansPerSecond(int qe, boolean flip){
  if (flip) {
    return -qe / VELOCITY_CONVERSION_CONSTANT_430;
  } else {
    return qe / VELOCITY_CONVERSION_CONSTANT_430;
  }
}

/*
   qe : joint in encoder units, [0, 4095], maps to [-180, 180] degrees
   flip : is motor flipped on robot, making our coordinates flipped from planned
   offset : degrees
   scale :
   return : [-PI, PI] radians
*/
float convertToRadians430(int qe, boolean flip, float scale, float offset) {
  offset *= PI / 180;
  if (flip) {
    return offset + PI - qe * scale * ANGLE_CONVERSION_CONSTANT_430;
  } else {
    return -offset - PI + qe * scale * ANGLE_CONVERSION_CONSTANT_430;
  }
}

/*
   qe : joint in encoder units, [0, 2047], maps to [-150, 150] degrees
   flip : is motor flipped on robot, making our coordinates flipped from planned
   offset : degrees
   scale :
   return : [-5*PI/6, 5*PI/6] radians
*/
float convertToRadians320(int qe, boolean flip, float scale, float offset) {
  offset *= PI / 180;
  if (flip) {
    return offset + 5 * PI / 6 - qe * scale * ANGLE_CONVERSION_CONSTANT_320;
  } else {
    return -offset - 5 * PI / 6 + qe * scale * ANGLE_CONVERSION_CONSTANT_320;
  }
}

void writeQ(Q430_t *Q430, Q320_t *Q320, dynamixel::GroupSyncWrite *groupSyncWrite430, dynamixel::GroupSyncWrite *groupSyncWrite320,  dynamixel::PacketHandler *packetHandler) {

  int q1 = convertToPositionCommand430(Q430->q1, false, Q1_SCALE, Q1_OFFSET);
  int q2 = convertToPositionCommand430(Q430->q2, true, Q2_SCALE, Q2_OFFSET);
  int q3 = convertToPositionCommand430(Q430->q3, true, Q3_SCALE, Q3_OFFSET);
  int q4 = convertToPositionCommand320(Q320->q4, false, Q4_SCALE, Q4_OFFSET);
  int q5 = convertToPositionCommand320(Q320->q5, false, Q5_SCALE, Q5_OFFSET);
  int q6 = convertToPositionCommand320(Q320->q6, false, Q6_SCALE, Q6_OFFSET);

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


  // Syncwrite goal position for 430's
  dxl_comm_result = groupSyncWrite430->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);
  // SyncWrite goal position for 320's
  dxl_comm_result = groupSyncWrite320->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

  // Clear syncwrite parameter storage
  groupSyncWrite430->clearParam();
  groupSyncWrite320->clearParam();
}

void writeQd(Q430_t *Qd430, Q320_t *Q320, dynamixel::GroupSyncWrite *groupSyncWriteVelocity430, dynamixel::GroupSyncWrite *groupSyncWrite320,  dynamixel::PacketHandler *packetHandler) {

  int q1 = convertToVelocityCommand430(Qd430->q1, false);
  int q2 = convertToVelocityCommand430(Qd430->q2, true);
  int q3 = convertToVelocityCommand430(Qd430->q3, true);

  int q4 = convertToPositionCommand320(Q320->q4, false, Q4_SCALE, Q4_OFFSET);
  int q5 = convertToPositionCommand320(Q320->q5, false, Q5_SCALE, Q5_OFFSET);
  int q6 = convertToPositionCommand320(Q320->q6, false, Q6_SCALE, Q6_OFFSET);

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

  dxl_addparam_result = groupSyncWriteVelocity430->addParam(DXL1_ID, q1_ba);
  dxl_addparam_result = groupSyncWriteVelocity430->addParam(DXL2_ID, q2_ba);
  dxl_addparam_result = groupSyncWriteVelocity430->addParam(DXL3_ID, q3_ba);

  dxl_addparam_result = groupSyncWrite320->addParam(DXL4_ID, q4_ba);
  dxl_addparam_result = groupSyncWrite320->addParam(DXL5_ID, q5_ba);
  dxl_addparam_result = groupSyncWrite320->addParam(DXL6_ID, q6_ba);

  // Syncwrite goal velociy for 430's
  dxl_comm_result = groupSyncWriteVelocity430->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);
  dxl_comm_result = groupSyncWrite320->txPacket();
  // SyncWrite goal position for 320's
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

  // Clear syncwrite parameter storage
  groupSyncWriteVelocity430->clearParam();
  groupSyncWrite320->clearParam();

}

void writeQdNo430Sync(Q430_t *Qd430, Q320_t *Q320, dynamixel::GroupSyncWrite *groupSyncWrite320,  dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler) {

  int q1 = convertToVelocityCommand430(Qd430->q1, false);
  int q2 = convertToVelocityCommand430(Qd430->q2, true);
  int q3 = convertToVelocityCommand430(Qd430->q3, true);

  int q4 = convertToPositionCommand320(Q320->q4, false, Q4_SCALE, Q4_OFFSET);
  int q5 = convertToPositionCommand320(Q320->q5, false, Q5_SCALE, Q5_OFFSET);
  int q6 = convertToPositionCommand320(Q320->q6, false, Q6_SCALE, Q6_OFFSET);

  uint8_t q4_ba[4];
  uint8_t q5_ba[4];
  uint8_t q6_ba[4];

  convertToByteArray(q4_ba, q4);
  convertToByteArray(q5_ba, q5);
  convertToByteArray(q6_ba, q6);

  setGoalVelocity430(DXL1_ID, q1, portHandler, packetHandler);
  setGoalVelocity430(DXL2_ID, q2, portHandler, packetHandler);
  setGoalVelocity430(DXL3_ID, q3, portHandler, packetHandler);

  dxl_addparam_result = groupSyncWrite320->addParam(DXL4_ID, q4_ba);
  dxl_addparam_result = groupSyncWrite320->addParam(DXL5_ID, q5_ba);
  dxl_addparam_result = groupSyncWrite320->addParam(DXL6_ID, q6_ba);

  // Syncwrite goal position
  dxl_comm_result = groupSyncWrite320->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

  // Clear syncwrite parameter storage
  groupSyncWrite320->clearParam();

}

void writePWM(Q430_t *Qd430, Q320_t *Q320, dynamixel::GroupSyncWrite *groupSyncWritePWM430, dynamixel::GroupSyncWrite *groupSyncWrite320,  dynamixel::PacketHandler *packetHandler) {

  int q1 = convertToPWMCommand430(Qd430->q1, false);
  int q2 = convertToPWMCommand430(Qd430->q2, true);
  int q3 = convertToPWMCommand430(Qd430->q3, true);

  int q4 = convertToPositionCommand320(Q320->q4, false, Q4_SCALE, Q4_OFFSET);
  int q5 = convertToPositionCommand320(Q320->q5, false, Q5_SCALE, Q5_OFFSET);
  int q6 = convertToPositionCommand320(Q320->q6, false, Q6_SCALE, Q6_OFFSET);

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

  dxl_addparam_result = groupSyncWritePWM430->addParam(DXL1_ID, q1_ba);
  dxl_addparam_result = groupSyncWritePWM430->addParam(DXL2_ID, q2_ba);
  dxl_addparam_result = groupSyncWritePWM430->addParam(DXL3_ID, q3_ba);

  dxl_addparam_result = groupSyncWrite320->addParam(DXL4_ID, q4_ba);
  dxl_addparam_result = groupSyncWrite320->addParam(DXL5_ID, q5_ba);
  dxl_addparam_result = groupSyncWrite320->addParam(DXL6_ID, q6_ba);

  // Syncwrite goal PWM for 430's
  dxl_comm_result = groupSyncWritePWM430->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);
  // Syncwrite goal position for 320's
  dxl_comm_result = groupSyncWrite320->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

  // Clear syncwrite parameter storage
  groupSyncWritePWM430->clearParam();
  groupSyncWrite320->clearParam();

}


/*
   qd (rad/s) --> rpm
   return : encoding units [0 1023]
*/
int convertToVelocityCommand430(float qd, boolean flip) {
  if (flip) {
    return -qd * VELOCITY_CONVERSION_CONSTANT_430;
  }
  else {
    return qd * VELOCITY_CONVERSION_CONSTANT_430;
  }
}

int convertToPWMCommand430(float qd, boolean flip) {
  if (flip) {
    return -qd * PWM_CONVERSION_CONSTANT_430;
  }
  else {
    return qd * PWM_CONVERSION_CONSTANT_430;
  }
}


/*
   flip : is motor flipped on robot, making our coordinates flipped from planned
   offset : degrees
   scale :
   return : encoder value [0, 4095], maps to [-180, 180] degrees
*/
int convertToPositionCommand430(float q, boolean flip, float scale, float offset) {
  offset *= PI / 180;
  if (flip) {
    return ( (-q + PI) / scale + offset) / ANGLE_CONVERSION_CONSTANT_430;
  }
  else {
    return ( (q + PI) / scale + offset) / ANGLE_CONVERSION_CONSTANT_430;
  }
}

/*
   flip : is motor flipped on robot, making our coordinates flipped from planned
   offset : degrees
   scale :
   return : encoder value [0, 2047], maps to [-150, 150] degrees
*/
int convertToPositionCommand320(float q, boolean flip, float scale, float offset) {
  offset *= PI / 180;
  if (flip) {
    return ( (-q + 150 * PI / 180) / scale + offset) / ANGLE_CONVERSION_CONSTANT_320;
  }
  else {
    return ( (q + 150 * PI / 180) / scale + offset) / ANGLE_CONVERSION_CONSTANT_320;
  }
}

void convertToByteArray(uint8_t *a, int val) {
  a[0] = DXL_LOBYTE(DXL_LOWORD(val));
  a[1] = DXL_HIBYTE(DXL_LOWORD(val));
  a[2] = DXL_LOBYTE(DXL_HIWORD(val));
  a[3] = DXL_HIBYTE(DXL_HIWORD(val));
}
