/* 
Testing .ino the kinematic functions library currently in USER/Documents/Arduino/libraries/kinematic_functions
*/

extern "C"{
  #include "kinematic_functions.h"
}
#include "math.h"
X_t X = {0.1, 0.1, 0.1, 0, 0, M_PI/2};
X_t Xd = {0.05, 0.05, 0.05, 0, 0, M_PI / 8};

unsigned long t1;
unsigned long t2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
}

void loop() {
  if (X.x > 0.25) {
    X.x = 0.1;
    X.y = 0.1;
    X.z = 0.1;
    Xd.x = 0.05;
    Xd.y = 0.05;
    Xd.z = 0.05;
    Xd.wx = 0;
    Xd.wy = 0;
    Xd.wz = M_PI/8;
  }

  //Inverse kinematics test
  Serial.print("Inverse Kinematics: ");
  Serial.print("Position = ");
  printX(X);
  Q_t Q;
  t1 = micros();
  inverse_kinematics(&Q, X);
  t2 = micros();
  Serial.print("Joint angles = ");
  printQ(Q);
  Serial.print("Time taken = ");
  Serial.println(pow(10, -6) * (t2 - t1), 6);

  //Forward Kinematics test
  Serial.print("Forward Kinematics: ");
  Serial.print("Joint angles = ");
  printQ(Q);
  X_t X_test;
  t1 = micros();
  forward_kinematics(&X_test, Q);
  t2 = micros();
  Serial.print("Position = ");
  printX(X_test);
  Serial.print("Time taken = ");
  Serial.println(pow(10, -6) * (t2 - t1), 6);

  //Inverse Jacobian calculation
  Serial.print("Inverse Jacobian: ");
  Serial.print("Task velocities = ");
  printX(Xd);
  Q_t Qd;
  t1 = micros();
  inverse_jacobian(&Qd, Xd, Q);
  t2 = micros();
  Serial.print("Joint velocities = ");
  printQ(Qd);
  Serial.print("Time taken = ");
  Serial.println(pow(10, -6) * (t2 - t1), 6);

  //Jacobian calculation
  Serial.print("Jacobian: ");
  Serial.print("Joint velocities = ");
  printQ(Qd);
  t1 = micros();
  jacobian(&Xd, Qd, Q);
  t2 = micros();
  Serial.print("Task velocities = ");
  printX(Xd);
  Serial.print("Time taken = ");
  Serial.println(pow(10, -6) * (t2 - t1), 6);

  Serial.println();
  X.x = X.x + 0.01;
  X.y = X.y + 0.01;
  X.z = X.z + 0.01;
  Xd.x = Xd.x + 0.025;
  Xd.y = Xd.y + 0.025;
  Xd.z = Xd.z + 0.025;
}

void printX(X_t X) {
  Serial.print(X.x);
  Serial.print(", ");
  Serial.print(X.y);
  Serial.print(", ");
  Serial.print(X.z);
  Serial.print(", ");
  Serial.print(X.wx);
  Serial.print(", ");
  Serial.print(X.wy);
  Serial.print(", ");
  Serial.print(X.wz);
  Serial.print(", ");
}

void printQ(Q_t Q) {
  Serial.print(Q.q1);
  Serial.print(", ");
  Serial.print(Q.q2);
  Serial.print(", ");
  Serial.print(Q.q3);
  Serial.print(", ");
  Serial.print(Q.q4);
  Serial.print(", ");
  Serial.print(Q.q5);
  Serial.print(", ");
}
