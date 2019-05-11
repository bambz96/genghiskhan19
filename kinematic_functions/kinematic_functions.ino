//needs to be placed in header file,
//currently unneccessary for testing purposes
#include <math.h>
//check file path
#include <dimensions.h>

//TODO: Include end-effector angles into structs/functions
//TODO: include dimensions array/struct, to take into account the changing lengths from calibration
typedef struct {
  float x;
  float y;
  float z;
  //Keeping all angular variables and removing them after calculation
  float wx; // 0 in frame {0} & {1}
  float wy; // ... don't really care
  float wz; //only really care about z rotation
} X_t;

typedef struct {
  float q1;
  float q2;
  float q3;
  float q4;
  float q5;
} Q_t;

//float L1 = 0.2;
//float L2 = 0.2;
//float L3 = 0.2;
//float L4 = 0.138;

float piOverTwo = M_PI / 2;

X_t X = {0.1, 0.1, 0.1, 0, 0, piOverTwo};
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

void inverse_kinematics(Q_t *Q, X_t X) {
  Q->q1 =  atan2(X.y, X.x);

  float L_b = sqrt(sq(X.x) + sq(X.y) + sq(X.z + L4 - L1));
  //make sure to check the home position ...
  Q->q3 = cosine_rule(L_b, L2, L3) - piOverTwo;

  float rad = atan2(X.z + L4 - L1, sqrt(sq(X.x) + sq(X.y)));
  Q->q2 = -(piOverTwo - rad - cosine_rule(L3, L_b, L2));

  Q->q4 = -(Q->q2 + Q->q3);

  Q->q5 = Q->q1 - X.wz;

}

void forward_kinematics(X_t *X, Q_t Q) {
  X->x = cos(Q.q1) * (L3 * cos(Q.q2 + Q.q3) - L2 * sin(Q.q2));
  X->y = sin(Q.q1) * (L3 * cos(Q.q2 + Q.q3) - L2 * sin(Q.q2));
  X->z = L3 * sin(Q.q2 + Q.q3) + L2 * cos(Q.q2) - L4 + L1;
  X->wz = Q.q1 - Q.q5;
}

float cosine_rule(float a, float b, float c) {
  // a is opposite the required angle
  return acos((sq(b) + sq(c) - sq(a)) / (2 * b * c));
}

//outputs joint velocities Qd, given task velocities Xd, and joint angles Q,
void inverse_jacobian(Q_t *Qd, X_t Xd, Q_t Q) {
  //Calculate inverse jacobian
  float iJ[5][5];
  inverse_jacobian_calculator(iJ, Q);

  //Caculate task space velocities into frame{1}
  //Currently making a copy, for efficiency change original struct...
  X_t Xd1 = rotation(Q, Xd);
  //Removed Xd1.wx from joint velocity calculation as per method 2
  Qd->q1 = iJ[0][0] * Xd1.x + iJ[0][1] * Xd1.y + iJ[0][2] * Xd1.z + iJ[0][3] * Xd1.wy + iJ[0][4] * Xd1.wz;
  Qd->q2 = iJ[1][0] * Xd1.x + iJ[1][1] * Xd1.y + iJ[1][2] * Xd1.z + iJ[1][3] * Xd1.wy + iJ[1][4] * Xd1.wz;
  Qd->q3 = iJ[2][0] * Xd1.x + iJ[2][1] * Xd1.y + iJ[2][2] * Xd1.z + iJ[2][3] * Xd1.wy + iJ[2][4] * Xd1.wz;
  Qd->q4 = iJ[3][0] * Xd1.x + iJ[3][1] * Xd1.y + iJ[3][2] * Xd1.z + iJ[3][3] * Xd1.wy + iJ[3][4] * Xd1.wz;
  Qd->q5 = iJ[4][0] * Xd1.x + iJ[4][1] * Xd1.y + iJ[4][2] * Xd1.z + iJ[4][3] * Xd1.wy + iJ[4][4] * Xd1.wz;
}

// Helper function to rotate task space velocities to frame {1}.
X_t rotation(Q_t Q, X_t Xd) {
  float R01[3][3] = {
    {cos(Q.q1), -sin(Q.q1), 0},
    {sin(Q.q1), cos(Q.q1), 0},
    {0, 0, 1}
  };
  X_t Xd1;
  Xd1.x = R01[0][0] * Xd.x +  R01[0][1]  * Xd.y; //Omit elements with 0 multiplication
  Xd1.y = R01[1][0] * Xd.x +  R01[1][1]  * Xd.y;
  Xd1.z = R01[2][2] * Xd.z;
  //Can omit Xd1.wx as it is removed later in outer loop
  Xd1.wx = R01[0][0] * Xd.wx +  R01[0][1] * Xd.wy;
  Xd1.wy = R01[1][0] * Xd.wx +  R01[1][1] * Xd.wy;
  Xd1.wz = R01[2][2] * Xd.wz;
  return Xd1;
}

//Helper function to calculate the inverse jacobian given joint angles.
void inverse_jacobian_calculator(float inv_jacobian[5][5], Q_t Q) {
  inv_jacobian[0][0] = -sin(2 * Q.q1) / (L3 * cos(Q.q2 + Q.q3) - L2 * sin(Q.q2));
  inv_jacobian[0][1] =  cos(2 * Q.q1) / (L3 * cos(Q.q2 + Q.q3) - L2 * sin(Q.q2));
  inv_jacobian[0][2] = 0; //Remove these lines with 0's when inv_jacobian is initialized to 0
  inv_jacobian[0][3] = 0;
  inv_jacobian[0][4] = 0;
  inv_jacobian[1][0] = cos(2 * Q.q1) / (L3 * cos(Q.q2 + Q.q3) - L2 * sin(Q.q2));
  inv_jacobian[1][1] = -(sin(2 * Q.q1) * cos(Q.q2 + Q.q3)) / (L2 * cos(Q.q3));
  inv_jacobian[1][2] = -sin(Q.q2 + Q.q3) / (L2 * cos(Q.q3));
  inv_jacobian[1][3] = 0;
  inv_jacobian[1][4] = 0;
  inv_jacobian[2][0] = (cos(2 * Q.q1) * (L3 * cos(Q.q2 + Q.q3) - L2 * sin(Q.q2))) / (L2 * L3 * cos(Q.q3));
  inv_jacobian[2][1] = (sin(2 * Q.q1) * (L3 * cos(Q.q2 + Q.q3) - L2 * sin(Q.q2))) / (L2 * L3 * cos(Q.q3));
  inv_jacobian[2][2] = (L3 * sin(Q.q2 + Q.q3) + L2 * cos(Q.q2)) / (L2 * L3 * cos(Q.q3));
  inv_jacobian[2][3] = 0;
  inv_jacobian[2][4] = 0;
  inv_jacobian[3][0] = (L3 * sq(cos(2 * Q.q1)) * sin(3 * Q.q2 + 2 * Q.q3 + Q.q4) - 2 * L3 * sin(Q.q2 + Q.q4) - 2 * L3 * sin(Q.q2 + 2 * Q.q3 + Q.q4) + L3 * sin(Q.q2 - Q.q4) * sq(cos(2 * Q.q1))  - 2 * L2 * cos(Q.q2 + Q.q3 + Q.q4) * sq(cos(2 * Q.q1)) + L2 * sq(cos(2 * Q.q1)) * cos(Q.q3 - Q.q2 + Q.q4) + L2 * sq(cos(2 * Q.q1))  * cos(3 * Q.q2 + Q.q3 + Q.q4) + L3 * sq(cos(2 * Q.q1)) * sin(Q.q2 + 2 * Q.q3 + Q.q4) + 3 * L3 * sq(cos(2 * Q.q1)) * sin(Q.q2 + Q.q4)) / (2 * L3 * cos(2 * Q.q1) * cos(Q.q3) * (L2 * sin(Q.q3 + Q.q4) - L2 * sin(2 * Q.q2 + Q.q3 + Q.q4) + L3 * cos(Q.q4) + L3 * cos(2 * Q.q2 + 2 * Q.q3 + Q.q4)));
  inv_jacobian[3][1] = (L2 * sin(2 * Q.q1) * (cos(Q.q3 - Q.q2 + Q.q4) - 2 * cos(Q.q2 + Q.q3 + Q.q4) + cos(3 * Q.q2 + Q.q3 + Q.q4)) + L3 * sin(2 * Q.q1) * (sin(Q.q2 - Q.q4) + sin(Q.q2 + 2 * Q.q3 + Q.q4) + 3 * sin(Q.q2 + Q.q4) + sin(3 * Q.q2 + 2 * Q.q3 + Q.q4))) / (L3 * (L3 * cos(2 * Q.q2 + Q.q3 + Q.q4) + L3 * cos(Q.q3 + Q.q4) + L3 * cos(2 * Q.q2 + 3 * Q.q3 + Q.q4) + L2 * sin(Q.q4) - L2 * sin(2 * Q.q2 + 2 * Q.q3 + Q.q4) + L3 * cos(Q.q3 - Q.q4) - L2 * sin(2 * Q.q2 + Q.q4) + L2 * sin(2 * Q.q3 + Q.q4)));
  inv_jacobian[3][2] = -cos(Q.q2) / (L3 * cos(Q.q3));
  inv_jacobian[3][3] = -1 / cos(2 * Q.q1);
  inv_jacobian[3][4] = -tan(Q.q2 + Q.q3 + Q.q4) * tan(2 * Q.q1);
  inv_jacobian[4][0] = -(2 * sin(2 * Q.q1)) / (L2 * sin(Q.q3 + Q.q4) - L2 * sin(2 * Q.q2 + Q.q3 + Q.q4) + L3 * cos(Q.q4) + L3 * cos(2 * Q.q2 + 2 * Q.q3 + Q.q4));
  inv_jacobian[4][1] = (2 * cos(2 * Q.q1)) / (L2 * sin(Q.q3 + Q.q4) - L2 * sin(2 * Q.q2 + Q.q3 + Q.q4) + L3 * cos(Q.q4) + L3 * cos(2 * Q.q2 + 2 * Q.q3 + Q.q4));
  inv_jacobian[4][2] = 0;
  inv_jacobian[4][3] = 0;
  inv_jacobian[4][4] = -1 / cos(Q.q2 + Q.q3 + Q.q4);
}

//Calculate task space velocities given joint angles and velocities
void jacobian(X_t *Xd, Q_t Qd, Q_t Q) {
  float j[6][5];
  jacobian_calculator(j, Q);
  Xd->x = j[0][0] * Qd.q1 + j[0][1] * Qd.q2 + j[0][2] * Qd.q3;
  Xd->y = j[1][0] * Qd.q1 + j[1][1] * Qd.q2 + j[1][2] * Qd.q3;
  Xd->z = j[2][1] * Qd.q2 + j[2][2] * Qd.q3;
  Xd->wx = j[3][1] * Qd.q2 + j[3][2] * Qd.q3 + j[3][3] * Qd.q4 + j[3][4] * Qd.q5; 
  Xd->wy = j[4][1] * Qd.q2 + j[4][2] * Qd.q3 + j[4][3] * Qd.q4 + j[4][4] * Qd.q5; 
  Xd->wz = j[5][0] * Qd.q1 + j[5][4] * Qd.q5;  
}

void jacobian_calculator(float jacobian[6][5], Q_t Q) {
  jacobian[0][0] = -sin(Q.q1) * (L3 * cos(Q.q2 + Q.q3) - L2 * sin(Q.q2));
  jacobian[0][1] = -cos(Q.q1) * (L3 * sin(Q.q2 + Q.q3) + L2 * cos(Q.q2));
  jacobian[0][2] = -L3 * sin(Q.q2 + Q.q3) * cos(Q.q1);
  jacobian[0][3] = 0;
  jacobian[0][4] = 0;

  jacobian[1][0] = cos(Q.q1) * (L3 * cos(Q.q2 + Q.q3) - L2 * sin(Q.q2));
  jacobian[1][1] = -sin(Q.q1) * (L3 * sin(Q.q2 + Q.q3) + L2 * cos(Q.q2));
  jacobian[1][2] = -L3 * sin(Q.q2 + Q.q3) * sin(Q.q1);
  jacobian[1][3] = 0;
  jacobian[1][4] = 0;

  jacobian[2][0] = 0;
  jacobian[2][1] = L3 * cos(Q.q2 + Q.q3) - L2 * sin(Q.q2);
  jacobian[2][2] = L3 * cos(Q.q2 + Q.q3);
  jacobian[2][3] = 0;
  jacobian[2][4] = 0;

  jacobian[3][0] = 0;
  jacobian[3][1] = sin(Q.q1);
  jacobian[3][2] = sin(Q.q1);
  jacobian[3][3] = sin(Q.q1);
  jacobian[3][4] = sin(Q.q1 + Q.q2 + Q.q3 + Q.q4) / 2 + sin(Q.q2 - Q.q1 + Q.q3 + Q.q4) / 2;

  jacobian[4][0] = 0;
  jacobian[4][1] = -cos(Q.q1);
  jacobian[4][2] = -cos(Q.q1);
  jacobian[4][3] = -cos(Q.q1);
  jacobian[4][4] = cos(Q.q2 - Q.q1 + Q.q3 + Q.q4) / 2 - cos(Q.q1 + Q.q2 + Q.q3 + Q.q4) / 2;

  jacobian[5][0] = 1;
  jacobian[5][1] = 0;
  jacobian[5][2] = 0;
  jacobian[5][3] = 0;
  jacobian[5][4] = -cos(Q.q2 + Q.q3 + Q.q4);
}

void printX(X_t X) {
  Serial.print(X.x);
  Serial.print(", ");
  Serial.print(X.y);
  Serial.print(", ");
  Serial.print(X.z);
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
