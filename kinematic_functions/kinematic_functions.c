#include "Arduino.h"
#include "math.h"
#include "kinematic_functions.h"
#include "dim.h" //Seperate "library" with just constants


//TODO: Include end-effector angles into structs/functions
//TODO: include dimensions array/struct, to take into account the changing lengths from calibration 

void inverse_kinematics(Q_t *Q, X_t X) {
  Q->q1 =  atan2(X.y, X.x);

  float L_b = sqrt(sq(X.x) + sq(X.y) + sq(X.z + L4 - L1));
  Q->q3 = cosine_rule(L_b, L2, L3) - (M_PI/2);

  float rad = atan2(X.z + L4 - L1, sqrt(sq(X.x) + sq(X.y)));
  Q->q2 = -((M_PI/2)- rad - cosine_rule(L3, L_b, L2));

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

/*outputs joint velocities Qd, given task velocities Xd, and joint angles Q*/
void inverse_jacobian(Q_t *Qd, X_t Xd, Q_t Q) {
  //Calculate inverse jacobian
  float iJ[5][5];
  inverse_jacobian_calculator(iJ, Q);

  //Caculate task space velocities into frame{1}
  //Currently making a copy, for efficiency change original struct...
  X_t Xd1 = rotation(Q, Xd);
  Xd1.wy = 0.0; //enforce constraint on y axis.
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

  //unneccessary 4th column which multiples with wy=0
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
  //Xd->wy = 0;
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