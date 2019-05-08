//needs to be placed in header file, 
//currently unneccessary for testing purposes

#include <math.h>  

//TODO: Include end-effector angles into structs/functions
//TODO: include dimensions array/struct, to take into account the changing lengths from calibration 
typedef struct {
  float x; 
  float y;
  float z;
  float theta; 
} X_t;

typedef struct {
  float q1;
  float q2;
  float q3;
  float q4;
  float q5;
} Q_t;

float L1 = 0.2;
float L2 = 0.2;
float L3 = 0.2;
float L4 = 0.138;

float piOverTwo = M_PI/2;

X_t X = {0.1, 0.1, 0.1, piOverTwo};

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
  }

  //Inverse kinematics test
  Serial.print("Inverse Kinematics: ");
  printX(X);
  Q_t Q; 
  t1 = micros();
  inverse_kinematics(&Q, &X);
  t2 = micros();
  printQ(Q);
  Serial.print("Time taken = ");
  Serial.println(pow(10,-6)*(t2-t1),6);

  //Forward Kinematics test
  Serial.print("Forward Kinematics: ");
  printQ(Q);
  X_t X_test;
  t1 = micros();
  forward_kinematics(&X_test, &Q);
  t2 = micros();
  printX(X_test);
  Serial.print("Time taken = ");
  Serial.println(pow(10,-6)*(t2-t1),6);
  Serial.println();
  
  X.x = X.x + 0.01;
  X.y = X.y + 0.01;
  X.z = X.z + 0.01;
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

void forward_kinematics(X_t *X, Q_t *Q){
  X->x = cos(Q->q1)*(L3*cos(Q->q2 + Q->q3) - L2*sin(Q->q2));
  X->y = sin(Q->q1)*(L3*cos(Q->q2 + Q->q3) - L2*sin(Q->q2));
  X->z = L3*sin(Q->q2 + Q->q3) + L2*cos(Q->q2) - L4 + L1;
  X->theta = Q->q1 - Q->q5;
}

float cosine_rule(float a, float b, float c) {
  // a is opposite the required angle
  return acos((sq(b) + sq(c) - sq(a))/(2*b*c));
}

void printX(X_t X){
  Serial.print("Position = ");
  Serial.print(X.x);
  Serial.print(", ");
  Serial.print(X.y);
  Serial.print(", ");
  Serial.print(X.z);
  Serial.print(", ");
  Serial.print(X.theta);
  Serial.print(", ");
}

void printQ(Q_t Q){
  Serial.print("Joint angles = ");
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
