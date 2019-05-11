#ifndef kinematic_functions_h
#define kinematic_functions_h
  typedef struct {
    float x;
    float y;
    float z;
    float wx; 
    float wy; 
    float wz;
  } X_t;
  
  typedef struct {
    float q1;
    float q2;
    float q3;
    float q4;
    float q5;
  } Q_t;

  void inverse_kinematics(Q_t *Q, X_t X);
  void forward_kinematics(X_t *X, Q_t Q);
  float cosine_rule(float a, float b, float c);
  void inverse_jacobian(Q_t *Qd, X_t Xd, Q_t Q);
  X_t rotation(Q_t Q, X_t Xd);
  void inverse_jacobian_calculator(float inv_jacobian[5][5], Q_t Q);
  void jacobian(X_t *Xd, Q_t Qd, Q_t Q);
  void jacobian_calculator(float jacobian[6][5], Q_t Q);
  void printX(X_t X);
  void printQ(Q_t Q);
#endif
