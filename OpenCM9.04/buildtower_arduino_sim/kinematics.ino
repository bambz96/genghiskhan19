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
