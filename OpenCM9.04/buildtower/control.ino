X_t feedback(X_t Xprev, X_t Xref, X_t X) {
  X_t Xe;
  float K = 0;
  Xe.x = Xref.x + K * (Xprev.x - X.x);
  Xe.y = Xref.y + K * (Xprev.y - X.y);
  Xe.z = Xref.z + K * (Xprev.z - X.z);
  Xe.wz = Xref.wz + K * (Xprev.wz - X.wz);
  Xe.grip = Xref.grip;
  return Xe;
}

X_t velocityFeedback(X_t Xdref, X_t Xref, X_t X) {
  X_t Xke;
  //  float K=5;
  Xke.x = 1 * (Xref.x - X.x);
  Xke.y = 1 * (Xref.y - X.y);
  Xke.z = 1 * (Xref.z - X.z);
  Xke.wx = 0;
  Xke.wy = 0;
  Xke.wz = 0; // K*(Xprev.theta - X.theta);
  Xke.grip = 0;
  return Xke;
}

X_t velocityControl(X_t Xdref, X_t Xref, X_t Xke) {
  X_t Xc;
  Xc.x = Xdref.x + Xke.x;
  Xc.y = Xdref.y + Xke.y;
  Xc.z = Xdref.z + Xke.z;
  Xc.wx = 0;
  Xc.wy = 0;
  Xc.wz = Xref.wz +  Xke.wz;
  Xc.grip = Xref.grip + Xke.grip;
  return Xc;
}
