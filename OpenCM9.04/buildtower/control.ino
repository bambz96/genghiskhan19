X_t feedback(X_t Xprev, X_t Xref, X_t X){
  X_t Xe;
  float K = 0;
  Xe.x = Xref.x + K*(Xprev.x - X.x);
  Xe.y = Xref.y + K*(Xprev.y - X.y);
  Xe.z = Xref.z + K*(Xprev.z - X.z);
  Xe.theta = Xref.theta + K*(Xprev.theta - X.theta);
  Xe.grip = Xref.grip;
  return Xe;
}

X_t velocityFeedback(X_t Xdref, X_t Xref, X_t X){
  X_t Xke;
  float K=5;
  Xke.x = K*(Xref.x - X.x);
  Xke.y = K*(Xref.y - X.y);
  Xke.z = K*(Xref.z - X.z);
  Xke.theta = 0; // K*(Xprev.theta - X.theta);
  Xke.grip = 0;
  return Xke;
}

X_t velocityControl(X_t Xdref, X_t Xref, X_t Xc) {
  X_t Xc;
  Xc.x = Xdref.x + Xke.x;
  Xc.y = Xdref.y + Xke.y;
  Xc.z = Xdref.z + Xke.z;
  Xc.theta = Xref.theta + Xke.theta;
  Xc.grip = Xref.grip + Xke.grip;
}
