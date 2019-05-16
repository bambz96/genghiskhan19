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
  X_t Xe;
  float K=5;
  Xe.x = Xdref.x + K*(Xref.x - X.x);
  Xe.y = Xdref.y + K*(Xref.y - X.y);
  Xe.z = Xdref.z + K*(Xref.z - X.z);
  Xe.theta = Xref.theta; //+ K*(Xprev.theta - X.theta);
  Xe.grip = Xref.grip;
  return Xe;
}
