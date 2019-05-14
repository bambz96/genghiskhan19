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
