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

X_t velocityFeedback(X_t Xdref, X_t Xref, X_t X, Q430_t Qm430, Q320_t Qm320, Q430_t Qr430, Q320_t Qr320, float dt) {

  Xerr = xSubtraction(Xref, X);
  Xtotalerr = xAddition(Xtotalerr, Xerr);
  Xdeltaerr = xSubtraction(Xerr, Xpreverr);

  X_t Xke = pid(Xerr, Xdeltaerr, Xtotalerr, dt);
  X_t Xor = orientation_error(Qm430, Qm320, Qr430, Qr320);
  //  float K=5;
  //Xke.x = 5 * (Xref.x - X.x);
  //Xke.y = 7 * (Xref.y - X.y);
//  Xke.z = Xke.z
  Xke.wx = 5 * Xor.wx;
  Xke.wy = 5 * Xor.wy;
  Xke.wz = 5 * Xor.wz;
  Xke.grip = 0;

  Xpreverr = Xerr;
  return Xke;
}

X_t pid(X_t Xerr, X_t Xdeltaerr, X_t Xtotalerr, float dt) {
  /*Xref = reference, X = measured*/
  //float Kp = 5, Ki = 1, Kd = 1;
  X_t Xc;
  Xc.x = 5 * Xerr.x + 0.7 * dt * Xtotalerr.x + (0.375 / dt) * Xdeltaerr.x;
  Xc.y = 7 * Xerr.y + 0.5 * dt * Xtotalerr.y + (0 / dt) * Xdeltaerr.y;
  Xc.z = 5 * Xerr.z + 1 * dt * Xtotalerr.z + (1 / dt) * Xdeltaerr.z;
//  Xc.wx = Kp * Xerr.wx + Ki * dt * Xtotalerr.wx + (Kd / dt) * Xdeltaerr.wx;
//  Xc.wy = Kp * Xerr.wy + Ki * dt * Xtotalerr.wy + (Kd / dt) * Xdeltaerr.wy;
//  Xc.wz = Kp * Xerr.wz + Ki * dt * Xtotalerr.wz + (Kd / dt) * Xdeltaerr.wz;
//  Xc.grip = Xerr.grip;
  return Xc;
}

X_t xAddition(X_t x1, X_t x2) {
  X_t x;
  x.x = x1.x + x2.x;
  x.y = x1.y + x2.y;
  x.z = x1.z + x2.z;
  x.wx = x1.wx + x2.wx;
  x.wy = x1.wy + x2.wy;
  x.wz = x1.wz + x2.wz;
  x.grip = x1.grip;
  //x.grip = x1.grip + x2.grip;
  return x;
}

X_t xSubtraction(X_t x1, X_t x2) {
  X_t x;
  x.x = x1.x - x2.x;
  x.y = x1.y - x2.y;
  x.z = x1.z - x2.z;
  x.wx = x1.wx - x2.wx;
  x.wy = x1.wy - x2.wy;
  x.wz = x1.wz - x2.wz;
  x.grip = x1.grip;
  //x.grip = x1.grip + x2.grip;
  return x;
}

X_t velocityControl(X_t Xdref, X_t Xref, X_t Xke) {
  X_t Xc;
  Xc.x = Xdref.x + Xke.x;
  Xc.y = Xdref.y + Xke.y;
  Xc.z = Xdref.z + Xke.z;
  Xc.wx = Xdref.wx + Xke.wx;
  Xc.wy = Xdref.wy + Xke.wy;
  Xc.wz = Xref.wz +  Xke.wz;
  Xc.grip = Xref.grip + Xke.grip;
  return Xc;
}
