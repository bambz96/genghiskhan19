void readData(struct Cubic *poly) {
  // "a3 a2 a1 a0 tf" has been sent, parse it in, create a new Cubic struct
  // and add it to the array provided
  if (Serial.available() > 0)
  {
    float a3 = Serial.parseFloat();
    float a2 = Serial.parseFloat();
    float a1 = Serial.parseFloat();
    float a0 = Serial.parseFloat();
    float tf = Serial.parseFloat();
    Serial.read(); // clear rest of input buffer (i.e. trailing \n
    // reply with read values
    Serial.print(a3, 5); Serial.print(' ');
    Serial.print(a2, 5); Serial.print(' ');
    Serial.print(a1, 5); Serial.print(' ');
    Serial.print(a0, 5); Serial.print(' ');
    Serial.print(tf, 5); Serial.print(' ');
    Serial.println();
    // create Cubic struct and save to given array of polynomials
    struct Cubic cubic;
    cubic.coef[0] = a0;
    cubic.coef[1] = a1;
    cubic.coef[2] = a2;
    cubic.coef[3] = a3;
    cubic.tf = tf * 1000; // convert s to ms
    poly[count] = cubic;
    count++;
  }
}

float poly(float t, float a0, float a1, float a2, float a3) {
  // evaluate the given cubic polynomial at time t
  return a3*t*t*t + a2*t*t + a1*t + a0;
}

void sendNPoly(int n, struct Cubic cubic[MAX_CUBICS]) {
  // send the first n polynomial path segments
  float t0 = 0.0;
  for (int i=0; i<n; i++) {
    for (int j=0; j<PLOTTED_PATH_RES; j++) {
      float t = j/float(PLOTTED_PATH_RES) * cubic[i].tf/1000.0; // convert to real duration
      sendPolyAtTime(t, t0, &cubic[i]);
    }
    t0 += cubic[i].tf/1000.0;
  }
}

void sendPolyAtTime(float t, float t0, struct Cubic *cubic) {
  // send ti and x(ti)
  float x = evaluate(cubic, t);
//  float a0 = cubic->coef[0];
//  float a1 = cubic->coef[1];
//  float a2 = cubic->coef[2];
//  float a3 = cubic->coef[3];
//  float x = poly(t, a0, a1, a2, a3);
  Serial.print(t+t0, 5); Serial.print(' ');
  Serial.print(x, 5); Serial.print(' ');
  Serial.println();
}

float evaluate(struct Cubic *cubic, float t) {
  // evaluate the given cubic at time t
  float a0 = cubic->coef[0];
  float a1 = cubic->coef[1];
  float a2 = cubic->coef[2];
  float a3 = cubic->coef[3];
  return poly(t, a0, a1, a2, a3);
}
