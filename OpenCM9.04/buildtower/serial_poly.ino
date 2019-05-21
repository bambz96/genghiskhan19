void readData(struct Cubic *poly) {
  // "a3 a2 a1 a0 t0 tf" has been sent, parse it in, create a new Cubic struct
  // and add it to the array provided
  if (Serial.available() > 0)
  {
    float a3 = Serial.parseFloat();
    float a2 = Serial.parseFloat();
    float a1 = Serial.parseFloat();
    float a0 = Serial.parseFloat();
    float t0 = Serial.parseFloat();
    float tf = Serial.parseFloat();
    Serial.read(); // clear rest of input buffer (i.e. trailing \n)
    // reply with read values
    Serial.print(truncateFloat(a3)); Serial.print(' ');
    Serial.print(truncateFloat(a2)); Serial.print(' ');
    Serial.print(truncateFloat(a1)); Serial.print(' ');
    Serial.print(truncateFloat(a0)); Serial.print(' ');
    Serial.print(truncateFloat(t0)); Serial.print(' ');
    Serial.print(truncateFloat(tf)); Serial.print(' ');
    Serial.println();
    // create Cubic struct and save to given array of polynomials
    struct Cubic cubic;
    cubic.coef[0] = a0;
    cubic.coef[1] = a1;
    cubic.coef[2] = a2;
    cubic.coef[3] = a3;
    cubic.t0 = t0 * 1000;
    cubic.tf = tf * 1000; // convert s to ms
    poly[count] = cubic;
    count++;
  }
}

String truncateFloat(float x) {
  if (abs(x) < 1) {
    return String(x, 6);
  } else if (abs(x) < 10) {
    return String(x, 5);
  } else if (abs(x) < 100) {
    return String(x, 4);
  } else if (abs(x) < 1000) {
    return String(x, 3);
  } else if (abs(x) < 10000) {
    return String(x, 2);
  } else {
    return String(x, 1);
  }
}

float cubicPoly(float t, float a0, float a1, float a2, float a3) {
  // evaluate the given cubic polynomial at time t
  return a3 * t * t * t + a2 * t * t + a1 * t + a0;
}

float quadPoly(float t, float a1, float a2, float a3) {
  return 3 * a3 * t * t + 2 * a2 * t + a1;
}

void sendNPoly(int n, struct Cubic cubic[MAX_CUBICS]) {
  // send the first n polynomial path segments
  for (int i=0; i<n; i++) {
    for (int j=0; j<PLOTTED_PATH_RES; j++) {
      float t = j/float(PLOTTED_PATH_RES)*(cubic[i].tf-cubic[i].t0)/1000.0 + cubic[i].t0/1000.0; // convert to real duration
      sendPolyAtTime(t, &cubic[i]);
    }
  }
}

void sendPolyAtTime(float t, struct Cubic *cubic) {
  // send ti and x(ti)
  float x = evaluate(cubic, t);
  Serial.print(t, 5); Serial.print(' ');
  Serial.print(x, 5); Serial.print(' ');
  Serial.println();
}

float cubicEvaluate(struct Cubic *cubic, float t) {
  // evaluate the given cubic at time t
  float a0 = cubic->coef[0];
  float a1 = cubic->coef[1];
  float a2 = cubic->coef[2];
  float a3 = cubic->coef[3];
  return cubicPoly(t, a0, a1, a2, a3);
}

float quadEvaluate(struct Cubic *cubic, float t) {
  // evaluate the given cubic at time t
  float a0 = cubic->coef[0];
  float a1 = cubic->coef[1];
  float a2 = cubic->coef[2];
  float a3 = cubic->coef[3];
  return quadPoly(t, a1, a2, a3);
}

float evaluate(struct Cubic *cubic, float t) {
  // evaluate the given cubic at time t
  float a0 = cubic->coef[0];
  float a1 = cubic->coef[1];
  float a2 = cubic->coef[2];
  float a3 = cubic->coef[3];
  return cubicPoly(t, a0, a1, a2, a3);
}
