void readData(struct Cubic *poly, File *file) {
  // "a3 a2 a1 a0 t0 tf" has been sent, parse it in, create a new Cubic struct
  // and add it to file provided, or arry if no SD card
  if (Serial.available() > 0)
  {
    float a3 = Serial.parseFloat();
    float a2 = Serial.parseFloat();
    float a1 = Serial.parseFloat();
    float a0 = Serial.parseFloat();
    float t0 = Serial.parseFloat();
    float tf = Serial.parseFloat();
    Serial.read(); // clear rest of input buffer (i.e. trailing \n
    String a3str = makeFloatString(a3);
    String a2str = makeFloatString(a2);
    String a1str = makeFloatString(a1);
    String a0str = makeFloatString(a0);
    String t0str = makeFloatString(t0);
    String tfstr = makeFloatString(tf);
    // reply with read values
    if (debugging) {
      String line = a3str+' '+a2str+' '+a1str+' '+a0str+' '+t0str+' '+tfstr;
      Serial.println(line);
    } else {
      Serial.println(count);
    }
    if (NO_SD_CARD) {
      // create Cubic struct and save to given array of polynomials
      struct Cubic cubic;
      cubic.coef[0] = a0;
      cubic.coef[1] = a1;
      cubic.coef[2] = a2;
      cubic.coef[3] = a3;
      cubic.t0 = t0*1000;
      cubic.tf = tf*1000;
      poly[count] = cubic;
    } else {
      t0 *= 1000; // convert s to ms
      tf *= 1000; // convert s to ms
      t0str = String((int)t0);
      tfstr = String((int)tf);
      String line = a3str+' '+a2str+' '+a1str+' '+a0str+' '+t0str+' '+tfstr;
      file->println(line);
      file->flush();
    }
    count++;
  }
}

String makeFloatString(float x) {
  // creates a string from x with 5 significant figures
  if (abs(x) < 1) {
    return String(x,6);
  } else if (abs(x) < 10) {
    return String(x,5);
  } else if (abs(x) < 100) {
    return String(x,4);
  } else if (abs(x) < 1000) {
    return String(x,3);
  } else if (abs(x) < 10000) {
    return String(x,2);
  } else {
    return String(x,1);
  }
}

File openFileOnSD(String fileName, byte mode) {
  int attempts = 0;
  File file = SD.open(fileName, mode);
  while (!file && attempts < 100) {
    file = SD.open(fileName, mode);
    delay(100);
    attempts++;
    if (attempts == 10) {
      Serial.println("Couldn't open file after 10 attempts over 1s");
    }
  }
  return file;
}

void sendFileForPlotting(String fileName) {
  File file = openFileOnSD(fileName, FILE_READ);
  file.seek(0);
  int i;
  for (i=0; i<nPolys; i++) {
    // read a line of the file
    float a3 = file.parseFloat();
    float a2 = file.parseFloat();
    float a1 = file.parseFloat();
    float a0 = file.parseFloat();
    float t0 = file.parseFloat();
    float tf = file.parseFloat();
//    float a3 = readFloat(&file);
//    float a2 = readFloat(&file);
//    float a1 = readFloat(&file);
//    float a0 = readFloat(&file);
//    float t0 = readFloat(&file);
//    float tf = readFloat(&file);
//    file.read(); // read and discard \n
    // create cubic struct
    struct Cubic cubic;
    cubic.coef[0] = a0;
    cubic.coef[1] = a1;
    cubic.coef[2] = a2;
    cubic.coef[3] = a3;
    cubic.t0 = t0;
    cubic.tf = tf;
//    // send the path this cubic generates
    sendAPoly(&cubic);
//      for (int j=0; j<PLOTTED_PATH_RES; j++) {
//        float t = j/(float)(PLOTTED_PATH_RES) * (tf-t0)/1000.0 + t0/1000;
//        float x = poly(t, a0, a1, a2, a3);
//        Serial.print(t, 5); Serial.print(' ');
//        Serial.print(x, 5); Serial.print(' ');
//        Serial.println();
//      }
//    String a3str = makeFloatString(a3);
//    String a2str = makeFloatString(a2);
//    String a1str = makeFloatString(a1);
//    String a0str = makeFloatString(a0);
//    String t0str = makeFloatString(t0);
//    String tfstr = makeFloatString(tf);
//    // reply with read values
//    String line = a3str+' '+a2str+' '+a1str+' '+a0str+' '+t0str+' '+tfstr;
//    Serial.println(line);
  }
}

void readFromFileNPolyAt(String fileName, struct Cubic *poly, int n, int *filePos) {
  File file = openFileOnSD(fileName, FILE_READ);
  file.seek(filePos);
  for (int i=0; i<n; i++) {
    // read a line of the file
    float a3 = file.parseFloat();
    float a2 = file.parseFloat();
    float a1 = file.parseFloat();
    float a0 = file.parseFloat();
    float t0 = file.parseFloat();
    float tf = file.parseFloat();
  //    file.read(); // read and discard \n
    // create cubic struct
    struct Cubic cubic;
    cubic.coef[0] = a0;
    cubic.coef[1] = a1;
    cubic.coef[2] = a2;
    cubic.coef[3] = a3;
    cubic.t0 = t0;
    cubic.tf = tf;
    poly[i] = cubic;
  }
  *filePos = file.position();
  file.close();
}

float readFloat(File *file) {
  // reads a character at a time, adding them to a string to be converted to a float
  // will stop at a \n, \r, space
  // note: if it immediately encounters one of these, it will exit with a value of 0.00
  String str = "";
  int ch = file->read();
//  Serial.print(ch); Serial.print(" "); Serial.println((char)ch);
  while (ch != '\n' && ch != '\r' && ch != ' ' && ch != -1 && ch < 255) {
    str += (char)ch;
    ch = file->read();
//    Serial.print(ch); Serial.print(" "); Serial.println((char)ch);
  }
  return str.toFloat();
}

void deleteAllPolysOnSD() {
  String files[] = {X_FILE, Y_FILE, Z_FILE, TH_FILE, GRIP_FILE};
  for (int i=0; i<5; i++) {
    while (SD.exists(files[i])) {
       SD.remove(files[i]);
       delay(50);
    }
  }
}

void sendNPoly(int n, struct Cubic cubic[MAX_CUBICS]) {
  // send the first n polynomial path segments
  for (int i=0; i<n; i++) {
    sendAPoly(&cubic[i]);
  }
}

void sendAPoly(struct Cubic *cubic) {
  for (int j=0; j<PLOTTED_PATH_RES; j++) {
    float t = j/float(PLOTTED_PATH_RES)*(cubic->tf - cubic->t0)/1000.0 + cubic->t0/1000.0; // convert to real duration
    sendPolyAtTime(t, cubic);
  }
}

void sendPolyAtTime(float t, struct Cubic *cubic) {
  // send ti and x(ti)
  float x = cubicEvaluate(cubic, t);
  Serial.print(makeFloatString(t)); Serial.print(' ');
  Serial.print(makeFloatString(x)); Serial.print(' ');
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

float cubicPoly(float t, float a0, float a1, float a2, float a3) {
  // evaluate the given cubic polynomial at time t
  return a3 * t * t * t + a2 * t * t + a1 * t + a0;
}

float quadPoly(float t, float a1, float a2, float a3) {
  return 3 * a3 * t * t + 2 * a2 * t + a1;
}
