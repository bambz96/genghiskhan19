/*
 * todo
 * delete/clear file doesn't seem to work correctly
 * why garbage at front of test2.txt when reading?
 * 
 * for robot
 * write coeffs to single or multiple files
 *    x01 y01 z01 th01 gr01
 *    x02 y02 z02 th02 gr02
 *    etc
 * read speed?
 * read 30 into memory at a time
 * 
 * https://www.arduino.cc/en/Reference/SD
 * 
 */

#include <SPI.h>
#include <SD.h>

File myFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println((char)13);

  Serial.print("Initializing SD card...");

  while (!SD.begin(53)) {
    Serial.println("initialization failed!");
    delay(100);
  }
  Serial.println("initialization done.");


  while (SD.exists("test.txt")) {
    delay(100);
     if (SD.remove("test.txt")) {
      Serial.println("Deleted test.txt");
     } else {
      Serial.println("Couldn't delete test.txt");
     }
  }

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File myFile;
  int attempts = 0;
  while (!myFile && attempts < 100) {
    myFile = SD.open("test2.txt", FILE_WRITE);
    delay(100);
    attempts++;
    if (attempts == 100) {
      Serial.println("Couldn't open file after 100 attempts over 10s");
    }
  }
  

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Read in "); Serial.print(attempts); Serial.println(" attempts.");
    
    /* Write 10k long lines, close after
     * Results:
     *  1588 to println 10k lines (repeated 3 times).
     *  0ms to close (ensures physically saved). Not sure if it simply returns immediately.
     */
     /*
    Serial.println("Write 10k long lines, close after.");
    int t0 = millis();
    int i = 0;
    while (i < 10000) {
      myFile.println("012345678901234567890123456789");
      i++;
    }
    Serial.print(millis()-t0); Serial.println("ms");
    t0 = millis();
    myFile.close(); // saves to SD
    Serial.print(millis()-t0); Serial.println("ms");
    */

    /* Write 10k long lines and flush after each.
     * Results:
     *  1618ms to print and flush 10k lines (repeated 3 times)
     *  0ms to close.
     */
     /*
    Serial.println("Write 10k long lines and flush after each.");
    int t0 = millis();
    int i = 0;
    while (i < 10000) {
      myFile.println("012345678901234567890123456789");
      myFile.flush();
      i++;
    }
    Serial.print(millis()-t0); Serial.println("ms");
    t0 = millis();
    myFile.close(); // saves to SD
    Serial.print(millis()-t0); Serial.println("ms");
    */

    // test data fro parsing floats
    myFile.println("0.1234 -12.97123 00.13746 0.20000 1.0000");
    myFile.println("0.1234 -12.97123 00.13746 0.20000 1.0000");
    myFile.close();

  } else {
    // if the file didn't open, print an error:
    Serial.println("Error opening test.txt");
  }

  // re-open the file for reading:
  myFile = SD.open("test2.txt");
  Serial.println("Reading test2.txt:");
  if (myFile) {    
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
//      Serial.println(myFile.parseFloat());
      Serial.println(readFloat(&myFile), 6);
      Serial.print("Available: "); Serial.println(myFile.available());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("Error opening test.txt");
  }
}

void loop() {
  // nothing happens after setup
}

float readFloat(File *file) {
  String str = "";
  int ch = file->read();
  Serial.print(ch); Serial.print(" "); Serial.println((char)ch);
  while (ch != '\n' && ch != '\r' && ch != ' ' && ch != -1 && ch < 255) {
    str += (char)ch;
    ch = file->read();
    Serial.print(ch); Serial.print(" "); Serial.println((char)ch);
  }
  return str.toFloat();
}
