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

#define CHIP_SELECT 53

File myFile;

Sd2Card card;
SdVolume volume;
SdFile root;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

//  Serial.print("\nInitializing SD card...");
//
//  // we'll use the initialization code from the utility libraries
//  // since we're just testing if the card is working!
//  if (!card.init(SPI_HALF_SPEED, CHIP_SELECT)) {
//    Serial.println("initialization failed. Things to check:");
//    Serial.println("* is a card inserted?");
//    Serial.println("* is your wiring correct?");
//    Serial.println("* did you change the chipSelect pin to match your shield or module?");
//    return;
//  } else {
//    Serial.println("Wiring is correct and a card is present.");
//  }
//
//  // print the type of card
//  Serial.print("\nCard type: ");
//  switch (card.type()) {
//    case SD_CARD_TYPE_SD1:
//      Serial.println("SD1");
//      break;
//    case SD_CARD_TYPE_SD2:
//      Serial.println("SD2");
//      break;
//    case SD_CARD_TYPE_SDHC:
//      Serial.println("SDHC");
//      break;
//    default:
//      Serial.println("Unknown");
//  }
//
//  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
//  if (!volume.init(card)) {
//    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
//    return;
//  }
//
//  // print the type and size of the first FAT-type volume
//  uint32_t volumesize;
//  Serial.print("\nVolume type is FAT");
//  Serial.println(volume.fatType(), DEC);
//  Serial.println();
//
//  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
//  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
//  volumesize *= 512;                            // SD card blocks are always 512 bytes
//  Serial.print("Volume size (bytes): ");
//  Serial.println(volumesize);
//  Serial.print("Volume size (Kbytes): ");
//  volumesize /= 1024;
//  Serial.println(volumesize);
//  Serial.print("Volume size (Mbytes): ");
//  volumesize /= 1024;
//  Serial.println(volumesize);
//
//  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
//  root.openRoot(volume);
//
//  // list all files in the card with date and size
//  root.ls(LS_R | LS_DATE | LS_SIZE);

  Serial.print("Initializing SD card...");

  while (!SD.begin(CHIP_SELECT)) {
    Serial.println("initialization failed!");
    delay(100);
  }
  Serial.println("initialization done.");

  while (SD.exists("test2.txt")) {
     if (SD.remove("test2.txt")) {
      Serial.println("Deleted test.txt");
     } else {
      Serial.println("Couldn't delete test.txt");
     }
     delay(100);
  }
  
  Serial.println("test2.txt doesn't exist");

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
  String fileName = "x.txt";
  myFile = SD.open(fileName);
  Serial.println("Reading "+fileName);
  int lines = 1;
  if (myFile) {    
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.print("Line "); Serial.println(lines);
      Serial.println(myFile.parseFloat(),6);
      Serial.println(myFile.parseFloat(),6);
      Serial.println(myFile.parseFloat(),6);
      Serial.println(myFile.parseFloat(),6);
      Serial.println(myFile.parseFloat(),6);
      Serial.println(myFile.parseFloat(),6);
//      Serial.println(readFloat(&myFile), 6);
//      Serial.print("Available: "); Serial.println(myFile.available());
      myFile.read(); // read and discard newline
      lines++;
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
  // reads a character at a time, adding them to a string to be converted to a float
  // will stop at a \n, \r, space
  // note: if it immediately encounters one of these, it will exit with a value of 0.00
  String str = "";
  int ch = file->read();
//  Serial.print(ch); Serial.print(" "); Serial.println((char)ch);
  while (ch != '\n' && ch != '\r' && ch != ' ' && ch != -1 && ch < 255) {
    str += (char)ch;
    ch = file->read();
    Serial.print(ch); Serial.print(" "); Serial.println((char)ch);
  }
  return str.toFloat();
}
