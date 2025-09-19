
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>


// SD Card CS pin (Chip Select)
const int chipSelect = 5;


void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  // Initialize the SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization successful.");


  // Open the file on the SD card (create it if it doesn't exist)
  File dataFile = SD.open("/data.txt", FILE_WRITE);


  // Check if the file is available
  if (dataFile) {
    Serial.println("Writing to data.txt...");
    dataFile.println("Hello, this is ESP32 writing to SD card!");
    dataFile.close();  // Close the file after writing
    Serial.println("Write complete.");
    digitalWrite(2, HIGH);
  } else {
    Serial.println("Error opening data.txt");
  }
}


void loop() {
  // Do nothing here
}
