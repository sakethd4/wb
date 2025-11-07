#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_HTS221.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <LittleFS.h>

#define LSM9DS1_XGCS 0x6A 
#define LSM9DS1_MCS 0x1C

#define SD_CS    5    // Chip Select
#define SD_MOSI  23   // MOSI
#define SD_MISO  19   // MISO
#define SD_SCK   18   // SCK
#define LED_PIN  2

const int chipSelect = 5;
Adafruit_BMP280 bmp;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_HTS221 hts = Adafruit_HTS221();

void initSDCard() {
  Serial.println("\nInit SD card...");
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);        // deselect card
  delay(50);                        // allow card to power up

  // Initialize SPI bus with pins used by your board
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  delay(10);

  // Try multiple speeds (lower speeds increase compatibility)
  bool ok = false;
  const uint32_t speeds[] = {8000000, 4000000, 2000000, 1000000};
  for (uint8_t i = 0; i < sizeof(speeds)/sizeof(speeds[0]); ++i) {
    Serial.printf("SD.begin() try at %lu Hz...\n", (unsigned long)speeds[i]);
    if (SD.begin(SD_CS, SPI, speeds[i])) {
      ok = true;
      Serial.println("SD.begin() succeeded");
      break;
    }
    delay(50);
  }

  if (!ok) {
    Serial.println("SD card initialization FAILED. Possible causes:");
    Serial.println("- Incorrect wiring (CS/MOSI/MISO/SCK/VCC/GND)");
    Serial.println("- Card needs 3.3V (not 5V)");
    Serial.println("- Bad/unsupported SD card or not FAT32");
    Serial.println("- CS pin conflict with other devices");
    Serial.println("Continuing without SD logging");
    return;
  }

  uint8_t type = SD.cardType();
  if (type == CARD_NONE) {
    Serial.println("No SD card detected after init");
    return;
  }

  Serial.print("SD card type: ");
  if (type == CARD_MMC) Serial.println("MMC");
  else if (type == CARD_SD) Serial.println("SDSC");
  else if (type == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");

  Serial.printf("Card size: %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));

  // quick write test
  File f = SD.open("/test.txt", FILE_WRITE);
  if (f) {
    f.println("sd test");
    f.close();
    Serial.println("Test write OK");
  } else {
    Serial.println("Test write FAILED");
  }
}

void initFS()
{
  // Mount the LittleFS filesystem
  if (!LittleFS.begin(/*formatOnFail=*/true, "/littlefs", 10, "spiffs")) {
    Serial.println("LittleFS mount failed!");
    while(1);
  }
  Serial.println("LittleFS mounted successfully.");

  const char* filename = "/sensor_data.txt";

  // ---- Read existing file content ----
  if (LittleFS.exists(filename)) {
    Serial.println("Existing file found. Contents:");
    File file = LittleFS.open(filename, FILE_READ);
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
    Serial.println("\n--- End of file ---\n");
  } else {
    Serial.println("No existing file found. Creating a new one...");
  }
}

void setup() {
  Serial.begin(9600);
  
  while (!Serial) delay(10);     // Wait for serial console
  Serial.println(F("BMP280 and LSM9DS1 Test"));

  if (!bmp.begin()) {
    Serial.println(F("Could not find BMP280 sensor!"));
    while (1) delay(10);
  }
  Serial.println(F("BMP280 sensor found!"));
   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                   Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                   Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                   Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                   Adafruit_BMP280::STANDBY_MS_500);  /* Standby time. */

  // Initialize LSM9DS1
  if (!lsm.begin()) {
    Serial.println(F("Could not find LSM9DS1 sensor!"));
    while (1) delay(10);
  }
  Serial.println(F("LSM9DS1 sensor found!"));
  // Set up the LSM9DS1 sensors
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  if (!hts.begin_I2C()) {
    Serial.println(F("Couldn't find HTS221 sensor!"));
    while (1) delay(10);  
  }
  Serial.println(F("HTS221 sensor found!"));
 
  initSDCard();
  initFS();
}

void logDataToSD(float temperature, float humidity, float pressure, float altitude, 
                 float xAccel, float yAccel, float zAccel, 
                 float xGyro, float yGyro, float zGyro, 
                 float xMag, float yMag, float zMag) {

  File dataFile = SD.open("/sensor_data.txt", FILE_APPEND);
 
  if (dataFile) {
    dataFile.print("Temp: ");
    dataFile.print(temperature);
    dataFile.print(" C, Humidity: ");
    dataFile.print(humidity);
    dataFile.println("% ");
    dataFile.print("Pressure: ");
    dataFile.print(pressure);
    dataFile.print(" hPa, Altitude: ");
    dataFile.print(altitude);
    dataFile.print(" m, Accel X: ");
    dataFile.print(xAccel);
    dataFile.print(" m/s^2, Y: ");
    dataFile.print(yAccel);
    dataFile.print(" m/s^2, Z: ");
    dataFile.print(zAccel);
    dataFile.print(" m/s^2, Gyro X: ");
    dataFile.print(xGyro);
    dataFile.print(" rad/s, Y: ");
    dataFile.print(yGyro);
    dataFile.print(" rad/s, Z: ");
    dataFile.print(zGyro);
    dataFile.print(" rad/s, Mag X: ");
    dataFile.print(xMag);
    dataFile.print(" uT, Y: ");
    dataFile.print(yMag);
    dataFile.print(" uT, Z: ");
    dataFile.print(zMag);
    dataFile.print(" uT\n");
    dataFile.close();
    Serial.println("Data logged to SD card.");
    digitalWrite(2, HIGH);
  } else {
    Serial.println("Error opening SD file for writing.");
  }
}

void logDataToFS(float temperature, float humidity, float pressure, float altitude, 
                 float xAccel, float yAccel, float zAccel, 
                 float xGyro, float yGyro, float zGyro, 
                 float xMag, float yMag, float zMag) {

  File dataFile = LittleFS.open("/sensor_data.txt", FILE_APPEND);
 
  if (dataFile) {
    dataFile.print("Temp: ");
    dataFile.print(temperature);
    dataFile.print(" C, Humidity: ");
    dataFile.print(humidity);
    dataFile.println("% ");
    dataFile.print("Pressure: ");
    dataFile.print(pressure);
    dataFile.print(" hPa, Altitude: ");
    dataFile.print(altitude);
    dataFile.print(" m, Accel X: ");
    dataFile.print(xAccel);
    dataFile.print(" m/s^2, Y: ");
    dataFile.print(yAccel);
    dataFile.print(" m/s^2, Z: ");
    dataFile.print(zAccel);
    dataFile.print(" m/s^2, Gyro X: ");
    dataFile.print(xGyro);
    dataFile.print(" rad/s, Y: ");
    dataFile.print(yGyro);
    dataFile.print(" rad/s, Z: ");
    dataFile.print(zGyro);
    dataFile.print(" rad/s, Mag X: ");
    dataFile.print(xMag);
    dataFile.print(" uT, Y: ");
    dataFile.print(yMag);
    dataFile.print(" uT, Z: ");
    dataFile.print(zMag);
    dataFile.print(" uT\n");
    dataFile.close();
    Serial.println("Data logged to LittleFS storage.");
    digitalWrite(2, HIGH);
  } else {
    Serial.println("Error opening LittleFS file for writing.");
  }
}

void loop() {
  // Format BMP280 data
  Serial.println(F("BMP280"));
  Serial.print(F("Temperature  = "));
  Serial.print(bmp.readTemperature());
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure()/100);  //displaying the Pressure in hPa, you can change the unit
  Serial.println(F("  hPa"));

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1019.66));  //The "1019.66" is the pressure(hPa) at sea level in day in your region
  Serial.println(F("  m"));                    //If you don't know it, modify it until you get your current  altitude

  Serial.println();

   // Read LSM9DS1 sensors
  lsm.read();
  sensors_event_t accel, gyro, mag, temp;
  lsm.getEvent(&accel, &gyro, &mag, &temp);

  // Print accelerometer data
  Serial.println("LSM9DS1");
  Serial.print(F("Accel X: ")); Serial.print(accel.acceleration.x); Serial.print(F(" m/s^2 "));
  Serial.print(F("Y: ")); Serial.print(accel.acceleration.y); Serial.print(F(" m/s^2 "));
  Serial.print(F("Z: ")); Serial.print(accel.acceleration.z); Serial.println(F(" m/s^2"));

  // Print gyroscope data
  Serial.print(F("Gyro X: ")); Serial.print(gyro.gyro.x); Serial.print(F(" rad/s "));
  Serial.print(F("Y: ")); Serial.print(gyro.gyro.y); Serial.print(F(" rad/s "));
  Serial.print(F("Z: ")); Serial.print(gyro.gyro.z); Serial.println(F(" rad/s"));

  // Print magnetometer data
  Serial.print(F("Mag X: ")); Serial.print(mag.magnetic.x); Serial.print(F(" uT "));
  Serial.print(F("Y: ")); Serial.print(mag.magnetic.y); Serial.print(F(" uT "));
  Serial.print(F("Z: ")); Serial.print(mag.magnetic.z); Serial.println(F(" uT"));

  Serial.println();

  Serial.println("HTS221");

  sensors_event_t humidityEvent, tempEvent;
  hts.getEvent(&humidityEvent, &tempEvent);

  Serial.print(F("Temp: ")); Serial.print(tempEvent.temperature); Serial.println(F(" C"));
  Serial.print(F("Humidity: ")); Serial.print(humidityEvent.relative_humidity); Serial.println(F("%"));  

  Serial.println(F("-------------------"));

  logDataToSD(bmp.readTemperature(), humidityEvent.relative_humidity, bmp.readPressure()/100, bmp.readAltitude(1019.66), 
              accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
              gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
              mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  static unsigned long lastFSWrite = 0;
  if (millis() - lastFSWrite > 60000)
  {
    lastFSWrite = millis();
    logDataToFS(bmp.readTemperature(), humidityEvent.relative_humidity, bmp.readPressure()/100, bmp.readAltitude(1019.66), 
              accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
              gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
              mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
  }
  delay(500);
  digitalWrite(2, LOW);
}