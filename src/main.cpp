#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

#define LSM9DS1_XGCS 0x6A 
#define LSM9DS1_MCS 0x1C

Adafruit_BMP280 bmp;
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

void setup() {
  Serial.begin(9600);
  
  while (!Serial) delay(10);     // Wait for serial console
  Serial.println(F("BMP280 and LSM9DS1 Test"));

  if (!bmp.begin()) {
    Serial.println(F("Could not find BMP280 sensor! Check wiring!"));
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

  // Set up the LSM9DS1 sensors
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
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

  Serial.println(F("-------------------"));
  delay(2000);
}