#include <Wire.h>
#include <Adafruit_HTS221.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>








const int chipSelect = 13;








Adafruit_HTS221 hts = Adafruit_HTS221();


 void initSDCard() {
 
  pinMode(chipSelect, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(chipSelect, HIGH);
 
  SPI.begin(18, 19, 23, chipSelect);


  delay(50);
 
  if (!SD.begin(chipSelect, SPI, 4000000)) {
    Serial.println("SD card initialization failed!");
    while (1) { delay(10); }
  }
  Serial.println("SD card initialized.");
}




void logDataToSD(float temperature, float humidity) {
  File dataFile = SD.open("/sensor_data.txt", FILE_APPEND);
 
  if (dataFile) {
    dataFile.print("Temperature: ");
    dataFile.print(temperature);
    dataFile.print(" C, Humidity: ");
    dataFile.print(humidity);
    dataFile.println(" %");
    dataFile.close();
    Serial.println("Data logged to SD card.");
    digitalWrite(2, HIGH);
  } else {
    Serial.println("Error opening file for writing.");
  }
}








void setup() {
  Serial.begin(115200);


  if (!hts.begin_I2C()) {
    Serial.println("Couldn't find HTS221 sensor.");
    while (1);  
  }
  Serial.println("HTS221 sensor initialized.");
 
  initSDCard();
}








void loop() {
  sensors_event_t humidityEvent, tempEvent;
  hts.getEvent(&humidityEvent, &tempEvent);  // fills both events


  float temperature = tempEvent.temperature;                 // °C
  float humidity    = humidityEvent.relative_humidity;       // %


  logDataToSD(temperature, humidity);
  delay(5000);
  digitalWrite(2, LOW);
}


