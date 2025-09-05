#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) delay(10);
  int result = myFunction(2, 3);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Hello world");
  delay(1000);
  

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}