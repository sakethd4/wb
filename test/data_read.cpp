#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

// --- List directory helper ---
void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root || !root.isDirectory()) {
    Serial.println(" - Failed to open directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels > 0) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.print(file.size());
      Serial.println(" bytes");
    }
    file = root.openNextFile();
  }
}

// --- Print file contents (fixed) ---
void printFile(const char * path) {
  // Ensure the path starts with '/'
  String fullPath = path;
  if (!fullPath.startsWith("/")) {
    fullPath = "/" + fullPath;
  }

  Serial.printf("\nReading file: %s\n", fullPath.c_str());

  File file = LittleFS.open(fullPath, "r");
  if (!file) {
    Serial.println(" - Failed to open file");
    return;
  }

  while (file.available()) {
    Serial.write(file.read());
  }
  Serial.println("\n--- End of file ---");
  file.close();
}

void setup() {
  Serial.begin(9600);
  delay(2000);

  Serial.println("\nMounting LittleFS...");

  // SAFE: no auto-format
  if (!LittleFS.begin(false, "/littlefs", 10, "spiffs")) {
    Serial.println("LittleFS mount FAILED. Data may be corrupted.");
    while (1);
  }

  Serial.println("LittleFS mounted successfully!\n");

  // List directory tree
  listDir(LittleFS, "/", 3);

  Serial.println("\nReading files...");

  File root = LittleFS.open("/");
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      printFile(file.name());
    }
    file = root.openNextFile();
  }

  Serial.println("\nAll files read. You can disconnect power.");
}

void loop() {
  // nothing
}