#include <SD.h>
#include <SPI.h>

// Pin assignments for SPI
#define SD_CS 15
#define SD_MISO 12
#define SD_MOSI 13
#define SD_SCK 14

#define BUTTON_1 18
#define BUTTON_2 19

#define VERBOSE 0

// File and read settings
char* filename = "/droplets-8k_16bit.wav";
const size_t CHUNK_SIZE = 1024 * 4;  // 4 KB

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // Wait until Serial is ready (on some boards)
  }
  delay(2000);

  pinMode(BUTTON_1, INPUT);
  pinMode(BUTTON_2, INPUT);

  // Initialize SPI with custom pins
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  // Initialize SD card
  if (!SD.begin(SD_CS, SPI, 40000000)) {
    Serial.println("SD initialization failed!");
    while (true) {
    }  // Stop here if SD can't initialize
  }
  Serial.println("SD initialization done.");

  // List files on the SD card
  Serial.println("Listing files in the root directory:");
  listFiles();
  Serial.println("Listing done.");
}

void read_sd_bench(char* filename) {
  // Open the file for reading
  File wavFile = SD.open(filename, FILE_READ);
  if (!wavFile) {
    Serial.print("Failed to open file: ");
    Serial.println(filename);
    while (true) {
    }  // Stop if file won't open
  }

  Serial.print("Reading from file: ");
  Serial.println(filename);

  unsigned long startBenchMicros = micros();
  unsigned long totalBytesRead = 0;

  // Read and time each 4 KB chunk
  uint8_t buffer[CHUNK_SIZE];
  while (true) {
    unsigned long startReadMicros = micros();
    int bytesRead = wavFile.read(buffer, CHUNK_SIZE);
    unsigned long endReadMicros = micros();
    totalBytesRead += bytesRead;

    unsigned long readDuration = endReadMicros - startReadMicros;
    if (VERBOSE) {
      Serial.print("Read ");
      Serial.print(bytesRead);
      Serial.print(" bytes in ");
      Serial.print(readDuration);
      Serial.println(" microseconds.");
    }

    if (bytesRead <= 0) {
      break;
    }

    // If we hit EOF before 1 KB, stop
    if (bytesRead < CHUNK_SIZE) {
      Serial.println("Reached end of file before reading full chunk.");
      break;
    }
  }

  unsigned long endBenchMicros = micros();
  unsigned long benchDuration = endBenchMicros - startBenchMicros;
  Serial.print("Read ");
  Serial.print(totalBytesRead);
  Serial.print(" bytes in ");
  Serial.print(benchDuration);
  Serial.print(" microseconds: ");
  Serial.print((totalBytesRead / 1024.0) / (benchDuration / 1000000.0));
  Serial.println(" mB/sec.");

  wavFile.close();
}

void listFiles() {
  // Open root directory
  File root = SD.open("/");
  if (!root) {
    Serial.println("Failed to open root directory!");
    return;
  }

  // List each file in root
  File entry = root.openNextFile();
  while (entry) {
    Serial.print("FILE: ");
    Serial.println(entry.path());
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
}

void loop() {
  if (digitalRead(BUTTON_1) == HIGH) {
    read_sd_bench(filename);
    delay(1000);
  }
}
