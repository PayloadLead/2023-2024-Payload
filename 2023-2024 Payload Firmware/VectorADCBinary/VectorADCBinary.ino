// Include libraries
#include <SPI.h>
#include <SD.h>


// Define the number of cells
const uint8_t N = 10;
const uint16_t BUF_SIZE = 512;

const long INT_REF = 1200;

// change this biaaaaaaaatch
const long watchDawg = 1000000;

uint8_t adcPins[N] = { PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1 };

// Cell input data variables
uint16_t adcValues[N] = { 0 };

uint8_t ADCBuf[BUF_SIZE];  // Data buffer
int ADCBufPos = 0;  // Current position in the buffer
uint8_t accBuf[BUF_SIZE];
int accBufPos = 0;

// SD module SPI pins
#define miso PB4
#define mosi PB5
#define sck PB3
#define cs PA15

// Accelerometer pins
#define sda PB11
#define scl PB10

File ADCDataFile;
File accDataFile;
File refVDataFile;
SPISettings sdSettings(20000000, MSBFIRST, SPI_MODE0);

void sdCardSetup(){

  Serial.print("\nInitializing SD card...");
  if (!SD.begin(cs)) {
    Serial.println("Card failed, or not present, dang");
    while (1);
  }

  Serial.println("Card initialized OH YEAH!");
}


int measureVref(int samples) {
  // Take samples
  int result;
  long accum = 0;
  for (int i = 0; i < samples; i++) {
    result = analogRead(AVREF);
    accum += ((INT_REF * 4096L) / result); // Back-calculate VDDA in mV
  }//for
  return accum/samples; //Average
}//measureVref(int)
//thank Kennan Bays

void selfAdjustDelay(){ //From Kennan Bays
  int countdown = 5000;
  while (countdown > 0 && !Serial.available()) {
    countdown -= 100;
    delay(100);
    //digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
  }//while
  //digitalWrite(STATUS_LED_PIN, LOW);
  Serial.println("Serial Started!");
}

void setup() {
  for (int i = 0; i < N; i++) {
    pinMode(adcPins[i], INPUT);
  }

  pinMode(cs, OUTPUT);

  // Will need to get the correct number from Kennan
  Serial.begin(115200);
  // Wait until serial port is connected
  selfAdjustDelay();

  // Configure SPI
  SPI.setSCLK(sck);
  SPI.setMISO(miso);
  SPI.setMOSI(mosi);

  sdCardSetup();
  SPI.beginTransaction(sdSettings);

  analogReadResolution(12);
  int fileCounter = 0;
  String ADCFileName = String(String(fileCounter) + "ADC");
  String accFileName = String(String(fileCounter) + "acc");
  String refVFileName = String(String(fileCounter) + "refV");
  while(SD.exists(ADCFileName)){
    fileCounter++;
    ADCFileName = String(String(fileCounter) + "ADC");
    accFileName = String(String(fileCounter) + "acc");
    refVFileName = String(String(fileCounter) + "refV");
  }
  ADCDataFile = SD.open(ADCFileName, O_CREAT | O_WRITE);
  accDataFile = SD.open(accFileName, O_CREAT | O_WRITE);
  refVDataFile = SD.open(refVFileName, O_CREAT | O_WRITE);

  int vRef = measureVref(100); 
  refVDataFile.print(vRef);
  refVDataFile.close(); 
}

void loop() {
  
  uint32_t startTime = micros();
  unsigned long timestamp = micros();

  // Add timestamp to buffer
  ADCBuf[ADCBufPos++] = timestamp >> 24;  // Byte 3
  ADCBuf[ADCBufPos++] = timestamp >> 16;  // Byte 2
  ADCBuf[ADCBufPos++] = timestamp >> 8;   // Byte 1
  ADCBuf[ADCBufPos++] = timestamp & 0xFF; // Byte 0

  accBuf[accBufPos++] = timestamp >> 24;  // Byte 3
  accBuf[accBufPos++] = timestamp >> 16;  // Byte 2
  accBuf[accBufPos++] = timestamp >> 8;   // Byte 1
  accBuf[accBufPos++] = timestamp & 0xFF; // Byte 0


  // Read ADC values
  for (int i = 0; i < N; i++) {
    adcValues[i] = analogRead(adcPins[i]);
  }

  // Add ADC values to buffer
  for (int i = 0; i < N; i++) {
    ADCBuf[ADCBufPos++] = adcValues[i] >> 8;  // High byte
    ADCBuf[ADCBufPos++] = adcValues[i] & 0xFF;  // Low byte
  }

  accBuf[accBufPos++] = adcValues[0] >> 8;  // High byte
  accBuf[accBufPos++] = adcValues[0] & 0xFF;  // Low byte

  // If buffer is full, write data to SD card
  if (ADCBufPos >= 504) { //21 whole readings
    if (ADCDataFile) {
      ADCDataFile.write(ADCBuf, ADCBufPos);
      ADCDataFile.flush();
    }
    ADCBufPos = 0;  // Reset buffer position
  }

  if (accBufPos >= 510) { //51 whole readings
    if (accDataFile) {
      accDataFile.write(accBuf, accBufPos);
      accDataFile.flush();
    }
    accBufPos = 0;  // Reset buffer position
  }
  
  uint32_t endTime = micros();
  Serial.println(1000000/(endTime - startTime));


  if(millis() > watchDawg){
    ADCDataFile.close();
    accDataFile.close();
    NVIC_SystemReset();
  }
}
