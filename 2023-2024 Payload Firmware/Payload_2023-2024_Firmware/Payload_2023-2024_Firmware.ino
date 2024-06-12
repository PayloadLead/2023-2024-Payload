#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h> //https://www.arduino.cc/reference/en/libraries/adafruit-neopixel/
#include <Adafruit_NAU7802.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h> //https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library/blob/master/examples/Example3_AdvancedWireSettings/Example3_AdvancedWireSettings.ino
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define DB_LED_PIN PA15
#define BUZZER_PIN PA9
#define BATT_SENSE_PIN PB0
#define RGBA_DATA PB3
#define ACCEL_INT1 PB13
#define ACCEL_INT2 PB14
#define GYRO_INT PB15
#define I2C_SDA_PIN PB7
#define I2C_SCL_PIN PB6
#define I2C_SWCLK_PIN PA14
#define I2C_SWDIO_PIN PA13
#define PUMP_PIN PA8
#define SOL1_PIN PA0
#define SOL2_PIN PA1
#define SOL3_PIN PA2
#define SOL4_PIN PA3

// Battery sense voltage divider ratio (default 4.00x)
const float BATT_SENSE_MULTIPLIER = 4.00;
// Internal Vref voltage (default 1200mV)
const long INT_REF = 1216;

// Expected Test Values //
#define EXPECTED_MCU_BENCH 324 // 340ms for 72MHz
#define EXPECTED_MCU_FREQ 72000000L
#define EXPECTED_VREG_MV 3300
#define EXPECTED_VSENSE_MV 7000
#define VOLT_TOLERANCE 100 //voltage reading tolerance (mV)
#define BENCHMARK_TOLERANCE 30 //CPU benchmark tolerance (ms)
#define EXPECTED_ACCEL_ADDR 0x13 //0x1F
#define EXPECTED_GYRO_ADDR 0x69
#define EXPECTED_PRESS_ADDR 0x76
#define EXPECTED_MULTI_ADDR 0x70
#define EXPECTED_BME_ADDR 0x77

// SD Module SPI Pins
#define MISO PA6
#define MOSI PA7
#define SCK PA5
#define CS PA4

// Prog Vars
byte statusCode = 0; //how many times to flash LED
Adafruit_NeoPixel rgbLEDs(2, RGBA_DATA, NEO_GRB + NEO_KHZ800);

// TCA9548a
QWIICMUX myMux;

// Define a second I2C data line to communicate with sensorboard            
TwoWire Wire2(I2C_SWDIO_PIN,I2C_SWDIO_PIN); //https://github.com/stm32duino/Arduino_Core_STM32/wiki/API
                                            //https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Wire/Wire.h#L41

// SD Module stuff
File ADCDataFile;
File accDataFile;
File refVDataFile;
SPISettings sdSettings(20000000, MSBFIRST, SPI_MODE0);

// Setup SD Card module
void sdCardSetup(){

  Serial.print("\nInitializing SD card...");
  //Check if SD card present
  if (!SD.begin(CS)) {
    Serial.println("Card failed, or not present, dang");
    while (1);
  }

  Serial.println("Card initialized OH YEAH!");
}


// ********************* SETUP **********************
void setup() {
  // Configure Pins
  pinMode(DB_LED_PIN, OUTPUT);
  pinMode(BATT_SENSE_PIN, INPUT_ANALOG);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(SOL1_PIN, OUTPUT);
  pinMode(SOL2_PIN, OUTPUT);
  pinMode(SOL3_PIN, OUTPUT);
  pinMode(SOL4_PIN, OUTPUT);

  // Add WS2812B
  rgbLEDs.begin(); // initialize WS2812Bs

  // Begin Serial
  Serial.begin(9600); //Baud doesn't matter for STM32 USB


  int countdown = 5000;
  while (countdown > 0 && !Serial.available()) {
    countdown -= 100;
    delay(100);
    digitalWrite(DB_LED_PIN, !digitalRead(DB_LED_PIN));
  }//while
  digitalWrite(DB_LED_PIN, LOW);
  
  // Maximize ADC resolution
  analogReadResolution(12);

  // Configure I2C (Motherboard)
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

// Configure I2C (Sensorboard)
  Wire2.begin();

  pinMode(cs, OUTPUT);

  // Configure SPI
  SPI.setSCLK(SCK);
  SPI.setMISO(MISO);
  SPI.setMOSI(MOSI);

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

// void loop() {
  
//   uint32_t startTime = micros();
//   unsigned long timestamp = micros();

//   // Add timestamp to buffer
//   ADCBuf[ADCBufPos++] = timestamp >> 24;  // Byte 3
//   ADCBuf[ADCBufPos++] = timestamp >> 16;  // Byte 2
//   ADCBuf[ADCBufPos++] = timestamp >> 8;   // Byte 1
//   ADCBuf[ADCBufPos++] = timestamp & 0xFF; // Byte 0

//   accBuf[accBufPos++] = timestamp >> 24;  // Byte 3
//   accBuf[accBufPos++] = timestamp >> 16;  // Byte 2
//   accBuf[accBufPos++] = timestamp >> 8;   // Byte 1
//   accBuf[accBufPos++] = timestamp & 0xFF; // Byte 0


//   // Read ADC values
//   for (int i = 0; i < N; i++) {
//     adcValues[i] = analogRead(adcPins[i]);
//   }

//   // Add ADC values to buffer
//   for (int i = 0; i < N; i++) {
//     ADCBuf[ADCBufPos++] = adcValues[i] >> 8;  // High byte
//     ADCBuf[ADCBufPos++] = adcValues[i] & 0xFF;  // Low byte
//   }

//   accBuf[accBufPos++] = adcValues[0] >> 8;  // High byte
//   accBuf[accBufPos++] = adcValues[0] & 0xFF;  // Low byte

//   // If buffer is full, write data to SD card
//   if (ADCBufPos >= 504) { //21 whole readings
//     if (ADCDataFile) {
//       ADCDataFile.write(ADCBuf, ADCBufPos);
//       ADCDataFile.flush();
//     }
//     ADCBufPos = 0;  // Reset buffer position
//   }

//   if (accBufPos >= 510) { //51 whole readings
//     if (accDataFile) {
//       accDataFile.write(accBuf, accBufPos);
//       accDataFile.flush();
//     }
//     accBufPos = 0;  // Reset buffer position
//   }
  
//   uint32_t endTime = micros();
//   Serial.println(1000000/(endTime - startTime));


//   if(millis() > watchDawg){
//     ADCDataFile.close();
//     accDataFile.close();
//     NVIC_SystemReset();
//   }
// }

  
