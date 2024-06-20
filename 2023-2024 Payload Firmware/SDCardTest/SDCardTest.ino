// Basic demo for accelerometer readings from Adafruit MPU6050
#include <SPI.h>
#include <SD.h>

const uint16_t N = 6;
const uint16_t BUF_SIZE = 512;

// change this biaaaaaaaatch
const long watchDawg = 20000;

uint8_t SensBuf[BUF_SIZE];  // Data buffer
int SensBufPos = 0;  // Current position in the buffer
uint8_t eventBuf[BUF_SIZE];
int eventBufPos = 0;

uint16_t sensValues[N] = { 0 };

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>

//#define USB_TX_PIN PA12 // D- pin
//#define USB_RX_PIN PA11 // D+ pin
#define I2C_SDA_PIN PB7
#define I2C_SCL_PIN PB6
#define ACCEL_INT1 PB5
#define ACCEL_INT2 PB6
#define DB_LED_PIN PA15
#define miso PA6
#define mosi PA7
#define sck PA5
#define cs PA4

File sensDataFile;
File eventDataFile;
SPISettings sdSettings(20000000, MSBFIRST, SPI_MODE0);

void sdCardSetup(){

  Serial.print("\nInitializing SD card...");
  if (!SD.begin(cs)) {
    Serial.println("Card failed, or not present, dang");
    while (1);
  }
    Serial.println("Card initialized OH YEAH!");
}

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


Adafruit_MPU6050 mpu;

bool potentialLaunch = false;
bool highGTime = 0;
int G_THRESHOLD = -5;

void setup(void) {
  pinMode(cs, OUTPUT);
  Serial.begin(115200);
  delay(2000);

  // Wait until serial port is connected
  selfAdjustDelay();

  // Configure SPI
  SPI.setSCLK(sck);
  SPI.setMISO(miso);
  SPI.setMOSI(mosi);

  sdCardSetup();
  SPI.beginTransaction(sdSettings);

  analogReadResolution(12);

  //softSerial.println("Adafruit MPU6050 test!");
  Serial.print("Adafruit MPU6050 test!");

  // Try to initialize!
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  
  if (!mpu.begin(0x69)) {
    //softSerial.println("Failed to find MPU6050 chip");
	Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  //softSerial.println("MPU6050 Found!");
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //softSerial.println("");
  Serial.print("");
  delay(100);

  int fileCounter = 0;
  String sensFileName = String(String(fileCounter) + "sens");
  while(SD.exists(sensFileName)){
    fileCounter++;
    sensFileName = String(String(fileCounter) + "sens");
  }
  sensDataFile = SD.open(sensFileName, O_CREAT | O_WRITE); 
}


void loop() {


  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (!potentialLaunch && a.acceleration.z < G_THRESHOLD) {
    potentialLaunch = true;
    highGTime = millis();
  }

  if (potentialLaunch && millis() - highGTime > 50) {
    if (a.acceleration.z < G_THRESHOLD) {
      //softSerial.print("LAUNCH");
	  Serial.print("LAUNCH");
    } else {
      potentialLaunch = false;
    }
  }

  //Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");


  Serial.println("");
  Serial.println("");
  delay(10);

   uint32_t startTime = micros();
  unsigned long timestamp = micros();

  // Add timestamp to buffer
  SensBuf[SensBufPos++] = timestamp >> 24;  // Byte 3
  SensBuf[SensBufPos++] = timestamp >> 16;  // Byte 2
  SensBuf[SensBufPos++] = timestamp >> 8;   // Byte 1
  SensBuf[SensBufPos++] = timestamp & 0xFF; // Byte 0

  // Read sensor values
  sensValues[0] = a.acceleration.x;
  sensValues[1] = a.acceleration.y;
  sensValues[2] = a.acceleration.z;
  sensValues[3] = g.gyro.x;
  sensValues[4] = g.gyro.y;
  sensValues[5] = g.gyro.z;

// Add Sensor values to buffer
  for (int i = 0; i < N; i++) {
    SensBuf[SensBufPos++] = sensValues[i] >> 8;  // High byte
    SensBuf[SensBufPos++] = sensValues[i] & 0xFF;  // Low byte
  }

// If buffer is full, write data to SD card
  if (SensBufPos >= 504) { //21 whole readings
    if (sensDataFile) {
      sensDataFile.write(SensBuf, SensBufPos);
      sensDataFile.flush();
    }
    SensBufPos = 0;  // Reset buffer position
  }

  uint32_t endTime = micros();
  Serial.println(1000000/(endTime - startTime));

  if(millis() > watchDawg){
    sensDataFile.close();
    NVIC_SystemReset();
  }
}