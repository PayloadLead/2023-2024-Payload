// Libraries
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h> //https://www.arduino.cc/reference/en/libraries/adafruit-neopixel/
#include <Adafruit_NAU7802.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h> //https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library/blob/master/examples/Example3_AdvancedWireSettings/Example3_AdvancedWireSettings.ino
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <MS5xxx.h>
#include <SD.h>

// Pinouts
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
#define miso PA6
#define mosi PA7
#define sck PA5
#define cs PA4

// Initialize data file
File sensDataFile;
File eventDataFile; 
SPISettings sdSettings(20000000, MSBFIRST, SPI_MODE0);

// Create SD card setup
void sdCardSetup(){

  Serial.print("\nInitializing SD card...");
  if (!SD.begin(cs)) {
    Serial.println("Card failed, or not present, dang");
    while (1);
  }
    Serial.println("Card initialized OH YEAH!");
}

//Create delay in data measurements
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

// BME680 ASL pressure declaration
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme;

// SD module constants
const uint16_t N = 9;
const uint16_t M = 2;
const uint16_t BUF_SIZE = 512;

// change this biaaaaaaaatch
const long watchDawg = 28800000;

uint8_t SensBuf[BUF_SIZE];  // Data buffer
uint8_t eventBuf[BUF_SIZE];
int SensBufPos = 0;  // Current position in the buffer
int eventBufPos = 0; // Current position in the buffer

// Initialize all sensor values as being equal to zero in SD card
uint16_t sensValues[N] = { 0 };
uint16_t eventValues[M] = { 0 };

// MPU6050 launch events declarations
#define G_THRESHOLD (9.80665F) * 1.5
#define LAUNCH_THRESHOLD  20  // [m/s]-mpu ; [g] mpu6050 Acceleration threshold to declare launch
#define LAUNCH_GRACE      50// [ms] Time for measurements to be above threshold before Launch declared
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;  //data object for mpu6050
unsigned long potentialLaunchStart; //saves time of possible launch start
bool potentialLaunch;

// MS5607 apogee event variables
double timeToApogee = 50000;

// MS5607 declaration
MS5xxx sensor(&Wire);

// SENSORS VALUES TO MEASURE:
float bmeTemp = 0;
float bmePress = 0;
float bmeHum = 0;
float bmeGas = 0;
float bmeAlt = 0;
float msTemp = 0;
float msPress = 0;
float msAlt = 0;
float mpuAccX = 0;
float mpuAccY = 0;
float mpuAccZ = 0;
float mpuGyroX = 0; 
float mpuGyroY = 0;
float mpuGyroZ = 0;

//EVENT VALUES TO MEASURE:
float eventTime = 0;
float launch = 0;

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

// Prog Vars
byte statusCode = 0; //how many times to flash LED
Adafruit_NeoPixel rgbLEDs(2, RGBA_DATA, NEO_GRB + NEO_KHZ800);

unsigned long getSetMCUSpeed() {
  //Returns core frequency
  return SystemCoreClock;
}//getSetMCUSpeed()


// Automatic test: MPU6050 gyroscope
bool autoTestGyroscope() {
  // Returns true if gyroscope works
  // SURFACE-LEVEL TEST
  Wire.beginTransmission(EXPECTED_GYRO_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.print(F("\tER: Gyroscope not at addr 0x"));
    Serial.println(EXPECTED_GYRO_ADDR,HEX);
    return false;
  }
  return true;
}//autoTestGyroscope()

// Automatic test: MS5607 pressure sensor
bool autoTestPressureSensor() {
  // Returns true if pressure sensor works
  // SURFACE-LEVEL TEST
  Wire.beginTransmission(EXPECTED_PRESS_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.print(F("\tER: Pressure sensor not at addr 0x"));
    Serial.println(EXPECTED_PRESS_ADDR,HEX);
    return false;
  }
  return true;
}//autoTestPressureSensor()\


// Benchmark test for MCU, returns execution time
uint32_t benchmarkMCU() {
  uint32_t startTime = millis();

  float pi = 0.0;
  float sign = 1.0;
  float denominator = 1.0;

  for (int i = 0; i < 50000; i++) {
    pi += sign / denominator;
    sign *= -1.0;
    denominator += 2.0;
  }

  pi *= 4.0;
// (need to use "pi" var so compiler
  // doesn't optimize the code out)
  return millis()-startTime* int(pi/pi);
}//benchmarkMCU()

int readBatteryVoltage(byte numSamples) {
  // Get Vref
  int vRef = measureVref(200);
  // Read ADC value
  int32_t aVal = 0;
  for (int i=0; i<numSamples; i++) {
    aVal += analogRead(BATT_SENSE_PIN);
  }//for
  aVal = aVal/numSamples;
  // Calculate voltage
  return int( vRef*aVal*BATT_SENSE_MULTIPLIER/4096 );
}//readBatteryVoltage()


/*
 * Reads internal voltage reference and back-calculates
 * the system Vdd (VDDA) using a calibrated Vref consant.
 * Returns the average across several samples (MAX ~32k
 * samples supported). Probably works with many STM32F1xx
 * chips.
 * 
 * Inspired by the STM32duino Internal_channels example
 */
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


// AIRFLOW FUNCTIONS
float hold1Fill2() {
	while (millis() < millis() + 15000){
	digitalWrite(SOL1_PIN, 0);
	digitalWrite(SOL2_PIN, 0);
	digitalWrite(SOL3_PIN, 1);
	digitalWrite(SOL4_PIN, 0);
	digitalWrite(PUMP_PIN, 1);
	} 
}

float hold1Hold2() {
	while (millis() < millis() + 15000){
	digitalWrite(SOL1_PIN, 0);
	digitalWrite(SOL2_PIN, 0);
	digitalWrite(SOL3_PIN, 0);
	digitalWrite(SOL4_PIN, 0);
	digitalWrite(PUMP_PIN, 0);
	}
}

float purge1Hold2() {
	while (millis() < millis() + 5000){
	digitalWrite(SOL1_PIN, 1);
	digitalWrite(SOL2_PIN, 1);
	digitalWrite(SOL3_PIN, 0);
	digitalWrite(SOL4_PIN, 0);
	digitalWrite(PUMP_PIN, 1);
	}
}

float fill1Hold2() {
	while (millis() < millis() + 10000){
	digitalWrite(SOL1_PIN, 1);
	digitalWrite(SOL2_PIN, 0);
	digitalWrite(SOL3_PIN, 0);
	digitalWrite(SOL4_PIN, 0);
	digitalWrite(PUMP_PIN, 1);
	}
}

float fill1Purge2() {
	while (millis() < millis() + 15000){
	digitalWrite(SOL1_PIN, 1);
	digitalWrite(SOL2_PIN, 0);
	digitalWrite(SOL3_PIN, 1);
	digitalWrite(SOL4_PIN, 1);
	digitalWrite(PUMP_PIN, 1);
	}
}

// Airflow function to use once apogee is detected
float airflow(){
  hold1Fill2();
  hold1Hold2();
  purge1Hold2();
  fill1Hold2();
  fill1Purge2();
}

 // Automatic test: BME680 voc sensor
 bool autoTestBME(){
   Wire.beginTransmission(EXPECTED_BME_ADDR);
   byte error = Wire.endTransmission();
   if (error != 0) {
     Serial.print(F("\tER: BME680 not at addr 0x"));
     Serial.println(EXPECTED_BME_ADDR,HEX);
     return false;
   }//if()
     return true;
 }// autoTestBMEr()


// Function to check if voltage is correct 
bool checkVoltPass(int expected, int measured) {
  //Returns true if "measured" is within a
  // tolerance around "expected".
  if (measured >= expected-VOLT_TOLERANCE) { // >= min
    if (measured <= expected+VOLT_TOLERANCE) { // <= max
      return true;
    }//if
  }//if
  return false;
}//checkVoltPass(int, int)


// Function to check if voltage is within tolerance
bool checkInTolerance(int expected, int measured, int tolerance) {
  //Returns true if "measured" is within a
  // tolerance around "expected".
  if (measured >= expected-tolerance) { // >= min
    if (measured <= expected+tolerance) { // <= max
      return true;
    }//if
  }//if
  return false;
}//checkInTolerance(int, int)

// Function to actually run the automatic test and return values
void runAutoTests(){
  int vReg = measureVref(100);
  int vSense = readBatteryVoltage(30);
  long mcuSpeed = getSetMCUSpeed();
  long mcuBenchmark = benchmarkMCU();
  bool mcuPass = (mcuSpeed == EXPECTED_MCU_FREQ) && checkInTolerance(EXPECTED_MCU_BENCH, mcuBenchmark, BENCHMARK_TOLERANCE);
  bool gyroPass = autoTestGyroscope();
  bool pressPass = autoTestPressureSensor();
  bool regPass = checkInTolerance(EXPECTED_VREG_MV, vReg, VOLT_TOLERANCE);
  bool vSensePass = checkInTolerance(EXPECTED_VSENSE_MV, vSense, VOLT_TOLERANCE);
  bool bmePass = autoTestBME();
}

  // Function to sound the buzzer when the payload is armed
  void buzzerSweep(byte loops) {
  // Sweeps from 500Hz to 1200Hz on buzzer
  for (int i=0; i<loops; i++) {
    for (int f=500; f<=1200; f+=10) {
      tone(BUZZER_PIN, f);
      delay(3);
    }//for
    noTone(BUZZER_PIN);
    delay(250);
  }//for
}//buzzerSweep()


// ************ SETUP **************
void setup(){
// Configure Pins
  pinMode(DB_LED_PIN, OUTPUT);
  pinMode(BATT_SENSE_PIN, INPUT_ANALOG);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(SOL1_PIN, OUTPUT);
  pinMode(SOL2_PIN, OUTPUT);
  pinMode(SOL3_PIN, OUTPUT);
  pinMode(SOL4_PIN, OUTPUT);
  pinMode(cs, OUTPUT);

  soundBuzzer();

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

  // Wait until serial port is connected
  selfAdjustDelay();

  // Configure SPI
  SPI.setSCLK(sck);
  SPI.setMISO(miso);
  SPI.setMOSI(mosi);

  sdCardSetup();
  SPI.beginTransaction(sdSettings);


  // Maximize ADC resolution
  analogReadResolution(12);
  // Configure I2C (Motherboard)
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  // Run automatic tests
  runAutoTests();

  //Setting range for MPU6050 measurements
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Checking connection to MPU6050
  while(!mpu.begin(0x69)) {
        digitalWrite(D3,HIGH);
        delay(250);
        digitalWrite(D3,LOW);
        delay(250);  
    }
  Serial.println("  - MPU6050 CONNECTED");    //ms6050 connected

  // Set range for MPU6050 gyroscope measurements
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Checking connection to MS5607
  if(sensor.connect()>0) {
    Serial.println("Error connecting...");
    delay(500);
    setup();
  }

  // Checking connection to BME680
   if (!bme.begin()) {
     Serial.println("Could not find a valid BME680 sensor, check wiring!");
     while (1);
   }

  // Set up oversampling and filter initialization for BME680
   bme.setTemperatureOversampling(BME680_OS_8X);
   bme.setHumidityOversampling(BME680_OS_2X);
   bme.setPressureOversampling(BME680_OS_4X);
   bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
   bme.setGasHeater(320, 150); // 320*C for 150 ms

  int fileCounter = 0;
  String sensFileName = String(String(fileCounter) + "sens");
  String eventFileName = String(String(fileCounter) + "event");
  while(SD.exists(sensFileName)){
    fileCounter++;
    sensFileName = String(String(fileCounter) + "sens");
    eventFileName = String(String(fileCounter) + "event");
  }
  sensDataFile = SD.open(sensFileName, O_CREAT | O_WRITE);
  eventDataFile = SD.open(eventFileName, O_CREAT | O_WRITE); 
}

// LAUNCH DETECTION ALGORITHM
bool detectLaunch(){
    //take mpu reading
    mpu.getEvent(&a, &g, &temp);

    //figure out if launch has been declared
    if (a.acceleration.z > LAUNCH_THRESHOLD){ //check if sensor reading is higher than 
      
      if(!potentialLaunch){
        //if first measurement, begin launch countdown
        potentialLaunch = 1;
        potentialLaunchStart = millis();
        
      }else{
        //if not first, check time since first
        if (millis() - potentialLaunchStart >= LAUNCH_GRACE){
          //THEN LAUNCH IS ON
          return 1;
        }//endif
      }//endif
      }else{
        potentialLaunch = 0;
        return 0;
     }//endif
     return 0;
}

// Sound buzzer during arming to confirm screwswitch worked
void soundBuzzer(){
  byte loops = 15;
  buzzerSweep(loops);
}



// **************** LOOP ****************

void loop(){
  detectLaunch();
  
  // Get data from BME680
  bme.performReading();
  bmeTemp = bme.temperature;
  bmePress = bme.pressure;
  bmeHum = bme.humidity;
  bmeGas = bme.gas_resistance;
  bmeAlt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  
  // Get data from MS5607
  msTemp = sensor.GetTemp();
  msPress = sensor.GetPres();
  msAlt = 4430 * (1- pow(msPress/101900, 1/5.255));
  
  // Get data from MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  mpuAccX = a.acceleration.x;
  mpuAccY = a.acceleration.y;
  mpuAccZ = a.acceleration.z;
  mpuGyroX = g.gyro.x; 
  mpuGyroY = g.gyro.y;
  mpuGyroZ = g.gyro.z;


  // Launch detection and event data
  if(detectLaunch != 0){
  delay(timeToApogee); //
  airflow();
  launch = detectLaunch();
  eventTime = millis();
  }
  

  uint32_t startTime = micros();
  unsigned long timestamp = micros();

  // Add timestamp to buffer
  SensBuf[SensBufPos++] = timestamp >> 24;  // Byte 3
  SensBuf[SensBufPos++] = timestamp >> 16;  // Byte 2
  SensBuf[SensBufPos++] = timestamp >> 8;   // Byte 1
  SensBuf[SensBufPos++] = timestamp & 0xFF; // Byte 0

  eventBuf[eventBufPos++] = timestamp >> 24;  // Byte 3
  eventBuf[eventBufPos++] = timestamp >> 16;  // Byte 2
  eventBuf[eventBufPos++] = timestamp >> 8;   // Byte 1
  eventBuf[eventBufPos++] = timestamp & 0xFF; // Byte 0

  // Read sensor values
  sensValues[0] = mpuAccX;
  sensValues[1] = mpuAccY;
  sensValues[2] = mpuAccZ;
  sensValues[3] = mpuGyroX;
  sensValues[4] = mpuGyroY;
  sensValues[5] = mpuGyroZ;
  sensValues[6] = msTemp;
  sensValues[7] = msPress;
  sensValues[8] = msAlt;

  eventValues[0] = eventTime;
  eventValues[1] = launch;

  // Add Sensor values and event values to buffer
  for (int i = 0; i < N; i++) {
    SensBuf[SensBufPos++] = sensValues[i] >> 8;  // High byte
    SensBuf[SensBufPos++] = sensValues[i] & 0xFF;  // Low byte
    eventBuf[eventBufPos++] = eventValues[i] >> 8;  // High byte
    eventBuf[eventBufPos++] = eventValues[i] & 0xFF;  // Low byte
  }

  // If buffer is full, write data to SD card
  if (SensBufPos >= 504) { //21 whole readings
    if (sensDataFile) {
      sensDataFile.write(SensBuf, SensBufPos);
      sensDataFile.flush();
    }
    SensBufPos = 0;  // Reset buffer position
  }

  // If buffer is full, write data to SD card
  if (eventBufPos >= 504) { //21 whole readings
    if (eventDataFile) {
      eventDataFile.write(eventBuf, eventBufPos);
      eventDataFile.flush();
    }
    eventBufPos = 0;  // Reset buffer position
  }
  uint32_t endTime = micros();
  //Serial.println(1000000/(endTime - startTime));

  if(millis() > watchDawg){
    sensDataFile.close();
    eventDataFile.close();
    digitalWrite(SOL1_PIN, 0);
	  digitalWrite(SOL2_PIN, 0);
	  digitalWrite(SOL3_PIN, 0);
	  digitalWrite(SOL4_PIN, 0);
	  digitalWrite(PUMP_PIN, 0);
    NVIC_SystemReset();
  }
}
