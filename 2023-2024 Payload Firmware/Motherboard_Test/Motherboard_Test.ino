/*
 * Author: Kennan
 * Created: May.1.2024
 * Hardware: QRET Motherboard (repurposed from OKA Altimeter (2023-2024 Altimeter PCB))
 * Purpose: Runs through some automated and some manual tests
 *          to verify the hardware functions of the payload Motherboard. Thrown together VERY quickly; not a good
 *          example of clean/proper code.
 *          
 *          Automatic Tests:
 *          - STM32F103 core speed
 *          - 3.3V regulator circuits
 *          - Battery-sense circuit
 *          - Accelerometer presense
 *          - Pressure sensor presense
 *          - Gyroscope sensor presense
 *          
 *          Manual Tests:
 *          - Buzzer
 *          - RGB LED function
 */


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h> //https://www.arduino.cc/reference/en/libraries/adafruit-neopixel/
#include <Adafruit_NAU7802.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h> //https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library/blob/master/examples/Example3_AdvancedWireSettings/Example3_AdvancedWireSettings.ino
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MS5xxx.h>

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

// MPU6050 launch events declarations
Adafruit_MPU6050 mpu;
bool potentialLaunch = false;
float highGTime = 0;
#define G_THRESHOLD -((9.80665F) * 1.5)

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

// TCA9548a
QWIICMUX myMux;

// Define a second I2C data line to communicate with sensorboard            
TwoWire Wire2(I2C_SWDIO_PIN,I2C_SWDIO_PIN); //https://github.com/stm32duino/Arduino_Core_STM32/wiki/API
                                            //https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Wire/Wire.h#L41




//**********MOTHERBOARD TEST FUNCTIONS**********//

unsigned long getSetMCUSpeed() {
  //Returns core frequency
  return SystemCoreClock;
}//getSetMCUSpeed()

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

bool autoTestAccelerometer() {
  // Returns true if accelerometer works
  // SURFACE-LEVEL TEST
  Wire.beginTransmission(EXPECTED_ACCEL_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.print(F("\tER: Accelerometer not at addr 0x"));
    Serial.println(EXPECTED_ACCEL_ADDR,HEX);
    return false;
  }
  return true;
}//autoTestAccelerometer()

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

/*
 * Performs a benchmark on the MCU, returning
 * the execution time. Lower is better.
 * 
 * Benchmark calculates Pi to appprox 4 decimals
 */
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

/*
 * Reads the battery voltage and
 * returns it in millivolts.
 */
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

// Airflow functions
void hold() {
  digitalWrite(SOL1_PIN, 0);
  digitalWrite(SOL2_PIN, 0);
  digitalWrite(SOL3_PIN, 0);
  digitalWrite(SOL4_PIN, 0);
  digitalWrite(PUMP_PIN, 0);
}

void open() {
  digitalWrite(SOL1_PIN, 1);
  digitalWrite(SOL2_PIN, 1);
  digitalWrite(SOL3_PIN, 1);
  digitalWrite(SOL4_PIN, 1);
  digitalWrite(PUMP_PIN, 0);
}


void purge() {
  digitalWrite(SOL1_PIN, 1);
  digitalWrite(SOL2_PIN, 1);
  digitalWrite(SOL3_PIN, 1);
  digitalWrite(SOL4_PIN, 1);
  digitalWrite(PUMP_PIN, 1);
}

void purgeCell1() {
  digitalWrite(SOL1_PIN, 1);
  digitalWrite(SOL2_PIN, 1);
  digitalWrite(SOL3_PIN, 0);
  digitalWrite(SOL4_PIN, 0);
  digitalWrite(PUMP_PIN, 1);
}

void purgeCell2() {
  digitalWrite(SOL1_PIN, 0);
  digitalWrite(SOL2_PIN, 0);
  digitalWrite(SOL3_PIN, 1);
  digitalWrite(SOL4_PIN, 1);
  digitalWrite(PUMP_PIN, 1);
}

void fillCell1() {
  digitalWrite(SOL1_PIN, 1);
  digitalWrite(SOL2_PIN, 0);
  digitalWrite(SOL3_PIN, 0);
  digitalWrite(SOL4_PIN, 0);
  digitalWrite(PUMP_PIN, 1);
  }

void fillCell2() {
  digitalWrite(SOL1_PIN, 0);
  digitalWrite(SOL2_PIN, 0);
  digitalWrite(SOL3_PIN, 1);
  digitalWrite(SOL4_PIN, 0);
  digitalWrite(PUMP_PIN, 1);
  }

void emptyCell1() {
  digitalWrite(SOL1_PIN, 0);
  digitalWrite(SOL2_PIN, 1);
  digitalWrite(SOL3_PIN, 0);
  digitalWrite(SOL4_PIN, 0);
  digitalWrite(PUMP_PIN, 0);
  }

void emptyCell2() {
  digitalWrite(SOL1_PIN, 0);
  digitalWrite(SOL2_PIN, 0);
  digitalWrite(SOL3_PIN, 0);
  digitalWrite(SOL4_PIN, 1);
  digitalWrite(PUMP_PIN, 0);
  }

void tryAirflow(){
  Serial.print("Purging");
  purge();
  delay(2000);
  Serial.print("Open");
  open();
  delay(2000);
  Serial.print("Purging Cell 1");
  purgeCell1();
  delay(2000);
  Serial.print("Purging Cell 2");
  purgeCell2();
  delay(2000);
  Serial.print("Filling Cell 1");
  fillCell1();
  delay(2000);
  Serial.print("Emptying Cell 1");
  emptyCell1();
  delay(2000);
  Serial.print("Filling Cell 2");
  fillCell2();
  delay(2000);
  Serial.print("Emptying Cell 2");
  emptyCell2();
  delay(2000);
  Serial.print("Holding");
  hold();
  delay(2000);
}

//**********SENSORBOARD TEST FUNCTIONS***********//


//Returns true if digital multiplexer works
  //SURFACE-LEVEL TEST
  // bool autoTestMultiplexer(){
  // Wire2.beginTransmission(EXPECTED_MULTI_ADDR);
  // byte error = Wire2.endTransmission();
  // if (error != 0) {
  //   Serial.print(F("\tER: Digital Multiplexer not at addr 0x"));
  //   Serial.println(EXPECTED_MULTI_ADDR,HEX);
  //   return false;
  // }//if()
  // return true;
  // }//autoTestMultiplexer()

   bool autoTestBME(){
  Wire.beginTransmission(EXPECTED_BME_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.print(F("\tER: BME680 not at addr 0x"));
    Serial.println(EXPECTED_BME_ADDR,HEX);
    return false;
  }//if()
  return true;
  }//autoTestBMEr()



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

  // MPU6050 begin test statement
  Serial.print("Adafruit MPU6050 test!");
  

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

  // Print details
  Serial.println(F("OKA Altimeter Integrity Test"));
  Serial.print(F("Uploaded "));
  Serial.print(__DATE__);
  Serial.print(F(" "));
  Serial.println(__TIME__);
  Serial.println();
  // Run automatic tests
  runAutoTests();
  // Prompt for manual tests
  Serial.println(F(" "));
  Serial.println(F("Press any key to begin manual testing"));  

  // MPU6050 initialization
  if (!mpu.begin(0x69)) {
    //softSerial.println("Failed to find MPU6050 chip");
	Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.print("");
  delay(100);

}//setup()

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


void runAutoTests() {
  int vReg = measureVref(100);
  int vSense = readBatteryVoltage(30);
  long mcuSpeed = getSetMCUSpeed();
  long mcuBenchmark = benchmarkMCU();
  bool mcuPass = (mcuSpeed == EXPECTED_MCU_FREQ) && checkInTolerance(EXPECTED_MCU_BENCH, mcuBenchmark, BENCHMARK_TOLERANCE);
  bool accelPass = autoTestAccelerometer();
  bool gyroPass = autoTestGyroscope();
  bool pressPass = autoTestPressureSensor();
  bool regPass = checkInTolerance(EXPECTED_VREG_MV, vReg, VOLT_TOLERANCE);
  bool vSensePass = checkInTolerance(EXPECTED_VSENSE_MV, vSense, VOLT_TOLERANCE);
  //bool multiPass = autoTestMultiplexer(); 
  bool bmePass = autoTestBME();

  Serial.println(F("\n\n"));
  Serial.println(F("      AUTOMATIC TEST RESULTS"));
  Serial.println(F("- - - - - - - - - - - - - - - - - -"));
  Serial.println(F("   TEST \t\tRESULT"));

  Serial.print(F("01. Voltage Reg: "));
  if (regPass) {  Serial.print(F("\tPass ("));
  } else {        Serial.print(F("\tFAIL ("));
  statusCode = 1; }//if
  Serial.print(vReg);
  Serial.println(F("mV)"));

  Serial.print(F("02. Batt Sense (7V): "));
  if (vSensePass) { Serial.print(F("\tPass ("));
  } else {          Serial.print(F("\tFAIL ("));
  statusCode = 2; }//if
  Serial.print(vSense);
  Serial.println(F("mV)"));
  
  Serial.print(F("04. STM32F103 Core: "));
  if (mcuPass) {  Serial.print(F("\tPass ("));
  } else {        Serial.print(F("\tFAIL ("));
  statusCode = 4; }//if
  Serial.print(mcuSpeed/1000000L);
  Serial.print(F("MHz, "));
  Serial.print(mcuBenchmark);
  Serial.println(F("ms)"));


  Serial.print(F("05. MPU6050 Gyro: "));
  if (gyroPass) { Serial.println(F("\tPass"));
  } else {         Serial.println(F("\tFAIL"));
  statusCode = 5; }//if

  Serial.print(F("06. QMA6100 Accel: "));
  if (accelPass) { Serial.println(F("\tPass"));
  } else {         Serial.println(F("\tFAIL"));
  statusCode = 6; }//if
  
  Serial.print(F("07. MS5607 Press: "));
  if (pressPass) { Serial.println(F("\tPass"));
  } else {         Serial.println(F("\tFAIL"));
  statusCode = 7; }//if

  // Serial.print(F("08. TCA9548a MuxL: "));
  // if (multiPass) { Serial.println(F("\tPass"));
  // } else {         Serial.println(F("\tFAIL"));
  // statusCode = 8; }//if

  Serial.print(F("08. BME680: "));
  if (bmePass) { Serial.println(F("\tPass"));
  } else {         Serial.println(F("\tFAIL"));
  statusCode = 8; }//if
  
}//runAutoTests()

void clearSerialBuffer() {
  // Clears contents of incoming serial buffer
  while (Serial.available()) {
    Serial.read();
    delay(1);
  }//while
}//clearSerialBuffer()

bool confirmPrompt() {
  // Blocks until a "Y" or "N" is received
  // over serial. Returns true if "Y", false
  // if "N"
  while (true) {
    if (Serial.available()) {
      char val = Serial.read();
      clearSerialBuffer();
      if (val == 'y' || val == 'Y') {
        return true;
      } else if (val == 'n' || val == 'N') {
        return false;
      }//if
    }//if
  }//while
}//confirmPrompt

void pressAnyKey() {
  while (!Serial.available()) { }
  clearSerialBuffer();
}//pressAnyKey

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

/*
 * Cycles through all the colours on the
 * given WS2812B LED
 */
void cycleTestLED(byte index) {
  rgbLEDs.setPixelColor(index, rgbLEDs.Color(255,0,0));
  rgbLEDs.show();
  Serial.print(F("\tLED: R"));
  delay(700);
  rgbLEDs.setPixelColor(index, rgbLEDs.Color(0,255,0));
  rgbLEDs.show();
  Serial.print(F("  G"));
  delay(700);
  rgbLEDs.setPixelColor(index, rgbLEDs.Color(0,0,255));
  rgbLEDs.show();
  Serial.print(F("  B"));
  delay(700);
  rgbLEDs.setPixelColor(index, rgbLEDs.Color(255,255,255));
  rgbLEDs.show();
  Serial.print(F("  W\n"));
  delay(700);
  rgbLEDs.clear();
  rgbLEDs.show();
}//cycleTestLED


void manualTestSuite() {

  // Test top RGB LED
  Serial.println(F("\nPress any key when ready to view top RGB LED"));
  pressAnyKey();
  cycleTestLED(0);
  cycleTestLED(1);
  Serial.println(F("[Y/N] Did top RGB LED cycle R/G/B?"));
  bool ledTopStatus = confirmPrompt();

  
  Serial.println(F("\nPress any key when ready to listen for buzzer"));
  pressAnyKey();
  byte loops = 5;
  buzzerSweep(loops);
  Serial.println(F("[Y/N] Did buzzer sound?"));
  bool buzzerStatus = confirmPrompt();

  Serial.println(F("\n\n"));
  Serial.println(F("      MANUAL TEST RESULTS"));
  Serial.println(F("- - - - - - - - - - - - - - - - -"));
  Serial.println(F("   TEST \t\tRESULT"));


  bool ledStatus = ledTopStatus;
  Serial.print(F("06. RGB LEDs: "));
  if (ledStatus) { Serial.print(F("\t\tPass ("));
  } else {         Serial.print(F("\t\tFAIL (")); }//if;
  Serial.print(ledTopStatus);
  Serial.print(F(", "));
 
  Serial.print(F("07. Buzzer: "));
  if (buzzerStatus) { Serial.println(F("\t\tPass"));
  } else {            Serial.println(F("\t\tFAIL")); }//if;

  Serial.println();

}//manualTestSuite

// Define pressure, temperature, altitude
float P_val,T_val,H_val;

void loop() {
  // Loop Code

  if (Serial.available()) {
    //Flush serial
    clearSerialBuffer();
    manualTestSuite();
    runAutoTests();
    tryAirflow();
  }//if

  if (statusCode == 0) {
    digitalWrite(DB_LED_PIN, HIGH);
    delay(50);
    digitalWrite(DB_LED_PIN, LOW);
    delay(50);
  } else {
    for (int i=0; i<statusCode; i++) {
      digitalWrite(DB_LED_PIN, HIGH);
      delay(200);
      digitalWrite(DB_LED_PIN, LOW);
      delay(200);
    }//for
    delay(1000);
  }//if


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
  
  
}//loop()
