//Lot of the multiplexer firmware was taken from this website: https://randomnerdtutorials.com/tca9548a-i2c-multiplexer-esp32-esp8266-arduino/
//For the NAU7802 library just go to Tools > Manage Libraries... > Search up library and install version 1.0.4

#include <Wire.h>
#define solenoidValvePin 1 // Set the pin of the solenoid valve
#include <Adafruit_NAU7802.h>

Adafruit_NAU7802 nau1;
Adafruit_NAU7802 nau2;
Adafruit_NAU7802 nau3;

// VOID SETUP
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(solenoidValvePin, OUTPUT); // Assigning the solenoid pin as output
  
  Wire.begin(); //start I2C communication with the Multiplexer
  // Initialize sensor on bus number 2
  TCA9548A(2);
  if (!nau1.begin(0x76)) {
    Serial.println("Could not find a valid NAU7802 sensor on bus 2, check wiring!");
    while (1);
  }
  Serial.println();
  
  // Initialize sensor on bus number 3
  TCA9548A(3);
  if (!nau2.begin(0x76)) {
    Serial.println("Could not find a valid NAU7802 sensor on bus 3, check wiring!");
    while (1);
  }
  Serial.println();
  
  // Initialize sensor on bus number 4
  TCA9548A(4);
  if (!nau3.begin(0x76)) {
    Serial.println("Could not find a valid NAU7802 ADC on bus 4, check wiring!");
    while (1);
  }
  Serial.println();
}


// VOID LOOP
void loop() {
  // put your main code here, to run repeatedly:
   //Print values for sensor 1
  printValuesCO(2);
  printValuesSO2(3);
  printValuesNO2(4);

  delay(100);
}


//**************************FUNCTIONS**************************

void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void printValuesCO(int bus) {
  TCA9548A (bus);
  Serial.print("Sensor number on bus");
  Serial.println(bus);
  Serial.print("Carbon Monoxide Concentration = ");
  while (! nau1.available()) {
    delay(1);
  }
  int32_t val = nau1.read();
  Serial.println(val);
  
  Serial.println();
}

void printValuesSO2(int bus) {
  TCA9548A (bus);
  Serial.print("Sensor number on bus");
  Serial.println(bus);
  Serial.print("Sulfate Concentration = ");
  while (! nau2.available()) {
    delay(1);
  }
  int32_t val = nau2.read();
  Serial.println(val);
  
  Serial.println();
}

void printValuesNO2(int bus) {
  TCA9548A (bus);
  Serial.print("Sensor number on bus");
  Serial.println(bus);
  Serial.print("Sulfate Concentration = ");
  while (! nau3.available()) {
    delay(1);
  }
  int32_t val = nau3.read();
  Serial.println(val);
  
  Serial.println();
  //can easily add more sensors if necessary, as well as conversion factors from bits to ppm and what not
}


bool solenoidValveState = false; // The state of the valve; false=>closed, true=>open (assuming starts closed)
void activateSolenoid(){
  solenoidValveState ? digitalWrite(solenoidValvePin, LOW) : digitalWrite(solenoidValvePin, HIGH); // Open/close the solenoid depending on the state
  solenoidValveState ? solenoidValveState = false : solenoidValveState = true; // Swap the states of the solenoid from either open=>close, or close=>open
}
