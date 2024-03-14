#include <Wire.h>
#define solenoidValvePin 1 // Set the pin of the solenoid valve

int dPin = 8;   //digital pin on STM32 that will read data

// VOID SETUP
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(solenoidValvePin, OUTPUT); // Assigning the solenoid pin as output
  
  pinMode(dPin, INPUT);   //configuring dPin to act as input so that it can be read
  
  
  Wire.begin(); //start I2C communication with the Multiplexer
}


// VOID LOOP
void loop() {
  // put your main code here, to run repeatedly:
   //Print values for sensor 1
  printValuesCO(2, dPin);
  printValuesSO2(3, dPin);
  printValuesNO2(4, dPin);

  delay(1000);
}


//**************************FUNCTIONS**************************

void TCA9548A(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  Serial.print(bus);
}




void printValuesCO(int bus, int digitalPinName) {
  TCA9548A (bus);
  Serial.print("Sensor number on bus");
  Serial.println(bus);
  Serial.print("Carbon Monoxide Concentration = ");
  Serial.print(digitalRead(digitalPinName));
  Serial.println(" ppm");

  Serial.println();
}

void printValuesSO2(int bus, int digitalPinName) {
  TCA9548A (bus);
  Serial.print("Sensor number on bus");
  Serial.println(bus);
  Serial.print("Sulfate Concentration = ");
  Serial.print(digitalRead(digitalPinName));
  Serial.println(" ppm");

  Serial.println();
}

void printValuesNO2(int bus, int digitalPinName) {
  TCA9548A (bus);
  Serial.print("Sensor number on bus");
  Serial.println(bus);
  Serial.print("Nitrate Concentration = ");
  Serial.print(digitalRead(digitalPinName));
  Serial.println(" ppm");
  
  Serial.println();

  //can easily add more sensors if necessary, as well as conversion factors from bits to ppm and what not
}

bool solenoidValveState = false; // The state of the valve; false=>closed, true=>open (assuming starts closed)
void activateSolenoid(){
  solenoidValveState ? digitalWrite(solenoidValvePin, LOW) : digitalWrite(solenoidValvePin, HIGH); // Open/close the solenoid depending on the state
  solenoidValveState ? solenoidValveState = false : solenoidValveState = true; // Swap the states of the solenoid from either open=>close, or close=>open
}
