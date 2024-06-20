/*example1-BasicReadings*/
#include <Wire.h>
#include <QMA6100P.h>

#define I2C_SDA_PIN PB11
#define I2C_SCL_PIN PB10
#define DB_LED_PIN PA15

bool buffer_enable = false;

QMA6100P qmaAccel;

outputData myData; // Struct for the accelerometer's data

void setup()
{
  Serial.begin(38400);  // POTENTIALLY CHANGE 
  delay(2000);
  Serial.println("serial start");

  // put your setup code here, to run once:
  pinMode(DB_LED_PIN, OUTPUT);

  // Configure I2C
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  if (!qmaAccel.begin())
  {
    Serial.println("ERROR: Could not communicate with the the QMA6100P. Freezing.");
    while (1)
      ;
  }

  if (!qmaAccel.softwareReset())
    Serial.println("ERROR: Failed to reset");
    
    delay(5);

  if(!qmaAccel.setRange(SFE_QMA6100P_RANGE32G)){      // 32g for the QMA6100P
    Serial.println("ERROR: failed to set range");
  }

  if(!qmaAccel.enableAccel()){
    Serial.println("ERROR: failed to set active mode");
  }   

  // if(!qmaAccel.calibrateOffsets()){
  //   softSerial.println("ERROR: calibration failed");
  // }
  qmaAccel.setOffset(1, 1, 1); // x, y, z

  Serial.println("Ready.");

}

void loop()
{

  // Check if data is ready.
  qmaAccel.getAccelData(&myData);
  qmaAccel.offsetValues(myData.xData, myData.yData, myData.zData);
  Serial.print("X: ");
  Serial.print(myData.xData, 2);
  Serial.print(" Y: ");
  Serial.print(myData.yData, 2);
  Serial.print(" Z: ");
  Serial.print(myData.zData, 2);
  Serial.println();
  

  delay(20); // Delay should be 1/ODR (Output Data Rate), default is 1/50ODR
}
