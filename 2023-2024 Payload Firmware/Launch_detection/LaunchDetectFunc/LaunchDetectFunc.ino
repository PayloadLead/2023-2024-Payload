#include <Wire.h>
#include <Adafruit_MPU6050.h>

#define USB_TX_PIN PA12 // D- pin
#define USB_RX_PIN PA11 // D+ pin
#define I2C_SDA_PIN PB11
#define I2C_SCL_PIN PB10
#define ACCEL_INT1 PB5
#define ACCEL_INT2 PB6

//--- LAUNCH SETTINGS
#define LAUNCH_THRESHOLD  20  // [m/s]-mpu ; [g]-qma. Acceleration threshold to declare launch
#define LAUNCH_GRACE      50// [ms] Time for measurements to be above threshold before Launch declared


Adafruit_MPU6050 mpu6050;    //Declare mpu object
sensors_event_t a, g, temp;  //data object for mpu6050

//Pre-Launch
unsigned long potentialLaunchStart; //saves time of possible launch start
bool potentialLaunch;

void setup(){
Serial.begin(115200);
  
//Configure I2C PINS
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();

//setup mpu6050
Serial.print("SEARCHING: MPU6050");

    while(!mpu6050.begin(0x69)) {
        digitalWrite(D3,HIGH);
        delay(250);
        digitalWrite(D3,LOW);
        delay(250);  
    }
    Serial.println("  - CONNECTED");    //ms6050 connected

    mpu6050.setAccelerometerRange(MPU6050_RANGE_16_G); 
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);



}

void loop(){

Serial.print(detectLaunch());

}

bool detectLaunch(){
    //take mpu reading
    mpu6050.getEvent(&a, &g, &temp);

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

}
