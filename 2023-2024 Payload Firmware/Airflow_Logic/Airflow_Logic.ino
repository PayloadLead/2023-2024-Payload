
#define minAltitude 150; //m
#define minPressure 100; //Pa
#define maxFillTime 15*1000; // milliseconds

void setup() {
  // put your setup code here, to run once:

}


void hold() {
    digitalWrite(PA0, 0);
    digitalWrite(PA1, 0);
    digitalWrite(PA2, 0);
    digitalWrite(PA3, 0);
    digitalWrite(PA8, 0);
}

void open() {
    digitalWrite(PA0, 1);
    digitalWrite(PA1, 1);
    digitalWrite(PA2, 1);
    digitalWrite(PA3, 1);
    digitalWrite(PA8, 0);
}

void purge() {
    digitalWrite(PA0, 1);
    digitalWrite(PA1, 1);
    digitalWrite(PA2, 1);
    digitalWrite(PA3, 1);
    digitalWrite(PA8, 1);
}

void purgeCell1() {
    digitalWrite(PA0, 1);
    digitalWrite(PA1, 1);
    digitalWrite(PA2, 0);
    digitalWrite(PA3, 0);
    digitalWrite(PA8, 1);
}

void purgeCell2() {
    digitalWrite(PA0, 0);
    digitalWrite(PA1, 0);
    digitalWrite(PA2, 1);
    digitalWrite(PA3, 1);
    digitalWrite(PA8, 1);
}

void fillCell1() {
    digitalWrite(PA0, 1);
    digitalWrite(PA1, 0);
    digitalWrite(PA2, 0);
    digitalWrite(PA3, 0);
    digitalWrite(PA8, 1);
}

void fillCell2() {
    digitalWrite(PA0, 0);
    digitalWrite(PA1, 0);
    digitalWrite(PA2, 1);
    digitalWrite(PA3, 0);
    digitalWrite(PA8, 1);
}

void emptyCell1() {
    digitalWrite(PA0, 0);
    digitalWrite(PA1, 1);
    digitalWrite(PA2, 0);
    digitalWrite(PA3, 0);
    digitalWrite(PA8, 0);
}

void emptyCell2() {
    digitalWrite(PA0, 0);
    digitalWrite(PA1, 0);
    digitalWrite(PA2, 0);
    digitalWrite(PA3, 1);
    digitalWrite(PA8, 0);
}





void inFlightRead(){
  unsigned long fillStart;

  //while altitude > 500 ft (~150m? idk I'm too lazy )
  while (altimeter.readAltitude() > minAltitude){
    //Read and Store Cell 1 Values
    //**NEED TO FILL IN**
    
    //Empty Cell 1
    emptyCell1()
    //Purge Cell 1
    purgeCell1()

    //Fill Cell 1 - in while loop, checking pressure > min OR fill_time < max_allowed_fill_time
    fillStart = millis();
    while (bmp1.readPressure() < minPressure && fillStart - micros < maxFillTime) {
      fillCell1();
    }

    //Hold until 15 seconds have passed
    while (fillStart - millis() < 15*1000);

    //Read and Store Cell 2 Values
    //**NEED TO FILL IN**

    //Empty Cell 2
    emptyCell2()
    //Purge Cell 2
    purgeCell2()

    //Fill Cell 2 - in while loop, checking pressure > min OR fill_time < max_allowed_fill_time
    while (bmp2.readPressure() < minPressure && fillStart - micros < maxFillTime) {
      fillCell2();
    }

    //Hold until 15 seconds have passed
    while (fillStart - millis() < 15*1000);
  }
}


void loop() {
  // put your main code here, to run repeatedly:

}
