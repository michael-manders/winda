// #include <Arduino.h>
// #include <AccelStepper.h>
// #include "ScioSense_ENS160.h"  
// #include <DHT.h>


// // MOTOR
// const int dirPin = 2;
// const int stepPin = 4;
// #define motorInterfaceType 1
// AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

// //VOC SENSOR
// ScioSense_ENS160      ens160(ENS160_I2CADDR_1);

// // TEMP AND HUMIDITY SENSOR
// #define DHTPIN 5
// #define DHTTYPE DHT11
// DHT dht(DHTPIN, DHTTYPE);

// void setup() {
//   Serial.begin(9600);

// 	// MOTOR
//   Serial.println("Stepper init");
// 	myStepper.setMaxSpeed(2000);
// 	myStepper.setAcceleration(200);
// 	myStepper.setSpeed(200);
// 	myStepper.moveTo(500);
//   Serial.println(" done");

//   // VOC SENSOR
//   Serial.println("VOC init");
//   ens160.begin();
//   Serial.println(ens160.available() ? "done." : "failed!");
//   if (ens160.available()) {
//     // Print ENS160 versions
//     Serial.print("\tRev: "); Serial.print(ens160.getMajorRev());
//     Serial.print("."); Serial.print(ens160.getMinorRev());
//     Serial.print("."); Serial.println(ens160.getBuild());
  
//     Serial.print("\tStandard mode ");
//     Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "done." : "failed!");
//   }


//   // TEMP AND HUMIDITY SENSOR
//   dht.begin();
// }

// void loop() {
// 	// Change direction once the motor reaches target position
// 	// if (myStepper.distanceToGo() == 0) 
// 	// 	myStepper.moveTo(-myStepper.currentPosition());

// 	// while (myStepper.distanceToGo() != 0) {
//   //   myStepper.run();  
//   // }

//   if (ens160.available()) {
//     ens160.measure(true);
//     ens160.measureRaw(true);
  
//     Serial.print("AQI: ");Serial.print(ens160.getAQI());Serial.print("\t");
//     Serial.print("TVOC: ");Serial.print(ens160.getTVOC());Serial.print("ppb\t");
//     Serial.print("eCO2: ");Serial.print(ens160.geteCO2());Serial.print("ppm\t");
//     Serial.println();
//   }

//   Serial.print("Temperature: ");
//   Serial.print(dht.readTemperature(true));
//   Serial.print("°F ");
//   Serial.print("Humidity: ");
//   Serial.print(dht.readHumidity());
//   Serial.print("%");
//   Serial.println();
  

  

// }


#include <Arduino.h>
#include <AccelStepper.h>
#include "ScioSense_ENS160.h"  
#include <Adafruit_AHTX0.h>
#include <DHT.h>


// MOTOR
const int dirPin = 15;
const int stepPin = 2;
#define motorInterfaceType 1
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

//VOC SENSOR
ScioSense_ENS160      ens160(ENS160_I2CADDR_1);

// TEMP 2 SENSOR
int tempC, tempF, humidity;
Adafruit_AHTX0 aht;

// TEMP AND HUMIDITY SENSOR
// #define DHTPIN 5
// #define DHTTYPE DHT11
// DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);

	// MOTOR
  Serial.println("Stepper init");
	myStepper.setMaxSpeed(2000);
	myStepper.setAcceleration(200);
	myStepper.setSpeed(200);
	myStepper.moveTo(500);
  Serial.println(" done");

  // VOC SENSOR
  Serial.println("VOC init");
  ens160.begin();
  Serial.println(ens160.available() ? "done." : "failed!");
  if (ens160.available()) {
    // Print ENS160 versions
    Serial.print("\tRev: "); Serial.print(ens160.getMajorRev());
    Serial.print("."); Serial.print(ens160.getMinorRev());
    Serial.print("."); Serial.println(ens160.getBuild());
  
    Serial.print("\tStandard mode ");
    Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "done." : "failed!");
  }


  
if (! aht.begin()) {

Serial.println("Could not find AHT? Check wiring");

while (1) delay(10);

}

  // TEMP AND HUMIDITY SENSOR
  // dht.begin();
}

void loop() {
	// Change direction once the motor reaches target position
	if (myStepper.distanceToGo() == 0) 
		myStepper.moveTo(-myStepper.currentPosition());

	while (myStepper.distanceToGo() != 0) {
    myStepper.run();  
  }

  if (ens160.available()) {
    ens160.measure(true);
    ens160.measureRaw(true);
  
    Serial.print("AQI: ");Serial.print(ens160.getAQI());Serial.print("\t");
    Serial.print("TVOC: ");Serial.print(ens160.getTVOC());Serial.print("ppb\t");
    Serial.print("eCO2: ");Serial.print(ens160.geteCO2());Serial.print("ppm\t");
    Serial.println();
  }

  // Serial.print("Temperature: ");
  // Serial.print(dht.readTemperature(true));
  // Serial.print("°F ");
  // Serial.print("Humidity: ");
  // Serial.print(dht.readHumidity());
  // Serial.print("%");
  // Serial.println();
  

  sensors_event_t humidity1, temp; //Tim had to change to humidity1

aht.getEvent(&humidity1, &temp);// populate temp and humidity objects with fresh data

tempC = (temp.temperature);

tempF = (temp.temperature)*1.8+32;

humidity = (humidity1.relative_humidity);

Serial.print("Temperature: ");

Serial.print(tempC);

Serial.println(" degrees C");

Serial.print("Temperature: ");

Serial.print(tempF);

Serial.println(" degrees F");

Serial.print("Humidity: ");

Serial.print(humidity);

Serial.println("% rH");




}
