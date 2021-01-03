/*
 * Code für SensorCube
 *
 *
 *
 *
 *
 *	V02:	Erweiterung um den Sensor MPU9250
 *
 *	V01:	Initiale Version, die die neue ESP32 Library von Phyphox verwendet und 2 Werte sendet  
 *			- separate Datenstruktur eingeführt
 *			
 */


#include <phyphoxBle.h>

// next lines are required for MPU9250 9-axes sensor
#include <MPU9250_asukiaaa.h>

#define SDA_PIN 21
#define SCL_PIN 22
 


// next lines are required for VL53L1X distance sensor
#include <Wire.h>

//#include "SparkFun_VL53L1X.h"
#include "DFRobot_VL53L1X.h" 


DFRobot_VL53L1X distanceSensor(&Wire);
MPU9250_asukiaaa accSensor;

const byte LaserReadInterval = 100;	// time in [ms] between each value is added to notification payload

enum dataSetTypes {	VARIOUS1, 	// 1
					ACC,		// 2
					GYRO,		// 3		
					MAG			// 4
				};
float	dataSet;

/*
Allowed values for timing budget with this library:
  eBudget_20ms = 20,
  eBudget_33ms = 33,
  eBudget_50ms = 50,
  eBudget_100ms = 100,
  eBudget_200ms = 200,
  eBudget_500ms = 500
*/
eTimingBudget lsTimingBudget = eBudget_33ms ;

// all sensor values - everything has to be a float
	float			accelX;			// Acceleration x
	float			accelY;			// Acceleration y
	float			accelZ;			// Acceleration z
	float			gyroX;			// Rotation of x axis
	float			gyroY;			// Rotation of y axis
	float			gyroZ;			// Rotation of z axis
	float			magX;			// Mag x axis
	float			magY;			// Mag y axis
	float			magZ;			// Mag z axis
	float			distance;		// distance
	float			temperature;	// temperature
	float			pressure;		// pressure

unsigned long timestamp;


//
const int	numDataValues = 5;
float measuredData[numDataValues];

void setup() {
	
	// for esp32
	Wire.begin(SDA_PIN, SCL_PIN); //sda, scl
		
	// initalize MPU9250
	accSensor.setWire(&Wire);

	accSensor.beginAccel();		// start acceleration measuring
	accSensor.beginGyro();
	accSensor.beginMag();
	
	
	
	//  PhyphoxBLE::start("CO2 Monitor", &CO2Monitor[0], sizeof(CO2Monitor));                 //Start the BLE server
	PhyphoxBLE::start("SensorCube");                 //Start the BLE server
	Serial.begin(115200);
		
	Wire.begin();			// initalize interface
	delay (1000);
	
	while (distanceSensor.begin() != true){//sensor initialization
	Serial.println("LaserSensor init failed!");
	delay(1000);
	}
	Serial.println("Distance sensor is online!");
	
	// set timing budget for distance sensor
	distanceSensor.setTimingBudgetInMs(lsTimingBudget); 	//Set the timing budget for a measurement; default: 33ms; value range: 20ms - 1000ms
	Serial.print("Timing budget is ");
	Serial.print(distanceSensor.getTimingBudgetInMs());	//Get the timing budget in ms for a measurement
	Serial.println(" ms");
	
	// Intermeasurement period must be >= timing budget. Default = 100 ms.
	distanceSensor.setInterMeasurementInMs(100);
	Serial.print("Intermeasurement period is ");
	Serial.print(distanceSensor.getInterMeasurementInMs());
	Serial.println(" ms");
	
	// Next 2 lines are selection of ONE of the 2 distance modes: uncomment the line you want to use
	//distanceSensor.setDistanceModeShort();
	distanceSensor.setDistanceModeLong();
	
	distanceSensor.getDistanceMode(); //Get the distance mode, returns 1 for short and 2 for long
	Serial.print("Distance mode is ");
	if (distanceSensor.getDistanceMode() == 1) {Serial.println(" short");}
	else if (distanceSensor.getDistanceMode() == 2) {Serial.println(" long");}
	else {Serial.println(" unknown !!!");}

	distanceSensor.startRanging(); 			// Start distance measurement

}

void loop() {

	timestamp = millis();
	
	distance = (float) distanceSensor.getDistance(); 	//Get the result of the measurement from the sensor
	Serial.printf("%d msec: \t", timestamp);
	Serial.printf("Distance: %d mm\r\n", distance);

	accSensor.accelUpdate();
	accelX = accSensor.accelX();
	accelY = accSensor.accelY();
	accelZ = accSensor.accelZ();

	gyroX = accSensor.gyroX();
	gyroY = accSensor.gyroY();
	gyroZ = accSensor.gyroZ();
	
	accSensor.magUpdate();
	magX = accSensor.magX();
	magY = accSensor.magY();
	magZ = accSensor.magZ();


	sendACCData();
	sendVARIOUS1Data();

	delay(LaserReadInterval);
}


void sendACCData() {
    // send acceleration data
	dataSet = (float) ACC;
	PhyphoxBLE::write(dataSet, accelX, accelY, accelZ);     //Send acc values to phyphox 
	Serial.printf("Acc: x:%f\t\ty:%f\t\tz:%f", accelX, accelY,accelZ);
	Serial.println("");

	
}

void sendGYROData() {
    // send acceleration data
	dataSet = (float) GYRO;
	PhyphoxBLE::write(dataSet, gyroX, gyroY, gyroZ);     //Send gyro values to phyphox  
	
}

void sendMAGData() {
    // send acceleration data
	dataSet = (float) MAG;
	PhyphoxBLE::write(dataSet, magX, magY, magZ);     //Send mag values to phyphox  
	
}

void sendVARIOUS1Data() {
    // send acceleration data
	dataSet = (float) VARIOUS1;
	PhyphoxBLE::write(dataSet, distance, temperature, pressure);     //Send distance, temp and pressure values to phyphox  
	Serial.printf("VARIOUS1: dist:%f\t\ttemp:%f\t\tpressure:%f", distance, temperature,pressure);
	Serial.println("");
	
}

