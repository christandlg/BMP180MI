// AS3935MI_LightningDetector_I2C.ino
//
// shows how to use the AS3935MI library with the lightning sensor connected using I2C.
//
// Copyright (c) 2018 Gregor Christandl
//
// connect the AS3935 to the Arduino like this:
//
// Arduino - AS3935
// 5V ------ VCC
// GND ----- GND
// D2 ------ IRQ		must be a pin supporting external interrupts, e.g. D2 or D3 on an Arduino Uno.
// SDA ----- MOSI
// SCL ----- SCL
// 5V ------ SI		(activates I2C for the AS3935)
// 5V ------ A0		(sets the AS3935' I2C address to 0x01)
// GND ----- A1		(sets the AS3935' I2C address to 0x01)
// 5V ------ EN_VREG !IMPORTANT when using 5V Arduinos (Uno, Mega2560, ...)
// other pins can be left unconnected.

#include <Arduino.h>
#include <Wire.h>

#include <BMP180MI.h>

#define I2C_ADDRESS 0x77

//create an AS3935 object using the I2C interface, I2C address 0x01 and IRQ pin number 2
BMP180I2C bmp180(I2C_ADDRESS);

void setup() {
  // put your setup code here, to run once:
	Serial.begin(9600);

	//wait for serial connection to open (only necessary on some boards)
	while (!Serial);

	Wire.begin(D2, D3);

	//begin() checks the Interface and I2C Address passed to the constructor and resets the AS3935 to 
	//default values.
	if (!bmp180.begin())
	{
		Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
		while (1);
	}

	Serial.print("Sensor ID: ");
	Serial.println(bmp180.readID(), HEX);

	//reset sensor to default parameters.
	bmp180.resetToDefaults();

	bmp180.setSamplingMode(BMP180MI::MODE_UHR);

	//...
}

void loop() {
  // put your main code here, to run repeatedly:

	delay(1000);

	//start a measurement
	if (!bmp180.measureTemperature())
	{
		Serial.println("could not start temperature measurement, is a measurement already running?");
		return;
	}

	bmp180.hasValue();

	//wait for the measurement to finish
	do
	{
		delay(100);
	} while (!bmp180.hasValue());

	Serial.print("Temperature: "); Serial.println(bmp180.getTemperature());

	//start a measurement
	if (!bmp180.measurePressure())
	{
		Serial.println("could not start perssure measurement, is a measurement already running?");
		return;
	}

	//wait for the measurement to finish
	do
	{
		delay(100);
	} while (!bmp180.hasValue());

	Serial.print("Pressure: "); Serial.println(bmp180.getPressure());
}