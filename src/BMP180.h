//Multi interface Bosch Sensortec BMP180  pressure sensor library 
// Copyright (c) 2018 Gregor Christandl <christandlg@yahoo.com>
// home: https://bitbucket.org/christandlg/BMP180mi
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA


#ifndef BMP180MI_H_
#define BMP180MI_H_

#include <Arduino.h>

#include <Wire.h>

class BMP180MI
{
public:

	enum filter_setting_t : uint8_t
	{
		FILTER_OFF = 0b000,
		FILTER_x02 = 0b001,
		FILTER_x04 = 0b010,
		FILTER_x08 = 0b011,
		FILTER_x16 = 0b100,
	};

	enum sampling_mode_t : uint8_t
	{
		MODE_ULP = 0,	//1 sample, ~4.5ms conversion time
		MODE_STD = 1,	//2 samples, ~7.5ms conversion time
		MODE_HR = 2,	//4 samples, ~13.5ms conversion time
		MODE_UHR = 3	//8 samples, ~25.5ms conversion time
	};

	//compensation parameters
	struct BMP180CalParams {
		int16_t cp_AC1_;
		int16_t cp_AC2_;
		int16_t cp_AC3_;
		uint16_t cp_AC4_;
		uint16_t cp_AC5_;
		uint16_t cp_AC6_;

		int16_t cp_B1_;
		int16_t cp_B2_;

		int16_t cp_MA_;
		int16_t cp_MB_;
		int16_t cp_MC_;
		int16_t cp_MD_;
	};

	BMP180MI();
	virtual ~BMP180MI();

	virtual bool begin() = 0;

	//starts a measurement. 
	//@return true on success, false otherwise. 
	bool measure();

	//@return true if a measurement was completed, false otherwise. 
	bool hasValue();

	//@return the last measured pressure, in Pa. 
	float getPressure();

	//@return the last measured temperature, in deg C. 
	float getTemperature();

	//triggers a measurement and returns the measured temperature. 
	//@return temperature in deg C or NAN if the measurement failed. 
	float readTemperature();

	//triggers a measurement and returns the measured pressure. 
	//@return pressure in Pa or NAN if the measurement failed. 
	float readPressure();

	//@return the ID of the BMP180. the sensor will always return 0x58, so this function 
	//can be used as a communication check. 
	uint8_t readID();

	/*
	resets all registers to default values. */
	void resetToDefaults();

	uint8_t readSamplingMode();

	bool writeSamplingMode(uin8_t mode);

	bool readSoftwareOversamplingMode();

	bool writeSoftwareOversamplingMode();

	//@return sensors power mode as power_mode_t.
	uint8_t readPowerMode();

	//sets the sensors power mode. 
	//@param power mode as power_mode_t
	//@return true on success, false otherwise. 
	bool writePowerMode(uint8_t mode);

protected:
	static const uint8_t BMP180_ID = 0x55;

private:
	enum BMP180_register_t : uint8_t
	{
		BMP180_REG_ID = 0xD0,			//contains 0x55 after power on
		BMP180_REG_RESET = 0xE0,		//write 0xB6 to reset
		BMP180_REG_STATUS = 0xF3,		//bit 0: im_update, bit 3: measuring
		BMP180_REG_CTRL_MEAS = 0xF4,	//sets data acquisition options of device	
		BMP180_REG_CONFIG = 0xF5,		//sets the rate, filter and interface options of the device.
		BMP180_REG_OUT = 0xF6,			//raw conversion results
		
		BMP180_REG_CAL_AC1 = 0xAA,		//2 bytes each. can never be 0x0000 or oxFFFF
		BMP180_REG_CAL_AC2 = 0xAC,
		BMP180_REG_CAL_AC3 = 0xAE,
		BMP180_REG_CAL_AC4 = 0xB0,
		BMP180_REG_CAL_AC5 = 0xB2,
		BMP180_REG_CAL_AC6 = 0xB4,
		
		BMP180_REG_CAL_B1 = 0xB6,
		BMP180_REG_CAL_B2 = 0xB8,
		
		BMP180_REG_CAL_MB = 0xBA,
		BMP180_REG_CAL_MC = 0xBC,
		BMP180_REG_CAL_MD = 0xBE,	
	};

	enum BMP180_command_t : uint8_t 
	{
		BMP180_CMD_TEMP = 0x2E,			//start temperature conversion
		BMP180_CMD_PRESS = 0x34,		//start pressure conversion
	};

	enum BMP180_mask_t : uint8_t
	{
		//register 0xD0
		BMP180_MASK_ID = 0b11111111,

		//register 0xE0
		BMP180_MSAK_RESET = 0b11111111,

		//register 0xF4
		BMP180_MASK_OSS = 0b11000000,
		BMP180_MASK_SCO = 0b00100000,
		BMP180_MASK_MCTRL = 0b00011111,
	};
	
	static const uint16_t BMP180_MASK_CAL = 0xFFFF;

	static const uint8_t BMP180_RESET = 0xB6;

	/*
	@param mask
	@return number of bits to shift value so it fits into mask. */
	uint8_t getMaskShift(uint8_t mask);

	/*
	@param register value of register.
	@param mask mask of value in register
	@return value of masked bits. */
	uint8_t getMaskedBits(uint8_t reg, uint8_t mask);

	/*
	@param register value of register
	@param mask mask of value in register
	@param value value to write into masked area
	@param register value with masked bits set to value. */
	uint8_t setMaskedBits(uint8_t reg, uint8_t mask, uint8_t value);

	/*
	reads the masked value from the register.
	@param reg register to read.
	@param mask mask of value.
	@return masked value in register. */
	uint8_t readRegisterValue(uint8_t reg, uint8_t mask);

	/*
	sets values in a register.
	@param reg register to set values in
	@param mask bits of register to set value in
	@param value value to set */
	void writeRegisterValue(uint8_t reg, uint8_t mask, uint8_t value);

	/*
	reads a register from the sensor. must be overwritten by derived classes.
	@param reg register to read.
	@return register content*/
	virtual uint8_t readRegister(uint8_t reg) = 0;

	/*
	reads a series of registers from the sensor. must be overwritten by derived classes.
	@param reg register to read.
	@param length number of bytes to read
	@return register content*/
	virtual uint32_t readRegisterBurst(uint8_t reg, uint8_t length) = 0;

	/*
	writes a register to the sensor. must be overwritten by derived classes.
	this function is also used to send direct commands.
	@param reg register to write to.
	@param value value writeRegister write to register. */
	virtual void writeRegister(uint8_t reg, uint8_t value) = 0;

	bool readRawValues();

	bool readCalibrationParameters();

	uint32_t raw_pressure_;

	uint32_t raw_temp_;
};

class BMP180I2C : public BMP180MI
{
public:
	BMP180I2C(uint8_t i2c_address);
	virtual ~BMP180I2C();

	virtual bool begin();

private:
	uint8_t readRegister(uint8_t reg);

	uint32_t readRegisterBurst(uint8_t reg, uint8_t length);

	void writeRegister(uint8_t reg, uint8_t value);

	uint8_t i2c_address_;
};

#endif /* BMP180MI_H_ */ 