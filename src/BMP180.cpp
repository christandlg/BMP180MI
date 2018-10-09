//Multi interface Bosch Sensortec BMP280  pressure sensor library 
// Copyright (c) 2018 Gregor Christandl <christandlg@yahoo.com>
// home: https://bitbucket.org/christandlg/BMP180
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


#include "BMP180MI.h"

BMP180MI::BMP180() : 
sampling_mode_(BMP180MI::MODE_ULP),
up_(0L),
ut_(0l)
{
	//nothing to do here...
}

BMP180MI::~BMP180()
{
	//nothing to do here...
}

bool BMP180MI::measurePressure()
{
	//return false if a measurement is already running. 
	if (readRegisterValue(BMP180_REG_CTRL_MEAS, BMP180_MASK_SCO))
		return false;
	
	uint8_t measure_cmd = (sampling_mode_ << 6) | BMP180_CMD_PRESS;

	//start a measurement. 
	writeRegisterValue(BMP180_REG_CTRL_MEAS, BMP180_MASK_OSS | BMP180_MASK_MCTRL, measure_cmd);

	return true;
}

bool BMP180MI::measureTemperature()
{
	//return false if a measurement is already running. 
	if (readRegisterValue(BMP180_REG_CTRL_MEAS, BMP180_MASK_SCO))
		return false;

	//start a measurement. 
	writeRegisterValue(BMP180_REG_CTRL_MEAS, BMP180_MASK_MCTRL, BMP180_CMD_TEMP);

	return true;
}

bool BMP180MI::hasValue()
{
	if(readRegisterValue(BMP180_REG_CTRL_MEAS, BMP180_MASK_SCO));
		return false;

	uint8_t mode = readRegisterValue(BMP180_REG_CTRL_MEAS, BMP180_MASK_MCTRL);

	switch (mode)
	{
		case BMP180_CMD_PRESS:
		up_ = readRegisterValueBurst(BMP180_REG_OUT, BMP180_MASK_PRESS, 3);
		up_ >>= (8 - sampling_mode_);
		break;
		case BMP180_CMD_TEMP:
		up_ = readRegisterValueBurst(BMP180_REG_OUT, BMP180_MASK_TEMP, 2);
		break;
		default:
		return false;
	}

	return true;
}

float BMP180MI::getPressure()
{
	int32_t p = 0;

	int32_t B6 = B5_ - 4000;

	int32_t X1 = (cal_params_.cp_B2_ * ((B6 * B6) >> 12) >> 11;

	int32_t X2 = (cal_params_.cp_AC2_ * B6) >> 11;

	int32_t X3 = X1 + X2;

	int32_t B3 = ((((cal_params_.cp_AC1_ << 2) + X3) << sampling_mode_) + 2) >> 2;

	X1 = (cal_params_.cp_AC3_ * B6) >> 13;

	X2 = (cal_params_.B1 * ((B6 * B6) >> 12 )) >> 16;

	X3 = (X1 + X2 + 2) >> 2;

	uint32_t B4 = cal_params_.cp_AC4_ * (static_cast<uint32_t>(X3 + 32768) >> 15);

	uint32_t B7 = static_cast<uint32_t>(up_ - B3) * (50000 >> sampling_mode_);

	if (B7 < 0x80000000)
		p = (B7 << 1) / B4;
	else
		p = (B7 / B4) << 1;

	X1 = (p << 8) * (p << 8);

	X1 = (X1 * 3038) >> 16;

	X2 = (-7357 * p) >> 16;

	p = p + ((X1 + X2 + 3791) >> 4);

	return static_cast<float>(p);
}

float BMP180MI::getTemperature()
{
	int32_t X1 = ((ut_ - cal_params_.AC6) * cal_params_.AC5) >> 15;

	int32_t X2 = (cal_params_.cp_MC_ << 11) / (X1 + cal_params_.cp_MD_);

	B5_ = X1 + X2;

	int32_t T = (B5_ + 8) >> 4;

	return static_cast<float>(T) * 0.01;
}

float BMP180MI::readTemperature()
{
	if (!measureTemperature())
		return NAN;

	do
	{
		delay(10);
	} while (!hasValue());

	return getTemperature();
}

float BMP180MI::readPressure()
{
	if (isNAN(readTemperature()))
		return NAN;

	if (!measurePressure())
		return NAN;

	do
	{
		delay(10);
	} while (!hasValue());

	return getPressure();
}

uint8_t BMP180MI::readID()
{
	return readRegisterValue(BMP180_REG_ID, BMP180_MASK_ID);
}

void BMP180MI::resetToDefaults()
{
	writeRegisterValue(BMP180_REG_RESET, BMP180_MASK_RESET, BMP180_CMD_RESET);
}

uint8_t BMP180MI::getSamplingMode()
{
	return sampling_mode_;
}

bool BMP180MI::setSamplingMode(uin8_t mode)
{
	if (mode > 3)
		return false;

	sampling_mode_ = mode;

	return true;
}	

uint8_t BMP180MI::getMaskShift(uint8_t mask)
{
	uint8_t return_value = 0;

	//count how many times the mask must be shifted right until the lowest bit is set
	if (mask != 0)
	{
		while (!(mask & 1))
		{
			return_value++;
			mask >>= 1;
		}
	}

	return return_value;
}

uint8_t BMP180MI::getMaskedBits(uint8_t reg, uint8_t mask)
{
	//extract masked bits
	return ((reg & mask) >> getMaskShift(mask));
}

uint8_t BMP180MI::setMaskedBits(uint8_t reg, uint8_t mask, uint8_t value)
{
	//clear mask bits in register
	reg &= (~mask);

	//set masked bits in register according to value
	return ((value << getMaskShift(mask)) & mask) | reg;
}

uint8_t BMP180MI::readRegisterValue(uint8_t reg, uint8_t mask)
{
	return getMaskedBits(readRegister(reg), mask);
}

void BMP180MI::writeRegisterValue(uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t reg_val = readRegister(reg);
	writeRegister(reg, setMaskedBits(reg_val, mask, value));
}

bool BMP180MI::readRawValues()
{
	return false;
}

bool BMP180MI::readCompensationParameters()
{
	return false;
}


//-----------------------------------------------------------------------
//BMP180I2C
BMP180I2C::BMP180I2C(uint8_t i2c_address) :
	i2c_address_(i2c_address)
{
	//nothing to do here...
}

BMP180I2C::~BMP180I2C()
{
	//nothing to do here...
}

bool BMP180I2C::beginInterface()
{
	return true;
}

uint8_t BMP180I2C::readRegister(uint8_t reg)
{
	#if defined(ARDUINO_SAM_DUE)
		//workaround for Arduino Due. The Due seems not to send a repeated start with the code above, so this 
		//undocumented feature of Wire::requestFrom() is used. can be used on other Arduinos too (tested on Mega2560)
		//see this thread for more info: https://forum.arduino.cc/index.php?topic=385377.0
		Wire.requestFrom(address_, 1, reg, 1, true);
	#else
		Wire.beginTransmission(i2c_address_);
		Wire.write(reg);
		Wire.endTransmission(false);
		Wire.requestFrom(address_, static_cast<uint8_t>(1));
	#endif
	
	return Wire.read();
}

uint32_t BMP180I2C::readRegisterBurst(uint8_t reg, uint8_t length)
{
	return uint32_t();
}

void BMP180I2C::writeRegister(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(i2c_address_);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}
