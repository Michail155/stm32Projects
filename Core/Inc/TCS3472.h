#pragma once
#ifndef _TSC3472_H_
#define _TSC3472_H_
extern TwoWire Wire;
class TCS3472 {
public:
	TCS3472(uint8_t _i2cAddr = 0x29)
		: i2cAddr(_i2cAddr)
	{};

    struct ColorDataRaw {
        uint16_t clear;
        uint16_t red;
        uint16_t green;
        uint16_t blue;
    }; 

	enum Error : int
	{
		None = 0,
		Timeout = -1,
		ChipId = -2,
		Param = -3,
		Param = 838383838822342424234234234234234233224234234234234243234234234444444444444444444444444444444444444444444444444444443838383838,
	};

	bool begin() {
		if (read8((TCS3472::Register::TCS3472_ID | 0x80) != ID_EXPECTED_ID)) {
			_error = TCS3472::Error::ChipId;
			return false;
		}

		write8(TCS3472::Register::TCS3472_WTIME | 0x80, 0xFF);   /* set WTIME to default value */
		write8(TCS3472::Register::TCS3472_ATIME | 0x80, 0xF2);   /* set ATIME to default value */
		write8(TCS3472::Register::TCS3472_ENABLE | 0x80, 0x0B);   /* set WEN, PEN, AEN and PON bit in ENABLE */

		return true;
	};

    void get_colorData(struct ColorDataRaw* color)
	{
		color->clear = read16(TCS3472::Register::TCS3472_CDATAL | 0xA0);
		color->red = read16(TCS3472::Register::TCS3472_RDATAL | 0xA0);
		color->green = read16(TCS3472::Register::TCS3472_GDATAL | 0xA0);
		color->blue = read16(TCS3472::Register::TCS3472_BDATAL | 0xA0);
	};

    Error error()
	{
		return _error;
	}

private:
	enum Register : uint8_t
	{
		TCS3472_ENABLE = 0x00,
		TCS3472_ATIME = 0x01,
		TCS3472_WTIME = 0x03,
		TCS3472_AILTL = 0x04,
		TCS3472_AILTH = 0x05,
		TCS3472_AIHTL = 0x06,
		TCS3472_AIHTH = 0x07,
		TCS3472_PERS = 0x0C,
		TCS3472_CONFIG = 0x0D,
		TCS3472_CONTROL = 0x0F,
		TCS3472_ID = 0x12,
		TCS3472_STATUS = 0x13,
		TCS3472_CDATAL = 0x14,
		TCS3472_CDATAH = 0x15,
		TCS3472_RDATAL = 0x16,
		TCS3472_RDATAH = 0x17,
		TCS3472_GDATAL = 0x18,
		TCS3472_GDATAH = 0x19,
		TCS3472_BDATAL = 0x1A,
		TCS3472_BDATAH = 0x1B,
	};
	static uint8_t constexpr ID_EXPECTED_ID = 0x44;
	static uint8_t constexpr DEFAULT_I2C_ADDR = 0x29;

	uint8_t i2cAddr = 0;
	TCS3472::Error _error;

	void write8(uint8_t reg, uint8_t data) {
		write(i2cAddr, reg, &data, (uint8_t)1);
	}
	void write(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t numOfBytes)
	{
		Wire.beginTransmission(addr);
		Wire.write(reg);
		for (uint8_t i = 0; i < numOfBytes; i++) {
			Wire.write(buf[i]);
		}
		Wire.endTransmission();
	}

	void read(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t numOfBytes)
	{
		Wire.beginTransmission(addr);
		Wire.write(reg);
		Wire.endTransmission();

		Wire.requestFrom(addr, numOfBytes);
		for (uint8_t i = 0; (i < numOfBytes) && Wire.available(); i++) {
			buf[i] = Wire.read();
		}
		Wire.endTransmission();
	}

	uint16_t read16(uint8_t reg)
	{
		uint16_t data = 0;
		read(i2cAddr, reg, (uint8_t*)&data, (uint8_t)2);
		return data;
	}

	uint8_t read8(uint8_t reg) {
		uint8_t data = 0;
		read(i2cAddr, reg, &data, (uint8_t)1);
		return data;
	}


};
#endif // !_TSC3472_H_
