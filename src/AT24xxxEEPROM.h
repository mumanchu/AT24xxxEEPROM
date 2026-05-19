#pragma once

/////////////////////////////////////////////////////////////////////
// I2C 2-Wire Serial EEPROM Library for AT24xxx EEPROMS
// Copyright(C) muman.ch, 2026.04.23
// https://github.com/mumanchu/AT24xxxEEPROM
/*
Supports 1K, 2K, 4K, 8K, 16K, 32K, 64K, 128K and 256K *BIT* I2C devices.
The old Atmel/Microchip versions with type numbers, AT24xx01 .. AT24xx256.
'xx' indicates the technology: C | AA | LC
e.g. AT24C01 AT24AA01 AT24LC01 

The base I2C address is 0x50 as 0b01010xxx, where xxx is taken from the 
hard-wired A2 A1 A0 bits. Note that the 4/8/16K bit chips do not use 
these bits, see begin().

I2C runs at up to 400KHz.

USAGE 
-----
i2cAddress is normally 0x50 + A2 A1 A0 bits.

Initialise 'Wire' before calling begin():

#include "AT24xxxEEPROM.h"
AT24xxxEEPROM eeprom;

	Wire.begin(sdaPin, sclPin);
	Wire.setClock(400000);
	Wire.setTimeout(100);
	if (!eeprom.begin(&Wire, 0x50, 16, wpPin)) {
		// fatal error
	}

EEPROM DATA SHEETS
------------------
1K 2K 4K 8K 16K bits
https://ww1.microchip.com/downloads/en/DeviceDoc/doc0180.pdf
32K bits
https://ww1.microchip.com/downloads/en/devicedoc/21713m.pdf
32K and 64K bits
https://ww1.microchip.com/downloads/en/DeviceDoc/doc0336.pdf
128k 256K bits
https://ww1.microchip.com/downloads/en/DeviceDoc/doc0670.pdf
*/

// for debugging, see MumanchuDebug.h
//#ifndef ASSERT
//#define ASSERT(b) if (!(b)) { Serial.println("ASSERT failed"); Serial.flush(); return false; }
//#define LOGERROR(s) { Serial.println(s); Serial.flush(); }
//#endif


class AT24xxxEEPROM
{
private:
	TwoWire* wire;
	uint i2cAdds;
	uint kBits;
	ulong nBytes;
	uint pageSize;
	uint wpPin = 0;
	bool writeProtected;

public:
	bool begin(TwoWire* twoWire, uint i2cAddress, uint sizeInBytes, 
		uint writeProtectPin);
	void writeProtect() { setWriteProtect(true); }
	void writeEnable() { setWriteProtect(false); }
	bool read(uint address, byte* buffer, uint length);
	bool write(uint address, const byte* buffer, uint length);
	bool erasePage(uint page);
	bool eraseChip();
	ulong getSizeInBytes() { return nBytes; }
	uint getPageSizeInBytes() { return pageSize; }
	bool test();	// warning, this test erases the chip!

protected:
	void sendAddress(uint address);
	void setWriteProtect(bool protect);
};


bool AT24xxxEEPROM::begin(TwoWire* twoWire, uint i2cAddress, uint sizeInKBits, 
	uint writeProtectPin = 0)
{
	ASSERT(twoWire != NULL && (i2cAddress & 0xf8) == 0x50);

	// For address bits of 4/8/16K bit chips:
	// https://ww1.microchip.com/downloads/en/DeviceDoc/doc0180.pdf#page=11

	switch (sizeInKBits) {
	case 1:
	case 2:
		pageSize = 8;
		break;
	case 4:
		// 4K bits, A0 not used
		ASSERT((i2cAddress & 1) == 0);
		pageSize = 16;
		break;
	case 8:
		// 8K bits, A1 A0 not used
		ASSERT((i2cAddress & 3) == 0);
		pageSize = 16;
		break;
	case 16:
		// 16K bits, A2 A1 A0 not used
		ASSERT((i2cAddress & 7) == 0);
		pageSize = 16;
		break;
	case 32:
	case 64:
		pageSize = 32;
		break;
	case 128:
	case 256:
		pageSize = 64;
		break;
	default:
		LOGERROR("invalid sizeInKBits");
		return false;
	}
	wire = twoWire;
	i2cAdds = i2cAddress;
	kBits = sizeInKBits;
	nBytes = (sizeInKBits * 1024UL) / 8;
	wpPin = writeProtectPin;

	// write protect pin, 1=write protected, 0=write enabled
	if (writeProtectPin > 0) {
		pinMode(wpPin, OUTPUT_OPEN_DRAIN);	// pin has 10K pullup
		writeProtect();		// write protected by default
	}

	// is the chip responding?
	wire->beginTransmission((byte)i2cAdds);
	return wire->endTransmission() == 0;
}

bool AT24xxxEEPROM::read(uint address, byte* buffer, uint length)
{
	ASSERT(address + length <= nBytes);

	// read from pageSize boundaries, it's more efficient
	int bufferOffset = 0;

	while (length) {
		int pageOffset = address % pageSize;
		int bytesToRead = pageSize - pageOffset;
		if (bytesToRead > length)
			bytesToRead = length;

		// transmit I2C and memory address according to the chip size
		sendAddress(address);

		if (wire->endTransmission() != 0) {
			LOGERROR("endTransmission failed");
			return false;
		}
		if (wire->requestFrom(i2cAdds, bytesToRead) != bytesToRead) {
			LOGERROR("requestFrom failed");
			return false;
		}
		if (wire->readBytes(buffer + bufferOffset, bytesToRead) != bytesToRead) {
			LOGERROR("readBytes failed");
			return false;
		}
		bufferOffset += bytesToRead;	// next buffer offset
		address += bytesToRead;			// next address
		length -= bytesToRead;			// number of bytes remaining to be written
	}
	return true;
}

bool AT24xxxEEPROM::write(uint address, const byte* buffer, uint length)
{
	ASSERT(address + length <= nBytes);

	if (writeProtected) {
		LOGERROR("write protected");
		return false;
	}
	bool ok = true;
	int offset = 0;

	// write in pageSize chunks
	while (length) {
		int pageOffset = address % pageSize;
		int bytesToWrite = pageSize - pageOffset;
		if (bytesToWrite > length)
			bytesToWrite = length;

		// transmit I2C and memory address according to the chip size
		sendAddress(address);

		if (wire->write(buffer + offset, bytesToWrite) != bytesToWrite) {
			LOGERROR("write failed");
			ok = false;
			break;
		}
		if (wire->endTransmission()) {
			LOGERROR("endTransmission failed");
			ok = false;
			break;
		}
		offset += bytesToWrite;		// next buffer offset
		address += bytesToWrite;	// next address
		length -= bytesToWrite;		// number of bytes remaining to be written

		// poll until finished, up to 10ms depending on the number of bytes
		while (1) {
			delay(1);
			wire->beginTransmission((byte)i2cAdds);
			if (wire->endTransmission() != 2)
				break;
		}
	}
	return ok;
}

bool AT24xxxEEPROM::erasePage(uint page)
{
	ASSERT(page < (nBytes / pageSize));

	// write a page of FFs
	byte data[pageSize];
	memset(data, 0xff, pageSize);
	return write(page * pageSize, data, pageSize);
}

bool AT24xxxEEPROM::eraseChip()
{
	// write all pages with FFs
	uint nPages = nBytes / pageSize;
	for (uint page = 0; page < nPages; ++page) {
		if (!erasePage(page))
			return false;
	}
	return true;
}

// Transmit I2C and memory address according to the chip size
void AT24xxxEEPROM::sendAddress(uint address)
{
	// message format depends on chip size
	byte startByte = i2cAdds;

	// these chips have the MS bits in the 1st byte,
	// in place of the A2 A1 A0 address bits
	if (kBits == 4 || kBits == 8 || kBits == 16)
		startByte |= (address >> 8);
	wire->beginTransmission(startByte);

	// large chips send 2 address bytes
	if (kBits > 16)
		wire->write((byte)(address >> 8));

	wire->write((byte)address);
}

void AT24xxxEEPROM::setWriteProtect(bool protect)
{
	writeProtected = protect;
	// write protect pin, 1=write protected, 0=write enabled
	if (wpPin > 0)
		digitalWrite(wpPin, protect ? 1 : 0);
}


// Patched out for safety
#if 1
// THIS TEST OVERWRITES *ALL* THE DATA IN THE EEPROM!
// Writes a random byte to each location, one byte at a time. 
// Then reads each byte back and verifies it is correct.
// The eeprom is then erased and verified that it's all FFs.
bool AT24xxxEEPROM::test()
{
	// write random data one byte at a time
	srandom(1234);
	for (int i = 0; i < nBytes; ++i) {
		byte b = (byte)random();
		if (!write(i, &b, 1))
			return false;
	}

	// read each byte back and verify the value
	srandom(1234);
	for (int i = 0; i < nBytes; ++i) {
		byte b;
		if (!read(i, &b, 1))
			return false;
		if (b != (byte)random())
			return false;
	}

	// erase the chip and check for all FFs
	if (!eraseChip())
		return false;
	for (int i = 0; i < nBytes; ++i) {
		byte b;
		if (!read(i, &b, 1))
			return false;
		if (b != 0xff)
			return false;
	}
	return true;
}
#endif
