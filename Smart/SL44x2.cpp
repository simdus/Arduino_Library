/*
  SCLib.cpp - Smart Card library (SLE4432 / SLE4442 / SLE4441 / SLE4440 Support Class)
  Copyright (c) 2013 Frank Bargstedt.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "SL44x2.h"

/**
 * Is card physically inserted?
 *
 * return true if card is inserted, otherwise false.
 */
boolean SL44x2::cardInserted() {
	return SmartCardReader::cardInserted();
}


/**
 * Just wait until card becomes ready.
 *
 * @return - true, if SL44x2 card was recognized, false (Card inserted, but not recognized)
 */
boolean SL44x2::cardReady() {
	uint16_t bytes_received = 0;
	uint8_t  data[MAX_ATR_BYTES];

	_recognized = false;
#if defined(SL44X2_SUPPORT_PSC)
	_authenticated = false;
#else
	// No authentication for write access needed
	_authenticated = true;
#endif

	while (!cardInserted())
		;

	// Just try to activate card
	bytes_received = activate(data, MAX_ATR_BYTES);

	// Data from card received ...
	if (bytes_received > 0) {
		_recognized = true;
	} else {
		// Card not recognized .. So deactivate
		deactivate();
	}
	return _recognized;
}

/**
 * read content from SL44x2 card
 *
 * start - Start position, to retrieve data from.
 * buf   - pointer to buffer containing the data provided by the card.
 * count - number of bytes requested from card (buf should have at least a size of count bytes).
 *
 * @return - number of bytes received from card.
 */
uint16_t SL44x2::readMainMemory(uint16_t start, uint8_t* buf, uint16_t count) {
	// Just some sanity checks
	if (buf != NULL && count > 0 && _isSL44X2Connected() && _isSupported(SL44X2_READ_MAIN_MEMORY)) {
		// Send read command to card
		_command.control = SL44X2_READ_MAIN_MEMORY;
		_command.adress  = start & 0xFF;
		_command.data    = 0x00;
		if (sendBytes((uint8_t*)&_command, 3)) {
			return receiveBytes(buf, count);
		}
	}
	return 0;
}

/**
 * Update card content.
 *
 * start - Start position, to write data to.
 * buf   - pointer to buffer containing the data to be written to the card.
 * count - number of bytes to be written to the card.
 */
boolean SL44x2::updateMainMemory(uint16_t start, uint8_t* buf, uint16_t count) {
	// Just some sanity checks
	boolean result = false;
	if (buf != NULL && count > 0 && _isSupported(SL44X2_UPDATE_MAIN_MEMORY) && _isAuthenticated()) {
		for(uint16_t i = 0; i < count && start + i < SL44X2_DATA_SIZE;i++) {
			// Only write if not protected
			if (!_isByteProtected(start+i)) {
				// Write byte
				if (!_updateMemory(SL44X2_UPDATE_MAIN_MEMORY, start + i, buf[i])) {
					break;;
				}
			}
		}
		result = true;
	}
	return result;
}

/**
 * Write one byte into card.
 *
 * control_command - control command used to write the value
 * addr            - address the value should be written to.
 * value           - value to be written to addr.
 */
boolean SL44x2::_updateMemory(SL44X2_Control_Commands_t control_command, uint8_t addr, uint8_t value) {
	if(_isSupported(control_command) && _isSL44X2Connected()) {
		// Send read command to card
		_command.control = control_command;
		_command.adress  = addr;
		_command.data    = value;
		if (sendBytes((uint8_t*)&_command, 3)) {
			return (SmartCardReader::waitForProcessingCompletion(SL44X2_MAX_WRITE_CLK, HIGH) <= SL44X2_MAX_WRITE_CLK);
		}
	}
	return false;
}

/**
 * Reads the protection bits from card.
 *
 * Available: SL4432 / SL4442
 *
 * @return - the protection bits
 */
uint32_t SL44x2::readProtectionMemory() {
	if (_isSL44X2Connected()) {
		return _readMemory(SL44X2_READ_PROTECTION_MEMORY);
	}
	return SL44X2_MEMORY_ERROR;
}

/**
 * Checks if the byte at pos is protected.
 *
 * Available : ALL
 *
 * @return true, if byte is protected, otherwise false
 */
boolean SL44x2::_isByteProtected(uint16_t pos)  {
	boolean result = false;
	if (SL44X2_PROTECTION_POSSIBLE(pos)) {
		// Check if byte is protected
		result = ((readProtectionMemory() & (1L<<pos)) == 0);
	}
	return result;
}

/**
 * Set Protection
 *
 * Available: SL4432 / SL4442
 *
 * addr : position of the byte to be protected.
 *
 * return true, if the byte is protected, otherwise false
 */
boolean SL44x2::protectByte(uint16_t addr) {
	boolean result = false;
	if (_isSupported(SL44X2_WRITE_PROTECTION_MEMORY) && _isAuthenticated()) {
		// Sanity Check
		if (SL44X2_PROTECTION_POSSIBLE(addr)) {
			// Byte can be protected?
			if (!_isByteProtected(addr)) {
				// Read Byte to be protected
				uint8_t byte_value = 0;
				if (readMainMemory(addr, &byte_value, 1) == 1) {
					// value was read successfully
					if (_updateMemory(SL44X2_WRITE_PROTECTION_MEMORY,addr, byte_value)) {
						result = _isByteProtected(addr);
					}
				}
			}
		}
	}
	return result;
}

/**
 * Checks is the feature is provided by the current card.
 *
 * Available : ALL
 *
 * feature to be checked.
 *
 * @return true, if feature is supported, otherwise false
 */
boolean SL44x2::_isSupported(SL44X2_Control_Commands_t cmd) {
	// All supported commands are supported ;-.)
	return _recognized;
}

/**
 * Is the card authenticated?
 *
 * @return true, if authentication was performed successfully or SL4432 is used, otherwise false.
 */
boolean SL44x2::_isAuthenticated() {
	return _isSL44X2Connected() && _authenticated;
}

/**
 * Checks if currently a supported card is connected..
 *
 * Available : ALL
 *
 * @return true, if card is connected, otherwise false
 */
boolean SL44x2::_isSL44X2Connected() {
	if (!cardInserted()) {
		_recognized = false;
#if defined(SL44X2_SUPPORT_PSC)
		_authenticated = false;
#else
		_authenticated = true;
#endif

	}
	return _recognized;
}

/**
 * Reads the Protected or Security Memory from card.
 *
 * Available: SL4432 / SL4442
 *
 * @return - the protection bits
 */
uint32_t SL44x2::_readMemory(SL44X2_Control_Commands_t control_command) {
	if (_isSL44X2Connected()) {
		uint8_t buf[4];

		// Send read command to card
		_command.control = control_command;
		_command.adress  = 0x00;
		_command.data    = 0x00;
		if (sendBytes((uint8_t*)&_command, 3)) {
			if (receiveBytes(buf, 4) == 4) {
				return (((uint32_t)buf[3])<<24) | (((uint32_t)buf[2])<<16) | (((uint32_t)buf[1])<<8) | ((uint32_t)buf[0]);
			}
		}
	}
	return SL44X2_MEMORY_ERROR;
}


/**
 * Just wait until users remove card from slot.
 */
void SL44x2::cardRemoved() {
	while (cardInserted())
		;

	_recognized = false;
	_authenticated = false;

	// Just be sure to have card deactivated
	deactivate();
}

#if defined(SL44X2_SUPPORT_PSC)
/**
 * Authenticate
 *
 * pin - pin code for authentication (PSC)
 *
 * @return true, if authentication was successful, false otherwise
 */
boolean SL44x2::authenticate(uint32_t pin) {
	_authenticated = false;
	if (_isSL44X2Connected()) {
		if (_isSupported(SL44X2_READ_SECURITY_MEMORY) && _isSupported(SL44X2_UPDATE_SECURITY_MEMORY) && _isSupported(SL44X2_COMPARE_VERIFICATION_DATA)) {
			uint32_t securityMemory = readSecurityMemory();

			// Check if authentication is possible?
			if ((securityMemory & 0x07) > 0) {
				// Search for bit to be set to 0
				uint8_t i = 0;
				uint8_t ec = securityMemory & 0x07;
				for(;i<3;i++) {
					if (ec & (1<<i)) {
						break;
					}
				}

				if (i < 3) {
					// Update EC
					ec &= ~(1<<i);
					_authenticated = _updateMemory(SL44X2_UPDATE_SECURITY_MEMORY,0,ec);
					// Send byte 1 of pin
					if (_authenticated) {
						_authenticated = _updateMemory(SL44X2_COMPARE_VERIFICATION_DATA,1,pin & 0xFF);
					}
					// Send byte 2 of pin
					if (_authenticated) {
						_authenticated = _updateMemory(SL44X2_COMPARE_VERIFICATION_DATA,2,(pin>>8) & 0xFF);
					}
					// Send byte 3 of pin
					if (_authenticated) {
						_authenticated = _updateMemory(SL44X2_COMPARE_VERIFICATION_DATA,3,(pin>>16) & 0xFF);
					}
					// Clear EC
					if (_authenticated) {
						_authenticated = _updateMemory(SL44X2_UPDATE_SECURITY_MEMORY,0,0xFF);
					}
					// Check if clear worked
					if (_authenticated) {
						securityMemory = readSecurityMemory();
						_authenticated = ((securityMemory & 0x07) == 0x07);
					}
				}
			}
		} else {
			// Authentication is not supported ...
			_authenticated = true;
		}
	}
	return _authenticated;
}

/**
 * Change PSC.
 *
 * Available: SL4442
 *
 * new_psc - new psc to be written to security memory
 *           (Old psc not needed, as you have to be authenticated anyway to access security memory)
 */
boolean SL44x2::changePSC(uint32_t new_pin) {
	boolean result = false;
	if (_isSupported(SL44X2_UPDATE_SECURITY_MEMORY) && _isAuthenticated()) {
		result = _updateMemory(SL44X2_UPDATE_SECURITY_MEMORY, 1, new_pin & 0xFF);
		if (result) {
			result = _updateMemory(SL44X2_UPDATE_SECURITY_MEMORY, 2, (new_pin >> 8) & 0xFF);
		}
		if (result) {
			result = _updateMemory(SL44X2_UPDATE_SECURITY_MEMORY, 3, (new_pin >> 16) & 0xFF);
		}
	}
	return result;
}

/**
 * Reads the security memory from card.
 *
 * Available: SL4442
 *
 * @return - the protection bits
 */
uint32_t SL44x2::readSecurityMemory() {
	if (_isSupported(SL44X2_READ_SECURITY_MEMORY) && _isSL44X2Connected()) {
		return _readMemory(SL44X2_READ_SECURITY_MEMORY);
	}
	return SL44X2_MEMORY_ERROR;
}

#endif


#if defined(SC_DEBUG)
void SL44x2::dumpHEX(uint8_t* values, uint16_t size) {
	SmartCardReader::dumpHEX(values, size);
}
#endif

