/*
  SL44x2.h - Smart Card library (SL4432 / SL4442 Support Class)
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

#ifndef SL44X2_H_
#define SL44X2_H_

#include <SCLib.h>

// I've experimented with an automatic recognition, but there might be many compatible
// types with different ATR signatures. May this comes in a later release, but this way
// is far more portable as the user of the class knows what card needs to be supported.

// Currently you have to specify the type used by activating ONE define below

// Please select the card type that needs to be supported
#define SLE4442
//#define SLE4440
//#define SLE4441
//#define SLE4432

//
// NO NEED TO CHANGE ANY DEFINES / CODE BELOW THIS LINE, UNLESS YOU KNOW WHAT YOU ARE DOING ;-.)
//

#if !defined(SLE4442) && !defined(SLE4441) && !defined(SLE4440) && !defined(SLE4432)
#error YOU NEED TO SPECIFY THE SUPPORTED CARD TYPE 'SLE4442, SLE4441, SLE4440 or SLE4432'
#endif

#if !defined(SYNCHRON_CARDS)
#error TO USE THE SL44x2 CLASS THE SUPPORT FOR SYNCHRON CARDS NEEDS TO BE ACTIVATED IN SCLib.h
#endif

#if defined(SLE4440)
#define SL44X2_DATA_SIZE                 64
#define SL44X2_SUPPORT_PSC
#endif

#if defined(SLE4441)
#define SL44X2_DATA_SIZE                 128
#define SL44X2_SUPPORT_PSC
#endif

#if defined(SLE4442)
#define SL44X2_DATA_SIZE                 256
#define SL44X2_SUPPORT_PSC
#endif

#if defined(SLE4432)
#define SL44X2_DATA_SIZE                 256
#undef SL44X2_SUPPORT_PSC
#endif

#define SL44X2_MAX_WRITE_CLK             255
#define SL44X2_MEMORY_ERROR              0xFFFFFFFF
#define SL44X2_PROTECTION_AREA_START     0
#define SL44X2_PROTECTION_AREA_END       31

#define SL44X2_PROTECTION_POSSIBLE(x)    ((x)>=SL44X2_PROTECTION_AREA_START && (x)<=SL44X2_PROTECTION_AREA_END)

enum SL44X2_Control_Commands_t {
	SL44X2_READ_MAIN_MEMORY=0x30,
	SL44X2_READ_PROTECTION_MEMORY=0x34,
	SL44X2_UPDATE_MAIN_MEMORY=0x38,
#if defined(SL44X2_SUPPORT_PSC)
	SL44X2_READ_SECURITY_MEMORY=0x31,
	SL44X2_UPDATE_SECURITY_MEMORY=0x39,
	SL44X2_COMPARE_VERIFICATION_DATA=0x33,
#endif
	SL44X2_WRITE_PROTECTION_MEMORY=0x3C };

typedef struct {
	uint8_t control;
	uint8_t adress;
	uint8_t data;
} SL44X2_Command_t;

class SL44x2 : private SmartCardReader {
public:
    /**
     * Creates a basic SL44X2 object, for communication.
     *
     * Available: ALL
     *
     * c7_io               - IO line, connected to Card (C7)
     * c2_rst              - RST for Card (C2)
     * c1_vcc              - Vcc for Card (C1)
     * card_present        - Card present - HIGH - Card available - LOW - Card removed (Can be changed by card_present_invert)
     * c3_clk              - clk signal to SC (C3) - (timer1 / pin9 used)
     * card_present_invert - Use inverted off signal (LOW Card available - HIGH Card removed)
     */
	SL44x2(uint8_t c7_io, uint8_t c2_rst, uint8_t c1_vcc, uint8_t card_present, uint8_t c3_clk, boolean card_present_invert=false) : SmartCardReader(c7_io, c2_rst, c1_vcc, card_present, c3_clk, card_present_invert) {
    	_recognized = false;
    	_authenticated = false;
    };

    /**
     * Is card physically inserted?
     *
     * return true if card is inserted, otherwise false.
     */
    boolean cardInserted();

    /**
     * Just wait until card becomes ready.
     *
     * Available: ALL
     *
     * @return - true, if SL44X2 card was recognized, false (Card inserted, but not recognized)
     */
    boolean cardReady();

    /**
     * Just wait until users remove card from slot.
     *
     * Available: ALL
     */
    void cardRemoved();

    /**
     * read content from SL44X2 card.
     *
     * Available: SL4432 / SL4442
     *
     * start - Start position, to retrieve data from.
     * buf   - pointer to buffer containing the data provided by the card.
     * count - number of bytes requested from card (buf should have at least a size of count bytes).
     *
     * @return - number of bytes received from card.
     */
    uint16_t readMainMemory(uint16_t start, uint8_t* buf, uint16_t count);

    /**
     * update content of SL44X2 card.
     *
     * Available: SL4432 / SL4442
     *
     * start - Start position, to update data.
     * buf   - pointer to buffer containing the data written to the card.
     * count - number of bytes to be written to the card.
     *
     */
    boolean updateMainMemory(uint16_t start, uint8_t* buf, uint16_t count);

    /**
     * Set Protection
     *
     * addr : position of the byte to be protected.
     *
     * return true, if the byte is protected, otherwise false
     */
    boolean protectByte(uint16_t addr);


    /**
     * Reads the protection bits from card.
     *
     * Available: SL4432 / SL4442
     *
     * @return - the protection bits or SL44X2_MEMORY_ERROR in case of an error.
     */
    uint32_t readProtectionMemory();

#if defined(SL44X2_SUPPORT_PSC)
    /**
     * Authenticate
     *
     * pin - pin code for authentication (PSC)
     *
     * @return true, if authentication was successful, false otherwise
     */
    boolean authenticate(uint32_t pin);

    /**
     * Change PSC.
     *
     * Available: SL4442
     *
     * new_psc - new psc to be written to security memory
     *           (Old psc not needed, as you have to be authenticated anyway to access security memory)
     */
    boolean changePSC(uint32_t new_pin);

    /**
     * Reads the security memory from card.
     *
     * Available: SL4442
     *
     * @return - the security memory or SL44X2_MEMORY_ERROR in case of an error
     */
    uint32_t readSecurityMemory();
#endif

	#if defined(SC_DEBUG)
	// Just some debug function
	void dumpHEX(uint8_t* values, uint16_t size);
	#endif

private:

	/**
	 * Write one byte into card.
	 *
	 * control_command - control command used to write the value
	 * addr            - address the value should be written to.
	 * value           - value to be written to addr.
	 */
	boolean _updateMemory(SL44X2_Control_Commands_t control_command, uint8_t addr, uint8_t value);

	/**
	 * Reads the Protected or Security Memory from card.
	 *
	 * Available: SL4432 / SL4442
	 *
	 * @return - the protection bits
	 */
	uint32_t _readMemory(SL44X2_Control_Commands_t control_command);

	/**
	 * Checks is the feature is provided by the current card.
	 *
	 * Available : ALL
	 *
	 * feature to be checked.
	 *
	 * @return true, if feature is supported, otherwise false
	 */
	boolean _isSupported(SL44X2_Control_Commands_t cmd);

	/**
	 * Checks if currently a supported card is connected..
	 *
	 * Available : ALL
	 *
	 * @return true, if card is connected, otherwise false
	 */
	boolean _isSL44X2Connected();

	/**
	 * Checks if the byte at pos is protected.
	 *
	 * Available : ALL
	 *
	 * @return true, if byte is protected, otherwise false
	 */
	boolean _isByteProtected(uint16_t pos);

	/**
	 * Is the card authenticated?
	 *
	 * @return true, if authentication was performed successfully or SL4432 is used, otherwise false.
	 */
	boolean _isAuthenticated();

    // SL44X2 recognized correctly?
    boolean _recognized;
    boolean _authenticated;

    // Smart Card Command
    SL44X2_Command_t _command;
};

#endif /* SL44X2_H_ */
