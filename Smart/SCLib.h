/*
  SCLib.h - Smart Card library
  Copyright (c) 2012 Frank Bargstedt.  All right reserved.

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

#ifndef SCLib_h
#define SCLib_h

#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include "WProgram.h"
#endif

//
// Here you can configure the features supported by the SCLib
//

// Per default it will support synchron and asynchron cards (Maximum size of sclib)
// You can reduce the size of the SCLib by deactivating features you don't need.
// Like when you use the SL44X2 card object to communication the SL44X2 cards.
// Only the SYNCHRON_CARDS define is used
#define SYNCHRON_CARDS
#define ASYNCHRON_CARDS

#if defined (ASYNCHRON_CARDS)
// Its only useful if asynchron cards are supported
// if you don't need the support of creating APDU commands etc.
// you can comment out the APDU_SUPPORT define to save some bytes
#define APDU_SUPPORT
#endif

// Currently if you comment out the SC_DEBUG define, the dumpHEX function
// will not be available anymore to reduce the size of the sclib binary
#define SC_DEBUG

// Uncomment this to activate debug pin toggle on DEFAULT_DEBUG_PIN (Normally 13)
//#define USE_DEBUG_PIN
#define DEFAULT_DEBUG_PIN           13

/* When you connect your SmartCard via an interface chip like TDA8204
   you have to enable this define here, or better in your source code,
   before you import SCLib.h 
   (This only inverts the Vcc control, as the TDA8204 uses an active low signal) */
//#define USE_TDA8204

//
// NO NEED TO CHANGE ANY DEFINES / CODE BELOW THIS LINE, UNLESS YOU KNOW WHAT YOU ARE DOING ;-.)
//

#if !defined(SYNCHRON_CARDS) && !defined(ASYNCHRON_CARDS)
#error AT LEAST ONE CARD TYPE SHOULD BE SUPPORTED (SYNCHRON_CARDS or ASYNCHRON_CARDS)
#endif

// CLK Values f >= 1MHz, leads to asynchronous transmission, otherwise synchronous (CLK_10KHZ)
typedef enum {
#if defined(SYNCHRON_CARDS)
  CLK_10KHZ,
#endif
#if defined(ASYNCHRON_CARDS)
  CLK_1MHZ,
  CLK_2MHZ,
  CLK_2DOT5MHZ,
  CLK_4MHZ,
#endif
  CLK_NO_CLK,
} frequency_t;

// Callback func pointer
typedef void (*SCLibVoidFuncPtr)(void);

// Default Config
#define DEFAULT_CLK_PIN              9
#define DEFAULT_ETU                372

// Maximum time of etu's to wait
#define MAX_WAIT_TIME            40000

// Maximum BYTES in ATR
#define MAX_ATR_BYTES 32


// APDU Command struct
#if defined(APDU_SUPPORT)
typedef struct {
  /* CLASS byte of command */
  uint8_t  cla;
  /* INS byte of command */
  uint8_t  ins;
  /* Parameter 1 of command */
  uint8_t  p1;
  /* Parameter 2 of command */
  uint8_t  p2;
  /* pointer to data buffer containing "data_size" bytes 
     (Can be NULL to signal no data required) */
  uint8_t* data_buf;
  /* number of bytes in data_buf is ignored, if data_buf is NULL. 
     Can also be 0 to signal no data is required. */
  uint16_t data_size;
  /* max. number of bytes for response size */
  uint16_t resp_size;
} APDU_t;
#endif

class SmartCardReader
{
  public:
    // Constructors to define communication pins
    /**
     * Creates a basic smart card reader object, with mandatory
     * pins assigned to establish smart card communication.
     *
     * c7_io               - IO line, connected to Card (C7)
     * c2_rst              - RST for Card (C2)
     * c1_vcc              - Vcc for Card (C1)
     * card_present        - Card present - HIGH - Card available - LOW - Card removed (Can be changed by card_present_invert)
     * c3_clk              - clk signal to SC (C3) - (timer1 / pin9 used)
     * card_present_invert - Use inverted off signal (LOW Card available - HIGH Card removed)
     */
    SmartCardReader(uint8_t c7_io, uint8_t c2_rst, uint8_t c1_vcc, uint8_t card_present, uint8_t c3_clk, boolean card_present_invert=false);
    
    /**
     * Is card inserted?
     *
     * return true if card is inserted, otherwise false.
     */
    boolean cardInserted();
    
    /**
     * Timeout occured, with last communication command?
     *
     * return true if timeout occured, otherwise false.
     */
    boolean timeout();
    
    /**
     * Start activation sequence and return ATR result in buf (buf should be at least 32 Bytes to be able to
     * receive maximum ATR response.
     *
     * return number of received bytes, or 0 if error occured (timeout etc.) 
     */
    uint16_t activate(uint8_t* buf, uint16_t buf_size);
    
    /**
     * Start deactivation sequence
     */
    void deactivate();
    
    /**
     * Get current ETU
     */
    uint16_t getCurrentETU();
    
    /**
     * Change guard time in micro seconds (Should be at least 9600)
     */
    void setGuardTime(unsigned long t);
    
    /**
     * Don't care about parity errors, as some cards screw it up on purpose as they use odd parity ;-)
     */
    void ignoreParityErrors(boolean in);
    
    /**
     * Set callback, to be noticed if time out occurs ...
     */
    void setTimeOutCallback(SCLibVoidFuncPtr cb);

#if defined(ASYNCHRON_CARDS)
    /**
     * Include TS byte in ATR (Only valid for ASync Cards)
     */
    void setIncludeTSinATR(boolean include);
    
    #if defined(APDU_SUPPORT)
    /**
     * Used to caculate ADPU crc value
     *
     * startValue should normaly be 0, only if sequend calls on a data structure are made the curent value 
     *            is used as input for the calculation of the subsequend call.
     * buf        pointer to the APDU command
     * size       number of bytes used for EDC calculation
     *
     * returns the current ECU value.
     */
    uint8_t calcEDC(uint8_t startValue, uint8_t *buf, uint16_t size);
    /**
     * Used to send APDU to smart card.
     *
     * command pointer to APDU command structure
     *
     * send if true data will be send, from command->data_buf (command->data_size bytes)
     *      to smartcard, otherwise data will be received and put into command->data_buf.
     *
     * returns SW1/SW2 byte or 0, if an error occured. 
     */
    uint16_t sendAPDU(APDU_t* command, boolean send=true);
    #endif
#endif

    /**
     * Send data to smartcard
     *
     * Return true, if the receiver signaled a parity error
     */
    boolean sendBytes(uint8_t* buf, uint16_t count);

    /**
     * Receive data from SmartCard until buffer full or timeout
     *
     * Return number of received bytes
     */    
    uint16_t receiveBytes(uint8_t* buf, uint16_t buf_size);

#if defined(SYNCHRON_CARDS)
    /**
     * Wait for sychron card to complete processing.
     *
     * maxClkCycle : Maximum number of cycle card needs to complete process (Use 0 for unlimited)
     * startValue  : Value clk should start with
     *
     * @return number of clk cycles until, processing ended - 0 in case of an error
     */
    uint16_t waitForProcessingCompletion(uint16_t maxClkCycle, uint8_t startValue = HIGH);
#endif

    #if defined(SC_DEBUG)
    // Just some debug function 
    void dumpHEX(uint8_t* values, uint16_t size);
    #endif

  private:
    
#if defined(ASYNCHRON_CARDS)
    // Internal receive functions
    int _receiveByte(uint8_t* buf, unsigned long timeout);
    uint8_t _receiveTSByte();
    void _receiveDataBits(uint8_t* buf, unsigned long startTime, uint8_t count=8);
    boolean _sendByte(uint8_t out);
#endif

    // Activate TDA8204T
    void _activateHW();

    /**
     * Init PINs and start CLK, if defined
     */
    void _init(frequency_t freq);

    void _timeOutoccured();

#if defined(SYNCHRON_CARDS)
    /**
     * Send bytes in sync mode
     */
    void _sendSyncBytes(uint8_t* buf, uint16_t buf_size);
    
    /**
     * Receive bytes is sync mode
     */
    uint16_t _receiveSyncBytes(uint8_t* buf, uint16_t buf_size);

    /**
     * Activate synchron card
     */
    uint16_t _activateSynchronCard(uint8_t* buf, uint16_t buf_size);
#endif

#if defined(ASYNCHRON_CARDS)
    /**
     * Send bytes in async mode
     */
    boolean _sendASyncBytes(uint8_t* buf, uint16_t buf_size);

    /**
     * Receive bytes is async mode
     */
    uint16_t _receiveASyncBytes(uint8_t* buf, uint16_t buf_size);

    /**
     * Activate asynchron card
     */
    uint16_t _activateASynchronCard(uint8_t* buf, uint16_t buf_size);

    /**
     * member to hold include TS in ATR value
     */
    boolean _includeTSinATR;
#endif

    #if defined(USE_DEBUG_PIN)
    // Toggle debug pin
    void _toggleDebugPin();
    #endif
    
    // PIN configuration
    uint8_t _io_in_pin;  // (INPUT/OUTPUT) : I/O pin to communication with Smart Card
    uint8_t _rstin_pin;  // (OUTPUT)       : LOW: reset active.  HIGH: inactive.
    uint8_t _cmdvcc_pin; // (OUTPUT)       : LOW: Activation sequence initiated HIGH: No activation in progress
    uint8_t _off_pin;    // (INPUT)        : HIGH: Card present LOW: No card present
    uint8_t _clk_pin;    // (OUTPUT)       : CLK signal for Smart Card (NOT_A_PIN, when no CLK generation)

    // Internal configuration
    boolean _ignoreParity;   // We don't care about screwd up parity from the smartcard
	boolean _off_invert;
    frequency_t _clkFrequency; // See CLK_* defines
    uint16_t _etu;
    uint16_t _initial_etu;
    uint8_t  _ocra1;
    SCLibVoidFuncPtr _timeOutCB;
    // Some timing constants
    unsigned long _guardTime;
    // Work Waiting Time (960 * D * Wi) etus
    unsigned long _wwt;
    // Maximum Waiting time (WWT + (D * 480)) etus
    unsigned long _max_wwt;
    
    // Internal state vars
    boolean  _activated;
    boolean  _high_active;
    boolean  _timeout;
    boolean  _synchronous;
};

#endif
