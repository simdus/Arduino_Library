/*
  SCLib.cpp - Smart Card library
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

/******************************************************************************
 * Includes
 ******************************************************************************/

#include "SCLib.h"

/******************************************************************************
 * Definitions
 ******************************************************************************/

// STATES
enum sync_state_t { SYNC_START_STATE, SYNC_START_CONDITION, SYNC_START_FINISHED, SYNC_CLK_LOW, SYNC_CLK_HIGH, SYNC_PRE_STOP_CONDITION, SYNC_STOP_CONDITION, SYNC_STOP_FINISHED, SYNC_FINISHED };
enum state_t { START_STATE, FOUND_FIRST_FALLING_EDGE, FOUND_FIRST_RAISING_EDGE, SYNC_FOUND, READING_BITS, SENDING_BITS, PARITY_BIT, PARITY_ERROR, PARITY_READ, PRE_FINISHED, FINISHED };
enum apdu_t0_state_t { SEND_HEADER, WAIT_ANSWER, SEND_DATA, RECEIVE_DATA, SEND_RESPONSE_SIZE, ADPU_FINISHED };

/******************************************************************************
 * Constructors
 ******************************************************************************/
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
SmartCardReader::SmartCardReader(uint8_t c7_io, uint8_t c2_rst, uint8_t c1_vcc, uint8_t card_present, uint8_t c3_clk, boolean card_present_invert) {
  _io_in_pin  = c7_io;
  _rstin_pin  = c2_rst;
  _cmdvcc_pin = c1_vcc;
  _off_pin    = card_present;
  _clk_pin    = c3_clk;

  _guardTime    = 2 * DEFAULT_ETU;
  _ignoreParity = true;
  _timeOutCB    = NULL;
  _off_invert   = card_present_invert;

#if defined(ASYNCHRON_CARDS)
  _includeTSinATR = true;
#endif  
}

/******************************************************************************
 * User API
 ******************************************************************************/

//
// CARD INDEPENDENT FUNCTIONS
//
void SmartCardReader::_init(frequency_t freq) {
  pinMode(_off_pin, INPUT);
  pinMode(_cmdvcc_pin, OUTPUT);
  pinMode(_rstin_pin, OUTPUT);

  // Start with reading from Card
  pinMode(_io_in_pin, INPUT);

  // Default value for CMDVCCn and RSTIN
#if defined(USE_TDA8204)
  digitalWrite(_cmdvcc_pin, HIGH);
#else
  digitalWrite(_cmdvcc_pin, LOW);
#endif

  digitalWrite(_rstin_pin, LOW);

  // Enable Debug PIN Toggling
  #if defined(USE_DEBUG_PIN)
  pinMode(DEFAULT_DEBUG_PIN, OUTPUT);
  digitalWrite(DEFAULT_DEBUG_PIN, HIGH);
  #endif
  
  _synchronous = false;
  _clkFrequency = freq;
  switch(freq) {
#if defined(ASYNCHRON_CARDS)
    case CLK_2MHZ:
      _initial_etu  = 186;
      _guardTime    = 2 * _initial_etu;
      _ocra1        = 3;
      break;
    case CLK_2DOT5MHZ:
      _initial_etu  = 148;
      _guardTime    = 2 * _initial_etu;
      _ocra1        = 2;
      break;
    case CLK_4MHZ:
      _initial_etu  = 93;
      _guardTime    = 2 * _initial_etu;
      _ocra1        = 1;
      break;
    case CLK_1MHZ:
      _initial_etu  = DEFAULT_ETU;
      _guardTime    = 2 * _initial_etu;
      _ocra1        = 7;
      break;
#endif
#if defined(SYNCHRON_CARDS)
    case CLK_10KHZ:
      // SYNCHRONOUS TRANSMISSION
      _synchronous  = true;
      _initial_etu  = 50;
      _ocra1        = 0;
      break;
#endif
    default:
      // NO CLK generation ...
      _initial_etu  = 50;
      _ocra1        = 0;
      break;

  }      
      
  _etu          = _initial_etu;
  
  _activated = false;
  _high_active = true;
  
  // Set Clock to Output
  pinMode(_clk_pin, OUTPUT);
  
#if defined(ASYNCHRON_CARDS)
  // Do we have to generate CLK, by our own?
  if (_ocra1 > 0) {
    TCNT1=0;
    // Toggle OC1A on Compare Match
    TCCR1A = 0x00;
    bitSet(TCCR1A, COM1A0);
    // Clear Timer on Compare Match
    TCCR1B = 0x00;
    bitSet(TCCR1B, WGM12);
    // Set frequency (1 - 4MHz, 2 - 2.5Mhz, 3 - 2MHz / 7 - 1MHz)
    OCR1A = _ocra1;
    // No prescaling
    bitSet(TCCR1B, CS10);
  } else {
    // We need to stop a previous active clk
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    OCR1A = _ocra1;
	// Set default LOW CLK
	digitalWrite(_clk_pin, LOW);
  }
#endif
}

void SmartCardReader::setGuardTime(unsigned long t) {
  if (t != 0) {
    _guardTime = t;
  }
}

void SmartCardReader::ignoreParityErrors(boolean in) {
  _ignoreParity = in;
}

#if defined(ASYNCHRON_CARDS)
void SmartCardReader::setIncludeTSinATR(boolean include) {
  _includeTSinATR = include;
}

#if defined(APDU_SUPPORT)
uint8_t SmartCardReader::calcEDC(uint8_t startValue, uint8_t *buf, uint16_t size) {
  uint8_t xorvalue = startValue;
  for(size_t i=0; i<size; i++) {
    xorvalue ^= buf[i];
  }
  return xorvalue;
}

uint16_t SmartCardReader::sendAPDU(APDU_t* command, boolean send) {
  // Internal State Machine
  apdu_t0_state_t state = SEND_HEADER;
  
  // Create ADPU Command header
  uint8_t buf[] = { 0, 0, 0, 0, 0, 0, 0 };
  uint16_t bytes_count = 0;
  uint16_t count = 0;
  uint16_t result = 0;
  uint16_t bytes_received = 0;
  
  if(command != NULL) { 
    // Fill buf with provided data
    buf[0] = command->cla;
    buf[1] = command->ins;
    buf[2] = command->p1;
    buf[3] = command->p2;
    
    while(state!=ADPU_FINISHED) {
      switch(state) {
        case SEND_HEADER:
          // Setup APDU command header data field
          if (command->data_size == 0 || command->data_buf == NULL) {
            // FIXME: NEEDS TO BE CHECKED FOR RECEIVE
            bytes_count = 4;
          } else if (command->data_size <= 0xFF) {
            bytes_count = 5;
            buf[4] = command->data_size & 0xFF;
          } else {
            bytes_count = 7;
            buf[5] = (command->data_size >> 8) & 0xFF;
            buf[6] = command->data_size & 0xFF;
          }
          
          // Send Header to card
          state = (!sendBytes(buf, bytes_count)) ? WAIT_ANSWER : ADPU_FINISHED;
          bytes_count = 0;
          break;
        case WAIT_ANSWER:      
          // Wait for Smart Card response
          if ((bytes_received = receiveBytes(buf, 1)) > 0) {

            // We check the first byte
            if (buf[0] == 0x60) {
              // Card needs more time to complete task .. 
              // So wait for next Heartbeat
              
              // NOOP
			  break;
            } else if ( (buf[0] > 0x60 && buf[0] <= 0x6F) || (buf[0] >= 0x90 && buf[0] <= 0x9F) ) {
              // Card send SW1
              result = buf[0];
              result = result << 8;

              // Wait for Smart Card response
              if (receiveBytes(buf, 1) > 0) {
                result |= buf[0];
              } else {
                result = 0;
              }
              state = ADPU_FINISHED;
              break;
            } else if (buf[0] == command->ins) {
              // The card wants data block
              
              // Send all remaining data at once
              count = command->data_size - count;

              // Check if we want to send or receive
              if (send) {
                // Just give the card time to switch to receive mode
                delayMicroseconds(_guardTime);
                
                state = SEND_DATA;
              } else {
                // No more data to be received
                state = (count>0)?RECEIVE_DATA:WAIT_ANSWER;
              }
              break;
            } else if (buf[0] == command->ins^0xFF) {
              // We just send one byte and wait for further instructions
              count = 1;
              state = (send)?SEND_DATA:RECEIVE_DATA;
              break;
            } 
          }
          result = 0;
          state = ADPU_FINISHED;
          break;
        case SEND_DATA:      
          // Do we really have to send something
          if (count > 0 && bytes_count < command->data_size) {
            // If something went wrong, just only what's available
            count = ((count + bytes_count) > command->data_size)?command->data_size - bytes_count:count;
  
            if( !sendBytes(&command->data_buf[bytes_count], count) ) {
              bytes_count += count;
              // All data is send .. Send Length Field
              state = (bytes_count >= command->data_size)?SEND_RESPONSE_SIZE:WAIT_ANSWER;
              break;
            }
          } 
          // An error occurred ..
          result = 0;
          state = ADPU_FINISHED;
          break;
        case RECEIVE_DATA:      
          // Do we really have to receive something
          if (count > 0 && bytes_count < command->data_size) {
            // If something went wrong, just only what's available
            count = ((count + bytes_count) > command->data_size)?command->data_size - bytes_count:count;
  
            if( (bytes_received = receiveBytes(&command->data_buf[bytes_count], count) ) > 0) {
              bytes_count += bytes_received;
              // Wait for answer
              state = (bytes_received>=count)?WAIT_ANSWER:RECEIVE_DATA;
              count = count - bytes_received;  
              break;
            }
          } 
          // An error occurred ..
          result = 0;
          state = ADPU_FINISHED;
          break;
        case SEND_RESPONSE_SIZE:      
          if (command->resp_size > 0) {
            if (command->resp_size <= 0x100) {
              bytes_count = 1;
              buf[0] = command->resp_size & 0xFF;
            } else if (command->data_size == 0 || command->data_buf == NULL) {
              bytes_count = 2;
              buf[0] = (command->resp_size >> 8) & 0xFF;
              buf[1] = command->resp_size & 0xFF;
            } else {
              bytes_count = 3;
              buf[0] = 00;
              buf[1] = (command->resp_size >> 8) & 0xFF;
              buf[2] = command->resp_size & 0xFF;
            }
    
            state = (!sendBytes(buf, bytes_count))?WAIT_ANSWER:ADPU_FINISHED;
          } else {
            state = WAIT_ANSWER;
          }
          break;
        default:
          state = ADPU_FINISHED;
          result = 0;
          break;
      }
    }
  }

  // As we might be a little bit to "fast", just prevent the user from
  // accessing the card to early
  delayMicroseconds(_guardTime);
  
  return result;
}
//APDU_SUPPORT
#endif

//ASYNCHRON_CARDS
#endif

void SmartCardReader::setTimeOutCallback(SCLibVoidFuncPtr cb) {
  _timeOutCB = cb;
}

boolean SmartCardReader::cardInserted() {
	if (_off_invert)
	  return (digitalRead(_off_pin) == LOW);
	return (digitalRead(_off_pin) == HIGH);
}

boolean SmartCardReader::timeout() {
  return _timeout;
}

uint16_t SmartCardReader::getCurrentETU() {
  return _etu;
}


uint16_t SmartCardReader::activate(uint8_t* buf, uint16_t buf_size) {
  uint16_t bytes_received = 0;

  // Just for consistency reasons
  if (_activated) {
    deactivate();
  }
  
  //
  // Activation tries to find out which type of card is found
  //
  if(buf != NULL && buf_size > 0) {
#if defined (ASYNCHRON_CARDS)
    bytes_received = _activateASynchronCard(buf, buf_size);

    _activated = (bytes_received > 0);
#endif
#if defined(SYNCHRON_CARDS)
    if (!_activated) {
      // Reset Card and try again as synchronous
      deactivate();

      bytes_received = _activateSynchronCard(buf, buf_size);

      _activated = (bytes_received > 0);
    }
#endif
  }

  return (_activated)?bytes_received:0;
}

uint16_t SmartCardReader::receiveBytes(uint8_t* buf, uint16_t buf_size) {
  uint16_t result = 0;
  if (buf != NULL && buf_size > 0) {
    if (_synchronous) {
#if defined(SYNCHRON_CARDS)
      result = _receiveSyncBytes(buf, buf_size);
#endif
    } else {
#if defined(ASYNCHRON_CARDS)
      result = _receiveASyncBytes(buf, buf_size);
#endif
    }
  }
  return result;
}

boolean SmartCardReader::sendBytes(uint8_t* buf, uint16_t count) {
  boolean result = false;
  if (buf != NULL && count > 0) {
    if (_synchronous) {
#if defined(SYNCHRON_CARDS)
      _sendSyncBytes(buf, count);
      result = true;
#endif
    } else {
#if defined(ASYNCHRON_CARDS)
      result = _sendASyncBytes(buf, count);
#endif
    }
  }
  return result;
}

void SmartCardReader::deactivate() {
  // Turn off power
#if defined(USE_TDA8204)  
  digitalWrite(_cmdvcc_pin, HIGH);
#else
  digitalWrite(_cmdvcc_pin, LOW);
#endif
  // Debug
  #if defined(USE_DEBUG_PIN)
  digitalWrite(DEFAULT_DEBUG_PIN, HIGH);
  #endif

  // Turn of CLK generation
  _init(CLK_NO_CLK);

  // Remove any previous found values
  _etu = _initial_etu;
  _activated = false;
  _high_active = true;
  _timeout = false;
}

/******************************************************************************
 * Private section
 ******************************************************************************/

//
// ASYNCHRONOUS FUNCTIONS
//

#if defined(ASYNCHRON_CARDS)
uint16_t SmartCardReader::_activateASynchronCard(uint8_t* buf, uint16_t buf_size) {
  uint16_t bytes_received = 0;
  uint8_t ts = 0;

  if(buf != NULL && buf_size > 0) {
    _init(CLK_1MHZ);

    _activateHW();

    // Read TS
    ts = _receiveTSByte();
    // try to read more
    if(ts != 0) {
      if (_includeTSinATR) {
        buf[bytes_received++] = ts;
      }
      bytes_received += receiveBytes(&buf[bytes_received], buf_size-bytes_received);
    }
  }

  return bytes_received;
}

uint16_t SmartCardReader::_receiveASyncBytes(uint8_t* buf, uint16_t buf_size) {
  uint16_t bytes_received = 0;
  _timeout = false;

  if (buf != NULL && buf_size > 0) {
    while (cardInserted() && !_timeout && bytes_received < buf_size) {
      if (_receiveByte(buf+bytes_received, _max_wwt) != 0) {
        // An error occured, while receiving ATR
        break;
      }
      bytes_received++;
    }
  }
  return bytes_received;
}

boolean SmartCardReader::_sendASyncBytes(uint8_t* buf, uint16_t count) {
  boolean parityError = false;
  if (buf != NULL && count > 0) {
    for(uint16_t i=0; i < count && cardInserted(); i++) {
      // _sendByte returns false in case parity error occured 
      if (!_sendByte(buf[i]) && !_ignoreParity) {
        delayMicroseconds(_guardTime);
        _sendByte(buf[i]);
        parityError = true;
      }
      if (i < count -1) {
        // Only wait between bytes and not at the end
        delayMicroseconds(_guardTime);
      }
      // Debug
      #if defined(USE_DEBUG_PIN)
      _toggleDebugPin();
      #endif
    }
  }
  return parityError;
}  

boolean SmartCardReader::_sendByte(uint8_t out) {
  uint8_t state = START_STATE;
  unsigned long nextBit = 0;
  boolean currentBit = false;
  boolean parity = false;
  boolean parityErrorSignaled = false;
  
  // 8 Data bits + parity
  uint8_t bits_left = 9;
  
  while (cardInserted() && state != FINISHED) {
    switch(state) {
      case START_STATE:
        // Change _io_in_pin to OUTPUT
        pinMode(_io_in_pin, OUTPUT);

        digitalWrite(_io_in_pin, LOW);

        // Debug
        #if defined(USE_DEBUG_PIN)
        _toggleDebugPin();
        #endif

        nextBit = micros() + _etu;
        state=SENDING_BITS;
        break;
      case SENDING_BITS:
        if (micros() >= nextBit) {
          currentBit = bitRead(out, 9 - bits_left);
          
          if (currentBit) {
            parity = !parity;
          }  
        
          digitalWrite(_io_in_pin, ((currentBit)?HIGH:LOW));
          
          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          nextBit += _etu;
          bits_left--;
          
          if (1 >= bits_left) {
            state = PARITY_BIT;
          }
        }
        break;
      case PARITY_BIT:
        if (micros() >= nextBit) {
          // Write Parity Bit
          digitalWrite(_io_in_pin, ((parity)?HIGH:LOW));

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif
          
          // Create Parity Bit          
          nextBit += _etu;
          state = PARITY_ERROR;
        }
        break;
      case PARITY_ERROR:
        if (micros() >= nextBit) {
          // Wait for parity signal from card
          digitalWrite(_io_in_pin, HIGH);
          pinMode(_io_in_pin, INPUT);
                 
          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          // Read 
          nextBit+= _etu/2;          
          state=PARITY_READ;
        }
        break;
      case PARITY_READ:
        if (micros() >= nextBit) {
          parityErrorSignaled = (digitalRead(_io_in_pin) != HIGH);

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          // Stop Sending Bity
          state=FINISHED;
        }
        break;
      default:
        state = FINISHED;
        break;
    }
  }
  return !parityErrorSignaled;
}   


int SmartCardReader::_receiveByte(uint8_t* buf, unsigned long timeout) {
  boolean startbit_found = false;
  
  // Calculate maximum wait time 
  unsigned long     endTime = micros() + timeout;
  int                result = 0;
  
  unsigned long nextBitTime = 0;
  
  // Wait for start bit
  while (cardInserted() && !startbit_found) {
    // First check if timeout occured
    if (micros() >= endTime) {
      // Timeout
      _timeOutoccured();
      result = -1;
      break;
    }
    // We are waiting for the falling edge of the start bit
    if (digitalRead(_io_in_pin) != HIGH) {
      // found it
      startbit_found = true;

      // Debug
      #if defined(USE_DEBUG_PIN)
      _toggleDebugPin();
      #endif

      // Set time for first bit
      nextBitTime = micros() + _etu + _etu / 2;
    }
  }
  
  if (startbit_found)
    _receiveDataBits(buf, nextBitTime, 8);
    
  return result;
}

void SmartCardReader::_receiveDataBits(uint8_t* buf, unsigned long startTime, uint8_t count) {
  // We also need to read the parity bit ;-.)
  uint8_t bits_left = count + 1;
  unsigned long nextBitTime = startTime;
  boolean parity = true;
  uint8_t state = READING_BITS;
  
  // Init buffer
  *buf = 0;

  while (cardInserted() && state != FINISHED && bits_left > 0) {
    switch(state) {
      case READING_BITS:
        if(micros() >= nextBitTime) {
          *buf = *buf >> 1;
          if (digitalRead(_io_in_pin) != LOW) {
            *buf |= 0x80;
            parity = !parity;
          } else {
            // Be sure to delete highest bit
            *buf &= 0x7F;
          }
          bits_left--;
        
          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          // Next is the Parity Bit
          if (bits_left <= 1) {
            state = PARITY_BIT;
          }          
          // Set arrival time for next data bit
          nextBitTime += _etu;
        }
        break;
      case PARITY_BIT:
        // Wait until next bit arrives
        if(micros() >= nextBitTime) {
          // Signal Parity Error, if requested
          if (!_ignoreParity && \
                ((parity && digitalRead(_io_in_pin) != HIGH) || \
                 (!parity && digitalRead(_io_in_pin) != LOW)) ) {
            state = PARITY_ERROR;
            nextBitTime += _etu / 2;
          } else {
            nextBitTime += _etu + _etu / 2;
            state = PRE_FINISHED;
          }

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif
        }          
        break;
      case PARITY_ERROR:
        // Wait until parity error window arrives
        if(micros() >= nextBitTime) {
          if (!_ignoreParity) {
            // Signal Parity Error
            pinMode(_io_in_pin, OUTPUT);
            digitalWrite(_io_in_pin, LOW);
            delayMicroseconds(_etu);
            digitalWrite(_io_in_pin, LOW);
            pinMode(_io_in_pin, INPUT);
          } else {          
            // For now we just use the debug pin
            // Debug
            #if defined(USE_DEBUG_PIN)
            _toggleDebugPin();
            delayMicroseconds(_etu);
            _toggleDebugPin();
            #endif
          }
          nextBitTime += _etu / 2;
          state = PRE_FINISHED;
        }
        break;
      case PRE_FINISHED:
        // Just wait time to fly by
        if (micros() >= nextBitTime) {
          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          state = FINISHED;
        }
        break;
      default:
        // Just be sure to bail out of while loop
        state = FINISHED;
        break;
    }
  }
}

uint8_t SmartCardReader::_receiveTSByte() {
  unsigned long start = 0;
  // Wait max 40.000 * etu
  unsigned long endTime = micros() + MAX_WAIT_TIME;
  // Init receiving state machine
  int state = START_STATE;
  
  // As we know TS, we preset bits_left and result, with predefined values
  int      bits_left = 6;
  uint8_t  result    = 0;
    
  // timeout did not occur and CARD still inserted
  while (cardInserted() && state != FINISHED && bits_left) {
    // Check for timeout
    if(micros() >= endTime) {
      result = 0;
      _timeOutoccured();
      break;
    }  
     switch (state) {
       case START_STATE:
         if (digitalRead(_io_in_pin) != HIGH) {
           start = micros();
           state = FOUND_FIRST_FALLING_EDGE;

           // Debug
           #if defined(USE_DEBUG_PIN)
           _toggleDebugPin();
           #endif
         }
         break;
       case FOUND_FIRST_FALLING_EDGE:
         if (digitalRead(_io_in_pin) != LOW) {
           state = FOUND_FIRST_RAISING_EDGE;

           // Debug
           #if defined(USE_DEBUG_PIN)
           _toggleDebugPin();
           #endif
         }
         break;
       case FOUND_FIRST_RAISING_EDGE:
         if (digitalRead(_io_in_pin) != HIGH) {
           // Calculate _etu
           _etu = ((micros() - start) / 3);
           // Calculate initial _guardtime (Wi * etu) (Wi = 2)
           _guardTime = 2 * _etu;
           // 960 * D * Wi ( D = 1, Wi = 2)
           _wwt       = 1920 * _etu; 
           // WWT + D * 480 etus ( D = 1 )
           _max_wwt   = _wwt + 480 * _etu; 
           
           state = SYNC_FOUND;

           // Debug
           #if defined(USE_DEBUG_PIN)
           _toggleDebugPin();
           #endif
         }
         break;
       case SYNC_FOUND:
         // Wait half etu, to read values in the middle of the etu
         _receiveDataBits(&result, micros() + _etu/2, bits_left);

         // Add predefined ATR BITS (First 2 are always high) 
         // Used for ETU calculation
         result |= 0x3;
         state = FINISHED;     
       default:
         break;
     }
  }

  return result;
}
#endif

//
// SYNCHRONOUS FUNCTIONS
//

#if defined(SYNCHRON_CARDS)

uint16_t SmartCardReader::_activateSynchronCard(uint8_t* buf, uint16_t buf_size) {
  uint16_t bytes_received = 0;

  if(buf != NULL && buf_size > 0) {
    uint8_t atr_buffer[4];
    uint16_t received = -1;
    _init(CLK_10KHZ);

    _activateHW();

    receiveBytes(atr_buffer, 4);

    // Copy data into buf
    for(received=0; received<4 && received < buf_size; received++) {
      if (atr_buffer[received] == 0xFF) {
        bytes_received = 0;
        break;
      }
      buf[received] = atr_buffer[received];
      bytes_received = received + 1;
    }
  }

  return bytes_received;
}

uint16_t SmartCardReader::_receiveSyncBytes(uint8_t* buf, uint16_t buf_size) {
  uint16_t bytes_received = 0;
  uint8_t received = 0;
  uint8_t bit_pos = 0;
  unsigned long nextChange = micros();

  if (buf != NULL && buf_size > 0) {
    sync_state_t state = SYNC_START_STATE;
    while (cardInserted() && state != SYNC_FINISHED) {
      switch(state) {
      case SYNC_START_STATE:
        if (micros() >= nextChange) {
          bit_pos = 0;
          received = 0;

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          state=SYNC_CLK_LOW;
          nextChange = micros();
        }
        break;
      case SYNC_CLK_LOW:
        if (micros() >= nextChange) {
          digitalWrite(_clk_pin, LOW);

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          nextChange += _etu;
          state = SYNC_CLK_HIGH;
        }
        break;
      case SYNC_CLK_HIGH:
        if (micros() >= nextChange) {
          // Data will be set on IO
          digitalWrite(_clk_pin, HIGH);

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          nextChange += _etu/2;
          state = SYNC_PRE_STOP_CONDITION;
        }
        break;
      case SYNC_PRE_STOP_CONDITION:
        if (micros() >= nextChange) {
          received = received >> 1;
          if (digitalRead(_io_in_pin) != LOW) {
            received |= 0x80;
          } else {
            received &=0x7F;
          }
          bit_pos++;

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          nextChange += _etu/2;
          state = SYNC_CLK_LOW;

          if (bit_pos>=8) {
            // Byte is complete
            buf[bytes_received++] = received;
            // Maximum Bytes received?
            if (bytes_received>=buf_size) {
              state = SYNC_STOP_FINISHED;
            } else {
              state = SYNC_START_STATE;
            }
          }
        }
        break;
      case SYNC_STOP_FINISHED:
        if (micros() >= nextChange) {

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          state = SYNC_FINISHED;
        }
        break;
      default:
        state = SYNC_FINISHED;
        break;
      }
    }
  }
  return bytes_received;
}

void SmartCardReader::_sendSyncBytes(uint8_t* buf, uint16_t buf_size) {
  sync_state_t state = SYNC_START_STATE;
  uint16_t pos = 0;
  uint8_t bit_pos = 0;
  unsigned long nextChange = 0;
  if (buf != NULL && buf_size > 0) {
    while(cardInserted() && state != SYNC_FINISHED) {
      switch(state) {
      case SYNC_START_STATE:
        // Setup send conditions
        digitalWrite(_clk_pin, LOW);

        // IO should be HIGH
        pinMode(_io_in_pin, OUTPUT);
        digitalWrite(_io_in_pin, HIGH);

        // Debug
        #if defined(USE_DEBUG_PIN)
        _toggleDebugPin();
        #endif

        // Activate next state
        nextChange = micros() + _etu;
        state = SYNC_START_CONDITION;
        break;
      case SYNC_START_CONDITION:
        if (micros() >= nextChange) {
          digitalWrite(_clk_pin, HIGH);

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          nextChange += _etu/2;
          state = SYNC_START_FINISHED;
        }
        break;
      case SYNC_START_FINISHED:
        if (micros() >= nextChange) {
          digitalWrite(_io_in_pin, LOW);

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          nextChange += _etu/2;
          state = SYNC_CLK_LOW;
        }
        break;
      case SYNC_CLK_LOW:
        if (micros() >= nextChange) {
          // Set CLK Pulse LOW
          digitalWrite(_clk_pin, LOW);
          // Set IO Value
          digitalWrite(_io_in_pin, (bitRead(buf[pos], bit_pos++) > 0)?HIGH:LOW);

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          nextChange += _etu;
          state = SYNC_CLK_HIGH;
        }
        break;
      case SYNC_CLK_HIGH:
        if (micros() >= nextChange) {
          // Set CLK Pulse HIGH
          digitalWrite(_clk_pin, HIGH);

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          // Use next byte
          if (bit_pos >= 8) {
            pos++;
            bit_pos = 0;
          }

          nextChange += _etu;
          state = (pos >= buf_size)?SYNC_PRE_STOP_CONDITION:SYNC_CLK_LOW;
        }
        break;
      case SYNC_PRE_STOP_CONDITION:
        if (micros() >= nextChange) {
          // Set CLK Pulse LOW
          digitalWrite(_clk_pin, LOW);
          // Set IO Value to LOW
          digitalWrite(_io_in_pin, LOW);

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          nextChange += _etu;
          state = SYNC_STOP_CONDITION;
        }
        break;
      case SYNC_STOP_CONDITION:
        if (micros() >= nextChange) {
          // Set CLK Pulse HIGH
          digitalWrite(_clk_pin, HIGH);

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          nextChange += _etu;
          state = SYNC_STOP_FINISHED;
        }
        break;
      case SYNC_STOP_FINISHED:
        if (micros() >= nextChange) {
          // Set IO HIGH, while CLK HIGH
          pinMode(_io_in_pin, INPUT);
          digitalWrite(_io_in_pin, HIGH);

          // Debug
          #if defined(USE_DEBUG_PIN)
          _toggleDebugPin();
          #endif

          state = SYNC_FINISHED;
        }
        break;
      case SYNC_FINISHED:
      default:
        state = SYNC_FINISHED;
        break;
      }
    }
  }
}

/**
 * Wait for sychron card to complete processing.
 *
 * maxClkCycle : Maximum number of cycle card needs to complete process (Use 0 for unlimited)
 * startValue  : Value clk should start with
 *
 * @return number of clk cycles until, processing ended - 0 in case of an error
 */
uint16_t SmartCardReader::waitForProcessingCompletion(uint16_t maxClkCycle, uint8_t startValue) {
	if (cardInserted()) {
		unsigned long nextChange = 0;
		// Set start value ...
		digitalWrite(_clk_pin, startValue);

		for(uint16_t i=0, nextChange = micros() + _etu; cardInserted() && (i < maxClkCycle || maxClkCycle == 0); i++ ) {
			// 2 x toggle -> 1 clk
			// Wait for next toggle ..
			while (cardInserted() && micros() < nextChange)
				;
			// Just toggle
			*portInputRegister(digitalPinToPort(_clk_pin)) = digitalPinToBitMask(_clk_pin);

			nextChange = micros() + _etu;

			// Wait for next toggle ..
			while (cardInserted() && micros() < nextChange)
				;

			// Just toggle
			*portInputRegister(digitalPinToPort(_clk_pin)) = digitalPinToBitMask(_clk_pin);

			nextChange = micros() + _etu;

			if (digitalRead(_io_in_pin) == HIGH) {
				return i;
			}
		}
	}
	return 0;
}

#endif

//
// CARD INDIPENDENT FUNCTIONS
//

#if defined(USE_DEBUG_PIN)
void SmartCardReader::_toggleDebugPin() {
  // Debug
  //PINB = _BV(PINB7);
  // Use a more portable version
  *portInputRegister(digitalPinToPort(DEFAULT_DEBUG_PIN)) = digitalPinToBitMask(DEFAULT_DEBUG_PIN);
}
#endif

#if defined(SC_DEBUG)
void SmartCardReader::dumpHEX(uint8_t* values, uint16_t size) {
  if (values != NULL && size > 0) {
    char ascii[17];
    for(uint16_t row=0; row<(size + 15)/16; row++) {
      // Print Adress
      if (row==0)
        Serial.print("0");
      Serial.print(row * 16, HEX);
      Serial.print("|");

      // Prefill ascii
      for(int i=0; i<16; i++)
        ascii[i] = '.';
      ascii[16] = (char)0x00;
      // colums
      for(uint16_t pos=row*16; pos<(row + 1) * 16; pos++ ) {
        if(pos < size) {
          if(values[pos] < 0x10)
            Serial.print("0");
          Serial.print(values[pos], HEX);
          if(isPrintable(values[pos]))
            ascii[pos - row*16] = (char)values[pos];
        } else {
          Serial.print("  ");
        }
        Serial.print(" ");
      }
      Serial.print("'");
      Serial.print(ascii);
      Serial.println("'");
    }
  }
}
#endif

void SmartCardReader::_activateHW() {
  // Different procedures needed for synchronous and asynchronous

#if defined(SYNCHRON_CARDS)
  // Just so that we can start with a normal clk pulse
  if (_synchronous) {
    digitalWrite(_clk_pin, LOW);
  }
#endif

  // Start activate sequence (RSTIN high)
  digitalWrite( _rstin_pin, HIGH );

  // Wait some time
  delayMicroseconds(100);

  // Start activate sequence (RSTIN high)
#if defined(USE_TDA8204)
  digitalWrite(_cmdvcc_pin, LOW);
#else
  digitalWrite(_cmdvcc_pin, HIGH);
#endif
  // Wait at least t3
  delayMicroseconds(100);

  // Revert RST Signal
  digitalWrite( _rstin_pin, LOW );

  // Get over t5
  delayMicroseconds(150);

  // This should trigger a ATR ...
  digitalWrite( _rstin_pin, HIGH );

#if defined(SYNCHRON_CARDS)
  if (_synchronous) {
    // Create CLK pulse, while reset active
    delayMicroseconds(_etu/2);
    digitalWrite( _clk_pin, HIGH );
    delayMicroseconds(_etu);
    digitalWrite( _clk_pin, LOW );
    delayMicroseconds(_etu);

    // Revert RST Signal
    digitalWrite( _rstin_pin, LOW );
  }
#endif
}

void SmartCardReader::_timeOutoccured() {
  _timeout = true;
  if (_timeOutCB != NULL) {
    _timeOutCB();
  }
}  

