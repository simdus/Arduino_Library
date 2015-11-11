#include <Arduino.h>

#include <SCLib.h>

// If you are using a Arduino Mega compatible board you need to change the SC_C2_CLK to 11 as the TIMER1A
// is used for asynchronous clock generation (1MHz with just plain arduino code is no fun ;-) )
// and the SC_C1_VCC can be changed to any other "free" digital pin.
#define SC_C2_RST              7
#define SC_C1_VCC              11
#define SC_C7_IO               10
#define SC_C2_CLK              9

// Default behavior of the signal connected to SC_SWITCH_CARD_PRESENT is
// that the signal is HIGH, when card is present and LOW otherwise.
#define SC_SWITCH_CARD_PRESENT 8

// If the signal on PIN SC_SWITCH_CARD_PRESENT has an inverted
// characteristic (LOW card present, HIGH otherwise) this can be signaled
// via the following define. The present signal will be inverted by SL44x2 object.
#define SC_SWITCH_CARD_PRESENT_INVERT false

#if !defined(ASYNC_CARDS) && !defined(APDU_SUPPORT)
#error This example only works if ASYNC_CARDS and APDU_SUPPORT is enabled in SCLib.h
#endif

// Create SmartCardReader object for further use
SmartCardReader sc(SC_C7_IO, SC_C2_RST, SC_C1_VCC, SC_SWITCH_CARD_PRESENT, SC_C2_CLK, SC_SWITCH_CARD_PRESENT_INVERT);

void setup() {
  Serial.begin(9600);
}

//
// Just send basic select command to async. smart card (T=0)
//
void loop() {
  uint16_t bytes_received = 0;
  APDU_t   command;

  // If you want the old behaviour without the TS byte in th ATR, just set
  // includeTSinATR to false ..
  //#if defined(ASYNCHRON_CARDS)
  //sc.setIncludeTSinATR(false);
  //#endif
  
  Serial.println("Waiting for Smartcard");

  // Wait for card to be inserted into smart card slot
  while (!sc.cardInserted())
  ;

  Serial.println("Smartcard found");

  uint8_t data[255];

  // Just try to activate card 
  //
  // Gives reset sequence and wait for ATR to be send by smart card
  bytes_received = sc.activate(data, MAX_ATR_BYTES);
  if ( bytes_received > 0) {
    Serial.println("Received ATR ...");

#if defined(SC_DEBUG)
    sc.dumpHEX(data, bytes_received);
#else
    Serial.print(bytes_received);
	Serial.println(" bytes ATR received from card.");
#endif

    // We just use the T=0 byte transfer
    command.cla       = 0x00;
    command.ins       = 0xA4;
    command.p1        = 0x04;
    command.p2        = 0x00;
    command.data_buf  = data;
    command.data_size = 7;
    command.resp_size = 0x100;
    
    data[0] = 0xA0;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x04;
    data[5] = 0x10;
    data[6] = 0x10;
    
    Serial.println("SELECT FILE Command ...");
    // Send Select File Command ...
    uint16_t result = sc.sendAPDU(&command);
    
    Serial.print("Received ... ");
    Serial.println(result, HEX);

    // The return value starting with 0x61 means that the
    // command was processed succesfully (Might not always
    // be the case. Depends on the used card, but if you receive
    // a return value != 0, this means that the card reacted to
    // your request.    
    if (((result >> 8) & 0xFF) == 0x61) {
      uint16_t receive_size = result & 0xFF;
      Serial.println("GET RESPONSE Command ...");
      
      // Read Coammnd result via GET RESPONSE
      command.cla       = 0x00;
      command.ins       = 0xC0;
      command.p1        = 0x00;
      command.p2        = 0x00;
      command.data_buf  = data;
      command.data_size = receive_size;
      
      result = sc.sendAPDU(&command, false);
      
      Serial.print("Received ... ");
      Serial.println(result, HEX);

#if defined(SC_DEBUG)
      sc.dumpHEX(data, receive_size);
#else
    Serial.print(receive_size);
	Serial.println(" bytes as command response received from card.");
#endif

    }
    
  } else {
    Serial.println("Unable to identify card ... Please remove");
  }
  delay(2000);

  // Deactivate smart card slot (Turn of power etc)
  sc.deactivate();

  // Wait for card to be removed physicaly from slot
  while (sc.cardInserted())
  ;
}


