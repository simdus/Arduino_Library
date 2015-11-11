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

// Create SmartCardReader object for further use
SmartCardReader sc(SC_C7_IO, SC_C2_RST, SC_C1_VCC, SC_SWITCH_CARD_PRESENT, SC_C2_CLK, SC_SWITCH_CARD_PRESENT_INVERT);

void setup() {
  Serial.begin(9600);
}

//
// This test just waits for the ATR initially sen dby the smart card
//
void loop() {
  uint16_t atr_received = 0;

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

  uint8_t data[MAX_ATR_BYTES];

  // Just try to activate card 
  //
  // Gives reset sequence and wait for ATR to be send by smart card
  // also tries to figure out if card is async or sync. only if 
  // both types are active in SCLIb.h
  // (SYNCHRON_CARDS and ASYNCHRON_CARDS are defined in SCLib.h)
  atr_received = sc.activate(data, MAX_ATR_BYTES);
  if ( atr_received > 0) {
#if defined(SC_DEBUG)
    sc.dumpHEX(data, atr_received);
#else
    Serial.print(atr_received);
	Serial.println(" bytes ATR received from card.");
#endif
    Serial.println();
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


