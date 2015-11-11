#include <WProgram.h>
#include <Wire.h>
#include <DS1307.h>

// vytvoří pole čísel do kterých se bude ukládat čas
int rtc[7];

void setup()
{    
  // nastaví seriovou komunakaci na 9600
  Serial.begin(9600);
}
void loop()
{
  // zapíše data z modulu do pole čísel rtc
  RTC.get(rtc,true);  
  // připraví pole znaků pro čas
  char cas[9];  
  // zapíše do pole znaků cas hodnoty z rtc
  sprintf(cas, "%02d:%02d:%02d", rtc[2],rtc[1],rtc[0]);  
  Serial.println(cas); // odesle čas na ser. port
  delay(1000); //počká jednu vteřinu
}

