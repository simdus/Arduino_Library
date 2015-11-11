
#include <LiquidCrystal_I2C.h>
#include <WProgram.h>
#include <Wire.h>
#include <DS1307.h>

// vytvoří objekt lcd a nastaví jeho adresu
// 0x20 a 16 zanků na 2 řádcích
LiquidCrystal_I2C lcd(0x20,16,2); 
// vytvoří pole čísel do kterých se bude ukládat čas
int rtc[7];

void setup()
{    
  lcd.init();// inicializuje displej   
  lcd.backlight(); // zapne podsvětlení
  lcd.setCursor(1,0); // nastaví kurzor na pozici
  lcd.print("Aktualni cas:"); // vypíše text
}
void loop()
{
  // zapíše data z modulu do pole čísel rtc
  RTC.get(rtc,true);  
  // připraví pole znaků pro čas
  char cas[9];  
  // zapíše do pole znaků cas hodnoty z rtc
  sprintf(cas, "%02d:%02d:%02d", rtc[2],rtc[1],rtc[0]);  
  lcd.setCursor(4,1); // nastaví kurzor na pozici
  lcd.print(cas); // vypíše cas na displej  
  delay(1000); //počká jednu vteřinu
}

