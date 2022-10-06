#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);

void setup()
{
  lcd.begin();

  lcd.setCursor(0, 0);
  lcd.print("12345678901234567890");
  lcd.setCursor(0, 1);
  lcd.print("This is Line 2!");
  lcd.setCursor(0, 2);
  lcd.print("This is Line 3!!");
  lcd.setCursor(0, 3);
  lcd.print("This is Line 4!");

}


void loop()
{


}