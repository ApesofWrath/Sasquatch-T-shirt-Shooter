#include <SPI.h>

void setup(){}

unsigned long ret = 0;
char count = 0;

void loop()
{
  SPI.begin();
  for (;;)
  {
    while (!SS && count++ < 16)
    {
      ret = 0;
      while (!SCK);
      ret = ret << 1 | MOSI;
      while (SCK);
    }
    count = 0;
    analogWrite(15, 0xFF & (ret >> 4));
    analogWrite(16, 0xFF & (ret >> 2));
    analogWrite(17, 0xFF & (ret >> 0));
  }
}
