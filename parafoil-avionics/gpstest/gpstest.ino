#include "TinyGPS.h"
#include <SoftwareSerial.h>

TinyGPS gps;
#define RXPIN 3
#define TXPIN 2
SoftwareSerial nss(7, 8);

void setup(){
  Serial.begin(9600);
}
void loop()
{
  Serial.println("s");
  while (nss.available())
  {
    int c = nss.read();
    if (gps.encode(c))
    {
      Serial.println(gps.satellites());
    }
  }
}
