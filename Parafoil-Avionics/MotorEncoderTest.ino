

#include <Encoder.h>
Encoder myEnc(14, 15);
void setup() {
  Serial.begin(9600);
  pinMode(18,OUTPUT);
  pinMode(20,OUTPUT);
  pinMode(19,OUTPUT);
  // put your setup code here, to run once:

}
long oldPosition  = -999;
void loop() {
  digitalWrite(18,LOW);
  digitalWrite(20,HIGH);
  analogWrite(19,255);
  long newPosition = myEnc.read();
  Serial.println(newPosition);
//  if (newPosition != oldPosition) {
//    oldPosition = newPosition;
//    Serial.println(newPosition);
//  }
  if (newPosition > 10000 || newPosition < -10000){
    digitalWrite(18,HIGH);
    digitalWrite(20,LOW);
    delay(10);
    digitalWrite(18,LOW);
    digitalWrite(20,LOW);
    delay(500000);
  }
  // put your main code here, to run repeatedly:

}
