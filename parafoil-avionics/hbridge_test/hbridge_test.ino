const int LOOPTIME = 3000;
int currentTime;
void setup() {
  // put your setup code here, to run once:]
  Serial.begin(9600);
  pinMode(A8, OUTPUT);
  pinMode(A9, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop() {
  
  analogWrite(A8, 75);
  analogWrite(A9, 0);
   currentTime = millis();
//  digitalWrite(3, HIGH);
//  digitalWrite(4, LOW);
//  delay(500);
  while  (currentTime <= millis() - LOOPTIME){ 
  }
  analogWrite(A8, -1);
  analogWrite(A9,-1);
  
  Serial.println("test");
  
}
