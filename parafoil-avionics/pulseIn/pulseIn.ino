unsigned long time;
unsigned long oldtime = 0;
int x;
int pin = 7;
unsigned long hpulse;

void setup(){
 Serial.begin(9600);
 pinMode(pin, INPUT);
}

void loop(){

 hpulse = pulseIn(pin, HIGH);
 Serial.println(hpulse);
 
// time = millis();
// x = (time - oldtime);
// if (x >= 997){       
//  Serial.print("x: ");
//  Serial.println(x);
//  Serial.print("PULSE: ");
//  Serial.println(hpulse);
// 
//  Serial.print("time: ");
//  Serial.println(time);
//    //prints time since program started
//  oldtime = time;
//  }
  
}

