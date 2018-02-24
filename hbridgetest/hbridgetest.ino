#define enA 10
#define in1 11
#define in2 12





void setup() {
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  
}
void loop() {
  Serial.println('y');


   digitalWrite(in1, HIGH);
   digitalWrite(in2, LOW);
   digitalWrite(enA, 250); // Send PWM signal to L298N Enable pin

}
