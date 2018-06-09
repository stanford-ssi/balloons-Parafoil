#include "Avionics.h"
#include "Log.h"
#include "Motor.h"
#include "Sensors.h"
#include "Constants.h"



int main(void){

  Avionics avionics;
  Encoder EncA(ENCODER_A_1, ENCODER_A_2);
  Encoder EncB(ENCODER_B_1, ENCODER_B_2);

  avionics.initialize(&EncA, &EncB);


  /* avionics.motors.setDirection(avionics.motors.CCW); */

  /* digitalWrite(MOTOR_A_DIR_1, HIGH); */
  /* digitalWrite(MOTOR_A_DIR_2, LOW); */

  /* digitalWrite(MOTOR_B_DIR_1, LOW); */
  /* digitalWrite(MOTOR_B_DIR_2, LOW); */

  /* while(true){ */
  /* analogWrite(MOTOR_A_SPEED, 100); */
  /* Serial.print("position "); */
  /* Serial.println(EncA.read()); */
  /* } */
  

  while(true){
    /* avionics.record(); */
    /* avionics.cutdown(); */
    Serial.println("break before fly");
    avionics.fly();
    avionics.smartSleep(50);
  }

  return 0;
}
