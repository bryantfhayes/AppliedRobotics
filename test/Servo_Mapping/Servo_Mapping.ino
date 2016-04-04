/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

// servo_1 = rod rotation (r)  700-2400 PIN(2)
// servo_2 = up/down (z)      1000-1600 PIN(3)
// servo_3 = forward/back (y) 1100-1700 PIN(4)
// servo_4 = main rotation (x) 500-2100 PIN(5)

#include <Servo.h>

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

Servo servo_x;
Servo servo_y;
Servo servo_z;
Servo servo_r;

int curr_x = 1460;
int curr_y = 1700;
int curr_z = 1000;
int curr_r = 1490;

void setup() {
  servo_x.attach(5);
  servo_y.attach(4);
  servo_z.attach(3);
  servo_r.attach(2);

  servo_x.writeMicroseconds(curr_x);
  servo_y.writeMicroseconds(curr_y);
  servo_z.writeMicroseconds(curr_z);
  servo_r.writeMicroseconds(curr_r);

  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

void loop() {
  if (stringComplete) {
    Serial.print("GOT NEW DATA: ");
    Serial.println(inputString);
    
    int commaIndex = inputString.indexOf(',');
    int secondCommaIndex = inputString.indexOf(',', commaIndex+1);
    int thirdCommaIndex = inputString.indexOf(',', secondCommaIndex+1);

    String firstValue = inputString.substring(0, commaIndex);
    String secondValue = inputString.substring(commaIndex+1, secondCommaIndex);
    String thirdValue = inputString.substring(secondCommaIndex+1, thirdCommaIndex);
    String fourthValue = inputString.substring(thirdCommaIndex+1);

    int x = firstValue.toInt();
    int y = secondValue.toInt();
    int z = thirdValue.toInt();
    int r = fourthValue.toInt();

    int done[4] = {0,0,0,0};
    int goal[4] = {x,y,z,r};
    int pos[4] = {curr_x, curr_y, curr_z, curr_r};
    Servo servos[4] = {servo_x, servo_y, servo_z, servo_r};
    while(1){  
      for(int i = 0; i < 4; i++){
        if(pos[i] < goal[i]){
          servos[i].writeMicroseconds(pos[i]++);
        } else if (pos[i] > goal[i]){
          servos[i].writeMicroseconds(pos[i]--);
        } else {
          done[i] = 1;
        }
      }
      if(done[0] && done[1] && done[2] && done[3]){
        // update current values
        curr_x = pos[0];
        curr_y = pos[1];
        curr_z = pos[2];
        curr_r = pos[3];
        break;
      }
      delay(1);
    }

/*
    if(curr_x != x){
      curr_x = x;
      //servo_x.writeMicroseconds(x);
    }
    if(curr_y != y){
      curr_y = y;
      //servo_y.writeMicroseconds(y);
    }
    if(curr_z != z){
      curr_z = z;
      //servo_z.writeMicroseconds(z);
    }
    if(curr_r != r){
      curr_r = r;
      //servo_r.writeMicroseconds(r);
    }
*/   
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
