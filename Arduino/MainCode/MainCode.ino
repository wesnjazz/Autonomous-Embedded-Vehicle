#include "DualMC33926MotorShield.h"


DualMC33926MotorShield md;



#define PRINT_DATA 600 // 10000 for print
#define SET_RIGHT_MOTOR 2 // 22400 for right motor, first digit from the left is the command, secend digit is the pos=1/neg=2 and the last 3 digits is the speed of the motor between 0-400
#define SET_LEFT_MOTOR 3 // 32400 for right motor, first digit from the left is the command, secend digit is the pos=1/neg=2 and the last 3 digits is the speed of the motor between 0-400
#define STOP_MOTORS 500 // 500 for stop the motors
#define SET_BOTH_MOTOR 1
#define RESET_COUNTER 700

const long serialBaund = 115200;
const unsigned long pingTimeout = 1000000; // microsecond

const int pingPin = 11;
volatile long enc_count_right = 0;
volatile long enc_count_left = 0;
int8_t lookup_table[] = {1, -1, -1, 1};
long distance = 0;

const unsigned long tickDebounceMillis = 5;
unsigned long lastLeftTickMillis = 0;
unsigned long lastRightTickMillis = 0;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(serialBaund);
  Serial.setTimeout(10);



  // all your normal setup code
  attachInterrupt(0, encoder_isr_right, CHANGE); // right pin 2 and pin 5
  attachInterrupt(1, encoder_isr_left, CHANGE); // left pin 3 and pin 6

  md.init();

}

void loop() {
  // put your main code here, to run repeatedly:
  parseReceivdData(readInt());

  /////////////////////////////////PING Start////////////////////////////////////
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:
  long duration;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH,pingTimeout);

  // convert the time into a distance
  distance = microsecondsToCentimeters(duration);

  //  Serial.print(duration);
  //  Serial.print(" time(microsecend), ");
  //  Serial.print(distance);
  //  Serial.print("cm");
  //  Serial.println();
  delay(10);

  /////////////////////////////////PING END////////////////////////////////////
}

float readInt() {
  if (Serial.available() > 0) {
    float receivedChar = Serial.parseFloat();
    //        newData = true;
    //        Serial.println(receivedChar);
    return receivedChar;
  }
  return 0;
}


void parseReceivdData(float data) {
  long dd = (long)(data);
  if (dd == 0) return;
  // Serial.println(dd);

  int code = 0;

  int rspeed = 0;
  int lspeed = 0;
  int rsign = 0;
  int lsign = 0;
  if (dd > 100000) {
    code = dd / 100000000;
    if (code == 1) {
      rsign = (dd % 100000000) / 10000000;
      rspeed = (dd / 10000) % 1000;
      if (rsign == 2)
        rspeed *= -1;

      lsign = (dd % 10000) / 1000;
      lspeed = dd  % 1000;
      if (lsign == 2)
        lspeed *= -1;
    } else {
      rsign = (dd % 10000) / 1000;
      rspeed = dd % 1000;
      if (rsign == 2)
        rspeed *= -1;
    }
//    Serial.print(rsign);
//    Serial.print(",");
//    Serial.print(rspeed);
//    Serial.print(",");
//    Serial.print(lsign);
//    Serial.print(",");
//    Serial.print(lspeed);
//    Serial.print(",");
//    Serial.println("");
  } else {
    code = data;
  }


  switch (code) {
    case PRINT_DATA:
      Serial.print(enc_count_right);
      Serial.print(",");
      Serial.print(enc_count_left);
      Serial.print(",");
      Serial.print(distance);
      Serial.println("");
//      delay(100);
      break;
    case SET_RIGHT_MOTOR:
      setRightMotorSpeed(rspeed);
      break;
    case SET_LEFT_MOTOR:
      setLeftMotorSpeed(rspeed);
      break;
    case SET_BOTH_MOTOR:
      setRightMotorSpeed(rspeed);
      setLeftMotorSpeed(lspeed);
      break;
    case STOP_MOTORS:
      setLeftMotorSpeed(0);
      setRightMotorSpeed(0);
      break;
    case RESET_COUNTER:
      resetCounter();
      break;
    default:
      break;
  }
}

void resetCounter() {
  enc_count_right = 0;
  enc_count_left = 0;
}

void setRightMotorSpeed(int speedd) {

  md.setM1Speed(speedd);
  stopIfFault();
  delay(2);
}

void setLeftMotorSpeed(int speedd) {
  md.setM2Speed(speedd);
  stopIfFault();
  delay(2);

}


void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while (1);
  }
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}


void encoder_isr_right() {
  unsigned long time = millis();
  if (time - lastRightTickMillis <= tickDebounceMillis) {
    return;
  }
  lastRightTickMillis = time;

  byte current = (PIND & 0b00100100) >> 2;

  if (current > 2) {
    current -= 0b1000;
    current += 0b0010;
  }

  enc_count_right = enc_count_right - lookup_table[current];

//        Serial.print("right enc_count = ");
//        Serial.println(enc_count_right);
}

void encoder_isr_left() {
  unsigned long time = millis();
  if (time - lastLeftTickMillis <= tickDebounceMillis) {
    return;
  }
  lastRightTickMillis = time;

  byte current = (PIND & 0b01001000) >> 3;

  if (current > 2) {
    current -= 0b1000;
    current += 0b0010;
  }

  enc_count_left = enc_count_left + lookup_table[current];
//        Serial.print("left enc_count = ");
//        Serial.println(enc_count_left);
}
