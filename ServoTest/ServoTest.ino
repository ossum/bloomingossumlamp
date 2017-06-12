#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int servoPin = 19; //variable to store which pin is the signal pin
int servoClosed = 1200;
int servoOpen = 1200;
int pos = servoOpen;
int ledPin = 13; // pin of the onboard LED


void setup() {
  myservo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  pinMode(ledPin, OUTPUT);
}

void loop() {
  for (pos = servoClosed; pos <= servoOpen; pos += 10) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
    digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = servoOpen; pos >= servoClosed; pos -= 10) { // goes from 180 degrees to 0 degrees
    myservo.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
    digitalWrite(ledPin, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

