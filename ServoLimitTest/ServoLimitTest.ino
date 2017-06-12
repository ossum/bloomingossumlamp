/* Jason Suter 2014
* This example code is in the public domain.
*
* Simple Script to determine a servo's limits
*
* Connect the servo signal to "servoPin" and provide power
* Connect to the arduino with serial port (115200)
* press 'q' to go to minimum
* press 'w' to go to midpoint
* press 'e' to go to maximum
*/ 
#include <Servo.h> 


//pin details
int servoPin = 10;

static int minMicros = 1375; //1000 is absolute min
static int midMicros = min(minMicros,maxMicros)+abs(minMicros-maxMicros)/2;
static int maxMicros = 1800; //2000 is absolute max
 
Servo servoUnderTest;  // create servo object to control a servo 
 
int posMicros = 1500;    // variable to store the servo position 
int lastMicros = posMicros;
 
void setup() 
{ 
  servoUnderTest.attach(servoPin);
  
  //configure serial port  
  Serial.begin(115200);
} 
 
 
void loop() 
{ 

  if (Serial.available() > 0) {
    char inByte = Serial.read();; //incoming serial byte
    if (inByte == 'q') {
      posMicros = minMicros;
    }
    else if (inByte == 'w') {
      posMicros = midMicros;
    }
    else if (inByte == 'e') {
      posMicros = maxMicros;
    }        
    else if (inByte == 'o') {
      posMicros = max(posMicros-5,minMicros);
    }        
    else if (inByte == 'p') {
      posMicros = min(posMicros+5,maxMicros);
    }
}
    //report current position
    if (posMicros != lastMicros) {
      Serial.println(posMicros);
      lastMicros = posMicros;
    }
    
    servoUnderTest.write(posMicros);

} 
