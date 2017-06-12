/*
* Code for the 3D printed Blooming Flower lamp by Jason "Ossum" Suter 
* 3D files: https://www.myminifactory.com/object/37752
* Website: www.ossum.co.za
* Facebook Page: www.facebook.com/ossumdesigns
* 
*
*Credit to Simon Merrett for his interrupt-based encoder code
*https://www.instructables.com/id/Improved-Arduino-Rotary-Encoder-Reading/
*
*Credit to Adafruit for their TiCoServo and NeoPixel Arduino Libraries
*/
#include <Adafruit_TiCoServo.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define pinNeopixels 4  //DI of first strip connected to this pin
#define numNeopixels 16 //we have two rings of 8

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(numNeopixels, pinNeopixels, NEO_GRB + NEO_KHZ800);

//encoder variables (rotary)
static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

//encoder variables (push button)
const int buttonPin = 7;    // the number of the pushbutton pin

int flowerGoalState = HIGH;         // HIGH = Open, LOW = Closed
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

//servo variables
Adafruit_TiCoServo petalServo; // create servo object to control a servo
static int servoPin = 10; //variable to store which pin is the signal pin
static int servoClosedMicros = 1800;
static int servoOpenMicros = 1400;
int pos = servoClosedMicros;

//petal animation timing variables
int frameDuration = 3000; //number of milliseconds for complete movement
int frameElapsed = 0; //how much of the frame has gone by, will range from 0 to frameDuration
unsigned long previousMillis = 0; //the last time we ran the position interpolation
unsigned long currentMillis = 0; //current time, we will update this continuosly
int interval = 0;

//petal animation status variables
int movementDirection = 0; //0 = stopped, 1 opening, -1 closing

//LED animation status variables
volatile float brightness = 0.0;

void setup() {
  //setting up the rotary encoder
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  Serial.begin(115200); // start the serial monitor link

  //setting up rotary encoder push button
  pinMode(buttonPin, INPUT_PULLUP);

  //setting up the neopixels
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  //setting up the servo
  petalServo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  
}

void PinA(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos -= 5; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos += 5; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, float brightness) {
  WheelPos = 255 - WheelPos;
  
  Serial.print("ratio:");
  Serial.println(brightness);
  
  if(WheelPos < 85) {
    return strip.Color(brightness*(255 - WheelPos * 3), 0, brightness*(WheelPos * 3));
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, brightness*(WheelPos * 3),brightness*( 255 - WheelPos * 3));
  }
  WheelPos -= 170;
  return strip.Color(brightness*(WheelPos * 3), brightness*(255 - WheelPos * 3), 0);
}


//apply colours from the wheel to the pixels
void setWheel(byte WheelPos, float brightness) {
  for(uint16_t i=0; i<strip.numPixels()/2; i++) {
    strip.setPixelColor(i, Wheel(WheelPos,brightness));
  }
  for(uint16_t i=strip.numPixels()/2; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, Wheel(128-WheelPos,brightness));
  }
  strip.show();
}




void updateFlower(){
  unsigned long currentMillis = millis();
  interval = currentMillis - previousMillis;
  previousMillis = currentMillis;

  frameElapsed += movementDirection*interval;
  float frameElapsedRatio = float(frameElapsed)/float(frameDuration);
  brightness = frameElapsedRatio;
  
  
  if (frameElapsed < 0) {
    movementDirection = 0;
    frameElapsed = 0;
    Serial.println("closed");
    brightness = 0.0;
    //analogWrite(ledPin,ledOffPWM);
    petalServo.detach();
  }
  if (frameElapsed > frameDuration) {
    movementDirection = 0;
    frameElapsed = frameDuration;
    Serial.println("opened");
    brightness = 1.0;
    petalServo.detach();
  }

  if (movementDirection != 0) {

  //determine new position/brightness by interpolation between endpoints
  //int newLedValue = (ledOffPWM + int(frameElapsedRatio*(ledOnPWM - ledOffPWM)));
  int newServoMicros = (servoClosedMicros + int(frameElapsedRatio*(servoOpenMicros - servoClosedMicros)));

  petalServo.write(newServoMicros);    
  }
  setWheel(encoderPos, brightness);


}

void loop(){
  if(oldEncPos != encoderPos) {
    Serial.println(encoderPos);
    updateFlower();
    oldEncPos = encoderPos;
  }

  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      
      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        if (movementDirection == 0) {
          flowerGoalState = !flowerGoalState;
          if (flowerGoalState == HIGH){
            movementDirection = 1;
            
          }
          else {
            movementDirection = -1;
          }
          Serial.print("movement direction:");
          Serial.println(movementDirection);
          petalServo.attach(servoPin); //we reattach each time the a movement is started
        }
        Serial.println("Push button pushed");
      }
    }
  }
 // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;
  
  updateFlower();

}


