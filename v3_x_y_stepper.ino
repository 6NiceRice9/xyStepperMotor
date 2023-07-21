#include <Encoder.h>
#include <AccelStepper.h>


// main motor pins
#define MAIN_ENABLE 9
#define MAIN_STEP 2
#define MAIN_DIR 3

//slave motor pins
#define SLAVE_ENABLE 10
#define SLAVE_STEP 4
#define SLAVE_DIR 5

// Encoder pins
#define ENCODER_A 6
#define ENCODER_B 7


//test settings
#define stepsPerRevolution (200*64/360*1.8) //200 stepper motor steps * 64 stepper driver steps / 360° MULTIPLY angle that should be driven 180°
#define drivingSPEED 50

// Button
const int buttonPin = 8;  // the number of the pushbutton pin
int buttonState = 0;  // variable for reading the pushbutton status

// Include multistepper pins
AccelStepper Xaxis(AccelStepper::FULL2WIRE, MAIN_STEP, MAIN_DIR);
AccelStepper Yaxis(AccelStepper::FULL2WIRE, SLAVE_STEP, SLAVE_DIR);

// Initialize Rotary Encoder
Encoder EncoderValue(ENCODER_A, ENCODER_B);

// Keep track of encoder value
long oldEncoderValue  = 0;

void setup() 
{
  // Serial.begin(9600); // open the serial port at 9600 bps:
  // MAIN motor
  pinMode(MAIN_ENABLE, OUTPUT);
  digitalWrite(MAIN_ENABLE, HIGH); // Keep motor disabled by default
  
  // SLAVE motor
  pinMode(SLAVE_ENABLE, OUTPUT);
  digitalWrite(SLAVE_ENABLE, HIGH); // Keep motor disabled by default
  

  // Setup pull-up resistors for Encoder Pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // Button
  pinMode(buttonPin, INPUT);  // initialize the pushbutton pin as an input
}


void loop()
{
  int encoderValue = EncoderValue.read();
  buttonState = digitalRead(buttonPin); // read the state of the pushbutton

  /////////////// DEBUG
  ////// read if encoder has beed rotated
  //Serial.print(oldPosition);
  //Serial.print("\t");
  //Serial.println(newPosition);
  ////// read in which direction encoder has beed rotated
  // Serial.print(oldEncoderValue);
  //Serial.println("_________________________________________");
  ////// Button
  //Serial.println(buttonState);
  ///////////////
  if (((encoderValue != oldEncoderValue) & (buttonState == 0)))
  {
    oldEncoderValue = encoderValue;   // reset encoder counter
    digitalWrite(MAIN_ENABLE, LOW);  // activate stepper motor driver
    if (encoderValue < 0)
    {
      digitalWrite(MAIN_DIR, LOW);    // Set the spinning direction clockwise:
      for (int i = 0; i < stepsPerRevolution; i++)
      {         // These four lines result in 1 step:
        digitalWrite(MAIN_STEP, HIGH);
        delayMicroseconds(drivingSPEED);
        digitalWrite(MAIN_STEP, LOW);
        delayMicroseconds(drivingSPEED);
      }
    }
    else if (encoderValue > 0)
    {
      digitalWrite(MAIN_DIR, HIGH);    // Set the spinning direction counterClockwise:
      for (int i = 0; i < stepsPerRevolution; i++)
      {
        digitalWrite(MAIN_STEP, HIGH);
        delayMicroseconds(drivingSPEED);
        digitalWrite(MAIN_STEP, LOW);
        delayMicroseconds(drivingSPEED);
      }
    }
  }
    // below is the slave motor section (= pushButton == HIGH)
  else if (((encoderValue != oldEncoderValue) & (buttonState == 1)))
  {
    oldEncoderValue = encoderValue;
    digitalWrite(SLAVE_ENABLE, LOW);
      if (encoderValue < 0)
      {
        digitalWrite(SLAVE_DIR, LOW);    // Set the spinning direction clockwise:
        for (int i = 0; i < stepsPerRevolution; i++)
        {          // These four lines result in 1 step:
          digitalWrite(SLAVE_STEP, HIGH);
          delayMicroseconds(drivingSPEED);
          digitalWrite(SLAVE_STEP, LOW);
          delayMicroseconds(drivingSPEED);
        }
      }
    else if (encoderValue > 0)
      {
        digitalWrite(SLAVE_DIR, HIGH);    // Set the spinning direction counterClockwise:
        for (int i = 0; i < stepsPerRevolution; i++)
        {
          digitalWrite(SLAVE_STEP, HIGH);
          delayMicroseconds(drivingSPEED);
          digitalWrite(SLAVE_STEP, LOW);
          delayMicroseconds(drivingSPEED);
        }
    }
    else
    {
      // Do Nothing!
    }
  }
  EncoderValue.write(0);  // reset encoder value
  digitalWrite(MAIN_ENABLE, HIGH); // deactivate main stepper motor driver
  digitalWrite(SLAVE_ENABLE, HIGH); // deactivate slave stepper motor driver
  delay(2);
}



