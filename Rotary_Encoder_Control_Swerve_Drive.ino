/*
Date:  Dec 19, 2025
-----------------------------------------------------------------------------------------
Purpose:
This code allows a rotary encoder to control the heading of a swerve drive robot.

A (ProMicro or Leonardo) aduino controller will operate as a USB device connected to a 
FIRST Robots Drivers station.

The robot's heading will be controlled by the rotary encoder's angular position.

The rotary encoder has 20 detents for 360 degrees. (18 degrees per detent)
[ May need to consider an encoder with higher resolution]
-----------------------------------------------------------------------------------------
Operation:
When started, the encoders position will be set to zero.
Rotating Counter-Clockwise (CCW) the encoders position value will increase by 18.
Rotating Clockwise (CW) will decrement the encoder value.

The default operation of the joystick library has the joystick outputing a value between 
0 and 1024 to the driver station.
The joystick will be configured to output a range of -180 to +180
  Joystick.setXAxisRange(-180, 180);

The WPI library receives this as -1 to +1
The code will be configured to have the joystick -1 to +1 similar to a common gamepad or joystick.

With a joystick output of -1 to +1, the robot software will map this to -180 to + 180.
A PID controller will use this value to control the heading of the robot.

When the rotary encoder button is pressed, the encoder will reset it position to zero.

When the rotary encoder is rotated is increased past +/- 180, the encoder will negate the sign and decrement the counter.
For example:  Output should be like this:  +170, +175, +180, -175, -170 OR -170, -175, -180, 175, 170

-----------------------------------------------------------------------------------------
Implementation:
The code to read the rotary encoder uses the Arduino Interupt feature.  
This feature configures the arduino to use an "Interupt Service Routine" to detect changes in the encoders clock and data lines. 
When a transition occurs (low to high or high to low), the service routine captures current values and returns.
This approach is much more efficient than polling routines that read the current values of the lines looking for transitions.

The ISR also provides better debounce implementation.  The goal is to have a single count increment or decrement for each 
rotary switch detent action.


-----------------------------------------------------------------------------------------
Resources:

Interrupt service routing
https://github.com/gfvalvo/NewEncoder/tree/master


documentation for MHeironimus ArduinoJoystickLibrary 

https://github.com/MHeironimus/ArduinoJoystickLibrary/tree/version-2.0
https://mheironimus.blogspot.com/2015/11/arduino-joystick-library.html

Joystick.setXAxis(byte value)
Sets the X axis value. Range -127 to 127 (0 is center).

*/
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// OLED Display Libraries and instantiation
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define the screen width and height
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The reset pin can be set to -1 if it shares the Arduino reset pin
#define OLED_RESET -1
// Address 0x3C is common for 128x32 displays
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// USB Joystick Emulation instantiation
#include "Joystick.h"

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
                   0, 0,                  // Button Count, Hat Switch Count
                   true, false, false,    // X and Y Axis, No Z Axis
                   false, false, false,   // No Rx, Ry, Rz
                   false, false,          // No rudder/throttle
                   false, false, false);  // No accelerator/brake/steering

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


int heading = 0;
int lastButtonPress = 0;
float encoder_output = 0;
bool counting_up_direction = true;
bool new_count_to_process = false;

int last_display_update = 0;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// New Encoder library (uses Interrupts, include debounce)
#include "Arduino.h"
#include "NewEncoder.h"

void ESP_ISR callBack(NewEncoder *encPtr, const volatile NewEncoder::EncoderState *state, void *uPtr);

// See README for meaning of constructor arguments.
// Use FULL_PULSE for encoders that produce one complete quadrature pulse per detnet, such as: https://www.adafruit.com/product/377
// Use HALF_PULSE for endoders that produce one complete quadrature pulse for every two detents, such as: https://www.mouser.com/ProductDetail/alps/ec11e15244g1/?qs=YMSFtX0bdJDiV4LBO61anw==&countrycode=US&currencycode=USD
// https://github.com/gfvalvo/NewEncoder/tree/master
NewEncoder encoder(0, 7, -30000, 30000, 0, FULL_PULSE);  // Parameters: Clock_pin, Data_pin, Minimim value, Max Value, Initial Value, type)
// Encoder Pin Assignments and variable initialization
// const int CLK_PIN =  0;        // Connect to CLK/A pin
// const int DT_PIN  =  7;        // Connect to DT/B pin
const int Encoder_SW_PIN = 15;  // Connect to SW (button) pin
int Encoder_SW_value = 1;

int16_t prevEncoderValue;
volatile NewEncoder::EncoderState newState;
volatile bool newValue = false;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void setup() {
  Serial.begin(115200);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // OLED Display setup
  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }

  // Set up text parameters
  display.setTextSize(2);               // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);  // Draw white text

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // USB Joystick Emulation Setup
  Joystick.begin();
  Joystick.setXAxisRange(-180, 180);  // +/- 180 matches robot heading
                                      // Arrives at the FIRST Driver Station as +/- 1.

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // New Encoder Library

  NewEncoder::EncoderState state;
  // The data structure "state" consists of the value "count" and the enumerate value "EncoderClick"
  // Notes from "NewEncoder.h"
  // enum EncoderClick {	NoClick, DownClick, UpClick }

  // Serial.begin(115200);
  delay(1000);
  Serial.println("Starting");

  if (!encoder.begin()) {
    Serial.println("Encoder Failed to Start. Check pin assignments and available interrupts. Aborting.");
    while (1) {
      yield();
    }
  } else {
    encoder.getState(state);
    Serial.print("Encoder Successfully Started at value = ");
    prevEncoderValue = state.currentValue;
    Serial.println(prevEncoderValue);
  }
  encoder.attachCallback(callBack);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void loop() {

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // New Encoder
  int16_t currentValue;
  NewEncoder::EncoderClick currentClick;

  if (newValue) {  // "newValue is updated in the Interrupt, true means new data present"
    noInterrupts();
    currentValue = newState.currentValue;
    currentClick = newState.currentClick;
    newValue = false;
    interrupts();
    // Serial.print("Encoder: ");
    if (currentValue != prevEncoderValue) {  //  The library counting is backwards compared to robot heading (CCW = positive)
      // Serial.print(currentValue);
      if (currentValue < prevEncoderValue) {  // Increasing value
        new_count_to_process = true;
        counting_up_direction = true;
        // Serial.println("Rotating CCW increasing");
      } else {
        new_count_to_process = true;
        counting_up_direction = false;
        // Serial.println("Rotating CW decreasing");
      }
      prevEncoderValue = currentValue;
    } else {                   // This library incorporates upper and lower limits
      switch (currentClick) {  // They are not being used (Setting value very high 30,000).
        case NewEncoder::UpClick:
          Serial.println("at upper limit.");
          break;

        case NewEncoder::DownClick:
          Serial.println("at lower limit.");
          break;

        default:
          break;
      }
    }
  }
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // This code handles the transition when passing +/- 180
  // When the rotary encoder is rotated is increased past +/- 180, the encoder will negate the sign and decrement the heading.
  // For example:  Output should be like this:  +170, +175, +180, -175, -170
  //               OR like this:                -170, -175, -180,  175,  170

  if (new_count_to_process == true) {
    if (counting_up_direction == true) {
      heading = heading + 18;
      if (heading >= 180) {
        // heading = -179;
        heading = -180 - (heading - 180);  // Make heading in the 0 to -180 range with correct offset
        counting_up_direction = false;
      }
    } else {
      heading = heading - 18;
      if (heading <= -180) {
        // heading = 179;
        heading = 180 + (heading + 180);
        counting_up_direction = true;
      }
    }
    new_count_to_process = false;
    // Serial.print("Heading: ");
    // Serial.println(heading);
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //  When the built in encoder switch is pressed, the heading resets to zero
  //
  Encoder_SW_value = digitalRead(Encoder_SW_PIN);

  // Check button press (with debouncing)
  if (Encoder_SW_value == LOW) {            // Button pressed (active LOW)
    if (millis() - lastButtonPress > 50) {  // Debounce delay
      // Serial.println("Button Pressed!");
      heading = 0;  // Reset heading on button press   ???
      Serial.println("heading reset to 0");
      lastButtonPress = millis();
    }
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //   Send the encoder value

  encoder_output = heading;
  Joystick.setXAxis(encoder_output);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //  Display the encoder values on the OLED display every 500 millisedcond

  if ((millis() - last_display_update) > 500) {
    display.clearDisplay();   // Clear the display buffer for each new frame
    display.setCursor(0, 0);  // Start at top-left corner
    display.println("heading: ");
    display.println(encoder_output);  // Print the dynamic heading value
    display.display();                // Transfer the buffer contents to the screen
    last_display_update = millis();
  }
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void ESP_ISR callBack(NewEncoder *encPtr, const volatile NewEncoder::EncoderState *state, void *uPtr) {
  (void)encPtr;
  (void)uPtr;
  memcpy((void *)&newState, (void *)state, sizeof(NewEncoder::EncoderState));
  newValue = true;
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
