/*
  Integrated Pressure/Flow Sensor and Pump Control Using Serial Port and I^2C

      The program integrates two sensors and pumps, namely a water pressure sensor and a flow sensor.
  The flow rate and pressure can be read through a personal computer, and the pump speed percentage (PWM)
  can be set through the serial port.

  - Water Pressure Sensor (DFRobot SEN0257)
      The output voltage of the sensor is converted by ADC to obtain the water pressure.

  - Compact Electromagnetic Flow Sensor (Aichi Tokei VN05)
      Use interrupts to calculate the number of unit pulses per interval to convert the water flow rate (mL/minute).

  - Kamoer KDS Peristaltic Pump (KDS-FE-2-S17B)
      12V DC brush motor

  Created 2 Nov. 2020
  by Yi-Xuan Wang

  References:
  https://wiki.dfrobot.com/Gravity__Water_Pressure_Sensor_SKU__SEN0257
  https://www.aichitokei.net/products/compact-electromagnetic-flow-sensor-vn/
  http://kamoer.com/Products/showproduct.php?id=234
*/

/*--- Preprocessor ---*/
#include <Wire.h>   // Import Wire library for Inter-Integrated Circuit (I^2C)

#define MAIN_CTRL 1 // Address of main controller w/ sensor
#define PUMP_CTRL 2 // Address of pump controller

#define flowPin 2   // Pin location of the sensor (D2) w/ interrupt (INT.1)
#define sigPin A0   // Potentiometer signal pin w/ ADC

#define N 800       // Measurment sampling number for smoothing

/*--- Constants ---*/
const unsigned long baudSpeed = 115200;           // Sets the data rate in bits per second (baud) for serial data transmission
const unsigned long period = 1000;                // The value is a number of milliseconds

const byte vIn = 5;                                   // Supply voltage from Arduino
const byte resBits = 10;                              // Resolution of ADC (10 bits)
const float vConv = vIn / (pow(2.0, resBits) - 1.0);  // Voltage of ADC level (2^bits)

// Spec. of water pressure sensor, Range: 0 - 16 MPa, Output: 0.5 - 4.5 V
const float pgMax = 16.0;             // Upper limit of pressure sensor
const float pgMin = 0.0;              // Lower limit of pressure sensor
const float pgVmax = 4.5;             // Maximum output voltage of pressure sensor
const float pgVmin = 0.5;             // Minimum output voltage of pressure sensor
const float offSet = 0.471772766113;  // Norminal value is 0.5 V

/*--- Global Variables ---*/
unsigned long startTime;            // Start time
unsigned long currentTime;          // Current time
unsigned long timer;                // Stopwatch

byte percent;                       // Percentage of pump PWM

float vOut;                         // Output of the ADC
float waterPres;                    // Value of water pressure

volatile unsigned long pulseCount;  // Measuring the falling edges of the signal
static unsigned long cumCount;      // Cumulative count
float flowRate;                     // Value of water flow rate
float flowML;                       // Unit converter (milliliter, mL)
float totalML;                      // Volume of cumulative water

/*--- Function Prototype ---*/
void getCounter(void);
float getwaterPres(float );
void waterPressure(byte );
void serialEvent(void);
void setup(void);
void loop(void);

/*--- Functions Definition ---*/
// Interrupt Service Routine (ISR) for Flow Sensor
void getCounter(void) {         
  pulseCount = pulseCount + 1;  // Every falling edge of the sensor signals to increment the pulse count
}

// Implementation of Water Pressure Calculation
float getwaterPres(float volt) {
    return ((volt - offSet) * ((pgMax - pgMin) / (pgVmax - pgVmin))) + pgMin;
}

// Water Pressure Sensor
void waterPressure(byte signalPin) {
    for (unsigned int i = 0; i < N; ++i) {    // Get samples for smooth the value
      vOut = vOut + analogRead(signalPin);
      delay(1);                               // delay in between reads for stability
    }
    vOut = (vOut * vConv) / N;                // ADC of voltage meter output voltage

    waterPres = getwaterPres(vOut);           // Calculate water pressure

    if (isinf(waterPres) || isnan(waterPres)) {
      waterPres = -1;
    }
}

void serialEvent(void) {
  if (Serial.available()) {
    percent = Serial.parseInt();

    Serial.print("Pump PWM Set to: ");
    Serial.print(percent);
    Serial.println(" %");

    Wire.beginTransmission(PUMP_CTRL);
    Wire.write(percent);
    Wire.endTransmission();
    // Flush the receive buffer
    Serial.flush(); 
    while (Serial.read() >= 0) { }
  }
}

/*--- Initialization ---*/
void setup(void) {
  Wire.begin(MAIN_CTRL);          // Initializes Wire and join I2C bus
  Serial.begin(baudSpeed);        // Initializes serial port
  pinMode(sigPin, INPUT);         // Initializes potentiometer pin
  pinMode(flowPin, INPUT_PULLUP); // Initializes interrupt digital pin 2 declared as an input and pull-up resitor enabled
  startTime = millis();           // Initial start time

  // Pump Percentage Initialization
  percent = 0;

  // Water Pressure Sensor Initialization
  vOut = 0.0;
  waterPres = 0.0;

  // Flow Sensor Initialization
  pulseCount = 0;
  cumCount = 0;
  flowRate = 0.0;
  flowML = 0.0;
  attachInterrupt(digitalPinToInterrupt(flowPin), getCounter, FALLING); // The interrupt is attached
}

/*--- Measurement ---*/
void loop(void) {
  // Every second, calculate and print the measured value
  currentTime = millis();                     // Get the current "time"

  if ((currentTime - startTime) >= period) {  // Test whether the period has elapsed
    timer = startTime / period;               // Calculate the period of time

    // Water Pressure Sensor
    waterPressure(sigPin);

    // Flow Sensor
    detachInterrupt(digitalPinToInterrupt(flowPin));  // Clears the function used to attend a specific interrupt
    cumCount = cumCount + pulseCount;                 // Count increment
    // Estimated Volume: 0.5004 ml/Pulse
    flowRate = abs(((-7.0 * pow(10.0, -18.0)) * sq(pulseCount)) + (0.5004 * pulseCount) - (8.0 * pow(10.0, -12.0)));
    flowML = flowRate * 60.0;                         // Milliliter per pulse converter to milliliter per minute

    if (isinf(flowML) || isnan(flowML) || (flowML <= 0.0)) {
      flowML = -1;
    }

    /*--- Sensor prompt ---*/
    Serial.print(percent);
    Serial.print(" %");

    Serial.print(", Voltage: ");
    Serial.print(vOut, 12);
    Serial.print(" V, ");
    // Unit converter for pressure, raw unit: MPa
    Serial.print("Pressure: ");
    Serial.print(waterPres * 1000, 1);    // 1 : 1000
    Serial.print(" kPa, ");
    Serial.print(waterPres, 2);           // Raw
    Serial.print(" MPa, ");
    Serial.print(waterPres * 10.1972, 1); // 1 : 10.1972
    Serial.print(" kg/cm^2, "); 

    Serial.print("Cumulative Count: ");
    Serial.print(cumCount);
    Serial.print(", Pulse Count: ");
    Serial.print(pulseCount);
    Serial.print(", Flow Rate: ");
    Serial.print(flowML);
    Serial.print(" mL/minute, ");

    Serial.print(timer);
    Serial.println(" sec.");

    /*--- System Return ---*/
    startTime = currentTime;                                              // Save the start time of the current state
    pulseCount = 0;                                                       // Set pulseCount to 0 ready for calculations
    attachInterrupt(digitalPinToInterrupt(flowPin), getCounter, FALLING); // Reattach interrupt
  } else {
    return;
  }
}
