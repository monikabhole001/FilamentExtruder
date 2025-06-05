 /* 
  Filament Extruder PID Temperature Control with MAX6675 Thermocouple and I2C LCD
  Author: Your Name
  Description:
    Controls the heater temperature of a filament extruder machine using PID.
    Reads temperature from MAX6675 thermocouple module,
    displays temperature and setpoint on I2C LCD,
    and controls heater power via PWM on a MOSFET.
    
  Hardware connections:
    MAX6675 Module:
      CS  -> Arduino D10
      SO  -> Arduino D12
      SCK -> Arduino D13
      Vcc -> 5V
      GND -> GND

    I2C LCD Module:
      SCL -> A5
      SDA -> A4
      Vcc -> 5V
      GND -> GND

    Heater control MOSFET:
      PWM signal -> Arduino D3

*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>

// LCD setup (address 0x27, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// MAX6675 SPI pins
#define MAX6675_CS   10
#define MAX6675_SO   12
#define MAX6675_SCK  13

// Heater control pin (PWM)
const int PWM_pin = 3;

// Variables
float temperature_read = 0.0;
float set_temperature = 100;  // Target temperature in °C
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;

// PID constants (tune these according to your system)
const float kp = 9.1;
const float ki = 0.3;
const float kd = 1.8;

float PID_p = 0, PID_i = 0, PID_d = 0;

void setup() {
  Serial.begin(9600);
  lcd.begin();
  pinMode(PWM_pin, OUTPUT);
  
  // Set PWM frequency for pin 3 (Timer2)
  TCCR2B = TCCR2B & B11111000 | 0x03; // ~980 Hz frequency
  
  Time = millis();
  
  lcd.backlight();
}

void loop() {
  lcd.clear();
  
  // Read temperature from thermocouple
  temperature_read = readThermocouple();
  Serial.println(temperature_read);

  // Calculate PID error (setpoint - actual)
  PID_error = set_temperature - temperature_read;

  // Proportional term
  PID_p = kp * PID_error;

  // Integral term (only update if error is within +-3)
  if (abs(PID_error) < 3) {
    PID_i += ki * PID_error;
  }

  // Derivative term - calculate based on elapsed time
  timePrev = Time;
  Time = millis();
  elapsedTime = (Time - timePrev) / 1000.0; // convert to seconds

  if(elapsedTime > 0) {
    PID_d = kd * ((PID_error - previous_error) / elapsedTime);
  } else {
    PID_d = 0;
  }

  // PID output
  PID_value = PID_p + PID_i + PID_d;

  // Limit PWM output between 0 and 255
  PID_value = constrain(PID_value, 0, 255);

  // Write PWM signal to heater MOSFET (inverted control)
  analogWrite(PWM_pin, 255 - PID_value);

  previous_error = PID_error;

  // Display on LCD
  lcd.setCursor(0, 0);
  lcd.print("PID TEMP control");

  lcd.setCursor(0, 1);
  lcd.print("S:");
  lcd.print(set_temperature, 1);

  lcd.setCursor(9, 1);
  lcd.print("R:");
  lcd.print(temperature_read, 1);

  delay(300);
}

// Reads temperature from MAX6675 thermocouple sensor
double readThermocouple() {
  uint16_t v;

  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);

  digitalWrite(MAX6675_CS, LOW);
  delay(1);

  // Read 16 bits from MAX6675
  v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);

  digitalWrite(MAX6675_CS, HIGH);

  // Check if thermocouple is connected
  if (v & 0x4) {
    return NAN; // No thermocouple attached
  }

  v >>= 3;  // Remove status bits

  // Each bit counts 0.25 degrees Celsius
  return v * 0.25;
}
