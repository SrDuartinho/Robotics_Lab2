#include "AK09918.h"
#include "ICM20600.h"
#include <Wire.h>
// This version uses an explicit PWM generator (in function MoveMotor1)

#include <Stepper.h>

// teste de velocidade e direcção para o balancer
// script baseado em codigo online na reference da biblioteca stepper

#include <Wire.h>


#define STEPS 200
#define dirPin 9
#define stepPin 8
#define sw1Pin 10
#define sw2Pin 11
// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
Stepper stepper(STEPS, 8, 9);
#define motorInterfaceType 1

// ===== Complementary filter =====
float pitch = 0.0;
float pitch_acc = 0.0;

unsigned long lastIMUTime = 0;
const float alpha = 0.98;   // filter weight


int left, right, middle;

AK09918_err_type_t err;
int32_t x, y, z;
AK09918 ak09918;
ICM20600 icm20600(true);
int16_t acc_x, acc_y, acc_z;
int32_t offset_x, offset_y, offset_z;
int32_t gyr_x, gyr_y, gyr_z;
double roll;
// Find the magnetic declination at your location
// http://www.magnetic-declination.com/
double declination_lisbon = -1.1;

//debounce variables
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 500;    // the debounce time; increase if the output flickers

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    Serial.begin(9600);

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(sw1Pin, INPUT);
    pinMode(sw2Pin, INPUT);

    
    left = 0;
    right = 0;

    Serial.println("Going left"); // when viewed from the robot frame
                                  // It's the right when facing the front of the robot
    while (digitalRead(sw2Pin)==1) {
      // go up until the switch is reached
      moveMotor1( -10, sw2Pin );
      left +=10;
    }
    Serial.print("Left at ");
    Serial.print(left);
    Serial.print("   ");
    Serial.print(digitalRead(sw2Pin));
    Serial.print("  ");
    Serial.println(digitalRead(sw1Pin));
    // at this point the motor is at the highest position

    Serial.println("Going right");
    while (digitalRead(sw1Pin)==1) {
      // go down until the switch is reached
      moveMotor1( 10, sw1Pin );
      right +=10;
    }
    Serial.print("Right at ");
    Serial.print(right);
    Serial.print("   ");
    Serial.print(digitalRead(sw2Pin));
    Serial.print("  ");
    Serial.println(digitalRead(sw1Pin));
    
    middle = right / 2;
    // put the motor at the middle - if needed an offset can be used
    moveMotor1(-middle, sw2Pin);
    Serial.print("Middle point at  ");
    Serial.println(middle);

    Serial.println("Setup completed");
    
    err = ak09918.initialize();
    icm20600.initialize();
    ak09918.switchMode(AK09918_POWER_DOWN);
    ak09918.switchMode(AK09918_CONTINUOUS_100HZ);


    err = ak09918.isDataReady();
    while (err != AK09918_ERR_OK) {
        Serial.println("Waiting Sensor");
        delay(100);
        err = ak09918.isDataReady();
    }

    lastIMUTime = millis();


    Serial.println("Start figure-8 calibration after 2 seconds.");
    delay(2000);
    calibrate(10000, &offset_x, &offset_y, &offset_z);
    Serial.println("");
}

void loop() {
    // get acceleration
    acc_x = icm20600.getAccelerationX();
    acc_y = icm20600.getAccelerationY();
    acc_z = icm20600.getAccelerationZ();

    gyr_x = icm20600.getGyroscopeX();
    gyr_y = icm20600.getGyroscopeY();
    gyr_z = icm20600.getGyroscopeZ();

    ak09918.getData(&x, &y, &z);
    x = x - offset_x;
    y = y - offset_y;
    z = z - offset_z;

    // ===== Time step =====
    unsigned long now = millis();
    float dt = (now - lastIMUTime) / 1000.0;
    lastIMUTime = now;

    // ===== Accelerometer angle (pitch) =====
    pitch_acc = atan2((float)acc_x, (float)acc_z) * 180.0 / PI;

    // ===== Gyro rate (deg/s) =====
    // ICM20600 gyro is usually in LSB → check your lib scaling
    float gyro_y_dps = gyr_y / 131.0;  // typical for ±250 dps

    // ===== Complementary filter =====
    pitch = alpha * (pitch + gyro_y_dps * dt) + (1 - alpha) * pitch_acc;


    if (pitch > 2.0) {
      moveMotor1(-5, sw1Pin);
    }
    else if (pitch < -2.0) {
      moveMotor1(5, sw2Pin);
    }


    if ((millis() - lastDebounceTime) > debounceDelay) {

      Serial.print("A:  ");
      Serial.print(acc_x);
      Serial.print(",  ");
      Serial.print(acc_y);
      Serial.print(",  ");
      Serial.print(acc_z);
      Serial.print(" mg      ");

      Serial.print("G:  ");
      Serial.print(gyr_x);
      Serial.print(",  ");
      Serial.print(gyr_y);
      Serial.print(",  ");
      Serial.print(gyr_z);
      Serial.print(" dps     ");

      Serial.print("M:  ");
      Serial.print(x);
      Serial.print(",  ");
      Serial.print(y);
      Serial.print(",  ");
      Serial.print(z);
      Serial.println(" uT");

      Serial.print(pitch);
      Serial.println();

      lastDebounceTime = millis();
    }

}

void calibrate(uint32_t timeout, int32_t* offsetx, int32_t* offsety, int32_t* offsetz) {
    int32_t value_x_min = 0;
    int32_t value_x_max = 0;
    int32_t value_y_min = 0;
    int32_t value_y_max = 0;
    int32_t value_z_min = 0;
    int32_t value_z_max = 0;
    uint32_t timeStart = 0;

    ak09918.getData(&x, &y, &z);

    value_x_min = x;
    value_x_max = x;
    value_y_min = y;
    value_y_max = y;
    value_z_min = z;
    value_z_max = z;
    delay(100);

    timeStart = millis();

    while ((millis() - timeStart) < timeout) {
        ak09918.getData(&x, &y, &z);

        /* Update x-Axis max/min value */
        if (value_x_min > x) {
            value_x_min = x;
            // Serial.print("Update value_x_min: ");
            // Serial.println(value_x_min);

        } else if (value_x_max < x) {
            value_x_max = x;
            // Serial.print("update value_x_max: ");
            // Serial.println(value_x_max);
        }

        /* Update y-Axis max/min value */
        if (value_y_min > y) {
            value_y_min = y;
            // Serial.print("Update value_y_min: ");
            // Serial.println(value_y_min);

        } else if (value_y_max < y) {
            value_y_max = y;
            // Serial.print("update value_y_max: ");
            // Serial.println(value_y_max);
        }

        /* Update z-Axis max/min value */
        if (value_z_min > z) {
            value_z_min = z;
            // Serial.print("Update value_z_min: ");
            // Serial.println(value_z_min);

        } else if (value_z_max < z) {
            value_z_max = z;
            // Serial.print("update value_z_max: ");
            // Serial.println(value_z_max);
        }

        Serial.print(".");
        delay(100);

    }

    *offsetx = value_x_min + (value_x_max - value_x_min) / 2;
    *offsety = value_y_min + (value_y_max - value_y_min) / 2;
    *offsetz = value_z_min + (value_z_max - value_z_min) / 2;
}

// Motor movement
// This function returns only after the movement is completed

void moveMotor1(int steps, int safetyPin1) {

  digitalWrite(dirPin, steps > 0 ? HIGH : LOW); // Set direction

  for (int i=0; i<abs(steps); i++) {

    if (digitalRead(safetyPin1)==1) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(2000);    //2500);  // Adjust for speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(2000);    //1500);
    }
    else {
      // get out of the loop and function
      break;
    }
  }
}

/*
int controller (int16_t acc_x,int16_t acc_y, int16_t acc_z, int32_t gyr_x, int32_t gyr_y, int32_t gyr_z){

  return steps;
}
*/

