/**
 * @file arduino_sketch2.ino
 * @brief Arduino sketch to control a 9 servo robot
 * @details This sketch animates a 9 servo bipedal robot using the ServoEasing library.
 * See the README.md file for more information.
 */

#include <Arduino.h>
#include "inverse_kinematics.h"

#define MAX_EASING_SERVOS 9
#include "ServoEasing.hpp"
#include "config.h"

// Left Leg
ServoEasing Servo1;
ServoEasing Servo2;
ServoEasing Servo3;
// Right Leg
ServoEasing Servo4;
ServoEasing Servo5;
ServoEasing Servo6;
// Neck elevation (unused)
ServoEasing Servo7;
// Neck tilt (unused)
ServoEasing Servo8;
// Neck pan (unused)
ServoEasing Servo9;

#define S1_REST 160 // Left Leg - Hip
#define S2_REST 180 // Left Leg - Knee
#define S3_REST 90  // Left Leg - Ankle
#define S4_REST 20  // Right Leg - Hip
#define S5_REST 0   // Right Leg - Knee
#define S6_REST 90  // Right Leg - Ankle
#define S7_REST 0   // Neck elevation (unused)
#define S8_REST 90  // Neck tilt
#define S9_REST 90  // Neck pan

// Arrays to store servo min / max positions to avoid mechanical issues due
// NOTE: attach() disregards this, set PosSleep to be within range of the servo's physical boundaries
int PosMin[MAX_EASING_SERVOS] = {20, 5, 15, 20, 5, 15, 40, 60, 20};
int PosMax[MAX_EASING_SERVOS] = {160, 175, 180, 160, 175, 180, 90, 120, 160};
int PosSleep[MAX_EASING_SERVOS] = {70, PosMin[1], PosMax[2], 110, PosMax[4], PosMin[5], S7_REST, PosMax[7], S9_REST};


void blinkLED();

uint16_t tSpeed;

InverseKinematics ik(PosMin[3], PosMax[3], PosMin[4], PosMax[4], PosMin[5], PosMax[5], 94.0, 94.0, 28.0);

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(11, OUTPUT);   // sets the digital pin as output
    digitalWrite(11, LOW); // sets the digital pin on
    Serial.begin(115200);
    tSpeed = SERVO_SPEED_MIN;

    // Seed random number generator
    randomSeed(analogRead(0));

    // Attach servos to pins at starting position
    Servo1.attach(SERVO1_PIN, PosSleep[0]);
    Servo2.attach(SERVO2_PIN, PosSleep[1]);
    Servo3.attach(SERVO3_PIN, PosSleep[2]);
    Servo4.attach(SERVO4_PIN, PosSleep[3]);
    Servo5.attach(SERVO5_PIN, PosSleep[4]);
    Servo6.attach(SERVO6_PIN, PosSleep[5]);
    Servo7.attach(SERVO7_PIN, PosSleep[6]);
    Servo8.attach(SERVO8_PIN, PosSleep[7]);
    Servo9.attach(SERVO9_PIN, PosSleep[8]);

    // Loop over ServoEasing::ServoEasingArray and attach each servo
    for (uint8_t tIndex = 0; tIndex < MAX_EASING_SERVOS; ++tIndex)
    {
        // Set easing type to EASING_TYPE
        ServoEasing::ServoEasingArray[tIndex]->setEasingType(EASING_TYPE);
        ServoEasing::ServoEasingArray[tIndex]->setMinMaxConstraint(PosMin[tIndex], PosMax[tIndex]);
    }
    // Wait for servos to reach start position.
    delay(3000);

    Serial.println("Test 2d Inverse Kinematics");
}

void test2dInverseK()
{
    
    float hipAngleL, kneeAngleL, ankleAngleL, hipAngleR, kneeAngleR, ankleAngleR;
    int thisMove[MAX_EASING_SERVOS] = {90, 90, 90, 90, 90, 90, 90, 90, 90};

    int y = 0; // Coming soon
    // Iterate through valid X values (leg height between joint in ankle and joint in hip)
    for (int x = 140; x <= 180; x += 40)
    {
        // Solve inverse kinematics for left leg
        if (!ik.inverseKinematics2D(x, y, hipAngleL, kneeAngleL, ankleAngleL)) 
        {
            Serial.println("No solution");
            continue;
        }

        // Solve right leg, assuming identical but mirrored
        ik.calculateOtherLeg(hipAngleL, kneeAngleL, ankleAngleL, hipAngleR, kneeAngleR, ankleAngleR);

        #ifdef IK_DEBUG
        Serial.print("X: ");
        Serial.print(x);
        Serial.print(" Y: ");
        Serial.println(y);
        Serial.print(" - L Hip: ");
        Serial.print(hipAngleL);
        Serial.print(" Knee: ");
        Serial.print(kneeAngleL);
        Serial.print(" Ankle: ");
        Serial.println(ankleAngleL);
        Serial.print(" - R Hip: ");
        Serial.print(hipAngleR);
        Serial.print(" Knee: ");
        Serial.print(kneeAngleR);
        Serial.print(" Ankle: ");
        Serial.println(ankleAngleR);
        #endif

        // Define new positions and move servos
        thisMove[0] = hipAngleR;
        thisMove[1] = kneeAngleR;
        thisMove[2] = ankleAngleR;
        thisMove[3] = hipAngleL;
        thisMove[4] = kneeAngleL;
        thisMove[5] = ankleAngleL;
        moveServos(thisMove);
    }
    delay(1000);
}


void blinkLED()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

void moveServos(int *Pos)
{
    for (uint8_t tIndex = 0; tIndex < MAX_EASING_SERVOS; ++tIndex)
    {
        if (Pos[tIndex] != -1)
            ServoEasing::ServoEasingNextPositionArray[tIndex] = Pos[tIndex];
        else 
            ServoEasing::ServoEasingNextPositionArray[tIndex] = moveRandom(tIndex); // If scripted value is -1, generate random position based on range of currently indexed servo
    }
    setEaseToForAllServosSynchronizeAndStartInterrupt(tSpeed);
    while (ServoEasing::areInterruptsActive())
    {
        blinkLED();
    }
}

long moveRandom(int index)
{
    return random(PosMin[index], PosMax[index]);
}


void setSpeed(uint16_t pSpeed)
{
    tSpeed = pSpeed;
    setSpeedForAllServos(tSpeed);
    Serial.print(F("Speed set: "));
    Serial.println(tSpeed);
}

void loop()
{
    test2dInverseK();
}
