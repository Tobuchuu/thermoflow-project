#include "gd32vf103.h"
void InitPWM(void);         // Fan

void InitServo(void);       // Servo

void FanPWMch0(int value);  // Control fan

void MoveServoA(int value); // Control servo 1

void MoveServoB(int value); // Control servo 2