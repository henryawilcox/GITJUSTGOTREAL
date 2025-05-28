#ifndef GYRO_H
#define GYRO_H

#include <stdint.h>

void Gyro_Init(void);
void Gyro_GetFilteredAngles(float *angleX, float *angleY);
int MapAngleWithDeadzone(float angle, float deadzone_deg, float max_deg);
void UpdateHeightLEDs(uint8_t height_percent);
uint8_t IsLimitSwitchPressed(void);

#endif


