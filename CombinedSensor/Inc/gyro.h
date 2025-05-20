#ifndef GYRO_H
#define GYRO_H

void Gyro_Init(void);
void Gyro_GetFilteredAngles(float *angleX, float *angleY);
int MapAngleWithDeadzone(float angle, float deadzone_deg, float max_deg);

#endif


