#ifndef INPUT_SMOOTHER_H
#define INPUT_SMOOTHER_H

void Smoother_Init(void);

int SmoothAngleX(int rawX, int minDelta, int maxDelta);
int SmoothAngleY(int rawY, int minDelta, int maxDelta);
int SmoothHeight(int rawZ, int maxDelta);

#endif
