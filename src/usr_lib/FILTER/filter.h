#ifndef FILTER_H
#define FILTER_H

#include "../QUEUE/queue.h"

void LowPassFilter_RC_1order(float *Vi, float *Vo, float *Vo_p, float sampleFrq, float cutFrq);

queue_t *SildingAvrgFilter_Init(int window_size);
double SildingAvrgFilter_Calc(queue_t *pSildingAvrgQueue, double new_data);
void SildingAvrgFilter_Kill(queue_t *pSildingAvrgQueue);

#endif
