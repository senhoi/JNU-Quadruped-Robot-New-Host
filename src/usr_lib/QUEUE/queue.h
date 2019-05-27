#ifndef QUEUE_H
#define QUEUE_H

#include "stdlib.h"

#define QUEUE_SUCCESS 0
#define QUEUE_FAILED 1

typedef struct queue_t
{
	int len;
	double *head;
	double *rear;
}queue_t;

queue_t* queue_malloc(int len);
void queue_free(queue_t* q);
void queue_push(queue_t* q, double new_data);
double queue_avrg(queue_t* q);
void queue_display(queue_t* q);

#endif