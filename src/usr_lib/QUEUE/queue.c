#include "queue.h"
#include "stdio.h"

queue_t *queue_malloc(int len)
{
	queue_t *q = (queue_t *)malloc(sizeof(queue_t));

	if (len > 0)
		q->len = len;

	q->head = (double *)calloc(len, sizeof(double));
	q->rear = q->head;
	return q;
}

void queue_free(queue_t *q)
{
	free(q->head);
	free(q);
}

void queue_push(queue_t *q, double new_data)
{
	*q->rear = new_data;

	if (q->rear == q->head + q->len - 1)
		q->rear = q->head;
	else
		q->rear++;
}

double queue_avrg(queue_t *q)
{
	double sum = 0;

	for (int i = 0; i < q->len; i++)
		sum = sum + *(q->head + i);
	//printf("Sum:%lf\n", sum);
	return sum / q->len;
}

void queue_display(queue_t *q)
{
	printf("Len:%d Member:", q->len);
	for (int i = 0; i < q->len; i++)
		printf("%6.2lf\t", *(q->head + i));
	printf("\n");
}
