/*
 FreeVario http://FreeVario.org

  Copyright (c), PrimalCode (http://www.primalcode.org)
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version. see <http://www.gnu.org/licenses/>
*/

#include "stackops.h"
#include <stdlib.h>

void SO_setQueue(Queue_t *queue, int capacity){

    queue->capacity = capacity;
    queue->front = queue->size = 0;
    queue->rear = capacity - 1;
    queue->array = (float*) malloc(queue->capacity * sizeof(float));

}

// Queue is full when size becomes equal to the capacity
int SO_qisFull(Queue_t *queue)
{  return (queue->size == queue->capacity);  }

// Queue is empty when size is 0
int SO_qisEmpty(Queue_t *queue)
{  return (queue->size == 0); }

// Function to add an item to the queue.
// It changes rear and size
void SO_enqueue(Queue_t *queue, float item)
{
    if (SO_qisFull(queue))
    	SO_dequeue(queue);
    queue->rear = (queue->rear + 1)%queue->capacity;
    queue->array[queue->rear] = item;
    queue->size = queue->size + 1;

}

// Function to remove an item from queue.
// It changes front and size
float SO_dequeue(Queue_t *queue)
{
    if (SO_qisEmpty(queue))
        return 0;
    int item = queue->array[queue->front];
    queue->front = (queue->front + 1)%queue->capacity;
    queue->size = queue->size - 1;
    return item;
}

// Function to get front of queue
float SO_front(Queue_t *queue)
{
    if (SO_qisEmpty(queue))
        return 0;
    return queue->array[queue->front];
}

// Function to get rear of queue
float SO_rear(Queue_t *queue)
{
    if (SO_qisEmpty(queue))
        return 0;
    return queue->array[queue->rear];
}


float SO_getAvarage(Queue_t *queue)  {
	float sum=0;
	int i;
	//TODO: test this
	for(i = queue->rear; i < queue->front +1; ++i) {
			sum += queue->array[i];
		}

	return sum / queue->size;
}

