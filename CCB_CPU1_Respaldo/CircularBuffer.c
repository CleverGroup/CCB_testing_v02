// recuperado de https://towardsdatascience.com/circular-queue-or-ring-buffer-92c7b0193326
#define NULL 0x00

struct queue {
	unsigned int tail;	    // current tail
	unsigned int head;	    // current head
	unsigned int size;	    // current number of items
	unsigned int capacity;      // Capacity of queue
	int* data; 		    // Pointer to array of data
};

// Create Global defenition of queue_t
typedef struct queue queue_t;

// NAME 	: create_queue()
// INPUT 	: _capacity : capacity of circular queue
// OUTPUT 	: NULL on failure.
// 		  pointer to created circular queue (queue_t*)
// DESCRIPTION	: determine if queue is empty
queue_t* create_queue(unsigned int _capacity){

	queue_t* myQueue = (queue_t*)malloc(sizeof(queue_t)); // allocate memory of size of queue struct

	if (myQueue == NULL ){
		return NULL; // if malloc was unsuccesful return NULL
	} else {
		// populate the variables of the queue :
		myQueue->tail = -1;
		myQueue->head = 0;
		myQueue->size = 0;
		myQueue->capacity = _capacity;
		myQueue->data = (int*)malloc(_capacity * sizeof(int)); // allocate memory for the array

		return myQueue;
	}
}

// NAME 	: queue_empty()
// INPUT 	: q : pointer to circular queue (queue_t*).
// OUTPUT 	: -1 if q is NULL
// 		  1 if q is empty
// 		  0 if q is not empty
// DESCRIPTION	: determine if queue is empty

int queue_empty(queue_t* q){
		if (q == NULL){
			return -1;
		}else if(q->size == 0) {
			return 1;
		}else {
			return 0;
		}
}

// NAME 	: queue_full()
// INPUT 	: q : pointer to circular queue (queue_t*).
// OUTPUT 	: -1 if q is NULL
// 		  1 if q is full
// 		  0 if q is not full
// DESCRIPTION	: determine if queue is full
int queue_full(queue_t* q){
	if (q == NULL){
		return -1;
	}else if(q->size == q->capacity){
		return 1;
	}else{
		return 0;
	}
}

// NAME 	:	queue_enqueue()
// INPUT 	: q : pointer to circular queue (queue_t*)
// 		  item : integer to be added to queue
// OUTPUT 	: -1 if q is NULL
// 		  1 if item was added successfully
// 		  0 otherwise
// DESCRIPTION	: Enqueue item into circular queue q.
int queue_enqueue(queue_t* q, int item){

		if (q == NULL){
			return -1;
		}	else if (queue_full(q) == 1){
			// make sure the queue isnt full.
			return 0;
		} else {
		// first we move the tail (insert) location up one (in the circle (size related to _capacity))
		q->tail = (q->tail + 1) % q->capacity; // this makes it go around in a circle
		// now we can add the actual item to the location
		q->data[q->tail] = item;
		// now we have to increase the size.
		q->size++;
		return 1;
		}
}

// NAME 	: queue_enqueue()
// INPUT 	: q : pointer to circular queue (queue_t*)
// OUTPUT 	: -1 if q is NULL
// 		  0 if q is Empty
//		  else returns value of item at front of line.
// DESCRIPTION	: Dequeue circular queue q, returning next value.
// Note : we are ASSUMING all values in q are greater than zero.
int queue_dequeue(queue_t *q){

		if (q == NULL){
			return -1;

		}	else if (queue_empty(q) == 1){
			return 0;
		}else{
			// firt capture the item
			 int item = q->data[q->head];
			 q->head = (q->head + 1) % q->capacity;
			 // decrease size by 1
			 q->size--;

			 return item;
		}
	}

// NAME 	: queue_size()
// INPUT 	: q : pointer to circular queue (queue_t*).
// OUTPUT 	: -1 if q is NULL
// 		  else return current size of circular queue q.
// DESCRIPTION	: determine size of queue q.
unsigned int queue_size(queue_t* q){
	if (q == NULL){
		return - 1;
	} else {
		return q->size;
	}
}

// NAME 	: free_queue()
// INPUT 	: q : pointer to circular queue (queue_t*).
// OUTPUT 	: NONE
// DESCRIPTION	: free memory associatioed with circular queue q
void free_queue(queue_t* q){
			// free the array
			free(q->data);
			// free queue
			free(q);
}
