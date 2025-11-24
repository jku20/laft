#include "request_queue.h"
#include <stdbool.h>

void reset_queue(request_queue *q) {
  q->tail = 0;
  q->head = 0;
  q->size = 0;
}

void push_queue(request_queue *q, struct usb_endpoint_configuration *ep,
                uint8_t *buf, uint16_t len) {
  q->size++;
  int i = q->tail++;
  q->tail %= REQUEST_QUEUE_BUFFER_SIZE;
  q->buf[i].len = len;
  for (int j = 0; j < len; j++) {
    q->buf[i].buf[j] = buf[j];
  }
  q->buf[i].ep = ep;
}

request *pop_front_queue(request_queue *q) {
  q->size--;
  int i = q->head++;
  q->head %= REQUEST_QUEUE_BUFFER_SIZE;
  return &q->buf[i];
}

bool is_empty_queue(request_queue *q) { return q->size == 0; }
