#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint8_t buf[64];
  uint16_t len;
  struct usb_endpoint_configuration *ep;
} request;

#define REQUEST_QUEUE_BUFFER_SIZE 100

typedef struct {
  request buf[REQUEST_QUEUE_BUFFER_SIZE];
  int tail;
  int head;
  int size;
} request_queue;

void reset_queue(request_queue *q);
void push_queue(request_queue *q, struct usb_endpoint_configuration *ep,
                uint8_t *buf, uint16_t len);
request *pop_front_queue(request_queue *q);
bool is_empty_queue(request_queue *q);
