#include <hardware/pio.h>
#include <stdint.h>

// This buffer must be large enough to hold all of the data collected by a SUMP
// request. That is 2^16 bits. We double this because our timing is not quite
// cycle accurate for stopping the DMA channel and we want the extra buffer room
// so we don't overwrite data we need when we jump back to the front of the
// buffer.
#define BUFFER_SIZE ((1 << 16) * 2)

/**
 * This is a circular buffer and associated functions to set up a startable and
 * stopable DMA channel and PIO state machine which will read 16 consecutive
 * bits, starting at a base GPIO, into the buffer.
 */
typedef struct {
  uint16_t buf[BUFFER_SIZE];
  PIO pio;
  uint sm;
  uint dma;
} CircularBuffer;

/**
 * Initializes the CircularBuffer and starts the DMA channel and GPIO. This
 * requires a DMA channel and a PIO state machine.
 */
void cb_init(CircularBuffer *self, uint base_pin, PIO pio, uint sm, uint dma);

/** Stops reading to the circular buffer from the PIO. */
void cb_stop_buf_population(CircularBuffer *self);

/** Starts reading to the circular buffer from the PIO. */
void cb_start_buf_population(CircularBuffer *self);
