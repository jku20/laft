#include <hardware/dma.h>
#include <hardware/pio.h>
#include <stdbool.h>

#include "circular_buffer.h"

void cb_init(CircularBuffer *self, uint base_pin, PIO pio, uint sm, uint dma) {
  self->pio = pio;
  self->sm = sm;
  self->dma = dma;

  // Create a pio program with a single "in" instruction.
  uint16_t in_insn = pio_encode_in(pio_pins, base_pin);
  struct pio_program capture_prog = {
      .instructions = &in_insn, .length = 1, .origin = -1};

  // Adds the pio program to the state machine and sets the base pin. Also sets
  // the shift parameters to be compatible with what is expected by the DMA
  // channel.
  int offset = pio_add_program(pio, &capture_prog);
  pio_sm_config pio_config = pio_get_default_sm_config();
  sm_config_set_wrap(&pio_config, offset, offset);
  sm_config_set_in_pin_base(&pio_config, base_pin);
  sm_config_set_in_shift(&pio_config, true, true, 16);
  pio_sm_set_enabled(pio, sm, false);

  // Sets the DMA channel to forever copy from the PIO to the circular buffer.
  dma_channel_config dma_data_config = dma_channel_get_default_config(dma);
  channel_config_set_read_increment(&dma_data_config, false);
  channel_config_set_write_increment(&dma_data_config, true);
  channel_config_set_dreq(&dma_data_config, pio_get_dreq(pio, sm, false));
  dma_channel_configure(
      dma, &dma_data_config, self->buf, pio->rxf,
      dma_encode_transfer_count_with_self_trigger(BUFFER_SIZE), true);
}

void cb_stop_buf_population(CircularBuffer *self) {
  pio_sm_set_enabled(self->pio, self->sm, false);
}

void cb_start_buf_population(CircularBuffer *self) {
  pio_sm_set_enabled(self->pio, self->sm, true);
}
