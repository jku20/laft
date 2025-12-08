#include <hardware/dma.h>
#include <hardware/pio.h>
#include <stdbool.h>

#include "circular_buffer.h"
#include "logic_analyzer.h"

void cb_init(CircularBuffer *self, uint base_pin, PIO pio, uint sm, uint dma) {
  self->pio = pio;
  self->sm = sm;
  self->dma = dma;

  // Disable the PIO so the DMA channel doesn't read nonsense.
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

void cb_arm_to_start_collecting(Bitset32 trigger_mask[NUM_STAGES],
                                Bitset32 trigger_value[NUM_STAGES],
                                Bitset32 trigger_config[NUM_STAGES],
                                uint32_t clock_div) {
  // This implementation is heavily inspired by the implementation of
  // [this](https://github.com/dotcypress/ula/blob/40756c4199c9f2ac605a39bfed2eddcc7fe0324b/src/trigger.rs#L65)
  // function in ula, a logic analyzer implemented for the pico.
}

void cb_stop_buf_population(CircularBuffer *self) {
  pio_sm_set_enabled(self->pio, self->sm, false);
}

void cb_start_buf_population(CircularBuffer *self) {
  pio_sm_set_enabled(self->pio, self->sm, true);
}
