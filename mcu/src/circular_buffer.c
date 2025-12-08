#include <hardware/dma.h>
#include <hardware/pio.h>
#include <hardware/pio_instructions.h>
#include <stdbool.h>
#include <string.h>

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

int trailing_zeros(uint32_t n) {
  int c = 0;
  while (!(n & 1) && c < 32) {
    c++;
    n >>= 1;
  }
  return c;
}

int trailing_ones(uint32_t n) {
  int c = 0;
  while (n & 1) {
    c++;
    n >>= 1;
  }
  return c;
}

void cb_arm_to_start_collecting(CircularBuffer *self,
                                Bitset32 trigger_mask[NUM_STAGES],
                                Bitset32 trigger_value[NUM_STAGES],
                                TriggerConfiguration trigger_config[NUM_STAGES],
                                uint32_t clock_div, uint32_t base_pin) {
  // This implementation is heavily inspired by the implementation of
  // [this](https://github.com/dotcypress/ula/blob/40756c4199c9f2ac605a39bfed2eddcc7fe0324b/src/trigger.rs#L65)
  // function in ula, a logic analyzer implemented for the pico.
  //
  // Generate the program for the PIO.
  uint16_t prog[32];
  memset(prog, 0, sizeof prog);
  int static_pc = 0;
  for (int i = 0; i < NUM_STAGES; i++) {
    int stage_offset = static_pc;
    if (bitset_is_empty(&trigger_mask[i])) {
      continue;
    }

    uint32_t mask = bitset_get_raw(&trigger_mask[i]);
    uint32_t value = bitset_get_raw(&trigger_value[i]);

    prog[static_pc++] = pio_encode_mov(pio_osr, pio_pins);
    while (mask) {
      int mask_zeros = trailing_zeros(mask);
      if (mask_zeros) {
        prog[static_pc++] = pio_encode_out(pio_null, mask_zeros);
        mask >>= mask_zeros;
        value >>= mask_zeros;
      }

      int mask_ones = MIN(trailing_ones(mask), 5);
      if (mask_ones) {
        prog[static_pc++] = pio_encode_out(pio_x, mask_ones);
        prog[static_pc++] =
            pio_encode_set(pio_y, value & ((1 << mask_ones) - 1));
        prog[static_pc++] = pio_encode_jmp_x_ne_y(stage_offset);
        mask >>= mask_ones;
        value >>= mask_ones;
      }
    }
  }
  prog[static_pc++] = pio_encode_in(pio_pins, 16);

  // Assign the PIO program to the state machine
  struct pio_program capture_prog = {
      .instructions = prog, .length = static_pc, .origin = -1};
  int offset = pio_add_program(self->pio, &capture_prog);
  pio_sm_config capture_config = pio_get_default_sm_config();
  sm_config_set_wrap(&capture_config, offset + static_pc - 1,
                     offset + static_pc - 1);
  sm_config_set_in_pin_base(&capture_config, base_pin);
  sm_config_set_in_pin_count(&capture_config, 16);
  sm_config_set_in_shift(&capture_config, true, true, 16);
  pio_sm_init(self->pio, self->sm, offset, &capture_config);
  pio_sm_set_enabled(self->pio, self->sm, true);
}

void cb_stop_buf_population(CircularBuffer *self) {
  pio_sm_set_enabled(self->pio, self->sm, false);
}

void cb_start_buf_population(CircularBuffer *self) {
  pio_sm_set_enabled(self->pio, self->sm, true);
}
