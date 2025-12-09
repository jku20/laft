#include <hardware/dma.h>
#include <hardware/pio.h>
#include <hardware/pio_instructions.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "logic_analyzer.h"

static int read_count = 0;
static LogicAnalyzer *la;

const uint8_t LOGIC_ANALYZR_ID[] = {'1', 'A', 'L', 'S'};
const uint8_t SUMP_ID_LEN = 4;

void dma_irq_0_handler() {
  for (int i = 0; i < read_count; i++) {
    putchar(0x00FF & la->buf.buf[i]);
    putchar(0xFF00 & la->buf.buf[i]);
    putchar(0x00FF & la->buf.buf[i]);
    putchar(0xFF00 & la->buf.buf[i]);
  }
}

void la_set_paused(LogicAnalyzer *self, bool paused) { self->paused = paused; }

void la_init(LogicAnalyzer *self, uint base_pin, PIO pio, uint sm, uint dma) {
  cb_init(&self->buf, base_pin, pio, sm, dma);
  la = self;
  irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_0_handler);
  irq_set_enabled(DMA_IRQ_0, true);
  for (int i = 0; i < NUM_STAGES; i++) {
    self->trigger_mask[i] = bitset_from_uint32(0);
    self->trigger_value[i] = bitset_from_uint32(0);
  }
  self->paused = false;
  self->clock_div = 1;
  self->read_count = 0;
  self->delay_count = 0;
}

void la_reset(LogicAnalyzer *self) {
  for (int i = 0; i < NUM_STAGES; i++) {
    self->trigger_mask[i] = bitset_from_uint32(0);
    self->trigger_value[i] = bitset_from_uint32(0);
  }
  self->paused = false;
  self->clock_div = 1;
  self->read_count = 0;
  self->delay_count = 0;
}

uint8_t *la_get_id(LogicAnalyzer *self) { return (uint8_t *)LOGIC_ANALYZR_ID; }

void la_arm(LogicAnalyzer *self) {
  cb_arm_to_start_collecting(&self->buf, self->trigger_mask,
                             self->trigger_value, self->trigger_config,
                             self->clock_div, self->read_count >> 1);
  read_count = self->read_count >> 1;
}

void la_set_trigger_mask(LogicAnalyzer *self, Bitset32 mask, int stage) {
  self->trigger_mask[stage] = mask;
}

void la_set_trigger_value(LogicAnalyzer *self, Bitset32 value, int stage) {
  self->trigger_value[stage] = value;
}

void la_set_trigger_config(LogicAnalyzer *self, TriggerConfiguration config,
                           int stage) {
  self->trigger_config[stage] = config;
}

void la_set_clock_divider(LogicAnalyzer *self, uint32_t clock_div) {
  self->clock_div = clock_div;
}

void la_set_read_count(LogicAnalyzer *self, uint16_t read_count) {
  self->read_count = read_count;
}

void la_set_delay_count(LogicAnalyzer *self, uint16_t delay_count) {
  self->delay_count = delay_count;
}

void la_set_flags(LogicAnalyzer *self, LogicAnalyzerFlags flags) {
  self->flags = flags;
}

void transmit_buf(uint8_t *buf, int len) {
  for (int i = 0; i < len; i++) {
    putchar(buf[i]);
  }
}

uint32_t reverse_bits(uint32_t bits) {
  uint32_t out = 0;
  while (bits) {
    out <<= 1;
    out |= bits & 1;
    bits >>= 1;
  }
  return out;
}

void la_exec_command(LogicAnalyzer *self, SumpCommand *cmd) {
  switch (sc_get_ty(cmd)) {
    int stage;
  case Reset:
    printf("Reset\n");
    la_set_paused(self, false);
    break;
  case Run:
    printf("Run\n");
    la_arm(self);
    break;
  case Id:;
    printf("Id\n");
    uint8_t *buf = la_get_id(self);
    transmit_buf(buf, SUMP_ID_LEN);
    break;
  case Xon:
    printf("Xon\n");
    la_set_paused(self, false);
    break;
  case Xoff:
    printf("Xoff\n");
    la_set_paused(self, true);
    break;
  case SetTriggerMaskStage0:
  case SetTriggerMaskStage1:
  case SetTriggerMaskStage2:
  case SetTriggerMaskStage3:
    stage = sc_get_stage(cmd);
    Bitset32 mask = sc_get_mask(cmd);
    printf("mask: %x\n", mask.data);
    la_set_trigger_mask(self, mask, stage);
    break;
  case SetTriggerValueStage0:
  case SetTriggerValueStage1:
  case SetTriggerValueStage2:
  case SetTriggerValueStage3:
    stage = sc_get_stage(cmd);
    Bitset32 value = sc_get_value(cmd);
    printf("value: %x\n", value.data);
    la_set_trigger_value(self, value, stage);
    break;
  case SetTriggerConfigStage0:
  case SetTriggerConfigStage1:
  case SetTriggerConfigStage2:
  case SetTriggerConfigStage3:
    stage = sc_get_stage(cmd);
    TriggerConfiguration config = sc_get_trigger_configuration(cmd);
    printf("(delay, channel, level, serial, start): (%hd, %hhd, %hhd, %d, %d)",
           config.delay, config.channel, config.level, config.serial,
           config.start);
    la_set_trigger_config(self, config, stage);
    break;
  case SetDivider:;
    uint32_t div = sc_get_clock_div(cmd);
    printf("div: %d\n", div);
    la_set_clock_divider(self, div);
    break;
  case SetReadAndDelayCount:
    printf("(read, delay): (%d, %d)\n", sc_get_read_count(cmd),
           sc_get_delay_count(cmd));
    la_set_read_count(self, sc_get_read_count(cmd));
    la_set_delay_count(self, sc_get_delay_count(cmd));
    break;
  case SetFlags:;
    LogicAnalyzerFlags flags = sc_get_flags(cmd);
    printf("(dmux, filter, channel groups, external, inverted): (%d, %d, %d, "
           "%d, %d)\n",
           flags.demux, flags.filter, flags.channel_groups, flags.external,
           flags.inverted);
    la_set_flags(self, flags);
    break;
  default:
    printf("Unknown\n");
    break;
  }
}

SumpCommandType sc_get_ty(SumpCommand *self) { return self->ty; }

Bitset32 sc_get_mask(SumpCommand *self) {
  return bitset_from_uint32(self->data);
}

Bitset32 sc_get_value(SumpCommand *self) {
  return bitset_from_uint32(self->data);
}

int sc_get_stage(SumpCommand *self) {
  SumpCommandType ty = sc_get_ty(self);
  return (ty & 0b00001100) >> 2;
}

TriggerConfiguration sc_get_trigger_configuration(SumpCommand *self) {
  TriggerConfiguration out;
  uint32_t data = reverse_bits(self->data);
  out.delay = ((0xFF000000 & data) >> 24) | ((0x00FF0000 & data) >> 16);
  out.channel = ((0x0000F000 & data) >> 12) | ((0x1 & data) << 4);
  out.level = (data >> 4) & 0x3;
  out.serial = (data >> 2) & 0x1;
  out.start = (data >> 3) & 0x1;
  return out;
}

uint32_t sc_get_clock_div(SumpCommand *self) {
  return (0x00FFFFFF & self->data) + 1;
}

uint16_t sc_get_read_count(SumpCommand *self) { return self->data & 0xFFFF; }

uint16_t sc_get_delay_count(SumpCommand *self) { return self->data >> 16; }

LogicAnalyzerFlags sc_get_flags(SumpCommand *self) {
  uint8_t flag_byte = self->data;
  LogicAnalyzerFlags out;
  out.demux = flag_byte & 0b00000001;
  out.filter = (flag_byte & 0b00000010) >> 1;
  out.channel_groups = (flag_byte & 0b00111100) >> 2;
  out.external = (flag_byte & 0b01000000) >> 6;
  out.inverted = (flag_byte & 0b10000000) >> 7;
  return out;
}

bool is_short_cmd(SumpCommandType ty) {
  switch (ty) {
  case Reset:
  case Run:
  case Id:
  case Xon:
  case Xoff:
    return true;
    break;
  case SetTriggerMaskStage0:
  case SetTriggerMaskStage1:
  case SetTriggerMaskStage2:
  case SetTriggerMaskStage3:
  case SetTriggerValueStage0:
  case SetTriggerValueStage1:
  case SetTriggerValueStage2:
  case SetTriggerValueStage3:
  case SetTriggerConfigStage0:
  case SetTriggerConfigStage1:
  case SetTriggerConfigStage2:
  case SetTriggerConfigStage3:
  case SetDivider:
  case SetReadAndDelayCount:
  case SetFlags:
    return false;
    break;
  }
}

SumpCommand sc_read_from_stdin() {
  SumpCommand out;
  out.ty = getchar();
  if (is_short_cmd(out.ty)) {
    return out;
  }
  out.data = 0;
  for (int i = 0; i < 4; i++) {
    out.data |= getchar() << (8 * i);
  }
  return out;
}

void cb_init(CircularBuffer *self, uint base_pin, PIO pio, uint sm, uint dma) {
  self->pio = pio;
  self->sm = sm;
  self->dma = dma;
  self->base_pin = base_pin;

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
                                uint32_t clock_div, int transfer_count) {
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
  sm_config_set_in_pin_base(&capture_config, self->base_pin);
  sm_config_set_in_pin_count(&capture_config, 16);
  sm_config_set_in_shift(&capture_config, true, true, 16);
  sm_config_set_clkdiv(&capture_config, clock_div);
  pio_sm_init(self->pio, self->sm, offset, &capture_config);
  pio_sm_set_enabled(self->pio, self->sm, true);

  // Sets the DMA channel to copy from the PIO to the circular buffer.
  dma_channel_config dma_data_config =
      dma_channel_get_default_config(self->dma);
  channel_config_set_read_increment(&dma_data_config, false);
  channel_config_set_write_increment(&dma_data_config, true);
  channel_config_set_dreq(&dma_data_config,
                          pio_get_dreq(self->pio, self->sm, false));
  dma_channel_configure(self->dma, &dma_data_config, self->buf, self->pio->rxf,
                        dma_encode_transfer_count(transfer_count), true);
  dma_set_irq0_channel_mask_enabled(1 << self->dma, true);
}

void cb_stop_buf_population(CircularBuffer *self) {
  pio_sm_set_enabled(self->pio, self->sm, false);
}

void cb_start_buf_population(CircularBuffer *self) {
  pio_sm_set_enabled(self->pio, self->sm, true);
}
