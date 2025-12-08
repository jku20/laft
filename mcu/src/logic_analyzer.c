#include "logic_analyzer.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void la_set_paused(LogicAnalyzer *self, bool paused) { self->paused = paused; }

void la_init(LogicAnalyzer *self, uint base_pin, PIO pio, uint sm, uint dma) {
  cb_init(&self->buf, base_pin, pio, sm, dma);
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

void la_arm(LogicAnalyzer *self) {}

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

void la_transmit(LogicAnalyzer *_self, uint8_t *buf, int len) {
  for (int i = 0; i < len; i++) {
    putchar(buf[i]);
  }
}

void la_exec_command(LogicAnalyzer *self, SumpCommand *cmd) {
  switch (sc_get_ty(cmd)) {
    int stage;
  case Reset:
    la_set_paused(self, false);
    break;
  case Run:
    la_arm(self);
    break;
  case Id:;
    uint8_t *buf = la_get_id(self);
    la_transmit(self, buf, SUMP_ID_LEN);
    break;
  case Xon:
    la_set_paused(self, false);
    break;
  case Xoff:
    la_set_paused(self, true);
    break;
  case SetTriggerMaskStage0:
  case SetTriggerMaskStage1:
  case SetTriggerMaskStage2:
  case SetTriggerMaskStage3:
    stage = sc_get_stage(cmd);
    Bitset32 mask = sc_get_mask(cmd);
    la_set_trigger_mask(self, mask, stage);
    break;
  case SetTriggerValueStage0:
  case SetTriggerValueStage1:
  case SetTriggerValueStage2:
  case SetTriggerValueStage3:
    stage = sc_get_stage(cmd);
    Bitset32 value = sc_get_value(cmd);
    la_set_trigger_value(self, value, stage);
    break;
  case SetTriggerConfigStage0:
  case SetTriggerConfigStage1:
  case SetTriggerConfigStage2:
  case SetTriggerConfigStage3:
    stage = sc_get_stage(cmd);
    TriggerConfiguration config = sc_get_trigger_configuration(cmd);
    la_set_trigger_config(self, config, stage);
    break;
  case SetDivider:
    la_set_clock_divider(self, sc_get_clock_div(cmd));
    break;
  case SetReadAndDelayCount:
    la_set_read_count(self, sc_get_read_count(cmd));
    la_set_delay_count(self, sc_get_delay_count(cmd));
    break;
  case SetFlags:
    la_set_flags(self, sc_get_flags(cmd));
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
  out.delay = (0xFFFF0000 & self->data) >> 16;
  out.channel = ((0x0000F000 & self->data) >> 11) | (0x1 & self->data);
  out.level = (self->data >> 4) & 0x3;
  out.serial = (self->data >> 2) & 0x1;
  out.start = (self->data >> 3) & 0x1;
  return out;
}

uint32_t sc_get_clock_div(SumpCommand *self) { return (self->data >> 8) + 1; }

uint16_t sc_get_read_count(SumpCommand *self) { return self->data >> 16; }

uint16_t sc_get_delay_count(SumpCommand *self) { return self->data & 0xFFFF; }

LogicAnalyzerFlags sc_get_flags(SumpCommand *self) {
  uint8_t flag_byte = (self->data >> 24);
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
  for (int i = 0; i < 4; i++) {
    out.data <<= 8;
    out.data |= getchar();
  }
  return out;
}
