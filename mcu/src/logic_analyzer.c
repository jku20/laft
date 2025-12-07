#include "logic_analyzer.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void la_set_paused(LogicAnalyzer *self, bool paused) { self->paused = paused; }

void la_reset(LogicAnalyzer *self) {
  for (int i = 0; i < NUM_STAGES; i++) {
    self->trigger_mask[i] = bitset_from_uint32(0);
    self->trigger_value[i] = bitset_from_uint32(0);
  }
  self->paused = false;
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

void la_transmit(LogicAnalyzer *_self, uint8_t *buf, int len) {
  for (int i = 0; i < len; i++) {
    putchar(buf[i]);
  }
}

void la_queue_command(LogicAnalyzer *self, SumpCommand *cmd) {
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
}
