#include "logic_analyzer.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void la_set_paused(LogicAnalyzer *self, bool paused) { self->paused = paused; }
void la_init(LogicAnalyzer *self) { self->paused = false; }
uint8_t *la_get_id(LogicAnalyzer *self) { return (uint8_t *)LOGIC_ANALYZR_ID; }
void la_arm(LogicAnalyzer *self) {}
void la_transmit(LogicAnalyzer *_self, uint8_t *buf, int len) {
  for (int i = 0; i < len; i++) {
    putchar(buf[i]);
  }
}

void la_queue_command(LogicAnalyzer *self, SumpCommand *cmd) {
  switch (sump_get_type(cmd)) {
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
  }
}

SumpCommandType sump_get_type(SumpCommand *self) { return self->ty; }
