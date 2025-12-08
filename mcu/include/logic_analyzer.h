#pragma once

#include <hardware/pio.h>
#include <stdbool.h>
#include <stdint.h>

#include "bitset.h"

#define NUM_STAGES 4

typedef struct {
  /** The samples stall for after match before either starting capture or
   * incrementing the trigger level. */
  uint16_t delay;

  /** Used when in serial mode. */
  uint8_t channel;

  /** The level at which to use this trigger. Initially this is at level 0 and
   * is incremented on successful matches to triggers. This is unsupported. */
  uint8_t level;

  /** True if the trigger is in serial mode, else false and the trigger is in
   * parallel mode. Serial mode is unsupported. */
  bool serial;

  /** If true start capturing, else simply increase the trigger level. */
  bool start;
} TriggerConfiguration;

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
  uint base_pin;
} CircularBuffer;

/**
 * Initializes the CircularBuffer and starts the DMA channel and GPIO. This
 * requires a DMA channel and a PIO state machine.
 */
void cb_init(CircularBuffer *self, uint base_pin, PIO pio, uint sm, uint dma);

/** Sets up the PIO state machine to start collecting data based on a trigger.
 * We do not implement triggering on specific levels as due to limitations from
 * PIO blocks, we can't well run multiple stages at the same time. Instead, we
 * run stages with nonzero trigger masks ordered 0 to NUM_STAGES. Additionally,
 * particularly bad masks (alternating 1s and 0s) may cause the PIO memory to
 * overflow. We just assume this doesn't happen and silently do undefined
 * horrors.*/
void cb_arm_to_start_collecting(CircularBuffer *self,
                                Bitset32 trigger_mask[NUM_STAGES],
                                Bitset32 trigger_value[NUM_STAGES],
                                TriggerConfiguration trigger_config[NUM_STAGES],
                                uint32_t clock_div, int transfer_count);

/** Stops reading to the circular buffer from the PIO. */
void cb_stop_buf_population(CircularBuffer *self);

/** Starts reading to the circular buffer from the PIO. */
void cb_start_buf_population(CircularBuffer *self);

/**
 * Of these flags, we only have support for channel groups and at that only
 * channel groups 0 and 1.
 */
typedef struct {
  /** Copy channels 0 and 1 to all 2 and 3. */
  bool demux;

  /** Use noise filter on input module, unsupported. */
  bool filter;

  uint8_t channel_groups;

  /** Use external clock, unsupported. */
  bool external;

  /** Invert the clock, unsupported. */
  bool inverted;
} LogicAnalyzerFlags;

typedef struct {
  CircularBuffer buf;

  bool paused;
  /** The logic analyzer's current trigger masks, indexed by their stage. */
  Bitset32 trigger_mask[NUM_STAGES];

  /** The logic analyzer's current trigger values, indexed by their stage. */
  Bitset32 trigger_value[NUM_STAGES];

  /** The logic analyzer's current trigger configurations, indexed by their
   * stage. */
  TriggerConfiguration trigger_config[NUM_STAGES];

  /** The clock divider for the sampler. */
  uint32_t clock_div;

  uint16_t read_count;
  uint16_t delay_count;

  LogicAnalyzerFlags flags;
} LogicAnalyzer;

typedef enum {
  Reset = 0x00,
  Run = 0x01,
  Id = 0x02,
  Xon = 0x11,
  Xoff = 0x13,
  SetTriggerMaskStage0 = 0xc0,
  SetTriggerMaskStage1 = 0xc4,
  SetTriggerMaskStage2 = 0xc8,
  SetTriggerMaskStage3 = 0xcc,
  SetTriggerValueStage0 = 0xc1,
  SetTriggerValueStage1 = 0xc5,
  SetTriggerValueStage2 = 0xc9,
  SetTriggerValueStage3 = 0xcd,
  SetTriggerConfigStage0 = 0xc2,
  SetTriggerConfigStage1 = 0xc6,
  SetTriggerConfigStage2 = 0xca,
  SetTriggerConfigStage3 = 0xce,
  SetDivider = 0x80,
  SetReadAndDelayCount = 0x81,
  SetFlags = 0x82,
} SumpCommandType;

typedef struct {
  SumpCommandType ty;
  uint32_t data;
} SumpCommand;

void la_init(LogicAnalyzer *self, uint base_pin, PIO pio, uint sm, uint dma);
void la_set_paused(LogicAnalyzer *self, bool paused);
void la_reset(LogicAnalyzer *self);
uint8_t *la_get_id(LogicAnalyzer *self);
void la_arm(LogicAnalyzer *self);
void la_set_trigger_mask(LogicAnalyzer *self, Bitset32 mask, int stage);
void la_set_trigger_value(LogicAnalyzer *self, Bitset32 value, int stage);
void la_set_trigger_config(LogicAnalyzer *self, TriggerConfiguration config,
                           int stage);
void la_set_clock_divider(LogicAnalyzer *self, uint32_t clock_div);
void la_set_read_count(LogicAnalyzer *self, uint16_t read_count);
void la_set_delay_count(LogicAnalyzer *self, uint16_t delay_count);
void la_set_flags(LogicAnalyzer *self, LogicAnalyzerFlags flags);
void la_exec_command(LogicAnalyzer *self, SumpCommand *cmd);

SumpCommandType sc_get_ty(SumpCommand *self);
Bitset32 sc_get_mask(SumpCommand *self);
Bitset32 sc_get_value(SumpCommand *self);
int sc_get_stage(SumpCommand *self);
TriggerConfiguration sc_get_trigger_configuration(SumpCommand *self);
uint32_t sc_get_clock_div(SumpCommand *self);
uint16_t sc_get_read_count(SumpCommand *self);
uint16_t sc_get_delay_count(SumpCommand *self);
LogicAnalyzerFlags sc_get_flags(SumpCommand *self);
SumpCommand sc_read_from_stdin();
