#include "bitset.h"
#include <stdbool.h>
#include <stdint.h>

#define NUM_STAGES 4

typedef struct {
  uint16_t delay;
  uint8_t channel;
  uint8_t level;
  bool serial;
  bool start;
} TriggerConfiguration;

typedef struct {
  bool demux;
  bool filter;
  uint8_t channel_groups;
  bool external;
  bool inverted;
} LogicAnalyzerFlags;

const uint8_t LOGIC_ANALYZR_ID[] = {'1', 'A', 'L', 'S'};
const uint8_t SUMP_ID_LEN = 4;
typedef struct {
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

void la_set_paused(LogicAnalyzer *self, bool paused);
void la_reset(LogicAnalyzer *self);
void la_init(LogicAnalyzer *self);
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
void la_transmit(LogicAnalyzer *self, uint8_t *buf, int len);
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
