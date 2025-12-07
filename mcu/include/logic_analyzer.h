#include <stdbool.h>
#include <stdint.h>

const uint8_t LOGIC_ANALYZR_ID[] = {'S', 'L', 'A', '1'};
const uint8_t SUMP_ID_LEN = 4;
typedef struct {
  bool paused;
} LogicAnalyzer;

typedef enum {
  Reset = 0x00,
  Run = 0x01,
  Id = 0x02,
  Xon = 0x11,
  Xoff = 0x13,
} SumpCommandType;

typedef struct {
  SumpCommandType ty;
} SumpCommand;

void la_set_paused(LogicAnalyzer *self, bool paused);
void la_init(LogicAnalyzer *self);
uint8_t *la_get_id(LogicAnalyzer *self);
void la_arm(LogicAnalyzer *self);
void la_transmit(LogicAnalyzer *self, uint8_t *buf, int len);

void la_queue_command(LogicAnalyzer *self, SumpCommand *cmd);

SumpCommandType sump_get_type(SumpCommand *self);
