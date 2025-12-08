#include "bitset.h"
#include <stdbool.h>

Bitset32 bitset_from_uint32(uint32_t value) {
  Bitset32 out = {.data = value};
  return out;
}

void bitset_set(Bitset32 *self, int idx, bool value) {
  self->data |= value << idx;
}

bool bitset_get(Bitset32 *self, int idx) {
  return (self->data & (1 << idx)) >> idx;
}

uint32_t bitset_get_raw(Bitset32 *self) { return self->data; }

bool bitset_is_empty(Bitset32 *self) { return self->data == 0; }
