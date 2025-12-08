#pragma once

#include <stdbool.h>
#include <stdint.h>
typedef struct {
  uint32_t data;
} Bitset32;

Bitset32 bitset_from_uint32(uint32_t value);
void bitset_set(Bitset32 *self, int idx, bool value);
bool bitset_get(Bitset32 *self, int idx);
uint32_t bitset_get_raw(Bitset32 *self);
