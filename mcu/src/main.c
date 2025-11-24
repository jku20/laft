#include "bsp/board_api.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include "tusb_config.h"
#include "usb_descriptors.h"

#define NUM_TRACES 16

#define TRACE_BUFFER_SIZE 1024

typedef struct {
  uint8_t trace_buffer[TRACE_BUFFER_SIZE];
} laft_state;

static laft_state state;

void reset_state(laft_state s) {}

int main() {
  stdio_init_all();
  board_init();
  tusb_init();

  // Initialize mock data
  for (int i = 0; i < TRACE_BUFFER_SIZE; i++) {
    state.trace_buffer[i] = i & 0xFF;
  }

  while (true) {
    tud_task();
  }
}

void tud_mount_cb() { reset_state(state); }

/**
 * Respond to write bulk transfers.
 */
void tud_vendor_rx_cb(uint8_t intf, uint8_t const *buffer, uint16_t bufsize) {
  static uint8_t out_buffer[64];

  uint16_t size = (buffer[1] << 8) + buffer[2];
  int bytes_in_out_buf = 0;
  // Multiply size by 2 because we have 16 traces, so we want 16 * size bits.
  for (int i = 0; i < size * NUM_TRACES / 8; i++) {
    if (bytes_in_out_buf == 64) {
      uint32_t written = tud_vendor_write(out_buffer, 64);
      tud_vendor_flush();
      printf("writing %d bytes\n", written);
      bytes_in_out_buf = 0;
    }
    out_buffer[bytes_in_out_buf++] = state.trace_buffer[i];
  }
  if (bytes_in_out_buf != 0) {
    uint32_t written = tud_vendor_write(out_buffer, 64);
    tud_vendor_flush();
    printf("writing %d bytes\n", written);
    bytes_in_out_buf = 0;
  }
  tud_vendor_read_flush();
}

void tud_vendor_tx_cb(uint8_t intf, uint32_t sent_bytes) {}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request) {}
