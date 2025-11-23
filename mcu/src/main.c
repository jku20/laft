#include "bsp/board_api.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include "tusb_config.h"
#include "usb_descriptors.h"

typedef struct {

} laft_state;

static laft_state state;
static bool write;

void reset_state(laft_state s) {}

int main() {
  stdio_init_all();
  board_init();
  tusb_init();

  printf("initialized\n");
  while (true) {
    tud_task();
  }
}

void tud_mount_cb() { reset_state(state); }

/**
 * Respond to write bulk transfers.
 */
void tud_vendor_rx_cb(uint8_t intf, uint8_t const *buffer, uint16_t bufsize) {
  tud_vendor_write(buffer, bufsize);
  tud_vendor_flush();

  tud_vendor_read_flush();
  printf("finished fx\n");
}

void tud_vendor_tx_cb(uint8_t intf, uint32_t sent_bytes) {
  printf("finished tx\n");
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request) {
  printf("got control\n");
}
