#include "bsp/board_api.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include "tusb_config.h"
#include "usb_descriptors.h"

int main() {
  stdio_init_all();
  board_init();
  tusb_init();

  while (true) {
    tud_task();
  }
}
