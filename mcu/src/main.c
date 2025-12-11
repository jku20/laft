#include "debug.h"
#include <hardware/dma.h>
#include <stdio.h>

// For memcpy
#include <string.h>

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <pico/stdio.h>

#include "logic_analyzer.h"

const int PICO_CLOCK_SPEED = 125000000;

int main() {
#ifdef ECHO
  static char buf[1000];
  set_sys_clock_hz(PICO_CLOCK_SPEED, true);
  printf("start\n");
  // Set the LED
  gpio_init(25);
  gpio_set_dir_out_masked(1 << 25);
  gpio_put(25, 1);
  stdio_init_all();
  while (1) {
    scanf("%s\n", buf);
    printf("%s\n", buf);
  }
#endif
#ifndef ECHO
  set_sys_clock_hz(PICO_CLOCK_SPEED, true);
  stdio_init_all();

  // Set the LED
  gpio_init(25);
  gpio_set_dir_out_masked(1 << 25);
  gpio_put(25, 1);

  // Initialize GPIOS
  for (int i = 8; i < 8 + 15; i++) {
    gpio_init(i);
    gpio_set_dir_in_masked(1 << i);
  }

  gpio_init(26);
  gpio_set_dir_in_masked(1 << 26);

  gpio_init(5);
  gpio_set_dir_out_masked(1 << 5);
  gpio_put(5, 0);

  // Initialize the logic analyzer
  LogicAnalyzer la;
  la_init(&la, 8, pio0, pio_claim_unused_sm(pio0, true),
          dma_claim_unused_channel(true));

  // Flicker GPIO 27 which is connected to GPIO 8 so we can read a signal.
  gpio_set_function(27, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(27);
  pwm_set_wrap(slice_num, 3);
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 3);
  pwm_set_enabled(slice_num, true);

#ifdef DEBUG
  printf("configured pwm, spinning forever now!");
#endif
  // Everything is interrupt driven so just loop here
  while (1) {
#ifdef DEBUG
    printf("reading command\n");
#endif
    SumpCommand cmd = sc_read_from_stdin();
#ifdef DEBUG
    printf("executing command\n");
#endif
    la_exec_command(&la, &cmd);
  }
#endif
}
