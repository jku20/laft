#include <stdio.h>

// For memcpy
#include <string.h>

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <pico/stdio.h>

const int PICO_CLOCK_SPEED = 125000000;

int main() {
  set_sys_clock_hz(PICO_CLOCK_SPEED, true);
  stdio_init_all();

  for (int i = 8; i < 8 + 15; i++) {
    gpio_init(i);
    gpio_set_dir_in_masked(1 << i);
  }

  gpio_init(5);
  gpio_set_dir_out_masked(1 << 5);
  gpio_put(5, 0);
  gpio_init(28);
  gpio_set_dir_out_masked(1 << 28);
  gpio_put(28, 0);

  printf("configured, usb, starting pwm!\n");

  // Flicker GPIO 27 which is connected to GPIO 8 so we can read a signal.
  gpio_set_function(27, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(27);
  pwm_set_wrap(slice_num, 3);
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 3);
  pwm_set_enabled(slice_num, true);

  printf("configured pwm, spinning forever now!");
  // Everything is interrupt driven so just loop here
  while (1) {
    tight_loop_contents();
  }
}
