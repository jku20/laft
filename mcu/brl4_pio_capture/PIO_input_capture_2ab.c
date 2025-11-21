 /*
 * input time capture using PIO as a timer/edge detector
 * TWO PIOs together give one-cycle esolution, although each of them has two cycle esolution
 * You get one-cycle resoluti by delaying one pIO BY ONE-CYCLE at startup
 * -- core0 readback dumps up to 8 time stamps from the FIFO at high frequency
 *    can run indefinitely at below a few KHz.
 */  
 // === VGA ================
#include "vga16_graphics.h"
// ========================

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <hardware/irq.h>
#include <hardware/adc.h>
#include <hardware/spi.h>
#include <hardware/pwm.h>

#include "hardware/pio.h"
#include "hardware/dma.h"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "stdio.h"
#include <string.h>
#include <pico/multicore.h>
#include "hardware/sync.h"
#include "stdlib.h"
#include "math.h"

// ==========================================
// === protothreads 
// ==========================================
// protothreads header
#include "pt_cornell_rp2040_v1_3.h"

// multicore sync functions
// NOTE -- SDK uses spin-locks 0-15 -- DO NOT use 16 to 31 OK
// sycc.h also has get_cpuid function
#include "hardware/sync.h"
//
// PIO program/setup include
#include "pio_input_capture_2a.pio.h"
#include "pio_input_capture_2b.pio.h"
//  

// ===========================================
//
// core 0 will run the UI for PIO testing
// 8-slot FIFO acts as buffer for values at high speed

// locations to store edge time 
unsigned int PIO_time_a[8], PIO_time_b[8] ;
// pwm slice
uint slice_num ;
// pio divider between 1 and 65536
// PWM divider  between 1 and 256
int PWM_divider=1, PIO_divider=1, PWM_duty=100, PWM_cycles=200 ; ;

 //  pio assembler link
PIO pio = pio1;
uint offset ;

// ==================================================
// === toggle25 thread 
// ==================================================
// the on-board LED blinks
static PT_THREAD (protothread_toggle25(struct pt *pt))
{
  PT_BEGIN(pt);
    static bool LED_state = false ;
    
    // set up LED p25 to blink
    gpio_init(25) ;	
    gpio_set_dir(25, GPIO_OUT) ;
    gpio_put(25, true);
    // data structure for interval timer
    PT_INTERVAL_INIT() ;
     
    while(1) {
      // yield time 0.1 second
      PT_YIELD_INTERVAL(200000) ;

      // toggle the LED on PICO
      LED_state = LED_state? false : true ;
      gpio_put(25, LED_state);
      //
      // NEVER exit while
    } // END WHILE(1)
  PT_END(pt);
} // blink thread

//
// ==================================================
// === serial and PIO read thread 
// ==================================================


// serial thead
static PT_THREAD (protothread_serial(struct pt *pt))
{
     PT_BEGIN(pt);
    // number of edge samples to grab
    static int N=8 ;
    static int delta_time ;
    static float f_delta_time ;

   // === 
   while(1){  
    //printf("hi");
    // test input and print
    // In PuTTY setup, Terminal panel:
    // set Local echo and Local editing to 'FORCE ON'
    sprintf(pt_serial_out_buffer, "PWM_cycles, PWM_duty, PWM_divider, PIO_divider> ");
    serial_write ;
    serial_read ;
    // convert input string to number
    sscanf(pt_serial_in_buffer,"%d %d %d %d", &PWM_cycles, &PWM_duty, &PWM_divider, &PIO_divider) ;
    if(PWM_cycles > 65535) {
      printf("!!Invalid cycles > 2^16\n\r");
      continue;
    }
    if(PWM_duty >= PWM_cycles) {
      printf("!!PWM_duty must be less than PWM_cycles\n\r");
      continue;
    }
    if(PWM_divider > 256) {
      printf("!!Invalid PWM divider > 2^8\n\r");
      continue;
    }
    if(PIO_divider > 65535) {
      printf("!!Invalid PIO divider > 2^16\n\r");
      continue;
    }

    if(PWM_cycles*PWM_divider/PIO_divider < 8) {
      printf("!!PWM period to short for PIO setting\n\r");
      printf("!!PIO may hang\n\r");
      continue;
    }

    if(PWM_duty*PWM_divider < 2*PIO_divider) {
      printf("!!PWM pulse length to short for PIO setting\n\r");
      printf("!!PIO may miss rising edges\n\r");
      continue;
    }

    // max count for PWM -- sets the period
    pwm_set_wrap(slice_num, PWM_cycles); 
    // initial duty cycle 
    pwm_set_chan_level(slice_num, PWM_CHAN_A , PWM_duty ) ; // two cycle pulse
    // print exact value to compare to meaured values below
    // to get period: add 1 to PWM_cycles since count starts at zero
   printf("Actual set PWM period=%6.3f uSec\n\r", (float)(PWM_cycles+1)*(float)PWM_divider/(125) ) ;  
   //
   pwm_set_clkdiv(slice_num, (float)PWM_divider);
   //
   // re synch the PIOs ater a clock divider change
   pio_set_sm_mask_enabled(pio, 0b0011, false) ;
   pio1_hw->sm[0].clkdiv = (PIO_divider<<16)  ;
   pio1_hw->sm[1].clkdiv = (PIO_divider<<16)  ;
   pio_sm_clkdiv_restart (pio, 0) ;
   pio_sm_clkdiv_restart (pio, 1) ;
   pio_set_sm_mask_enabled(pio, 0b0011, true) ;
   //

    //delete what ever is left in the capture FIFO
    pio_sm_clear_fifos (pio, 0) ;
    pio_sm_clear_fifos (pio, 1) ;
    // zero the PWM counter and start it
    pwm_set_counter (slice_num, 0);
    pwm_set_enabled(slice_num, true);

    // get 8 time stamps (FIFO depth)
    for(int i=0; i<N; i++){
      PIO_time_a[i] = pio_sm_get_blocking (pio, 0);
      PIO_time_b[i] = pio_sm_get_blocking (pio, 1);
    }
    
    // and stop thee pwm
    pwm_set_enabled(slice_num, false);

    // read back the time stamps
    printf("Measured PWM periods:\n\r") ;  
    printf("time stamp, pio cycles, time\n\r");
    //
    printf("ta=%x  tb=%x  \n\r", PIO_time_a[0], PIO_time_b[0]);
    for(int i=1; i<N; i++){
      // take the average of the two PIO elapsed times
      f_delta_time = (float)(PIO_time_a[i-1] - PIO_time_a[i] + PIO_time_b[i-1] - PIO_time_b[i]) * 0.5 ;
      // and print 
      printf("ta=%x  tb=%x  dt_a=%d  dt_b= %d  uSec=%6.3f\n\r", 
                PIO_time_a[i], PIO_time_b[i], PIO_time_a[i-1] - PIO_time_a[i], PIO_time_b[i-1] - PIO_time_b[i],
                (f_delta_time) * (float)PIO_divider/(62.5)) ;   
    }  
   }
   PT_END(pt);
 }

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){ 
  //  === add threads  ====================
  // for core 1
  // Initialize the VGA screen//
  //pt_add_thread(protothread_graphics);
  // === initalize the scheduler ==========
  pt_schedule_start ;
  // NEVER exits
  // ======================================
}

// ========================================
// === core 0 main
// ========================================
int main(){
  // set the clock
  //set_sys_clock_khz(250000, true); // 171us
  // start the serial i/o
  stdio_init_all() ;
  // announce the threader version on system reset
  printf("\n\rProtothreads RP2040 v1.3 two-core, priority\n\r");
     
  // ===================
  // === PWM channel ===
  // pwm set up
  #define pin4 4
  gpio_set_function(pin4, GPIO_FUNC_PWM);
  slice_num = pwm_gpio_to_slice_num(pin4);
  // want 1 mSec sq wave
  pwm_set_clkdiv(slice_num, (float)PWM_divider);
  pwm_set_clkdiv_mode(slice_num, PWM_DIV_FREE_RUNNING) ;
  // max count 
  pwm_set_wrap(slice_num, PWM_cycles); //
  // initial to ~half
  pwm_set_chan_level(slice_num, PWM_CHAN_A , PWM_duty) ; //
  // enable later
  pwm_set_enabled(slice_num, 0);

  // Initialize the VGA screen
  //initVGA() ;

  // ====================
  
  // ===  pio
  // init the PIO
    offset = pio_add_program(pio, &capture_program);   
    capture_program_init(pio, 0, offset);
    //
    offset = pio_add_program(pio, &capture_b_program);   
    capture_program_init(pio, 1, offset);
    //int clk_div_int=PIO_divider, clk_div_fract=0 ;
    //pio0_hw->sm[0].clkdiv = (clk_div_int<<16) | (clk_div_fract<<8) ;
    //pio_sm_set_enabled(pio, 0, true);
    // turn them both on at the same time
    pio_set_sm_mask_enabled(pio, 0b0011, true) ;

  // === add threads ======================
  // start core 1 threads
  //multicore_reset_core1();
  //multicore_launch_core1(&core1_main);

  // === config threads ===================
  // for core 0
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_toggle25);
  //
  // === initalize the scheduler ===============
  pt_schedule_start ;
  // NEVER exits
  // ===========================================
} // end main
///////////
// end ////
///////////
