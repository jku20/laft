 /*
 * input time capture using PIO as a timer/edge detector
 * 51 clock ticks per cycle at 1 MHz input
 * works to at least 10  MHz, but no usable accuracy above ~2 MHz
 * -- PIO has 8 instructions which implements a counter and edge state machine
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
#include "pio_input_capture_3.pio.h"
//  

// ===========================================
//
// core 0 will run the UI for PIO testing
//   and blinks an LED
// set up PWM for 50% duty cycle up to 1 MHz
// 8-slot FIFO acts as buffer for values at high speed

// locations to store edge time 
unsigned int PIO_time[8] ;
// pwm slice
uint slice_num ;
// pio divider between 1 and 65536
// PWM divider  between 1 and 256
int PWM_divider=1, PIO_divider=1, PWM_duty=500, PWM_cycles=1000 ; ;

 //  pio assembler link
PIO pio = pio1;
uint offset ;
// set up pio state machine
/*
void pio_test_start(PIO pio, uint sm, uint offset, uint pin) {
    offset = pio_add_program(pio, &capture_program);
    capture_program_init(pio, sm, offset, pin);
    pio_sm_set_clkdiv (pio, sm, 1.0) ;
    // drain the 4-slot input fifo
    // drain the transmit fifo
    pio_sm_clear_fifos (pio, sm);
   // pio_sm_drain_tx_fifo (pio, sm);
   // pio_sm_get (pio, sm);
   // pio_sm_get (pio, sm);
    //pio_sm_get (pio, sm);
   // pio_sm_get (pio, sm);
    // and turn on the state machine
    pio_sm_set_enabled(pio, sm, true);
}
*/

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

    if(PWM_duty*PWM_divider < 6*PIO_divider) {
      printf("!!PWM pulse length to short for PIO setting\n\r");
      printf("!!PIO may miss rising edges\n\r");
      continue;
    }

    // max count for PWM -- sets the period
    pwm_set_wrap(slice_num, PWM_cycles); 
    // initial duty cycle 
    pwm_set_chan_level(slice_num, PWM_CHAN_A , PWM_duty ) ; // two cycle pulse
    // print exact value to compare to meaured values below
   printf("Actual set PWM period=%6.3f uSec\n\r", (float)(PWM_cycles)*(float)PWM_divider/(125) ) ;  
   //
   pwm_set_clkdiv(slice_num, (float)PWM_divider);
   //
   
   pio_sm_set_enabled (pio, 0, false) ;
   pio1_hw->sm[0].clkdiv = (PIO_divider<<16)  ;
   pio_sm_clkdiv_restart (pio, 0) ;
   pio_sm_set_enabled (pio, 0, true) ;
   //

    //delete what ever is left in the capture FIFO
    pio_sm_clear_fifos (pio, 0) ;
    // zero the PWM counter and start it
    pwm_set_counter (slice_num, 0);
    pwm_set_enabled(slice_num, true);

    // get 8 time stamps (FIFO depth)
    for(int i=0; i<N; i++){
      PIO_time[i] = pio_sm_get_blocking (pio, 0);
    }
    
    // and stop it
    pwm_set_enabled(slice_num, false);

    // read back the time stamps
    printf("Measured PWM periods:\n\r") ;  
    printf("time stamp, pio cycles, time\n\r");
    for(int i=1; i<N; i++){
      delta_time = (signed int)PIO_time[i-1] - (signed int)PIO_time[i] ;
      printf("t=%x  dt=%d uSec=%6.3f\n\r", 
                PIO_time[i], delta_time, (float)(delta_time)*(float)PIO_divider/(20.83333)) ;   //125/6
    }

   }PT_END(pt);
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
    //int clk_div_int=PIO_divider, clk_div_fract=0 ;
    //pio0_hw->sm[0].clkdiv = (clk_div_int<<16) | (clk_div_fract<<8) ;
    pio_sm_set_enabled(pio, 0, true);

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
/* the PIO program
.program capture
; In which we try to use the PIO as a timer
; then blast the captured times to the IN FIFO

	; gpio 3 rising edge occurance will be time
	; stamped and transfered to FIFO
	; This program just counts while waiting for the edge,
	; loads a time into a FIFO,
	; Then counts and waits for a falling edge and loops
  ; time stamp resolution is 2 PIO cycles
	;
	set x 0x00		; init x timer 0x00000000 
	mov x ~x		; NOT the bits into x 0xffffffff

.wrap_target
wait1:
	jmp pin got1 	; wait for rising edge + delay a cycle
	jmp x-- wait1	; loop is 2 cycles (two jmps+cycle-to-jmp) add one wait
got1:
	in x 32		; send the counter to isr and autopush
   push noblock
	; now we need to debpunce to wait for a low level on the pin
wait0:
	jmp pin not0
	jmp wait1       ; got the 0, so wait for 1
not0:
	jmp x-- wait0	; loop is 2 cycles
	;
.wrap             ; 

% c-sdk {
// this is a raw helper function for use by the user which sets up the GPIO output, 
//   and configures the SM to output on a particular pin

void capture_program_init(PIO pio, uint sm, uint offset) {
   // the jump pin
   pio_gpio_init(pio, 3); 
   //
   pio_sm_config c = capture_program_get_default_config(offset);  
   // JMP pin is specified separately as GPIO #, GPIO 4
   sm_config_set_jmp_pin (&c, 3) ;
   // no output FIFO from core0, all input to core0
   sm_config_set_fifo_join (&c, PIO_FIFO_JOIN_RX) ;
   // autopush the isr to eliminate one istrcution
   //sm_config_set_in_shift (&c, true, true, 1) ; 
   pio_sm_init(pio, sm, offset, &c);
}
%}
*/