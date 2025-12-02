/**
 * Logic Analyzer Final Project
 * by Jeremy Ku-Benjet, Colin Muessig, Matthew Hurford
 *
 * Base code from a Raspberry Pi demo: 
 * https://github.com/raspberrypi/pico-examples/blob/master/pio/logic_analyser/logic_analyser.c
 * 
 * This program captures samples from a group of pins, at a fixed rate, once a trigger condition is 
 * detected (level condition on one pin). The samples are transferred to a capture buffer using the 
 * system DMA.
 * 
 * 1 to 32 pins can be captured, at a sample rate no greater than system clock frequency.
 */

#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"

// Some logic to analyse:
#include "hardware/structs/pwm.h"

// Constants for DMA bit capture
const uint CAPTURE_PIN_BASE_LS1 = 8;  // Base GPIO # for level shifter 1 (A1 to A8)
const uint CAPTURE_PIN_COUNT_LS1 = 8; // # of pins to sample from for level shifter 1 (GP8 to GP15)
const uint CAPTURE_N_SAMPLES = 96;

static inline uint bits_packed_per_word(uint pin_count) {
    // If the number of pins to be sampled divides the shift register size, we
    // can use the full SR and FIFO width, and push when the input shift count
    // exactly reaches 32. If not, we have to push earlier, so we use the FIFO
    // a little less efficiently.
    const uint SHIFT_REG_WIDTH = 32;
    return SHIFT_REG_WIDTH - (SHIFT_REG_WIDTH % pin_count);
}

void logic_analyser_init(PIO pio, uint sm, uint pin_base, uint pin_count, float div) {
    // Load a program to capture n pins. This is just a single `in pins, n` instruction with a wrap.
    uint16_t capture_prog_instr = pio_encode_in(pio_pins, pin_count);
    struct pio_program capture_prog = {
            .instructions = &capture_prog_instr,
            .length = 1, // 1 instruction
            .origin = -1 // Load it anywhere there is room
    };
    uint offset = pio_add_program(pio, &capture_prog); // Offset = starting address for program

    // Configure state machine to loop over this `in` instruction forever, with autopush enabled.
    pio_sm_config c = pio_get_default_sm_config(); // Get state machine defaults
    sm_config_set_in_pins(&c, pin_base); // Tell state machine which GPIO # to use as a base
    sm_config_set_wrap(&c, offset, offset); // Causes program to loop at 'offset' address
    sm_config_set_clkdiv(&c, div); // Set state machine clock divider (1.f = PIO clock speed)

    // Note that we may push at a < 32 bit threshold if pin_count does not divide into 32. We are 
    // using shift-to-right, so the sample data ends up left-justified in the FIFO in this case, 
    // with some zeroes at the LSBs.
    sm_config_set_in_shift(&c, true, true, bits_packed_per_word(pin_count));
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c); // Instantiate the program on this state machine
}

void logic_analyser_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words,
                        uint trigger_pin, bool trigger_level) {
    // Disable state machine to begin with
    pio_sm_set_enabled(pio, sm, false);

    // Need to clear input shift counter, as well as FIFO, because there may be partial ISR contents 
    // left over from a previous run.
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    // Configure the DMA channel
    dma_channel_config c = dma_channel_get_default_config(dma_chan); // Get DMA defaults
    channel_config_set_read_increment(&c, false); // Do not increment read address (fixed at PIO FIFO)
    channel_config_set_write_increment(&c, true); // Do increment the write address (location in buffer)
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false)); // Make DMA transfer when a sample is ready
   
    // Set the DMA channel with new configs and start it
    dma_channel_configure(dma_chan, &c,
        capture_buf,        // Destination pointer
        &pio->rxf[sm],      // Source pointer
        capture_size_words, // Number of transfers
        true                // Start immediately
    );

    // Insert a wait instruction (freezes until trigger is detected) and enable the state machine
    pio_sm_exec(pio, sm, pio_encode_wait_gpio(trigger_level, trigger_pin));
    pio_sm_set_enabled(pio, sm, true);
}

void print_capture_buf(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples) {
    // Display the capture buffer in text form, like this:
    // 00: __--__--__--__--__--__--
    // 01: ____----____----____----

    printf("Capture:\n");
    // Each FIFO record may be only partially filled with bits, depending on whether pin_count is a factor of 32.
    uint record_size_bits = bits_packed_per_word(pin_count);
    for (uint pin = 0; pin < pin_count; ++pin) {
        printf("%02d: ", pin + pin_base);
        for (uint32_t sample = 0; sample < n_samples; ++sample) {
            uint bit_index = pin + sample * pin_count;
            uint word_index = bit_index / record_size_bits;
            // Data is left-justified in each FIFO entry, hence the (32 - record_size_bits) offset
            uint word_mask = 1u << (bit_index % record_size_bits + 32 - record_size_bits);
            printf(buf[word_index] & word_mask ? "-" : "_");
        }
        printf("\n");
    }
}

int main() {
    stdio_init_all();
    printf("Starting PIO Logic Analyzer\n");

    // We capture into a uint32_t buffer for best DMA efficiency. Here, we calculate how many
    // 32-bit words the DMA must store to hold all samples.
    // - First line determines the number of bits actually being sampled.
    // - Second and third lines implement ceiling division to determine the # of words needed
    //   (i.e. ceil(a/b) = (a+b-1)/b ).
    uint total_sample_bits = CAPTURE_N_SAMPLES * CAPTURE_PIN_COUNT_LS1;
    total_sample_bits += bits_packed_per_word(CAPTURE_PIN_COUNT_LS1) - 1;
    uint buf_size_words = total_sample_bits / bits_packed_per_word(CAPTURE_PIN_COUNT_LS1);

    // Allocate space for bit capture buffer
    uint32_t *capture_buf = malloc(buf_size_words * sizeof(uint32_t));
    hard_assert(capture_buf);

    // Grant high bus priority to the DMA so it can shove the processors out of the way. This 
    // is needed if pushing things up to >16bits/clk, i.e. if you need to saturate the bus completely.
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    // Select state machine 0 on pio0, as well as DMA channel 0
    PIO pio = pio0;
    uint sm = 0;
    uint dma_chan = 0;

    // Initialize logic analyzer
    logic_analyser_init(pio, sm, CAPTURE_PIN_BASE_LS1, CAPTURE_PIN_COUNT_LS1, 1.f);

    // Arm logic analyzer to do a burst capture on the first transition of the base pin
    printf("Arming Trigger\n");
    bool trigger_level = true; // true = rising edge, false = falling edge
    logic_analyser_arm(pio, sm, dma_chan, capture_buf, buf_size_words, CAPTURE_PIN_BASE_LS1, trigger_level);

    // PWM Example Code
    // ------------------------------------------------------------------------
    printf("Starting PWM example\n");
    gpio_set_function(CAPTURE_PIN_BASE_LS1, GPIO_FUNC_PWM);
    gpio_set_function(CAPTURE_PIN_BASE_LS1 + 1, GPIO_FUNC_PWM);

    // Topmost value of 3: count from 0 to 3 and then wrap, so period is 4 cycles
    pwm_hw->slice[0].top = 3;
    // Divide frequency by two to slow things down a little
    pwm_hw->slice[0].div = 4 << PWM_CH0_DIV_INT_LSB;
    // Set channel A to be high for 1 cycle each period (duty cycle 1/4) and
    // channel B for 3 cycles (duty cycle 3/4)
    pwm_hw->slice[0].cc =
        (1 << PWM_CH0_CC_A_LSB) |
        (3 << PWM_CH0_CC_B_LSB);
    // Enable this PWM slice
    pwm_hw->slice[0].csr = PWM_CH0_CSR_EN_BITS;
    // ------------------------------------------------------------------------

    // Wait until the last sample comes in from the DMA.
    dma_channel_wait_for_finish_blocking(dma_chan);

    // Print capture buffer over serial
    print_capture_buf(capture_buf, CAPTURE_PIN_BASE_LS1, CAPTURE_PIN_COUNT_LS1, CAPTURE_N_SAMPLES);
}
