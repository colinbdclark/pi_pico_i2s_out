/*
Copyright 2025 Kinetic Light and Lichen Community Systems
Licensed under the BSD-3 License.
*/

#include <stdlib.h>
#include "pi_pico_i2s_out.pio.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "synth.h"

#define DATA_PIN 9
#define BCLK_PIN 10
#define LRCLK_PIN 11
#define BIT_DEPTH 32
#define SAMPLE_RATE 48000
#define NUM_CHANNELS 2
#define PIO_CYCLES_PER_INSTRUCTION 2
#define PIO_INSTRUCTIONS_PER_BIT 2
#define CPU_CLOCK_SPEED 153600 // KHz
#define BLOCK_SIZE 6
#define STEREO_BLOCK_SIZE (BLOCK_SIZE * NUM_CHANNELS)
#define DOUBLE_BUFFER_SIZE (STEREO_BLOCK_SIZE * 2)
#define GAIN 0.5f
#define FREQUENCY 120.0f

int32_t output[DOUBLE_BUFFER_SIZE] = {0};
int32_t* bufferPointers[2];
int32_t bufferPointerIdx = 0;
int dataChannel;
Oscillator<BLOCK_SIZE> osc;

void pi_pico_i2s_out_program_init(PIO pio, int sm) {
    int offset = pio_add_program(pio, &pi_pico_i2s_out_program);
    pio_sm_config sm_config = pi_pico_i2s_out_program_get_default_config(
        offset);

    pio_gpio_init(pio, DATA_PIN);
    pio_gpio_init(pio, BCLK_PIN);
    pio_gpio_init(pio, LRCLK_PIN);
    sm_config_set_out_pins(&sm_config, DATA_PIN, 1);
    sm_config_set_sideset_pin_base(&sm_config, BCLK_PIN);

    sm_config_set_out_shift(&sm_config, false, false, BIT_DEPTH);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);

    float clockSpeed = (float) clock_get_hz(clk_sys);
    float bclkSpeed = (float) SAMPLE_RATE * (float) NUM_CHANNELS *
        (float) BIT_DEPTH;
    float pioSpeed = bclkSpeed * (float) PIO_CYCLES_PER_INSTRUCTION *
        (float) PIO_INSTRUCTIONS_PER_BIT;
    float clockDivision = clockSpeed / pioSpeed;
    sm_config_set_clkdiv(&sm_config, clockDivision);

    pio_sm_init(pio, sm, offset, &sm_config);

    uint32_t pinMask = (1u << DATA_PIN) | (3u << BCLK_PIN);
    pio_sm_set_pindirs_with_mask(pio, sm, pinMask, pinMask);
    pio_sm_set_pins(pio, sm, 0);
}

void data_dma_init(PIO pio, int sm, int dataChannel) {
    dma_channel_config config = dma_channel_get_default_config(
        dataChannel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, false);
    channel_config_set_dreq(&config, pio_get_dreq(pio, sm, true));

    dma_channel_configure(dataChannel,
        &config, &pio->txf[sm], bufferPointers[0], STEREO_BLOCK_SIZE,
        true);
}

void dma_irq_init(int dataChannel, void(*dma_handler)(void)) {
    dma_channel_set_irq0_enabled(dataChannel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

void dma_init(PIO pio, int sm, int dataChannel,
    void(*dma_handler)(void)) {
    bufferPointers[0] = &output[0];
    bufferPointers[1] = &output[STEREO_BLOCK_SIZE];

    data_dma_init(pio, sm, dataChannel);
    dma_irq_init(dataChannel, dma_handler);
}

void handler() {
    osc.generateBlock();

    int32_t* bufferToFill = bufferPointers[bufferPointerIdx];
    for (size_t i = 0; i < BLOCK_SIZE; i++) {
        float sample = osc.output[i];
        int32_t converted = (int32_t)(sample * 2147483647.0f * GAIN);
        bufferToFill[i * 2] = converted;
        bufferToFill[i * 2 + 1] = converted;
    }

    bufferPointerIdx = 1 - bufferPointerIdx;

    dma_hw->ints0 = 1u << dataChannel;
    dma_channel_set_read_addr(dataChannel,
        bufferPointers[bufferPointerIdx], true);
}

int main() {
    set_sys_clock_khz(CPU_CLOCK_SPEED, true);

    osc.init({SAMPLE_RATE}, FREQUENCY, WaveShapes::sine);

    PIO pio = pio0;
    int sm = pio_claim_unused_sm(pio, true);
    pi_pico_i2s_out_program_init(pio, sm);

    dataChannel = dma_claim_unused_channel(true);
    dma_init(pio, sm, dataChannel, handler);

    pio_sm_set_enabled(pio, sm, true);

    while (true) {
        tight_loop_contents();
    }
}
