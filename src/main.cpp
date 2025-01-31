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

#define CPU_CLOCK_SPEED_KHZ 153600
#define PIO_CYCLES_PER_INSTRUCTION 2
#define PIO_INSTRUCTIONS_PER_BIT 2

#define DATA_PIN 9
#define BCLK_PIN 10
#define LRCLK_PIN 11

#define BIT_DEPTH 32
#define SAMPLE_RATE 48000
#define NUM_CHANNELS 2
#define BLOCK_SIZE 48
#define STEREO_BLOCK_SIZE (BLOCK_SIZE * NUM_CHANNELS)
#define DOUBLE_BUFFER_SIZE (STEREO_BLOCK_SIZE * 2)

#define GAIN 0.5f
#define FREQUENCY 120.0f

int32_t output[DOUBLE_BUFFER_SIZE] = {0};
__attribute__((aligned(8))) int32_t* bufferPointers[2];
int32_t bufferPointerIdx = 0;
int dataChannel;
int controlChannel;
Oscillator<BLOCK_SIZE> osc;

float calculatePIOClockDivision() {
    float clockSpeed = (float) clock_get_hz(clk_sys);
    float bclkSpeed = (float) SAMPLE_RATE * (float) NUM_CHANNELS *
        (float) BIT_DEPTH;
    float pioSpeed = bclkSpeed * (float) PIO_CYCLES_PER_INSTRUCTION *
        (float) PIO_INSTRUCTIONS_PER_BIT;
    float clockDivision = clockSpeed / pioSpeed;

    return clockDivision;
}

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

    float clockDivision = calculatePIOClockDivision();
    sm_config_set_clkdiv(&sm_config, clockDivision);
    pio_sm_init(pio, sm, offset, &sm_config);

    uint32_t pinMask = (1u << DATA_PIN) | (3u << BCLK_PIN);
    pio_sm_set_pindirs_with_mask(pio, sm, pinMask, pinMask);
    pio_sm_set_pins(pio, sm, 0);
}

void control_dma_init(int dataChannel, int controlChannel) {
    dma_channel_config config = dma_channel_get_default_config(
        controlChannel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, false);
    channel_config_set_ring(&config, false, 3);

    dma_channel_configure(controlChannel, &config,
        &dma_hw->ch[dataChannel].al3_read_addr_trig,
        &bufferPointers, 1, false);
}

void data_dma_init(PIO pio, int sm, int dataChannel,
    int controlChannel) {
    dma_channel_config config = dma_channel_get_default_config(
        dataChannel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, false);
    channel_config_set_dreq(&config, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&config, controlChannel);

    dma_channel_configure(dataChannel, &config,
        &pio->txf[sm],
        NULL, // Read address is set by the control channel.
        STEREO_BLOCK_SIZE, false);
}

void dma_irq_init(int dataChannel, void(*dma_handler)(void)) {
    dma_channel_set_irq0_enabled(dataChannel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);
}

void dma_init(PIO pio, int sm, int dataChannel, int controlChannel,
    void(*dma_handler)(void)) {
    bufferPointers[0] = &output[0];
    bufferPointers[1] = &output[STEREO_BLOCK_SIZE];

    data_dma_init(pio, sm, dataChannel, controlChannel);
    control_dma_init(dataChannel, controlChannel);
    dma_irq_init(dataChannel, dma_handler);
    dma_channel_start(controlChannel);
}

inline int32_t floatToInt32(float sample) {
    int32_t converted = (int32_t)(sample * 2147483647.0f);
    return converted;
}

inline void writeSamples(int32_t* interleavedOutput,
    size_t i, int32_t left, int32_t right) {
    interleavedOutput[i * 2] = left;
    interleavedOutput[i * 2 + 1] = right;
}

inline void writeStereo(float* left, float* right,
    size_t blockSize, int32_t* interleavedOutput) {
    for (size_t i = 0; i < blockSize; i++) {
        float leftSample = left[i];
        int32_t leftConverted = floatToInt32(leftSample);
        float rightSample = right[i];
        int32_t rightConverted = floatToInt32(rightSample);
        writeSamples(interleavedOutput, i, leftConverted,
            rightConverted);
    }
}

inline void writeMono(float* mono, size_t blockSize,
    int32_t* interleavedOutput) {
    for (size_t i = 0; i < blockSize; i++) {
        float sample = mono[i];
        sample = sample;
        int32_t converted = floatToInt32(sample);
        writeSamples(interleavedOutput, i, converted, converted);
    }
}

void handler() {
    osc.generateBlock();

    int32_t* bufferToFill = bufferPointers[bufferPointerIdx];
    writeMono(osc.output, BLOCK_SIZE, bufferToFill);
    bufferPointerIdx = 1 - bufferPointerIdx;

    dma_hw->ints0 = 1u << dataChannel;
}

int main() {
    set_sys_clock_khz(CPU_CLOCK_SPEED_KHZ, true);

    osc.init({SAMPLE_RATE}, WaveShapes::sine);
    osc.frequency = FREQUENCY;
    osc.gain = GAIN;

    PIO pio = pio0;
    int sm = pio_claim_unused_sm(pio, true);
    pi_pico_i2s_out_program_init(pio, sm);

    dataChannel = dma_claim_unused_channel(true);
    controlChannel = dma_claim_unused_channel(true);
    dma_init(pio, sm, dataChannel, controlChannel, handler);

    pio_sm_set_enabled(pio, sm, true);

    while (true) {
        tight_loop_contents();
    }
}
