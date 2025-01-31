/*
* Copyright 2025 Kinetic Light and Lichen Community Systems
* Licensed under the BSD-3 License.
*/

#ifndef PIOI2S_H
#define PIOI2S_H

#include <math.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

/** Compiled version of pio_i2s_out.pio */
#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ---------- //
// PioI2S_out //
// ---------- //

#define PioI2S_out_wrap_target 0
#define PioI2S_out_wrap 7
#define PioI2S_out_pio_version 1

#define PioI2S_out_offset_entry_point 0u

static const uint16_t PioI2S_out_program_instructions[] = {
            //     .wrap_target
    0x8080, //  0: pull   noblock         side 0
    0xe83e, //  1: set    x, 30           side 1
    0x6001, //  2: out    pins, 1         side 0
    0x0842, //  3: jmp    x--, 2          side 1
    0x9080, //  4: pull   noblock         side 2
    0xf83e, //  5: set    x, 30           side 3
    0x7001, //  6: out    pins, 1         side 2
    0x1846, //  7: jmp    x--, 6          side 3
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program PioI2S_out_program = {
    .instructions = PioI2S_out_program_instructions,
    .length = 8,
    .origin = -1,
    .pio_version = PioI2S_out_pio_version,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = 0x0
#endif
};

static inline pio_sm_config PioI2S_out_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + PioI2S_out_wrap_target, offset + PioI2S_out_wrap);
    sm_config_set_sideset(&c, 2, false, false);
    return c;
}
#endif
/** End compiled version of pio_i2s_out.pio */


#define PioI2S_PIO_CYCLES_PER_INSTRUCTION 2
#define PioI2S_PIO_INSTRUCTIONS_PER_BIT 2
#define PioI2S_NUM_CHANNELS 2

/**
 * @brief Configuration for a PioI2S instance.
 *
 */
struct PioI2S_Config {
    /**
     * @brief The data pin.
     */
    int dataPin;

    /**
     * @brief The bit clock pin.
     */
    int bclkPin;

    /**
     * @brief The bit depth to use.
     * This must match the bit depth supported by your I2S device.
     * Note currently only 32 bit samples are supported.
     */
    int bitDepth;

    /**
     * @brief The sample rate in Hz.
     * This must match a sample rate supported by your I2S device.
     */
    int32_t sampleRate;

    /**
     * @brief The audio sample block size to use.
     */
    int blockSize;

    /**
     * @brief The PIO instance to use.
     */
    PIO pio;
};

/**
 * @brief The PioI2S object.
 *
 */
struct PioI2S {
    /**
     * @brief A pointer to the configuration for this PioI2S instance.
     */
    struct PioI2S_Config* config;

    /**
     * @brief The PIO state machine index.
     */
    int sm;

    /**
     * @brief The index of the data DMA channel.
     */
    int dataChannel;

    /**
     * @brief The index of the control DMA channel.
     */
    int controlChannel;

    /**
     * @brief The size of a stereo audio block.
     */
    size_t stereoBlockSize;

    /**
     * @brief The size of internal double output buffer.
     */
    size_t doubleBufferSize;

    /**
     * @brief The output buffer, which must be twice the size of a stereo block.The client is responsible for allocating this buffer.
     */
    int32_t* outputDoubleBuffer;

    /**
     * @brief An array storing addresses to the two output buffers.
     */
    __attribute__((aligned(8))) int32_t* bufferPointers[2];

    /**
     * @brief The index of the current output buffer.
     */
    int32_t bufferPointerIdx;

    /**
     * @brief A pointer to the DMA interrupt handler function.
     */
    void(*dmaHandler)(void);
};

/**
 * @brief Calculates the appropriate PIO clock division based on the
 * system clock speed and the required I2S bit clock rate.
 *
 * @param self the PioI2S instance
 * @return float the calculated clock division
 */
float PioI2S_calculateClockDivision(struct PioI2S* self) {
    float clockSpeed = (float) clock_get_hz(clk_sys);
    float bclkSpeed = (float) self->config->sampleRate *
        (float) PioI2S_NUM_CHANNELS * (float) self->config->bitDepth;
    float pioSpeed = bclkSpeed * (float) PioI2S_PIO_CYCLES_PER_INSTRUCTION *
        (float) PioI2S_PIO_INSTRUCTIONS_PER_BIT;
    float clockDivision = clockSpeed / pioSpeed;

    return clockDivision;
}

/**
 * @brief Verifies that the calculated PIO clock division is valid.
 * This function will panic if an invalid clock ratio is detected.
 *
 * @param frequencyRatio the calculated clock division ratio
 */
void PicoI2S_verifyPIOClockDivision(float frequencyRatio) {
    if (frequencyRatio < 1.0f) {
        panic("PIO clock ratio %f is faster than the system clock.",
            frequencyRatio);
    }

    float integral;
    float fractional = modff(frequencyRatio, &integral);

    // Check if the fractional part is a power of 2
    // by multiplying by 16 (the maximum number of fractional bits
    // supported by a PIO's clock divider)
    float scaled = fractional * 16.0f;
    float scaledIntegral;
    float remainder = modff(scaled, &scaledIntegral);

    if (remainder > 0.0f) {
        panic("PIO clock ratio %f cannot be represented accurately.",
            frequencyRatio);
    }
}

/**
 * @brief Initializes the I2S PIO program and configures the PIO appropriately.
 *
 * @param self the PioI2S instance
 */
void PioI2s_initPIOProgram(struct PioI2S* self) {
    int offset = pio_add_program(self->config->pio, &PioI2S_out_program);
    pio_sm_config sm_config = PioI2S_out_program_get_default_config(
        offset);

    pio_gpio_init(self->config->pio, self->config->dataPin);
    pio_gpio_init(self->config->pio, self->config->bclkPin);
    pio_gpio_init(self->config->pio, self->config->bclkPin + 1);
    sm_config_set_out_pins(&sm_config, self->config->dataPin, 1);
    sm_config_set_sideset_pin_base(&sm_config, self->config->bclkPin);
    sm_config_set_out_shift(&sm_config, false, false, self->config->bitDepth);
    sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);

    float clockDivision = PioI2S_calculateClockDivision(self);
    PicoI2S_verifyPIOClockDivision(clockDivision);
    sm_config_set_clkdiv(&sm_config, clockDivision);
    pio_sm_init(self->config->pio, self->sm, offset, &sm_config);

    uint32_t pinMask = (1u << self->config->dataPin) |
        (3u << self->config->bclkPin);
    pio_sm_set_pindirs_with_mask(self->config->pio, self->sm, pinMask, pinMask);
    pio_sm_set_pins(self->config->pio, self->sm, 0);
}

/**
 * @brief Initializes the control DMA channel.
 *
 * @param self the PioI2S instance
 */
void PioI2S_initControlChannel(struct PioI2S* self) {
    dma_channel_config config = dma_channel_get_default_config(
        self->controlChannel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, false);
    channel_config_set_ring(&config, false, 3);

    dma_channel_configure(self->controlChannel, &config,
        &dma_hw->ch[self->dataChannel].al3_read_addr_trig,
        &self->bufferPointers, 1, false);
}

void PioI2S_initDataChannel(struct PioI2S* self) {
    dma_channel_config config = dma_channel_get_default_config(
        self->dataChannel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, true);
    channel_config_set_write_increment(&config, false);
    channel_config_set_dreq(&config, pio_get_dreq(
        self->config->pio, self->sm, true));
    channel_config_set_chain_to(&config, self->controlChannel);

    dma_channel_configure(self->dataChannel, &config,
        &self->config->pio->txf[self->sm],
        NULL, // Read address is set by the control channel.
        self->stereoBlockSize, false);
}

/**
 * @brief Initializes the DMA interrupt handler.
 *
 * @param self the PioI2S instance
 */
void PioI2S_initDMAIRQ(struct PioI2S* self) {
    dma_channel_set_irq0_enabled(self->dataChannel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, self->dmaHandler);
    irq_set_enabled(DMA_IRQ_0, true);
}

/**
 * @brief Initializes the DMA channels and interrupt handler.
 *
 * @param self the PioI2S instance
 */
void PioI2S_initDMA(struct PioI2S* self) {
    PioI2S_initDataChannel(self);
    PioI2S_initControlChannel(self);
    PioI2S_initDMAIRQ(self);
}

/**
 * @brief Initializes a PioI2S instance.
 *
 * @param self the PioI2S instance
 * @param config the configuration for the PioI2S instance
 * @param outputDoubleBuffer the output buffer to use
 * @param dmaHandler the DMA interrupt handler function
 */
void PioI2S_init(struct PioI2S* self, struct PioI2S_Config* config,
    int32_t* outputDoubleBuffer, void(*dmaHandler)(void)) {
    self->config = config;
    self->sm = pio_claim_unused_sm(self->config->pio, true);
    self->dataChannel = dma_claim_unused_channel(true);
    self->controlChannel = dma_claim_unused_channel(true);
    self->stereoBlockSize = self->config->blockSize * PioI2S_NUM_CHANNELS;
    self->doubleBufferSize = self->stereoBlockSize * 2;
    self->outputDoubleBuffer = outputDoubleBuffer;
    self->bufferPointers[0] = &self->outputDoubleBuffer[0];
    self->bufferPointers[1] = &self->outputDoubleBuffer[self->stereoBlockSize];
    self->bufferPointerIdx = 0;
    self->dmaHandler = dmaHandler;

    PioI2s_initPIOProgram(self);
    PioI2S_initDMA(self);
}

/**
 * @brief Starts outputting I2S audio.
 *
 * @param self the PioI2S instance
 */
void PioI2S_start(struct PioI2S* self) {
    dma_channel_start(self->controlChannel);
    pio_sm_set_enabled(self->config->pio, self->sm, true);
}

/**
 * @brief Gets the output buffer that needs to be refilled during the
 * current DMA interrupt.
 *
 * This function should only be called within your I2S DMA interrupt handler.
 *
 * @param self the PioI2S instance
 * @return int32_t* the buffer to fill
 */
inline int32_t* PioI2S_nextOutputBuffer(struct PioI2S* self) {
    int32_t* bufferToFill = self->bufferPointers[self->bufferPointerIdx];
    self->bufferPointerIdx = 1 - self->bufferPointerIdx;
    return bufferToFill;
}

/**
 * @brief Clears the DMA interrupt flag for the data channel.
 * Call this function at the end of your I2S DMA interrupt handler.
 *
 * @param self the PioI2S instance
 */
inline void PioI2S_endDMAInterruptHandler(struct PioI2S* self) {
    dma_hw->ints0 = 1u << self->dataChannel;
}

/**
 * @brief Converts a float sample in the range of 1.0 to -1.0 to a 32-bit
 * signed integer.
 *
 * @param sample the sample to convert
 * @return int32_t the converted sample
 */
inline int32_t PioI2s_floatToInt32(float sample) {
    int32_t converted = (int32_t)(sample * 2147483647.0f);
    return converted;
}

/**
 * @brief Writes a stereo pair of samples to an interleaved output buffer.
 *
 *  @param left the left channel sample
 * @param right the right channel sample
 * @param interleavedOutput the output buffer to write into
 * @param i the audio frame index
 */
inline void PioI2s_writeSamples(int32_t left, int32_t right,
    int32_t* interleavedOutput, size_t i) {
    interleavedOutput[i * 2] = left;
    interleavedOutput[i * 2 + 1] = right;
}

/**
 * @brief Writes a pair of stereo audio blocks to the interleaved output buffer.
 *
 * @param left the left channel samples
 * @param right the right channel samples
 * @param blockSize the block size
 * @param interleavedOutput the output buffer to write into
 */
inline void PioI2s_writeStereo(float* left, float* right,
    size_t blockSize, int32_t* interleavedOutput) {
    for (size_t i = 0; i < blockSize; i++) {
        float leftSample = left[i];
        int32_t leftConverted = PioI2s_floatToInt32(leftSample);
        float rightSample = right[i];
        int32_t rightConverted = PioI2s_floatToInt32(rightSample);
        PioI2s_writeSamples(leftConverted, rightConverted,
            interleavedOutput, i);
    }
}

/**
 * @brief Writes a mono audio block to the interleaved output buffer.
 *
 * @param mono the sample block to write
 * @param blockSize the block size
 * @param interleavedOutput the output buffer to write into
 */
inline void PioI2s_writeMono(float* mono, size_t blockSize,
    int32_t* interleavedOutput) {
    for (size_t i = 0; i < blockSize; i++) {
        float sample = mono[i];
        sample = sample;
        int32_t converted = PioI2s_floatToInt32(sample);
        PioI2s_writeSamples(converted, converted, interleavedOutput, i);
    }
}

#endif /* PIOI2S_H */
