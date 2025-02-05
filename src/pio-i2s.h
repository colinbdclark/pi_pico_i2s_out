/*
* Copyright 2025 Kinetic Light and Lichen Community Systems
* Licensed under the BSD-3 License.
*/

#ifndef PIOI2S_H
#define PIOI2S_H

#ifdef __cplusplus
extern "C" {
#endif

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
float PioI2S_calculateClockDivision(struct PioI2S* self);

/**
 * @brief Verifies that the calculated PIO clock division is valid.
 * This function will panic if an invalid clock ratio is detected.
 *
 * @param frequencyRatio the calculated clock division ratio
 */
void PicoI2S_verifyPIOClockDivision(float frequencyRatio);

/**
 * @brief Initializes the I2S PIO program and configures the PIO appropriately.
 *
 * @param self the PioI2S instance
 */
void PioI2s_initPIOProgram(struct PioI2S* self);

/**
 * @brief Initializes the control DMA channel.
 *
 * @param self the PioI2S instance
 */
void PioI2S_initControlChannel(struct PioI2S* self);

void PioI2S_initDataChannel(struct PioI2S* self);

/**
 * @brief Initializes the DMA interrupt handler.
 *
 * @param self the PioI2S instance
 */
void PioI2S_initDMAIRQ(struct PioI2S* self);

/**
 * @brief Initializes the DMA channels and interrupt handler.
 *
 * @param self the PioI2S instance
 */
void PioI2S_initDMA(struct PioI2S* self);

/**
 * @brief Initializes a PioI2S instance.
 *
 * @param self the PioI2S instance
 * @param config the configuration for the PioI2S instance
 * @param outputDoubleBuffer the output buffer to use
 * @param dmaHandler the DMA interrupt handler function
 */
void PioI2S_init(struct PioI2S* self, struct PioI2S_Config* config,
    int32_t* outputDoubleBuffer, void(*dmaHandler)(void));

/**
 * @brief Starts outputting I2S audio.
 *
 * @param self the PioI2S instance
 */
void PioI2S_start(struct PioI2S* self);

/**
 * @brief Gets the output buffer that needs to be refilled during the
 * current DMA interrupt.
 *
 * This function should only be called within your I2S DMA interrupt handler.
 *
 * @param self the PioI2S instance
 * @return int32_t* the buffer to fill
 */
int32_t* PioI2S_nextOutputBuffer(struct PioI2S* self);

/**
 * @brief Clears the DMA interrupt flag for the data channel.
 * Call this function at the end of your I2S DMA interrupt handler.
 *
 * @param self the PioI2S instance
 */
void PioI2S_endDMAInterruptHandler(struct PioI2S* self);

/**
 * @brief Converts a float sample in the range of 1.0 to -1.0 to a 32-bit
 * signed integer.
 *
 * @param sample the sample to convert
 * @return int32_t the converted sample
 */
int32_t PioI2s_floatToInt32(float sample);

/**
 * @brief Writes a stereo pair of samples to an interleaved output buffer.
 *
 *  @param left the left channel sample
 * @param right the right channel sample
 * @param interleavedOutput the output buffer to write into
 * @param i the audio frame index
 */
void PioI2s_writeSamples(int32_t left, int32_t right,
    int32_t* interleavedOutput, size_t i);

/**
 * @brief Writes a pair of stereo audio blocks to the interleaved output buffer.
 *
 * @param left the left channel samples
 * @param right the right channel samples
 * @param blockSize the block size
 * @param interleavedOutput the output buffer to write into
 */
void PioI2s_writeStereo(float* left, float* right,
    size_t blockSize, int32_t* interleavedOutput);

/**
 * @brief Writes a mono audio block to the interleaved output buffer.
 *
 * @param mono the sample block to write
 * @param blockSize the block size
 * @param interleavedOutput the output buffer to write into
 */
void PioI2s_writeMono(float* mono, size_t blockSize,
    int32_t* interleavedOutput);

#ifdef __cplusplus
}
#endif

#endif /* PIOI2S_H */
