/*
* Copyright 2025 Kinetic Light and Lichen Community Systems
* Licensed under the BSD-3 License.
*/

#include <math.h>
#include <functional>

#define TABLE_SIZE 2048
#define TWO_PI 6.28318530718f

inline float interpoloateLinear(float idx, float* table, size_t length) {
    int32_t idxIntegral = (int32_t) idx;
    float idxFractional = idx - (float) idxIntegral;
    float a = table[idxIntegral];
    float b = table[(idxIntegral + 1) % length];

    return a + (b - a) * idxFractional;
}

struct AudioSettings {
    float sampleRate;
};

struct WaveShapes {
    static void sine(float* table, size_t size) {
        for (size_t i = 0; i < size; i++) {
            table[i] = sinf(TWO_PI * (float) i / (float) size);
        }
    }

    static void silence(float* table, size_t size) {
        for (size_t i = 0; i < size; i++) {
            table[i] = 0.0f;
        }
    }
};

template <size_t size> class Wavetable {
    public:

    float table[size];
    size_t tableSize = size;

    inline float readLinear(float normalizedIndex) {
        float index = normalizedIndex * (float) size;
        return interpoloateLinear(index, this->table, size);
    }
};

template <size_t blockSize> class Oscillator {
    public:
    float sampleRate;
    Wavetable<2048> wavetable = {0};
    float output[blockSize] = {0};
    float frequency = 440;
    float gain = 1.0f;
    float phaseAccumulator = 0.0f;

    void init(AudioSettings settings,
        std::function<void(float*, size_t)> shapeFiller) {
        this->sampleRate = settings.sampleRate;
        shapeFiller(wavetable.table, wavetable.tableSize);
    }

    inline void accumulatePhase() {
        float phaseAccumulator = this->phaseAccumulator;
        float phaseStep = this->frequency / this->sampleRate;

        phaseAccumulator += phaseStep;
        while (phaseAccumulator >= 1.0f) {
            phaseAccumulator -= 1.0f;
        }
        this->phaseAccumulator = phaseAccumulator;
    }

    inline float generate() {
        float sample = this->wavetable.readLinear(this->phaseAccumulator);
        this->accumulatePhase();

        return sample * gain;
    }

    inline void generateBlock() {
        for (size_t i = 0; i < blockSize; i++) {
            float sample = generate();
            output[i] = sample * gain;
        }
    }
};
