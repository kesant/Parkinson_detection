#include <sirdeniel-project-1_inferencing.h>
#include "LSM6DS3.h"
#include "Wire.h"

// Crear una instancia de la clase LSM6DS3
LSM6DS3 xIMU(I2C_MODE, 0x6A); // Dirección del dispositivo I2C 0x6A

#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  2.0f
static bool debug_nn = false; // Agrega esta línea al inicio del archivo

void setup() {
    Serial.begin(115200);
    //while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    pinMode(LED_BUILTIN, OUTPUT); // Configurar el LED como salida

    if (xIMU.begin() != 0) {
        ei_printf("Failed to initialize IMU!\r\n");
    } else {
        ei_printf("IMU initialized\r\n");
    }

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
        return;
    }
}

float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

void loop() {
    ei_printf("\nStarting inferencing in 1 second...\n");
    delay(1000);

    ei_printf("Sampling...\n");

    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        buffer[ix]     = xIMU.readFloatAccelX();
        buffer[ix + 1] = xIMU.readFloatAccelY();
        buffer[ix + 2] = xIMU.readFloatAccelZ();

        for (int i = 0; i < 3; i++) {
            if (fabs(buffer[ix + i]) > MAX_ACCEPTED_RANGE) {
                buffer[ix + i] = ei_get_sign(buffer[ix + i]) * MAX_ACCEPTED_RANGE;
            }
        }

        buffer[ix + 0] *= CONVERT_G_TO_MS2;
        buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix + 2] *= CONVERT_G_TO_MS2;

        delayMicroseconds(next_tick - micros());
    }

    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");

    bool detected_walk = false;

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);

        // Encender LED si la predicción es "walk" con alta confianza
        if (strcmp(result.classification[ix].label, "walk") == 0 && result.classification[ix].value > 0.8) {
            detected_walk = true;
        }
    }

    if (detected_walk) {
        digitalWrite(LED_BUILTIN, LOW); // Enciende el LED
    } else {
        digitalWrite(LED_BUILTIN, HIGH); // Apaga el LED
    }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
}

