#include <sirdeniel-project-1_inferencing.h>
#include "LSM6DS3.h"
#include "Wire.h"

// Crear una instancia de la clase LSM6DS3
LSM6DS3 xIMU(I2C_MODE, 0x6A); // Dirección del dispositivo I2C 0x6A

// Pines de entrada/salida
#define BUTTON_START D2
const int pinSignal1 = D10; // Pin para la señal 1
const int pinSignal2 = D9;  // Pin para la señal 2
const int pinButtonTest = D3; // Pin para el pulsador

// Variables para señales
const int tiempoOnSignal1 = 300; // 300 microsegundos
const int tiempoOffSignal1 = 200; // 200 microsegundos
const int frecuenciaSignal2 = 20; // Frecuencia de 20 Hz
const int periodoSignal2 = 1000 / frecuenciaSignal2; // Periodo en milisegundos

// Variables para el control de tiempos
unsigned long lastToggleSignal1 = 0;
unsigned long lastToggleSignal2 = 0;
volatile bool signal1On = false;

// Variables de estado
bool program_enabled = false; // Estado de habilitación
bool last_button_state = HIGH; // Estado previo del botón
bool buttonState = false;
bool buttonPressedLastState = false;

// Funciones adicionales
#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  2.0f
static bool debug_nn = false; // Agrega esta línea al inicio del archivo

void setup() {
    Serial.begin(115200);
    Serial.println("Edge Impulse Inferencing Demo");

    // Configuración de pines
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BUTTON_START, INPUT_PULLUP);
    pinMode(pinSignal1, OUTPUT);
    pinMode(pinSignal2, OUTPUT);
    pinMode(pinButtonTest, INPUT_PULLUP);

    // Inicializar los pines
    digitalWrite(pinSignal1, LOW);
    digitalWrite(pinSignal2, LOW);

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
    // Botón habilitador
    bool current_button_state = digitalRead(BUTTON_START);
    if (last_button_state == HIGH && current_button_state == LOW) {
        program_enabled = !program_enabled;
        ei_printf("Program state: %s\n", program_enabled ? "ENABLED" : "DISABLED");
        delay(200); // Anti-rebote
    }
    last_button_state = current_button_state;

    // Lógica del botón Test (siempre activa)
    buttonState = digitalRead(pinButtonTest);
    if (buttonState == LOW && !buttonPressedLastState) {
        Serial.println("Pulsador presionado");
        buttonPressedLastState = true;
    } else if (buttonState == HIGH && buttonPressedLastState) {
        Serial.println("Pulsador liberado");
        buttonPressedLastState = false;
    }

    unsigned long now = millis();

    // Generar señal 1 si el botón test está presionado
    if (buttonState == LOW) {
        static bool stateSignal1 = false;
        if (now - lastToggleSignal1 >= (stateSignal1 ? tiempoOnSignal1 / 1000.0 : tiempoOffSignal1 / 1000.0)) {
            stateSignal1 = !stateSignal1;
            digitalWrite(pinSignal1, stateSignal1);
            signal1On = stateSignal1;
            lastToggleSignal1 = now;
        }

        // Generar señal 2 (solo si señal 1 está activa)
        static bool stateSignal2 = false;
        if (signal1On && (now - lastToggleSignal2 >= periodoSignal2 / 2)) {
            stateSignal2 = !stateSignal2;
            digitalWrite(pinSignal2, stateSignal2);
            lastToggleSignal2 = now;
        } else if (!signal1On) {
            digitalWrite(pinSignal2, LOW);
        }
    } else {
        // Apagar ambas señales si no está presionado
        digitalWrite(pinSignal1, LOW);
        digitalWrite(pinSignal2, LOW);
        signal1On = false;
    }

    // Continuar con la lógica principal solo si el programa está habilitado
    if (!program_enabled) {
        return; // No ejecutar inferencias si no está habilitado
    }

    // Inferencia
    ei_printf("\nStarting inferencing...\n");
    delay(500);

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

    // Análisis de resultados
    bool detected_walk = false;
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (strcmp(result.classification[ix].label, "walk") == 0 && result.classification[ix].value > 0.8) {
            detected_walk = true;
        }
    }

    // Si se detecta caminata
    if (detected_walk) {
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("Walk detected: Activating signals!");
        digitalWrite(pinSignal1, HIGH);
        digitalWrite(pinSignal2, HIGH);
        delay(500); // Activar ambas señales por 500ms
        digitalWrite(pinSignal1, LOW);
        digitalWrite(pinSignal2, LOW);
    }
}