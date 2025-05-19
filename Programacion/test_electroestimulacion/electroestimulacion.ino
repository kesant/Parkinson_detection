#include "Wire.h"

// Definición de pines (adaptados a la Seeed XIAO nRF52840 Sense)
const int pinSignal1 = D10;  // Salida señal 1
const int pinSignal2 = D9;   // Salida señal 2
const int pinButtonTest = D3; // Pulsador de prueba

// Variables para señales
const int tiempoOnSignal1 = 300;   // Tiempo encendido en microsegundos
const int tiempoOffSignal1 = 200;  // Tiempo apagado en microsegundos
const int frecuenciaSignal2 = 20;  // Frecuencia de 20 Hz
const int periodoSignal2 = 1000 / frecuenciaSignal2; // Periodo en milisegundos

// Variables de estado y control de tiempos
unsigned long lastToggleSignal1 = 0;
unsigned long lastToggleSignal2 = 0;
bool signal1On = false;
bool buttonState = false;
bool buttonPressedLastState = false;

void setup() {
    Serial.begin(115200);
    Serial.println("Electroestimulación activada por pulsador");

    // Configuración de pines
    pinMode(pinSignal1, OUTPUT);
    pinMode(pinSignal2, OUTPUT);
    pinMode(pinButtonTest, INPUT_PULLUP);

    // Inicializar pines en LOW
    digitalWrite(pinSignal1, LOW);
    digitalWrite(pinSignal2, LOW);
}

void loop() {
    // Lectura del botón de prueba
    buttonState = digitalRead(pinButtonTest);
    if (buttonState == LOW && !buttonPressedLastState) {
        Serial.println("Pulsador presionado - Activando electroestimulación");
        buttonPressedLastState = true;
    } else if (buttonState == HIGH && buttonPressedLastState) {
        Serial.println("Pulsador liberado - Desactivando electroestimulación");
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

        // Generar señal 2 solo si señal 1 está activa
        static bool stateSignal2 = false;
        if (signal1On && (now - lastToggleSignal2 >= periodoSignal2 / 2)) {
            stateSignal2 = !stateSignal2;
            digitalWrite(pinSignal2, stateSignal2);
            lastToggleSignal2 = now;
        } else if (!signal1On) {
            digitalWrite(pinSignal2, LOW);
        }
    } else {
        // Apagar ambas señales si el botón no está presionado
        digitalWrite(pinSignal1, LOW);
        digitalWrite(pinSignal2, LOW);
        signal1On = false;
    }
}
