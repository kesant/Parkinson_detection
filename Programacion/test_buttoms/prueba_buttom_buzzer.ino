#define BUTTON_PIN D2  // Pin donde está conectado el botón

volatile int buttonPresses = 0; // Contador de pulsaciones
volatile bool buttonState = HIGH; // Estado actual del botón
volatile bool lastButtonState = HIGH; // Último estado del botón

void setup() {
  // Configurar el pin del botón como entrada con resistencia pull-up
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Iniciar comunicación serial para mostrar los resultados
  Serial.begin(9600);
}

void loop() {
  // Leer el estado actual del botón
  buttonState = digitalRead(BUTTON_PIN);
  
  // Detectar si el botón pasó de HIGH a LOW (flanco descendente)
  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPresses++; // Incrementar el contador
    Serial.print("Pulsaciones: ");
    Serial.println(buttonPresses); // Imprimir en el monitor serial
  }
  
  // Actualizar el último estado del botón
  lastButtonState = buttonState;

  delay(10); // Pequeño retraso para evitar rebotes (debouncing)
}
