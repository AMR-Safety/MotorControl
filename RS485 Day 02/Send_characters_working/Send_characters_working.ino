#define RE_DE 2           // RE+DE tied together and connected to D2
#define LED_PIN 13        // Built-in LED on Uno

void setup() {
  pinMode(RE_DE, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(RE_DE, LOW);   // Start in receive mode
  digitalWrite(LED_PIN, LOW);

  Serial.begin(9600);         // RS485 serial
}

void loop() {
  if (Serial.available()) {
    byte received = Serial.read();

    // Blink to indicate reception
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);

    // Switch to transmit mode
    digitalWrite(RE_DE, HIGH);
    delayMicroseconds(10);   // Wait for driver to activate

    Serial.write(received);  // Echo the received byte

    Serial.flush();          // Wait until transmission is complete
    delayMicroseconds(10);   // Hold RE/DE high briefly

    // Switch back to receive mode
    digitalWrite(RE_DE, LOW);
  }
}
