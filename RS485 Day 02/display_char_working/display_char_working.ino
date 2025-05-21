#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define RE_DE 2           // RE+DE tied together and connected to D2
#define LED_PIN 13        // Built-in LED on Uno

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  pinMode(RE_DE, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RE_DE, LOW);   // Start in receive mode
  digitalWrite(LED_PIN, LOW);

  Serial.begin(9600);         // RS485

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // OLED failed
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Ready...");
  display.display();
}

void loop() {
  if (Serial.available()) {
    byte received = Serial.read();

    // Blink to indicate reception
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);

    // Show character on OLED
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("Received:");
    display.setCursor(0, 30);
    display.setTextSize(4);
    display.write(received);
    display.display();

    // Echo the received byte
    digitalWrite(RE_DE, HIGH);
    delayMicroseconds(10);
    Serial.write(received);
    Serial.flush();
    delayMicroseconds(10);
    digitalWrite(RE_DE, LOW);
  }
}
