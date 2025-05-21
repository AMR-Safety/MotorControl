#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define RE_DE 4              // RE+DE tied to pin 4 (PD4)
#define LED_PIN 17           // TX LED on Pro Micro

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  pinMode(RE_DE, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RE_DE, LOW);    // Receive mode
  digitalWrite(LED_PIN, LOW);

  Serial1.begin(9600);         // RS485 communication

  // OLED init (uses SDA=pin 0, SCL=pin 1 on Pro Micro)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1);  // Display failed
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Ready...");
  display.display();
}

void loop() {
  if (Serial1.available()) {
    byte received = Serial1.read();

    // Blink LED
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);

    // Display on OLED
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("Received:");
    display.setCursor(0, 30);
    display.setTextSize(4);
    display.write(received);
    display.display();

    // Echo back over RS485
    digitalWrite(RE_DE, HIGH);  // Enable transmit
    delayMicroseconds(10);
    Serial1.write(received);
    Serial1.flush();
    delayMicroseconds(10);
    digitalWrite(RE_DE, LOW);   // Back to receive
  }
}
