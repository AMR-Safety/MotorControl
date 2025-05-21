#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1); // OLED not found
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  // 🌟 Yellow Zone (Top)
  display.setCursor(0, 0);     // Y = 0 to 15 → Yellow
  display.println("YELLOW");

  // 🔵 Blue Zone (Bottom)
  display.setCursor(0, 32);    // Y = 16+ → Blue
  display.println("BLUE");

  display.display();
}

void loop() {
  // Nothing here
}
