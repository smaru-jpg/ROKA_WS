#include <Adafruit_NeoPixel.h>

// Validated ROKA section-1 hardware contract: Arduino Nano, D6, 16 pixels.
#define PIN 6
#define NUMPIXELS 16

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setColor(uint32_t color);
void setHalfColor(uint32_t color1, uint32_t color2);

void setup() {
  Serial.begin(115200);
  pixels.begin();
  pixels.clear();
  pixels.show();
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  const char state = Serial.read();
  if (state == 'E') {  // Enemy: all red.
    setColor(pixels.Color(255, 0, 0));
  } else if (state == 'F') {  // Friend: all green.
    setColor(pixels.Color(0, 255, 0));
  } else if (state == 'B') {  // Both: green half, red half.
    setHalfColor(
      pixels.Color(0, 255, 0),
      pixels.Color(255, 0, 0)
    );
  } else if (state == 'N') {  // None: off.
    setColor(pixels.Color(0, 0, 0));
  }
}

void setColor(uint32_t color) {
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

void setHalfColor(uint32_t color1, uint32_t color2) {
  const int half = NUMPIXELS / 2;
  for (int i = 0; i < half; i++) {
    pixels.setPixelColor(i, color1);
  }
  for (int i = half; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, color2);
  }
  pixels.show();
}
