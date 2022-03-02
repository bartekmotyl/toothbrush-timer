#include <NeoPixelAnimator.h>
#include <NeoPixelBus.h>
#include <driver/rtc_io.h>

const uint16_t PixelCount = 16;
const uint8_t PixelPin = 2;
const gpio_num_t ButtonPin = GPIO_NUM_33;

float maximumBrightness = 255;
float speedFactor = 0.004;
float stepDelay = 5;

int numPeriods = 16;
int timerSeconds = 120;
int waitingSeconds = 10;

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

long millisSinceLastWakeUp = 0;

class ColorScheme {
 public:
  virtual RgbColor getColor(uint ledIndex, uint elapsedPeriods,
                            ulong millisSincePeriod) = 0;
};

class ColorScheme1 : public ColorScheme {
  RgbColor getColor(uint ledIndex, uint elapsedPeriods, ulong millisSincePeriod) {
    static uint ledMapping[16] = {0, 4, 8,  12, 13, 9,  5, 1,
                                 2, 6, 10, 14, 15, 11, 7, 3};
    static RgbColor colorFinish(0, 128, 0);
    //static RgbColor colorInProgress(128, 0, 0);

    RgbColor colorInProgress = Wheel(ledIndex + elapsedPeriods*4);

    uint i = millisSincePeriod % 65535;
    float intensity = 128 * (1.0 + sin(speedFactor * i));

    ledIndex = ledMapping[ledIndex];

    if (elapsedPeriods == numPeriods) {
      return colorFinish.Dim((uint8_t)intensity);
    } else if (ledIndex < elapsedPeriods) {
      return colorFinish;
    } else if (ledIndex == elapsedPeriods) {
      return colorInProgress.Dim((uint8_t)intensity);
    } else {
      return RgbColor(0);
    }
  }

  RgbColor Wheel(uint8_t WheelPos) {
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) {
      return RgbColor(255 - WheelPos * 3, 0, WheelPos * 3);
    } else if (WheelPos < 170) {
      WheelPos -= 85;
      return RgbColor(0, WheelPos * 3, 255 - WheelPos * 3);
    } else {
      WheelPos -= 170;
      return RgbColor(WheelPos * 3, 255 - WheelPos * 3, 0);
    }
  }
};

void setup() {
  Serial.begin(57600);
  delay(250);
  Serial.println("starting");
  millisSinceLastWakeUp = millis();
  strip.Begin();
  strip.Show();
}

void enterDeepSleep() {
  Serial.println("entring deep sleep");
  strip.ClearTo(RgbColor(0));
  strip.Show();
  rtc_gpio_pullup_en(ButtonPin);
  esp_sleep_enable_ext0_wakeup(ButtonPin, 0);
  esp_deep_sleep_start();
}

void calculateFrame(ColorScheme& colorScheme) {
  unsigned long currentMillis = millis();

  float periodSeconds = (float)timerSeconds / numPeriods;
  unsigned long periodMillis = periodSeconds * 1000;
  int elapsedPeriods = currentMillis / periodMillis;

  unsigned long millisSincePeriod = currentMillis % periodMillis;

  for (int ledIndex = 0; ledIndex < PixelCount; ledIndex++) {
    RgbColor color =
        colorScheme.getColor(ledIndex, elapsedPeriods, millisSincePeriod);
    strip.SetPixelColor(ledIndex, color);
  }

  strip.Show();
}

void loop() {
  ColorScheme1 colorScheme = ColorScheme1();
  calculateFrame(colorScheme);

  if (millis() > (timerSeconds + waitingSeconds) * 1000) {
    enterDeepSleep();
  }
  delay(stepDelay);
}