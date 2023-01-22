#include <NeoPixelAnimator.h>
#include <NeoPixelBus.h>
#include <driver/adc.h>
#include <driver/rtc_io.h>

const uint16_t PixelCount = 16;
const uint8_t PixelPin = 2;
const gpio_num_t ButtonPin = GPIO_NUM_33;
const adc1_channel_t BatteryVoltageADC = ADC1_CHANNEL_0;

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
  RgbColor getColor(uint ledIndex, uint elapsedPeriods,
                    ulong millisSincePeriod) {
    static uint ledMapping[16] = {0, 4, 8,  12, 13, 9,  5, 1,
                                  2, 6, 10, 14, 15, 11, 7, 3};
    static RgbColor colorFinish(0, 128, 0);
    // static RgbColor colorInProgress(128, 0, 0);

    RgbColor colorInProgress = Wheel(ledIndex + elapsedPeriods * 4);

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

float getBatteryVoltage() {
  // 12 bit range ADC (0..4095), 0 to 1.1 V
  //
  // R1 = 100k (99,60k)
  // R2 = 33k (32,48k)
  static float r1 = 99600.0;
  static float r2 = 32480.0;
  static float multiplier = r2 / (r1 + r2);

  int rawValue = adc1_get_raw(BatteryVoltageADC);
  float readVoltage = (rawValue / 4095.0) * 1.1;
  float batteryVoltage = readVoltage / multiplier;
  return batteryVoltage;
}

void setup() {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(BatteryVoltageADC, ADC_ATTEN_DB_0);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(57600);
  delay(250);
  Serial.println("starting");
  Serial.println(getBatteryVoltage());
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
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  /*
  ColorScheme1 colorScheme = ColorScheme1();
  calculateFrame(colorScheme);

  if (millis() > (timerSeconds + waitingSeconds) * 1000) {
    enterDeepSleep();
  }
  delay(stepDelay);
  */
}