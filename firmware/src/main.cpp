#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>
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

NeoPixelBus<NeoRgbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

long millisSinceLastWakeUp = 0;
RgbColor myColor(0, 128, 0);

void setup()
{
    Serial.begin(57600);
    delay(250); 
    Serial.println ("starting");
    millisSinceLastWakeUp = millis();
    
    strip.Begin();
    strip.Show();
}

void deepSleep() {
    strip.ClearTo(RgbColor(0));
    strip.Show();
    rtc_gpio_pullup_en(ButtonPin);
    esp_sleep_enable_ext0_wakeup(ButtonPin, 0);
    esp_deep_sleep_start();
}


void breathe() {
    long currentMillis = millis();

    float periodSeconds = (float)timerSeconds / numPeriods;
    long periodMillis = periodSeconds * 1000;
    int elapsedPeriods = currentMillis / periodMillis;
    
    long millisSincePeriod = currentMillis % periodMillis;
    int i = millisSincePeriod  % 65535;

    float intensity = myColor.G / 2.0 * (1.0 + sin(speedFactor * i));
    RgbColor newColor = myColor.Dim((uint8_t)intensity);

    for (int ledNumber = 0; ledNumber < PixelCount; ledNumber++) {
      if (ledNumber < elapsedPeriods) {
        strip.SetPixelColor(ledNumber, myColor);
      } else if (ledNumber == elapsedPeriods) {
        strip.SetPixelColor(ledNumber, newColor);
      } else {
        strip.SetPixelColor(ledNumber, 0);
      }
    }
    strip.Show();
  }
void loop()
{
    breathe();
    if (millis() > (timerSeconds + waitingSeconds) * 1000) {
      Serial.println ("entring deep sleep");
      deepSleep();
    }
    delay(stepDelay);
}