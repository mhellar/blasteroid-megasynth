// Keypad Synth - short finder. Every header GPIO gets INPUT_PULLUP; a pin that still reads LOW is tied to GND
// (or held low by something). Then each pin in turn is driven LOW and any other pin that follows it is shorted to it.
// Unplug nothing - the keypad is passive and harmless, the OLED/amp inputs are high impedance.
// Serial Monitor 115200, USB CDC On Boot: Enabled. Prints a report every 3 s.
#include <Arduino.h>

const int PINS[] = {43, 44, 1, 2, 3, 4, 5, 6, 7, 13, 12, 11, 10, 9, 8};
const char *NAMES[] = {"TX43 amp DIN", "RX44 amp BCLK", "IO1 amp LRC", "IO2 key R1", "IO3 key R2", "IO4 key R3",
                       "IO5 key R4", "IO6 key C1", "IO7 key C2", "IO13 key C3", "IO12 key C4", "IO11 pot HI",
                       "IO10 pot W", "IO9 OLED SDA", "IO8 OLED SCL"};
const int N = sizeof(PINS) / sizeof(PINS[0]);

void allPullup() { for (int i = 0; i < N; i++) pinMode(PINS[i], INPUT_PULLUP); }

void setup() {
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);
  delay(1500);
}

void loop() {
  allPullup();
  delay(5);
  Serial.println("\n=== pins reading LOW with pull-up (tied to GND?) ===");
  int lows = 0;
  for (int i = 0; i < N; i++) if (!digitalRead(PINS[i])) { Serial.printf("  %s\n", NAMES[i]); lows++; }
  if (!lows) Serial.println("  none");
  Serial.println("=== pin-to-pin shorts (drive one LOW, who follows) ===");
  int shorts = 0;
  for (int i = 0; i < N; i++) {
    allPullup();
    pinMode(PINS[i], OUTPUT); digitalWrite(PINS[i], LOW);
    delayMicroseconds(200);
    for (int j = i + 1; j < N; j++) {
      if (!digitalRead(PINS[j])) {
        // ignore pins that are low anyway (already reported as GND)
        pinMode(PINS[i], INPUT_PULLUP); delayMicroseconds(200);
        bool lowAnyway = !digitalRead(PINS[j]);
        pinMode(PINS[i], OUTPUT); digitalWrite(PINS[i], LOW); delayMicroseconds(200);
        if (!lowAnyway) { Serial.printf("  %s  <->  %s\n", NAMES[i], NAMES[j]); shorts++; }
      }
    }
  }
  allPullup();
  if (!shorts) Serial.println("  none (keys held down will show up here as R-C pairs)");
  delay(3000);
}
