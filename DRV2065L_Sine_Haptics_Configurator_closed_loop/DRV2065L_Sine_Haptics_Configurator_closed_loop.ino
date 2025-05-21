#include <Wire.h>
#include <Adafruit_DRV2605.h>
#include <math.h>

// === USER SETTING ===
#define MOTOR_TYPE 0  // 1 = ERM, 0 = LRA
#define PWM_PIN 9     // PWM output pin to DRV2605L IN/TRIG

Adafruit_DRV2605 drv;

// === Configurable Parameters ===
int amplitude = 100;      // 0-127
int tickDuration = 30;    // ms the motor is on
int sineDuration = 5000;  // half breath duration
int numTicks = 43;        // ticks per half sine

// === State Variables ===
int tickIndex = 0;
bool running = false;
unsigned long lastTickTime = 0;
unsigned long nextTickDelay = 0;
bool buzzing = false;
unsigned long buzzStartTime = 0;
unsigned long cycleStartTime = 0;
int ticksThisCycle = 0;
unsigned long lastTickStartTime = 0;
unsigned long nextTickStartTime = 0;
double tickPhaseTime = 0;

int amplitudeMin = 0;
int amplitudeMax = 0;
int minTickDuration = 0;
int dynamicAmplitude = 0;

// === Setup ===
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  if (!drv.begin()) {
    Serial.println("DRV2605 not found");
    while (1);
  }

  pinMode(PWM_PIN, OUTPUT);

#if MOTOR_TYPE
  drv.selectLibrary(3);
  drv.useERM();
  drv.setMode(DRV2605_MODE_REALTIME);
  amplitudeMin = 50;
  amplitudeMax = 100;
  minTickDuration = 15;
  tickDuration = 35;
#else
  drv.selectLibrary(6);
  drv.useLRA();
  drv.setMode(DRV2605_MODE_PWMANALOG);

  uint8_t ctrl3 = drv.readRegister8(DRV2605_REG_CONTROL3);
  ctrl3 &= ~(1 << 1);  // Clear N_PWM_ANALOG
  ctrl3 &= ~(1 << 0);  // Clear LRA_OPEN_LOOP
  drv.writeRegister8(DRV2605_REG_CONTROL3, ctrl3);

  drv.writeRegister8(DRV2605_REG_RATEDV, 82);  // 1.8V
  drv.writeRegister8(DRV2605_REG_CLAMPV, 91);  // 2.0V

  amplitudeMin = 3;
  amplitudeMax = 127;
  minTickDuration = 1;
  tickDuration = 10;
#endif

  drv.setRealtimeValue(0);
  analogWrite(PWM_PIN, 0);  // Ensure motor is off

  Serial.println("Enter 'start' or 'stop' to turn haptics on/off");
  Serial.println("Use 'a' for Amplitude (e.g. a20-100)");
  Serial.println("Use 'd' for Tick Duration");
  Serial.println("Use 's' for Sine Duration");
  Serial.println("Use 'n' for Ticks per Sine");
  Serial.println("Use 'p' to print settings");
}

void loop() {
  readSerialInput();

  unsigned long now = millis();

  if (running && !buzzing && now >= nextTickStartTime) {
    unsigned long tickInterval = timeUntilNextTick();
    float tickRate = 1000.0 / tickInterval;

    double phase = (double)tickIndex / (numTicks - 1);
    double envelope = sin(phase * PI);
    dynamicAmplitude = amplitudeMin + (int)((amplitudeMax - amplitudeMin) * envelope);

    // Convert 0–127 to 0–255 for analogWrite
    int pwmValue = map(dynamicAmplitude, 0, 127, 0, 255);
    analogWrite(PWM_PIN, pwmValue);

    buzzing = true;
    buzzStartTime = now;
  }

  if (buzzing && now - buzzStartTime >= tickDuration) {
    analogWrite(PWM_PIN, 0);
    buzzing = false;

    tickIndex++;
    Serial.print("Tick: ");
    Serial.print(tickIndex);
    Serial.print(" @ ");
    Serial.print(now - cycleStartTime);
    Serial.print(" ms, Amplitude: ");
    Serial.println(dynamicAmplitude);

    if (tickIndex >= numTicks - 1) {
      Serial.print("Cycle completed in ");
      Serial.print(now - cycleStartTime);
      Serial.println(" ms");
      tickIndex = 0;
      cycleStartTime = now;
    }

    nextTickStartTime = lastTickStartTime + timeUntilNextTick();
    lastTickStartTime = nextTickStartTime;
  }
}

unsigned long timeUntilNextTick() {
  int i = tickIndex;
  double step = 2.0 / (numTicks - 1);
  double y_now = -1.0 + step * i;
  double y_next = -1.0 + step * (i + 1);

  y_now = constrain(y_now, -1.0, 1.0);
  y_next = constrain(y_next, -1.0, 1.0);

  double t_now = ((asin(y_now) + PI / 2.0) / PI) * sineDuration;
  double t_next = ((asin(y_next) + PI / 2.0) / PI) * sineDuration;

  return (unsigned long)max(1.0, t_next - t_now);
}

String inputString = "";
void readSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      inputString.trim();
      if (inputString.length() > 0) processCommand(inputString);
      inputString = "";
    } else {
      inputString += c;
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.equalsIgnoreCase("start")) {
    cycleStartTime = millis();
    tickIndex = 0;
    running = true;
    buzzing = false;
    lastTickStartTime = millis();
    nextTickStartTime = lastTickStartTime;
    nextTickDelay = timeUntilNextTick();
    Serial.println("Started.");
  } else if (cmd.equalsIgnoreCase("stop")) {
    running = false;
    analogWrite(PWM_PIN, 0);
    Serial.println("Stopped.");
  } else if (cmd.equalsIgnoreCase("p")) {
    printCurrentSettings();
  } else if (cmd.length() >= 2) {
    char code = cmd.charAt(0);
    int val = cmd.substring(1).toInt();
    switch (code) {
      case 'a':
        if (cmd.indexOf('-') > 0) {
          int sep = cmd.indexOf('-');
          int minVal = cmd.substring(1, sep).toInt();
          int maxVal = cmd.substring(sep + 1).toInt();
          amplitudeMin = constrain(minVal, 0, 127);
          amplitudeMax = constrain(maxVal, amplitudeMin, 127);
          Serial.print("Amplitude range set to ");
          Serial.print(amplitudeMin);
          Serial.print(" - ");
          Serial.println(amplitudeMax);
        } else {
          Serial.println("Use format: aMIN-MAX (e.g. a30-100)");
        }
        break;
      case 'd':
        tickDuration = constrain(val, 10, 100);
        Serial.print("Tick duration set to ");
        Serial.print(tickDuration);
        Serial.println(" ms");
        break;
      case 's':
        sineDuration = max(val, 100);
        Serial.print("Sine half-cycle set to ");
        Serial.print(sineDuration);
        Serial.println(" ms");
        break;
      case 'n':
        numTicks = constrain(val, 1, 100);
        Serial.print("Number of ticks set to ");
        Serial.println(numTicks);
        break;
      default:
        Serial.println("Unknown command. Use a/d/s/n/p/start/stop");
        break;
    }
  } else {
    Serial.println("Invalid command. Use a/d/s/n/p/start/stop");
  }
}

void printCurrentSettings() {
  Serial.println("=== Current Settings ===");
  Serial.print("Amplitude Range (a): ");
  Serial.print(amplitudeMin);
  Serial.print(" - ");
  Serial.println(amplitudeMax);
  Serial.print("Tick Duration (d): ");
  Serial.print(tickDuration);
  Serial.println(" ms");
  Serial.print("Sine Half Duration (s): ");
  Serial.print(sineDuration);
  Serial.println(" ms");
  Serial.print("Ticks per Half Sine (n): ");
  Serial.println(numTicks);
  Serial.println("========================");
}
