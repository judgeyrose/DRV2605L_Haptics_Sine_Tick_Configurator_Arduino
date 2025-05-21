#include <Wire.h>
#include <Adafruit_DRV2605.h>
#include <math.h>




//USER SETTING
//
//
// 
#define MOTOR_TYPE 0  // Set to 1 for ERM, 0 for LRA            //DONT CHANGE ANYTHING IN THE PROGRAM EXCEPT FOR THIS LINE 
//
//
//
// NO CHANGING ANYTHING ELSE - ADJUSTMENTS ARE MADE DURING RUNTIME IN SERIAL MONITOR




Adafruit_DRV2605 drv;

// === Configurable Parameters ===
int amplitude = 100;      // 0-127
int tickDuration = 30;    // ms the motor is on
int sineDuration = 5000;  // half breath duration (Inhale or Exhale sengment) in ms (1 full sine wave per half breath) peaks mid breath
int numTicks = 43;        // ticks per half sine, loosely related to baffles (14 x 3)

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
int dynamicAmplitude = 0;  // current value applied in RTP


// === Setup ===
void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Wire.begin();
  if (!drv.begin()) {
    Serial.println("DRV2605 not found");
    while (1)
      ;
  }
#if MOTOR_TYPE
  drv.selectLibrary(3);
  drv.useERM();
  amplitudeMin = 50;
  amplitudeMax = 100;
  minTickDuration = 15;
  tickDuration = 35;
#else
  drv.selectLibrary(6);
  drv.useLRA();

  // === BEGIN custom LRA closed-loop PWM config ===
  // Set mode to PWM input
  drv.setMode(DRV2605_MODE_PWMANALOG);  // MODE = 0x03

  // Force N_PWM_ANALOG = 0 (PWM mode), and LRA_OPEN_LOOP = 0 (closed loop)
  uint8_t ctrl3 = drv.readRegister8(DRV2605_REG_CONTROL3);
  ctrl3 &= ~(1 << 1);  // Clear N_PWM_ANALOG bit (bit 1)
  ctrl3 &= ~(1 << 0);  // Clear LRA_OPEN_LOOP bit (bit 0)
  drv.writeRegister8(DRV2605_REG_CONTROL3, ctrl3);

  // Rated voltage = 1.8V, Clamp = 2.0V → 1LSB = 21.9mV
  drv.writeRegister8(DRV2605_REG_RATEDV, 82);  // 1.8V
  drv.writeRegister8(DRV2605_REG_CLAMPV, 91);  // 2.0V

  amplitudeMin = 3;
  amplitudeMax = 127;
  minTickDuration = 1;
  tickDuration = 10;
#endif

  //drv.setMode(DRV2605_MODE_REALTIME);
  drv.setRealtimeValue(0);

  Serial.println("Enter 'start' or 'stop' to turn haptics on/off");
  Serial.println("Parameters can be changed with the following commands:");
  Serial.println("Use 'a' for Amplitude (0-127)                            (Ex. a95)");
  Serial.println("Use 'd' for Tick Duration ~0-100ms                       (Ex. d35)");
  Serial.println("Use 's' for Inhale/Exhale Duration ~3000-9000ms          (Ex. s5000)");
  Serial.println("Use 'n' for Ticks per Inhale/Exhale (n-1)                (Ex. n15 = 14 ticks)");
  Serial.println("Use 'p' to print current parameters");
}

// === Main Loop ===
void loop() {
  readSerialInput();

  unsigned long now = millis();

  // Start new tick on schedule
  if (running && !buzzing && now >= nextTickStartTime) {
    unsigned long tickInterval = timeUntilNextTick();
    float tickRate = 1000.0 / tickInterval;  // Approx Hz

    double phase = (double)tickIndex / (numTicks - 1);
    double envelope = sin(phase * PI);  // smooth half-sine envelope
    dynamicAmplitude = amplitudeMin + (int)((amplitudeMax - amplitudeMin) * envelope);

    drv.setRealtimeValue(dynamicAmplitude);
    buzzing = true;
    buzzStartTime = now;
  }


  // Stop the tick after tickDuration
  if (buzzing && now - buzzStartTime >= tickDuration) {
    drv.setRealtimeValue(0);
    buzzing = false;

    tickIndex++;
    Serial.print("Tick: ");
    Serial.print(tickIndex);
    Serial.print(" @ ");
    Serial.print(now - cycleStartTime);
    Serial.print(" ms, Amplitude: ");
    Serial.println(dynamicAmplitude);
    if (tickIndex >= numTicks - 1) {
      unsigned long cycleEnd = now;
      Serial.print("Cycle completed in ");
      Serial.print(cycleEnd - cycleStartTime);
      Serial.println(" ms");

      Serial.print("Ticks this cycle: ");
      Serial.println(tickIndex);

      tickIndex = 0;
      cycleStartTime = now;
    }

    // Schedule next tick absolutely
    nextTickStartTime = lastTickStartTime + timeUntilNextTick();
    lastTickStartTime = nextTickStartTime;
  }
}

double timeUntilNextTickDouble() {
  int i = tickIndex;
  double step = 2.0 / (numTicks - 1);
  double y_now = -1.0 + step * i;
  double y_next = -1.0 + step * (i + 1);

  y_now = constrain(y_now, -1.0, 1.0);
  y_next = constrain(y_next, -1.0, 1.0);

  double t_now = ((asin(y_now) + PI / 2.0) / PI) * sineDuration;
  double t_next = ((asin(y_next) + PI / 2.0) / PI) * sineDuration;

  return max(1.0, t_next - t_now);  // now returns *double*, not long
}


// === Time Between Ticks on Sine ===
unsigned long timeUntilNextTick() {
  int i = tickIndex;
  double step = 2.0 / (numTicks - 1);  // full span from -1 to 1
  double y_now = -1.0 + step * i;
  double y_next = -1.0 + step * (i + 1);

  y_now = constrain(y_now, -1.0, 1.0);
  y_next = constrain(y_next, -1.0, 1.0);

  double t_now = ((asin(y_now) + PI / 2.0) / PI) * sineDuration;
  double t_next = ((asin(y_next) + PI / 2.0) / PI) * sineDuration;

  double delta = max(1.0, t_next - t_now);
  //Serial.print("Tick Interval: ");
  //Serial.println(delta);
  return (unsigned long)delta;
}


// === Serial Input Parsing ===
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
    drv.setRealtimeValue(0);
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
