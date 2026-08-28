/*
  Rescue Float - Movement Module
  Board: VIA Banh Mi (ESP32-WROVER) + VIA Motorshield (PCA9685 + TA6586 H-bridges)

  Behavior:
    - Forward (D-pad Up OR left stick Up): both motors (MT3 + MT4) ramp up together.
    - Left    (D-pad Left OR left stick Left): MT4 ramps up, MT3 is actively braked.
    - Right   (D-pad Right OR left stick Right): MT3 ramps up, MT4 is actively braked.
    - Down (D-pad Down / stick Down), or releasing all inputs: whatever is running
      decelerates back to 0 at the same rate it accelerated.

  Hardware mapping (confirmed against schematic + netlist):
    MT3 -> PCA9685 channel 10 (BI / backward input), channel 11 (FI / forward input)
    MT4 -> PCA9685 channel 12 (BI / backward input), channel 13 (FI / forward input)

  TA6586 truth table (per datasheet):
    FI=H, BI=L -> spins "forward" (this is the direction we use for driving)
    FI=L, BI=H -> spins "backward" (not used in this module)
    FI=H, BI=H -> active brake (short-brake)
    FI=L, BI=L -> coast (free-wheel)

  Required libraries (install before uploading):
    - Adafruit PWM Servo Driver Library (Library Manager: "Adafruit PWM Servo Driver")
    - PS2X_lib, ESP32-compatible fork: https://github.com/makerviet/Arduino-PS2X-ESP32-Makerbot
      (the classic PS2X library on Library Manager is not guaranteed to work on ESP32 - use this fork)

  IMPORTANT: unplug the PS2 receiver from the board before uploading new firmware,
  same as noted in the VIA board's own setup guide.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

// ---------------- PS2 controller pins (per schematic: matches VIA board's SPI header) ----------------
#define PS2_DAT 12  // MISO
#define PS2_CMD 13  // MOSI
#define PS2_SEL 15  // SS / Attention
#define PS2_CLK 14  // SCK

PS2X ps2x;

// ---------------- PCA9685 / motor shield ----------------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // default I2C address 0x40

const uint8_t MT3_BI_CH = 10;  // MT3 backward-input channel
const uint8_t MT3_FI_CH = 11;  // MT3 forward-input channel
const uint8_t MT4_BI_CH = 12;  // MT4 backward-input channel
const uint8_t MT4_FI_CH = 13;  // MT4 forward-input channel

// If a motor spins the "wrong" physical direction once tested, flip its flag to true.
// This swaps which channel acts as BI vs FI for that motor, without touching any other logic.
bool invertMotorMT3 = false;
bool invertMotorMT4 = false;

// ---------------- Speed ramp settings ----------------
const uint16_t START_PWM = 700;            // PWM applied the instant a direction is first triggered
const uint16_t MAX_PWM   = 1000;             // full-speed cap
const uint16_t BRAKE_PWM = 2800;             // both channels high = active short-brake
const unsigned long RAMP_TIME_MS = 3000;     // time to go from START_PWM to MAX_PWM
// PWM units per millisecond; same rate is used for acceleration and deceleration
const float RAMP_RATE = (float)(MAX_PWM - START_PWM) / (float)RAMP_TIME_MS;

// ---------------- Control loop timing ----------------
const unsigned long CONTROL_INTERVAL_MS = 20;  // ~50Hz: matches recommended PS2 gamepad poll rate
unsigned long lastControlTime = 0;

// ---------------- Analog stick deadzone ----------------
// PS2 stick reads 0-255, center ~128. Must move this far from center to register as a press.
const int STICK_CENTER = 128;
const int STICK_DEADZONE = 64;

// ---------------- Current commanded speed per motor (0..MAX_PWM) ----------------
float speedMT3 = 0;
float speedMT4 = 0;

enum DriveMode { MODE_NONE, MODE_FORWARD, MODE_LEFT, MODE_RIGHT };

void setup() {
  Serial.begin(115200);
  delay(200);

  // --- PCA9685 init (matches manufacturer example: 50Hz for combined servo/motor use) ---
  Wire.begin();  // ESP32 default pins: SDA=21, SCL=22 - matches this board's I2C routing
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);

  // make sure both motors are off at boot, before anything else can command them
  coastMotor(MT3_BI_CH, MT3_FI_CH);
  coastMotor(MT4_BI_CH, MT4_FI_CH);

  // --- PS2 controller init ---
  int error = -1;
  for (int i = 0; i < 10; i++) {
    delay(1000);
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
    Serial.print(".");
    if (!error) break;
  }
  Serial.println();
  if (error) {
    Serial.println("PS2 controller NOT connected - check wiring/power. Motors will stay idle.");
  } else {
    Serial.println("PS2 controller connected.");
  }
}

void loop() {
  unsigned long now = millis();
  if (now - lastControlTime < CONTROL_INTERVAL_MS) {
    return;  // non-blocking: skip until it's time for the next control tick
  }
  unsigned long dt = now - lastControlTime;
  lastControlTime = now;

  ps2x.read_gamepad(false, false);

  DriveMode mode = readDriveMode();

  bool mt3Driving = (mode == MODE_FORWARD || mode == MODE_RIGHT);
  bool mt3Braking = (mode == MODE_LEFT);
  bool mt4Driving = (mode == MODE_FORWARD || mode == MODE_LEFT);
  bool mt4Braking = (mode == MODE_RIGHT);

  updateMotor(mt3Driving, mt3Braking, speedMT3, MT3_BI_CH, MT3_FI_CH, invertMotorMT3, dt);
  updateMotor(mt4Driving, mt4Braking, speedMT4, MT4_BI_CH, MT4_FI_CH, invertMotorMT4, dt);

  Serial.print("Mode: ");
  Serial.print(modeName(mode));
  Serial.print("  MT3 PWM: ");
  Serial.print((int)speedMT3);
  Serial.print(mt3Braking ? " (BRAKE)" : "");
  Serial.print("  MT4 PWM: ");
  Serial.print((int)speedMT4);
  Serial.println(mt4Braking ? " (BRAKE)" : "");
}

// Reads D-pad + left analog stick and resolves them into a single drive mode.
// Priority: Down (forces decel) > Up (forward) > Left/Right.
// Left+Right pressed together is treated as neutral (both motors decelerate).
DriveMode readDriveMode() {
  bool up    = ps2x.Button(PSB_PAD_UP)    || (ps2x.Analog(PSS_LY) < (STICK_CENTER - STICK_DEADZONE));
  bool down  = ps2x.Button(PSB_PAD_DOWN)  || (ps2x.Analog(PSS_LY) > (STICK_CENTER + STICK_DEADZONE));
  bool left  = ps2x.Button(PSB_PAD_LEFT)  || (ps2x.Analog(PSS_LX) < (STICK_CENTER - STICK_DEADZONE));
  bool right = ps2x.Button(PSB_PAD_RIGHT) || (ps2x.Analog(PSS_LX) > (STICK_CENTER + STICK_DEADZONE));

  if (down) return MODE_NONE;         // explicit brake input -> decelerate everything
  if (up) return MODE_FORWARD;
  if (left && right) return MODE_NONE;
  if (left) return MODE_LEFT;
  if (right) return MODE_RIGHT;
  return MODE_NONE;                   // nothing pressed -> decelerate everything
}

// Ramps one motor's speed up/down, or holds it at active brake, and writes the PCA9685 channels.
void updateMotor(bool isDriving, bool isBraking, float &speed, uint8_t biCh, uint8_t fiCh,
                  bool invert, unsigned long dt) {
  uint8_t BI = invert ? fiCh : biCh;
  uint8_t FI = invert ? biCh : fiCh;

  if (isBraking) {
    speed = 0;  // so the next time this motor drives, it restarts cleanly at START_PWM
    pwm.setPin(BI, BRAKE_PWM);
    pwm.setPin(FI, BRAKE_PWM);
    return;
  }

  if (isDriving) {
    if (speed <= 0) {
      speed = START_PWM;  // jump straight to the starting speed on first press
    } else {
      speed += RAMP_RATE * dt;
      if (speed > MAX_PWM) speed = MAX_PWM;
    }
  } else {
    if (speed > 0) {
      speed -= RAMP_RATE * dt;
      if (speed < 0) speed = 0;
    }
  }

  pwm.setPin(BI, 0);
  pwm.setPin(FI, (uint16_t)speed);
}

void coastMotor(uint8_t biCh, uint8_t fiCh) {
  pwm.setPin(biCh, 0);
  pwm.setPin(fiCh, 0);
}


const char* modeName(DriveMode m) {
  switch (m) {
    case MODE_FORWARD: return "FORWARD";
    case MODE_LEFT:     return "LEFT";
    case MODE_RIGHT:    return "RIGHT";
    default:            return "NONE/DECEL";
  }
}
