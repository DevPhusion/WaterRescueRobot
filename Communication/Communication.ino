/*
  ============================================================
  ROBOT MAIN - GOP 2 CHUC NANG TREN 1 MACH ESP32:
    1) BO DAM 2 CHIEU (WiFi UDP + I2S mic/loa) - RECEIVER
    2) DIEU KHIEN DI CHUYEN (tay PS2 + PCA9685 + TA6586)
  ============================================================
  KIEN TRUC:
  Audio walkie-talkie co nhieu vong while() CHAN (blocking) toi
  ~1 giay moi lan (doc I2S, gui/nhan UDP). Neu nhet thang vao
  loop() chung voi dieu khien dong co, robot se "dung hinh",
  khong phan hoi tay cam trong luc dang noi/nghe.

  => Giai phap: dung 2 LOI (core) cua ESP32.
     - CORE 1 (loop() mac dinh cua Arduino): dieu khien dong co,
       chay 50Hz, KHONG bi chan boi audio.
     - CORE 0 (task rieng do code nay tao ra bang FreeRTOS):
       toan bo logic bo dam 2 chieu (WiFi/UDP/I2S), y het logic
       goc, chi bọc trong vong for(;;) vo han.

  PIN VA CONFIG GIU NGUYEN 100% TU 2 FILE GOC - KHONG DOI GI:
    - Audio (ROBOT/RECEIVER): GPIO 32, 25, 36, 2, 39 (PTT, INPUT thuong)
    - PS2:   GPIO 12, 13, 14, 15
    - I2C (PCA9685): GPIO 21 (SDA), 22 (SCL) - mac dinh ESP32
  Khong co xung dot chan giua 2 he thong.

  FIX LOI BIEN DICH (so voi ban truoc):
  Enum I2SMode va enum DriveMode ca hai deu co thanh vien ten
  "MODE_NONE" -> trung ten trong C++ (unscoped enum chia se
  chung namespace bao quanh) -> loi "does not name a type" /
  "conflicts with a previous declaration".
  => Da doi ten cac thanh vien cua DriveMode:
     MODE_NONE/MODE_FORWARD/MODE_LEFT/MODE_RIGHT
     -> DRIVE_NONE/DRIVE_FORWARD/DRIVE_LEFT/DRIVE_RIGHT
  Enum I2SMode giu nguyen khong doi.

  FIX LOI BIEN DICH LAN 2 (loi "DriveMode does not name a type"):
  Arduino IDE tu dong chen prototype cho MOI ham len dau file,
  TRUOC ca khi enum DriveMode duoc dinh nghia trong file. Da them
  prototype thu cong cho readDriveMode() va modeName() ngay sau
  enum, de Arduino khong tu sinh ban prototype sai vi tri nua.
  ============================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

// ============================================================
// ============  PHAN 1: BO DAM 2 CHIEU (AUDIO)  =============
// ============================================================

// ================== CAU HINH WIFI ==================
const char* WIFI_SSID     = "Redmi Note 13 Pro 5G";
const char* WIFI_PASSWORD = "Hailam09";

const uint16_t UDP_PORT = 3333;
IPAddress broadcastIP(255, 255, 255, 255);

WiFiUDP udp;

// ================== CAU HINH CHAN - ROBOT (KHONG DOI) ==================
#define I2S_PORT       I2S_NUM_0

#define MIC_SCK_PIN    32
#define MIC_WS_PIN     25
#define MIC_SD_PIN     36

#define SPK_BCLK_PIN   32
#define SPK_LRC_PIN    25
#define SPK_DIN_PIN    2

#define PTT_PIN        39   // active HIGH

// ================== CAU HINH AM THANH ==================
#define SAMPLE_RATE         16000
#define DMA_BUF_LEN         256
#define SAMPLES_PER_CHUNK   128

int16_t sampleBuffer[SAMPLES_PER_CHUNK];

enum I2SMode { MODE_NONE, MODE_MIC_RX, MODE_SPEAKER_TX };
I2SMode currentI2SMode = MODE_NONE;

void switchToMicMode() {
  if (currentI2SMode == MODE_MIC_RX) return;
  if (currentI2SMode != MODE_NONE) {
    i2s_driver_uninstall(I2S_PORT);
  }
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = MIC_SCK_PIN,
    .ws_io_num = MIC_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = MIC_SD_PIN
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  currentI2SMode = MODE_MIC_RX;
  Serial.println("[I2S] Chuyen sang che do MIC (RX)");
}

void switchToSpeakerMode() {
  if (currentI2SMode == MODE_SPEAKER_TX) return;
  if (currentI2SMode != MODE_NONE) {
    i2s_driver_uninstall(I2S_PORT);
  }
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = SPK_BCLK_PIN,
    .ws_io_num = SPK_LRC_PIN,
    .data_out_num = SPK_DIN_PIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  currentI2SMode = MODE_SPEAKER_TX;
  Serial.println("[I2S] Chuyen sang che do LOA (TX)");
}

// Mot lan lap cua logic bo dam - y het noi dung ben trong loop() goc
// cua file receiver, chi doi ten thanh ham rieng de goi trong vong
// for(;;) cua task audio.
void audioLoopOnce() {
  if (digitalRead(PTT_PIN) == HIGH) {
    Serial.println("[ROBOT] Bat dau phat (Started transmitting)");
    switchToMicMode();

    unsigned long start_time = millis();
    int packetCounter = 0;
    while (millis() - start_time < 1000 || digitalRead(PTT_PIN) == HIGH) {
      size_t bytesRead = 0;
      int32_t rawBuffer[SAMPLES_PER_CHUNK];
      i2s_read(I2S_PORT, rawBuffer, sizeof(rawBuffer), &bytesRead, portMAX_DELAY);
      int samplesRead = bytesRead / sizeof(int32_t);

      int32_t maxAmp = 0;
      int32_t minAmp = 0;
      long sumAbs = 0;
      for (int i = 0; i < samplesRead; i++) {
        // FIXED: Chuyen tu >> 11 sang >> 14 de chong re tieng
        int32_t s = rawBuffer[i] >> 14;
        if (s > 32767) s = 32767;
        if (s < -32768) s = -32768;
        sampleBuffer[i] = (int16_t)s;

        if (s > maxAmp) maxAmp = s;
        if (s < minAmp) minAmp = s;
        sumAbs += abs(s);
      }
      long avgAbs = (samplesRead > 0) ? (sumAbs / samplesRead) : 0;

      udp.beginPacket(broadcastIP, UDP_PORT);
      udp.write((uint8_t*)sampleBuffer, samplesRead * sizeof(int16_t));
      udp.endPacket();

      packetCounter++;
      if (packetCounter % 10 == 0) {
        Serial.printf("[MIC] Goi %d | samplesRead=%d | minAmp=%ld maxAmp=%ld avgAbs=%ld | ",
                      packetCounter, samplesRead, (long)minAmp, (long)maxAmp, avgAbs);
        Serial.print("mau dau: ");
        for (int i = 0; i < 5 && i < samplesRead; i++) {
          Serial.printf("%d ", sampleBuffer[i]);
        }
        Serial.println();
      }
    }
    Serial.println("[ROBOT] Ket thuc phat (Finished transmitting)");
  }

  Serial.println("[ROBOT] Bat dau nhan (Started Receiving)");
  switchToSpeakerMode();

  unsigned long start_time = millis();
  while (millis() - start_time < 1000 || digitalRead(PTT_PIN) == LOW) {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      IPAddress senderIP = udp.remoteIP();
      int len = udp.read((uint8_t*)sampleBuffer, sizeof(sampleBuffer));

      if (senderIP != WiFi.localIP()) {
        size_t bytesWritten = 0;
        i2s_write(I2S_PORT, sampleBuffer, len, &bytesWritten, portMAX_DELAY);
      }
    } else {
      delay(2);
    }
  }
  Serial.println("[ROBOT] Ket thuc nhan (Finished Receiving)");
}

// Task FreeRTOS chay rieng tren CORE 0 - chi lam viec voi audio,
// khong bao gio dung yen (for(;;) vo han).
void audioTask(void *pvParameters) {
  for (;;) {
    audioLoopOnce();
    // Nhuong CPU 1 tick cho scheduler, tranh trigger watchdog trong
    // cac truong hop bat thuong (vd mat WiFi lien tuc parsePacket=0).
    vTaskDelay(1);
  }
}

// ============================================================
// ==========  PHAN 2: DIEU KHIEN DI CHUYEN (PS2)  ============
// ============================================================
/*
  Board: VIA Banh Mi (ESP32-WROVER) + VIA Motorshield (PCA9685 + TA6586 H-bridges)

  Hanh vi:
    - Forward (D-pad Up hoac left stick Up): ca 2 dong co (MT3+MT4) tang toc cung luc.
    - Left    (D-pad Left hoac left stick Left): MT4 tang toc, MT3 bi phanh chu dong.
    - Right   (D-pad Right hoac left stick Right): MT3 tang toc, MT4 bi phanh chu dong.
    - Down (D-pad Down / stick Down), hoac tha het cac nut: dong co dang chay se
      giam toc ve 0 voi cung toc do luc tang toc.

  MT3 -> PCA9685 channel 10 (BI), channel 11 (FI)
  MT4 -> PCA9685 channel 12 (BI), channel 13 (FI)

  TA6586 truth table:
    FI=H, BI=L -> quay "forward" (dung de lai xe)
    FI=L, BI=H -> quay "backward" (khong dung trong module nay)
    FI=H, BI=H -> phanh chu dong (short-brake)
    FI=L, BI=L -> tha troi (coast)

  Thu vien can cai truoc khi nap:
    - Adafruit PWM Servo Driver Library
    - PS2X_lib, ban fork tuong thich ESP32:
      https://github.com/makerviet/Arduino-PS2X-ESP32-Makerbot

  QUAN TRONG: rut bo thu PS2 truoc khi nap firmware moi, giong huong dan
  cua board VIA.
*/

// ---------------- Chan tay cam PS2 ----------------
#define PS2_DAT 12  // MISO
#define PS2_CMD 13  // MOSI
#define PS2_SEL 15  // SS / Attention
#define PS2_CLK 14  // SCK

PS2X ps2x;

// ---------------- PCA9685 / motor shield ----------------
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // dia chi I2C mac dinh 0x40

const uint8_t MT3_BI_CH = 10;
const uint8_t MT3_FI_CH = 11;
const uint8_t MT4_BI_CH = 12;
const uint8_t MT4_FI_CH = 13;

// Neu 1 dong co quay "sai" chieu vat ly sau khi test, doi flag nay thanh true.
bool invertMotorMT3 = false;
bool invertMotorMT4 = false;

// ---------------- Cau hinh toc do ramp ----------------
const uint16_t START_PWM = 1400;
const uint16_t MAX_PWM   = 4095;
const uint16_t BRAKE_PWM = 4095;
const unsigned long RAMP_TIME_MS = 3000;
const float RAMP_RATE = (float)(MAX_PWM - START_PWM) / (float)RAMP_TIME_MS;

// ---------------- Timing vong dieu khien ----------------
const unsigned long CONTROL_INTERVAL_MS = 20;  // ~50Hz
unsigned long lastControlTime = 0;

// ---------------- Deadzone can stick ----------------
const int STICK_CENTER = 128;
const int STICK_DEADZONE = 64;

// ---------------- Toc do dang lenh cho tung dong co ----------------
float speedMT3 = 0;
float speedMT4 = 0;

// Da doi ten cac thanh vien de tranh trung "MODE_NONE" voi enum I2SMode
// o tren (I2SMode va DriveMode deu la unscoped enum, chia se chung
// namespace bao quanh -> trung ten se gay loi bien dich).
enum DriveMode { DRIVE_NONE, DRIVE_FORWARD, DRIVE_LEFT, DRIVE_RIGHT };

// QUAN TRONG: Arduino IDE tu dong chen "function prototype" cho MOI ham
// len DAU file (ngay sau cac dong #include), TRUOC ca khi enum DriveMode
// o tren duoc nhin thay. Vi 2 ham ben duoi tra ve/nhan kieu DriveMode,
// prototype tu sinh cua Arduino se bi dat truoc enum -> loi "DriveMode
// does not name a type". Khai bao thu cong prototype ngay tai day (sau
// khi enum da duoc dinh nghia) de Arduino KHONG tu sinh ban loi nua.
DriveMode readDriveMode();
const char* modeName(DriveMode m);

DriveMode readDriveMode() {
  bool up    = ps2x.Button(PSB_PAD_UP)    || (ps2x.Analog(PSS_LY) < (STICK_CENTER - STICK_DEADZONE));
  bool down  = ps2x.Button(PSB_PAD_DOWN)  || (ps2x.Analog(PSS_LY) > (STICK_CENTER + STICK_DEADZONE));
  bool left  = ps2x.Button(PSB_PAD_LEFT)  || (ps2x.Analog(PSS_LX) < (STICK_CENTER - STICK_DEADZONE));
  bool right = ps2x.Button(PSB_PAD_RIGHT) || (ps2x.Analog(PSS_LX) > (STICK_CENTER + STICK_DEADZONE));

  if (down) return DRIVE_NONE;
  if (up) return DRIVE_FORWARD;
  if (left && right) return DRIVE_NONE;
  if (left) return DRIVE_LEFT;
  if (right) return DRIVE_RIGHT;
  return DRIVE_NONE;
}

void updateMotor(bool isDriving, bool isBraking, float &speed, uint8_t biCh, uint8_t fiCh,
                  bool invert, unsigned long dt) {
  uint8_t BI = invert ? fiCh : biCh;
  uint8_t FI = invert ? biCh : fiCh;

  if (isBraking) {
    speed = 0;
    pwm.setPin(BI, BRAKE_PWM);
    pwm.setPin(FI, BRAKE_PWM);
    return;
  }

  if (isDriving) {
    if (speed <= 0) {
      speed = START_PWM;
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
    case DRIVE_FORWARD: return "FORWARD";
    case DRIVE_LEFT:     return "LEFT";
    case DRIVE_RIGHT:    return "RIGHT";
    default:             return "NONE/DECEL";
  }
}

// ============================================================
// ====================  SETUP / LOOP  =======================
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ---------- Khoi tao audio (giu nguyen y het file goc) ----------
  pinMode(PTT_PIN, INPUT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[ROBOT] Dang ket noi WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  WiFi.setSleep(WIFI_PS_NONE);

  Serial.print("[ROBOT] Da ket noi! IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);
  Serial.printf("[ROBOT] Dang lang nghe UDP broadcast tren cong %d\n", UDP_PORT);

  switchToSpeakerMode();

  // ---------- Khoi tao PCA9685 (dong co) ----------
  Wire.begin();  // SDA=21, SCL=22 mac dinh ESP32
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.setClock(400000);

  coastMotor(MT3_BI_CH, MT3_FI_CH);
  coastMotor(MT4_BI_CH, MT4_FI_CH);

  // ---------- Khoi tao tay cam PS2 ----------
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

  // ---------- Tao task audio rieng tren CORE 0 ----------
  // loop() mac dinh cua Arduino chay tren CORE 1 se lo dieu khien dong co.
  xTaskCreatePinnedToCore(
    audioTask,      // ham task
    "AudioTask",    // ten task (debug)
    8192,           // stack size (byte)
    NULL,           // tham so truyen vao
    1,              // priority
    NULL,           // handle (khong can giu)
    0               // pin vao CORE 0
  );

  Serial.println("=== [ROBOT] SETUP HOAN TAT: AUDIO (core 0) + MOTOR (core 1) ===");
}

// loop() chay tren CORE 1 - CHI lo dieu khien dong co, khong bao gio bi
// audio lam nghen vi audio chay o task rieng tren core 0.
void loop() {
  unsigned long now = millis();
  if (now - lastControlTime < CONTROL_INTERVAL_MS) {
    return;
  }
  unsigned long dt = now - lastControlTime;
  lastControlTime = now;

  ps2x.read_gamepad(false, false);

  DriveMode mode = readDriveMode();

  bool mt3Driving = (mode == DRIVE_FORWARD || mode == DRIVE_RIGHT);
  bool mt3Braking = (mode == DRIVE_LEFT);
  bool mt4Driving = (mode == DRIVE_FORWARD || mode == DRIVE_LEFT);
  bool mt4Braking = (mode == DRIVE_RIGHT);

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