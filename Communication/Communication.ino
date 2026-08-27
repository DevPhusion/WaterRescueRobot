/*
  ============================================================
  BO DAM 2 CHIEU - THIET BI: ROBOT (RECEIVER) - FIXED AUDIO
  ============================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"

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

void setup() {
  Serial.begin(115200);
  delay(1000);
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

  Serial.println("=== [ROBOT] SETUP HOAN TAT ===");
}

void loop() {
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