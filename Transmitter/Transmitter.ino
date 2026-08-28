#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"

//Wifi config
const char* WIFI_SSID     = "Redmi Note 13 Pro 5G";
const char* WIFI_PASSWORD = "Hailam09";

const uint16_t UDP_PORT = 3333;
IPAddress broadcastIP(255, 255, 255, 255);

WiFiUDP udp;

//Pins
#define I2S_PORT       I2S_NUM_0

#define MIC_SCK_PIN    32
#define MIC_WS_PIN     25
#define MIC_SD_PIN     33

#define SPK_BCLK_PIN   26
#define SPK_LRC_PIN    27
#define SPK_DIN_PIN    22
#define SPK_SD_PIN     21   // Set to HIGH -> Turn on speaker

#define PTT_PIN        17   // active HIGH

bool isPttPressed() {
  return digitalRead(PTT_PIN) == HIGH;
}

// Audio config
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

  pinMode(PTT_PIN, INPUT_PULLDOWN);

  pinMode(SPK_SD_PIN, OUTPUT);
  digitalWrite(SPK_SD_PIN, HIGH);
  Serial.println("[SD] Da keo GPIO21 (SD) len HIGH de bat chip MAX98357A");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[TAY CAM] Dang ket noi WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  WiFi.setSleep(WIFI_PS_NONE);

  Serial.print("[TAY CAM] Da ket noi! IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);
  Serial.printf("[TAY CAM] Dang lang nghe UDP broadcast tren cong %d\n", UDP_PORT);

  switchToSpeakerMode();

  Serial.println("=== [TAY CAM] SETUP HOAN TAT ===");
}

void loop() {
  if (isPttPressed()) {
    Serial.println("[TAY CAM] Bat dau phat (Started transmitting)");
    switchToMicMode();

    unsigned long start_time = millis();
    while (millis() - start_time < 1000 || isPttPressed()) {
      size_t bytesRead = 0;
      int32_t rawBuffer[SAMPLES_PER_CHUNK];
      i2s_read(I2S_PORT, rawBuffer, sizeof(rawBuffer), &bytesRead, portMAX_DELAY);
      int samplesRead = bytesRead / sizeof(int32_t);

      for (int i = 0; i < samplesRead; i++) {
        // Chong re tieng: giam bit shift tu >>11 xuong >>14
        int32_t s = rawBuffer[i] >> 14;
        if (s > 32767) s = 32767;
        if (s < -32768) s = -32768;
        sampleBuffer[i] = (int16_t)s;
      }

      udp.beginPacket(broadcastIP, UDP_PORT);
      udp.write((uint8_t*)sampleBuffer, samplesRead * sizeof(int16_t));
      udp.endPacket();
    }
    Serial.println("[TAY CAM] Ket thuc phat (Finished transmitting)");
  }

  Serial.println("[TAY CAM] Bat dau nhan (Started Receiving)");
  switchToSpeakerMode();

  unsigned long start_time = millis();
  int recvCounter = 0;
  int ignoredSelfCounter = 0;
  while (millis() - start_time < 1000 || !isPttPressed()) {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      IPAddress senderIP = udp.remoteIP();
      int len = udp.read((uint8_t*)sampleBuffer, sizeof(sampleBuffer));

      if (senderIP != WiFi.localIP()) {
        size_t bytesWritten = 0;
        i2s_write(I2S_PORT, sampleBuffer, len, &bytesWritten, portMAX_DELAY);

        recvCounter++;
        if (recvCounter % 10 == 0) {
          Serial.printf("[SPK] Da nhan %d goi tu %s, len=%d byte, i2s da ghi=%d byte\n",
                        recvCounter, senderIP.toString().c_str(), len, bytesWritten);
        }
      } else {
        ignoredSelfCounter++;
        if (ignoredSelfCounter % 20 == 0) {
          Serial.printf("[SPK] Da bo qua %d goi TU CHINH MINH (senderIP == myIP == %s)\n",
                        ignoredSelfCounter, senderIP.toString().c_str());
        }
      }
    } else {
      delay(2);
    }
  }
  Serial.println("[TAY CAM] Ket thuc nhan (Finished Receiving)");
}