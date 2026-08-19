/*
  ============================================================
  HE THONG BO DAM 2 CHIEU KHONG DAY - ESP32
  ============================================================
  Mic: INMP441 (I2S input)  -> thu am nguoi dung 1, gui qua WiFi/UDP den may tinh
  Loa: MAX98357A (I2S output) + loa chong nuoc 3W 8 Ohm -> phat am thanh nhan tu may tinh
  Nut: Mach tact switch 12x12mm (module 3 chan OUT) lam nut PTT (Push-To-Talk)
       - Active HIGH: THA = 0, GIU = 1

  ------------------ SO DO DAU DAY (CAP NHAT) ------------------
  INMP441 (Mic):
    VDD  -> JP2 (chan so 2 dau cam receiver PS2, la nguon 3.3V)
    GND  -> GND
    L/R  -> GND (chon kenh trai)
    WS   -> GPIO 25
    SCK  -> GPIO 32
    SD   -> GPIO 36  (chan input-only, phu hop lam data-in)

  MAX98357A (Amp loa):
    VIN  -> 5V
    GND  -> GND
    BCLK -> GPIO 32  (CHIA SE CHUNG VOI SCK CUA MIC)
    LRC  -> GPIO 25  (CHIA SE CHUNG VOI WS CUA MIC)
    DIN  -> GPIO 2
    SD   -> bo troi
    GAIN -> bo troi

  Nut PTT (module 3 chan): VCC -> 3.3V, GND -> GND, OUT -> GPIO 39
  (GPIO 39 la chan input-only, khong co pull-up/down noi, nhung module
   da tu co pull-up/down tren board nen khong can INPUT_PULLUP)

  ------------------ LUU Y QUAN TRONG ------------------
  BCLK/SCK va LRC/WS cua mic va loa dang DUNG CHUNG 2 chan GPIO32, GPIO25.
  Ca 2 I2S port (I2S_NUM_0 cho mic, I2S_NUM_1 cho loa) deu la MASTER va
  deu tu tao xung nhip -> co the xay ra xung dot dien khi ca 2 cung day
  tin hieu ra chung 1 chan vat ly. Neu gap loi khoi dong I2S hoac am
  thanh bi nhieu/loi, can tach BCLK/LRC cua loa sang 2 GPIO khac.

  GPIO2 la mot "strapping pin" (anh huong che do boot), can dam bao
  khong bi keo HIGH tu ben ngoai luc ESP32 khoi dong, neu khong co the
  gay loi boot / khong nap duoc code.
  ============================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"

// ================== CAU HINH WIFI ==================
const char* WIFI_SSID     = "A06 của Phú";
const char* WIFI_PASSWORD = "Chiphuhang123";

const uint16_t UDP_PORT = 3333;
IPAddress computerIP(10, 100, 124, 143); // SUA THEO IP MAY TINH HIEN TAI

WiFiUDP udp;

// ================== CAU HINH CHAN (DA CAP NHAT) ==================
// I2S_NUM_0: MIC INMP441 (input)
#define MIC_I2S_PORT   I2S_NUM_0
#define MIC_SCK_PIN    32
#define MIC_WS_PIN     25
#define MIC_SD_PIN     36

// I2S_NUM_1: LOA MAX98357A (output)
#define SPK_I2S_PORT   I2S_NUM_1
#define SPK_BCLK_PIN   32
#define SPK_LRC_PIN    25
#define SPK_DIN_PIN    2

#define PTT_PIN        39   // module tact switch 3 chan, active HIGH

// ================== CAU HINH AM THANH ==================
#define SAMPLE_RATE     16000
#define DMA_BUF_LEN     512
#define UDP_PACKET_SAMPLES 512

int16_t micBuffer[UDP_PACKET_SAMPLES];
int16_t spkBuffer[UDP_PACKET_SAMPLES];

TaskHandle_t micTaskHandle;
TaskHandle_t spkTaskHandle;

// ============================================================
//  CAU HINH I2S CHO MIC (INMP441) - CHI DOC (RX)
// ============================================================
void setupMicI2S() {
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

  esp_err_t err1 = i2s_driver_install(MIC_I2S_PORT, &i2s_config, 0, NULL);
  esp_err_t err2 = i2s_set_pin(MIC_I2S_PORT, &pin_config);
  Serial.printf("[MIC I2S] driver_install=%d, set_pin=%d\n", err1, err2);
}

// ============================================================
//  CAU HINH I2S CHO LOA (MAX98357A) - CHI GHI (TX)
// ============================================================
void setupSpeakerI2S() {
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

  esp_err_t err1 = i2s_driver_install(SPK_I2S_PORT, &i2s_config, 0, NULL);
  esp_err_t err2 = i2s_set_pin(SPK_I2S_PORT, &pin_config);
  Serial.printf("[SPK I2S] driver_install=%d, set_pin=%d\n", err1, err2);
}

// ============================================================
//  TASK 1: DOC MIC + GUI UDP KHI GIU NUT PTT (CO DEBUG LOG)
// ============================================================
void micTask(void *param) {
  int32_t rawBuffer[UDP_PACKET_SAMPLES];
  size_t bytesRead = 0;
  bool lastPttState = false;
  int packetCounter = 0;

  while (true) {
    bool pttPressed = (digitalRead(PTT_PIN) == HIGH); // active HIGH: giu = 1, tha = 0

    if (pttPressed != lastPttState) {
      Serial.println(pttPressed ? "[PTT] NUT DUOC NHAN - bat dau ghi am" : "[PTT] NUT DUOC THA - dung ghi am");
      lastPttState = pttPressed;
    }

    if (pttPressed) {
      i2s_read(MIC_I2S_PORT, rawBuffer, sizeof(rawBuffer), &bytesRead, portMAX_DELAY);
      int samplesRead = bytesRead / sizeof(int32_t);

      int32_t maxAmp = 0;
      for (int i = 0; i < samplesRead; i++) {
        int32_t sample32 = rawBuffer[i] >> 11;
        if (sample32 > 32767) sample32 = 32767;
        if (sample32 < -32768) sample32 = -32768;
        micBuffer[i] = (int32_t)sample32;
        if (abs(micBuffer[i]) > maxAmp) maxAmp = abs(micBuffer[i]);
      }

      udp.beginPacket(computerIP, UDP_PORT);
      udp.write((uint8_t*)micBuffer, samplesRead * sizeof(int32_t));
      udp.endPacket();

      packetCounter++;
      if (packetCounter % 20 == 0) {
        Serial.printf("[MIC] Da gui %d goi, samplesRead=%d, do lon am thanh (maxAmp)=%d\n",
                      packetCounter, samplesRead, maxAmp);
      }

    } else {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
}

// ============================================================
//  TASK 2: NHAN UDP TU MAY TINH -> PHAT RA LOA (CO DEBUG LOG)
// ============================================================
void speakerTask(void *param) {
  int recvCounter = 0;

  while (true) {
    int packetSize = udp.parsePacket();

    if (packetSize > 0) {
      int len = udp.read((uint8_t*)spkBuffer, sizeof(spkBuffer));
      size_t bytesWritten = 0;
      i2s_write(SPK_I2S_PORT, spkBuffer, len, &bytesWritten, portMAX_DELAY);

      recvCounter++;
      if (recvCounter % 20 == 0) {
        Serial.printf("[SPK] Da nhan %d goi UDP, kich thuoc goi gan nhat=%d byte, da ghi %d byte ra I2S\n",
                      recvCounter, len, bytesWritten);
      }
    } else {
      vTaskDelay(2 / portTICK_PERIOD_MS);
    }
  }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PTT_PIN, INPUT); // KHONG dung INPUT_PULLUP - GPIO39 khong co pull noi

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Dang ket noi WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Da ket noi! Dia chi IP cua ESP32: ");
  Serial.println(WiFi.localIP());
  Serial.print("Se gui du lieu toi may tinh dia chi: ");
  Serial.println(computerIP);

  udp.begin(UDP_PORT);
  Serial.printf("Dang lang nghe UDP tren cong %d\n", UDP_PORT);

  setupMicI2S();
  setupSpeakerI2S();

  xTaskCreatePinnedToCore(micTask,     "MicTask",     4096, NULL, 1, &micTaskHandle, 1);
  xTaskCreatePinnedToCore(speakerTask, "SpeakerTask", 4096, NULL, 1, &spkTaskHandle, 0);

  Serial.println("=== SETUP HOAN TAT, HE THONG SAN SANG ===");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}