#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// --- 腳位與通訊定義 ---
#define SELF_TEST_BTN_PIN   1  // 自檢按鈕 (GPIO 1, 綠色)
#define INSTANT_OFF_BTN_PIN 2  // Instant-Off 觸發按鈕 (GPIO 2, 紅色)
#define RELAY_CTRL_PIN      7  // 光耦合繼電器控制腳位 (GPIO 7, HIGH=通電, LOW=斷電)

// Wi-Fi 與 MQTT 設定
const char* WIFI_SSID     = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";
const char* MQTT_SERVER   = "192.168.1.101"; // 執行 Golang Broker 的電腦 IP
const int   MQTT_PORT     = 1883;
const char* MQTT_TOPIC_CMD = "ts_rmu/cmd";
const char* MQTT_TOPIC_TEL = "ts_rmu/telemetry";

// I2C 腳位 (ADS1219 24-Bit ADC)
#define I2C_SDA 4
#define I2C_SCL 5
#define ADS1219_ADDR 0x40

// TFT ILI9341 SPI 腳位
#define TFT_CS   10
#define TFT_DC    9
#define TFT_RST  14
#define TFT_MOSI 11
#define TFT_SCLK 12
#define TFT_MISO 13

// 陰極防蝕防護標準 (NACE SP0169)
const float STRAY_CURRENT_LIMIT_J = 30.0f;   // 雜散電流保護門檻 30.0 A/m²
const float CP_THRESHOLD_MV       = -850.0f;  // 陰極保護標準 -850mV/CSE

// CP 系統運作狀態列舉
enum CpSystemState {
  STATE_PROTECTED = 0,    // 保護達標 (E_off <= -850mV)
  STATE_UNDER_PROTECTION, // 電位不足 (E_off > -850mV)
  STATE_STRAY_ALARM,      // 雜散電流過載 (> 30 A/m²)
  STATE_INSTANT_OFF_TEST, // 正進行單次 Instant-Off 測試
  STATE_CYCLE_4S1S_ACTIVE // 正進行遠端 4s ON / 1s OFF 採集循環
};

enum Cycle4s1sPhase {
  CYCLE_PHASE_IDLE = 0,
  CYCLE_PHASE_ON_4S,
  CYCLE_PHASE_OFF_1S
};

// --- 100ms 批次採集數據結構 ---
struct SampleData {
  uint16_t timeMs;   // 週期內相對時間 (ms)
  float    pipeMV;   // 電位 (mV)
  float    couponJ;  // 試片電流密度 (A/m²)
  bool     relayOn;  // 繼電器狀態 (1=通電, 0=斷電)
};

const int MAX_SAMPLES = 60; // 暫存最多 6 秒高頻數據 (60 筆)
SampleData cycleBuffer[MAX_SAMPLES];
int sampleIndex = 0;
unsigned long currentCycleStartMs = 0;

// 物件建立
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// 按鈕去彈跳變數
int lastSelfTestBtnState   = HIGH;
int lastInstantOffBtnState = HIGH;
unsigned long lastSelfTestDebounce   = 0;
unsigned long lastInstantOffDebounce = 0;
const unsigned long debounceDelay    = 50;

// ADC 採集定時器 (100ms = 10Hz)
unsigned long lastAdcReadTime = 0;
const unsigned long ADC_SAMPLE_INTERVAL_MS = 100;

// 單次 Instant-Off 按鈕測試狀態
bool isInstantOffTesting       = false;
unsigned long instantOffStartTime = 0;

// 遠端 4s ON / 1s OFF 循環控制狀態
bool isRemoteCycleActive         = false;
Cycle4s1sPhase currentCyclePhase = CYCLE_PHASE_IDLE;
unsigned long cyclePhaseStartTime= 0;

// ★ 關鍵修復標記：確保每輪斷電必定鎖定一次 300ms 電位
bool capturedOffThisCycle        = false;

float lastE_on   = -1280.0f; // 通電瞬間電位 E_on
float lastE_off  = -1100.0f; // 斷電 300ms 極化電位 E_off
float lastIrDrop = 180.0f;   // IR Drop (mV)

// 系統自檢數據結構
struct SysSelfTestResult {
  float flashTotalMB;  
  float flashUsedMB;   
  float sramTotalKB;
  float sramFreeKB;
};

// --- ADS1219 底層控制 ---
void writeADS1219Cmd(uint8_t cmd) {
  Wire.beginTransmission(ADS1219_ADDR);
  Wire.write(cmd);
  Wire.endTransmission();
}

void initADS1219() {
  writeADS1219Cmd(0x06); // RESET
  delay(10);
}

float readADS1219Channel(uint8_t channelMux) {
  Wire.beginTransmission(ADS1219_ADDR);
  Wire.write(0x40); 
  uint8_t configVal = (channelMux << 5) | 0x08; // 330 SPS
  Wire.write(configVal);
  Wire.endTransmission();

  writeADS1219Cmd(0x08); // START/SYNC
  delay(4);              
  writeADS1219Cmd(0x10); // RDATA

  Wire.requestFrom((uint8_t)ADS1219_ADDR, (uint8_t)3);
  if (Wire.available() >= 3) {
    uint32_t raw = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    int32_t signedRaw = raw;
    if (signedRaw & 0x00800000) signedRaw |= 0xFF000000;
    return (float)signedRaw * (2.048f / 8388607.0f);
  }
  return 0.0f;
}

// --- MQTT 批次 JSON 遙測數據上傳 ---
void publishBatchTelemetry() {
  if (!mqttClient.connected() || sampleIndex == 0) return;

  String jsonPayload = "{\"cycle_id\":" + String(millis()) +
                       ",\"E_on\":" + String(lastE_on, 1) +
                       ",\"E_off\":" + String(lastE_off, 1) +
                       ",\"IR_Drop\":" + String(lastIrDrop, 1) +
                       ",\"count\":" + String(sampleIndex) +
                       ",\"samples\":[";

  for (int i = 0; i < sampleIndex; i++) {
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"t\":%d,\"v\":%.1f,\"j\":%.2f,\"r\":%d}",
             cycleBuffer[i].timeMs,
             cycleBuffer[i].pipeMV,
             cycleBuffer[i].couponJ,
             cycleBuffer[i].relayOn ? 1 : 0);
    jsonPayload += buf;
    if (i < sampleIndex - 1) jsonPayload += ",";
  }
  jsonPayload += "]}";

  bool success = mqttClient.publish(MQTT_TOPIC_TEL, jsonPayload.c_str());
  if (success) {
    Serial.printf("[MQTT Publish SUCCESS] 上傳 %d 筆 100ms 數據 (大小: %d Bytes)\r\n", 
                  sampleIndex, jsonPayload.length());
  } else {
    Serial.println("[MQTT Publish FAIL] 上傳失敗，請檢查 MQTT Buffer 大小設定！");
  }

  sampleIndex = 0;
}

// --- MQTT 下行指令處理 ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();
  Serial.printf("[MQTT Recv] Topic: %s | Cmd: %s\r\n", topic, message.c_str());

  if (message.equalsIgnoreCase("START_CYCLE")) {
    isRemoteCycleActive    = true;
    currentCyclePhase      = CYCLE_PHASE_ON_4S;
    cyclePhaseStartTime    = millis();
    currentCycleStartMs    = cyclePhaseStartTime;
    sampleIndex            = 0;
    capturedOffThisCycle   = false;
    digitalWrite(RELAY_CTRL_PIN, HIGH);
    Serial.println(" -> 啟動遠端 4s On / 1s Off 連續採集循環");
  } 
  else if (message.equalsIgnoreCase("STOP_CYCLE")) {
    isRemoteCycleActive    = false;
    currentCyclePhase      = CYCLE_PHASE_IDLE;
    digitalWrite(RELAY_CTRL_PIN, HIGH);
    Serial.println(" -> 停止遠端採集循環，恢復常態閉合");
  } 
  else if (message.equalsIgnoreCase("SINGLE_TEST")) {
    if (!isInstantOffTesting && !isRemoteCycleActive) {
      isInstantOffTesting  = true;
      capturedOffThisCycle = false;
      instantOffStartTime  = millis();
      currentCycleStartMs  = instantOffStartTime;
      sampleIndex          = 0;
      digitalWrite(RELAY_CTRL_PIN, LOW); // 切斷 Relay 1 秒
      Serial.println(" -> 觸發單次 Instant-Off 採集測試");
    }
  }
}

void reconnectMqtt() {
  if (!mqttClient.connected()) {
    String clientId = "ESP32S3-RMU-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("[MQTT] 已成功連線至 Golang Broker!");
      mqttClient.subscribe(MQTT_TOPIC_CMD);
      Serial.printf("[MQTT] 已訂閱指令主題: %s\r\n", MQTT_TOPIC_CMD);
    }
  }
}

// --- TFT UI 繪製邏輯 ---
void drawStaticUI() {
  tft.fillScreen(ILI9341_BLACK);

  // Header 標題欄
  tft.fillRect(0, 0, 320, 30, ILI9341_NAVY);
  tft.drawRect(0, 0, 320, 30, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE, ILI9341_NAVY);
  tft.setTextSize(2);
  tft.setCursor(10, 7);
  tft.print("TS RMU LTE/WiFi-MQTT");

  // 靜態標籤 - 系統自檢區
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  tft.setCursor(10, 35); tft.print("CPU  : ");
  tft.setCursor(10, 48); tft.print("Flash: ");
  tft.setCursor(10, 61); tft.print("SRAM : ");

  tft.drawFastHLine(10, 74, 300, ILI9341_WHITE);

  // 靜態標籤 - CP 數據區
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  tft.setCursor(10, 82);  tft.print("E_on  (ON) :");
  tft.setCursor(10, 107); tft.print("E_off(300ms):");
  tft.setCursor(10, 132); tft.print("IR Drop    :");
  tft.setCursor(10, 157); tft.print("Coupon J   :");

  // Status Box 初繪
  tft.drawRect(10, 185, 300, 48, ILI9341_WHITE);
  tft.fillRect(12, 187, 296, 44, ILI9341_DARKGREEN);
  tft.setCursor(15, 201);
  tft.setTextColor(ILI9341_WHITE, ILI9341_DARKGREEN);
  tft.setTextSize(2);
  tft.print("E_off <= -850mV PASS");
}

void updateSystemInfoUI(const SysSelfTestResult& res) {
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(55, 35); tft.printf("%s@%dMHz", ESP.getChipModel(), ESP.getCpuFreqMHz());
  tft.setCursor(55, 48); tft.printf("%.1f/%.0fMB", res.flashUsedMB, res.flashTotalMB);
  tft.setCursor(55, 61); tft.printf("%.1f/%.1fKB", res.sramFreeKB, res.sramTotalKB);
}

void updateAdcValuesUI(float currentPipeMV, float couponJ_Am2, CpSystemState currentState, bool relayIsOn) {
  tft.setTextSize(2);

  // 1. E_on (通電時直接顯示 10Hz 實時跳動電位；斷電時顯示鎖定之 lastE_on)
  tft.setCursor(170, 82);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  if (relayIsOn) {
    tft.printf("%6.1f mV", currentPipeMV); // 實體高頻刷新感！
  } else {
    tft.printf("%6.1f mV", lastE_on);      // 斷電時保留通電瞬間 E_on
  }

  // 2. E_off (斷電未滿 300ms 顯示 SAMPLING..，滿 300ms 即刻顯示最新鎖定數值)
  tft.setCursor(170, 107);
  if (!relayIsOn && !capturedOffThisCycle) {
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.print("SAMPLING..");
  } else {
    if (lastE_off <= CP_THRESHOLD_MV) tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    else tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
    tft.printf("%6.1f mV", lastE_off);
  }

  // 3. IR Drop
  tft.setCursor(170, 132);
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.printf("%6.1f mV", lastIrDrop);

  // 4. Coupon J
  tft.setCursor(170, 157);
  if (!relayIsOn) {
    tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
    tft.print(" 0.00 A/m2"); // 斷電時電流為 0
  } else if (currentState == STATE_STRAY_ALARM) {
    tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
    tft.printf("%5.1f[LIM]", couponJ_Am2);
  } else {
    tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
    tft.printf("%6.2f A/m2", couponJ_Am2);
  }

  // 5. 狀態列更新
  static CpSystemState lastState = (CpSystemState)-1;
  if (currentState != lastState) {
    lastState = currentState;
    tft.setTextSize(2);

    switch (currentState) {
      case STATE_PROTECTED:
        tft.fillRect(12, 187, 296, 44, ILI9341_DARKGREEN);
        tft.setCursor(15, 201);
        tft.setTextColor(ILI9341_WHITE, ILI9341_DARKGREEN);
        tft.print("E_off <= -850mV PASS");
        break;

      case STATE_UNDER_PROTECTION:
        tft.fillRect(12, 187, 296, 44, ILI9341_MAROON);
        tft.setCursor(15, 201);
        tft.setTextColor(ILI9341_YELLOW, ILI9341_MAROON);
        tft.print("WARN: E_off > -850mV");
        break;

      case STATE_STRAY_ALARM:
        tft.fillRect(12, 187, 296, 44, ILI9341_RED);
        tft.setCursor(16, 201);
        tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
        tft.print("ALARM: STRAY >30A/m2");
        break;

      case STATE_INSTANT_OFF_TEST:
        tft.fillRect(12, 187, 296, 44, ILI9341_NAVY);
        tft.setCursor(12, 201);
        tft.setTextColor(ILI9341_YELLOW, ILI9341_NAVY);
        tft.print("BTN 300ms OFF TEST..");
        break;

      case STATE_CYCLE_4S1S_ACTIVE:
        tft.fillRect(12, 187, 296, 44, ILI9341_BLUE);
        tft.setCursor(12, 201);
        tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE);
        tft.print("REMOTE 4s-ON/1s-OFF");
        break;
    }
  }
}

void runSystemSelfTest() {
  SysSelfTestResult res = {0, 0, 0, 0};
  res.flashTotalMB = ESP.getFlashChipSize() / (1024.0f * 1024.0f);
  res.flashUsedMB  = ESP.getSketchSize() / (1024.0f * 1024.0f);
  res.sramTotalKB  = ESP.getHeapSize() / 1024.0f;
  res.sramFreeKB   = ESP.getFreeHeap() / 1024.0f;

  updateSystemInfoUI(res);
}

void setup() {
  Serial.begin(115200);

  pinMode(SELF_TEST_BTN_PIN, INPUT_PULLUP);
  pinMode(INSTANT_OFF_BTN_PIN, INPUT_PULLUP);
  pinMode(RELAY_CTRL_PIN, OUTPUT);
  digitalWrite(RELAY_CTRL_PIN, HIGH); // 預設導通

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  initADS1219();

  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(3);
  drawStaticUI();
  runSystemSelfTest();

  // Wi-Fi 連線
  Serial.print("[Wi-Fi] 連線至: "); Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("\r\n[Wi-Fi] 連線成功! IP: " + WiFi.localIP().toString());

  // MQTT 初始化與加大 Buffer 至 4KB
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(4096);
}

void loop() {
  unsigned long currentMillis = millis();

  // 維持 MQTT 連線
  if (!mqttClient.connected()) {
    reconnectMqtt();
  }
  mqttClient.loop();

  // -----------------------------------------------------------------------
  // A. 遠端指令觸發：4 秒 ON / 1 秒 OFF 循環狀態機
  // -----------------------------------------------------------------------
  if (isRemoteCycleActive) {
    unsigned long phaseElapsed = currentMillis - cyclePhaseStartTime;

    if (currentCyclePhase == CYCLE_PHASE_ON_4S) {
      digitalWrite(RELAY_CTRL_PIN, HIGH);

      if (phaseElapsed >= 4000) {
        currentCyclePhase    = CYCLE_PHASE_OFF_1S;
        cyclePhaseStartTime  = currentMillis;
        capturedOffThisCycle = false;       // 重置鎖定標記，準備抓取 300ms
        digitalWrite(RELAY_CTRL_PIN, LOW);  // 切斷繼電器
      }
    } 
    else if (currentCyclePhase == CYCLE_PHASE_OFF_1S) {
      digitalWrite(RELAY_CTRL_PIN, LOW);

      if (phaseElapsed >= 1000) {
        publishBatchTelemetry();            // 滿 1 秒上傳 100ms 數據

        currentCyclePhase    = CYCLE_PHASE_ON_4S;
        cyclePhaseStartTime  = currentMillis;
        currentCycleStartMs  = currentMillis;
        capturedOffThisCycle = false;
        digitalWrite(RELAY_CTRL_PIN, HIGH); // 恢復通電
      }
    }
  }

  // -----------------------------------------------------------------------
  // B. 本地按鈕觸發：單次 Instant-Off 斷電測試機制 (1 秒)
  // -----------------------------------------------------------------------
  if (isInstantOffTesting && !isRemoteCycleActive) {
    unsigned long elapsed = currentMillis - instantOffStartTime;

    if (elapsed >= 1000) {
      isInstantOffTesting = false;
      digitalWrite(RELAY_CTRL_PIN, HIGH); // 恢復導通
      publishBatchTelemetry();             // 上傳此單次測試的波形
    }
  }

  // -----------------------------------------------------------------------
  // C. 定時高頻數據採集 (100ms / 10Hz) 與真實陰極防蝕物理計算
  // -----------------------------------------------------------------------
  if (currentMillis - lastAdcReadTime >= ADC_SAMPLE_INTERVAL_MS) {
    lastAdcReadTime = currentMillis;

    bool relayIsOn = (digitalRead(RELAY_CTRL_PIN) == HIGH);

    // 1. 模擬高頻採集微幅雜訊 (±1.5mV 抖動，模擬真實環境)
    float adcNoise = (random(-15, 16) / 10.0f);

    // 2. 取得試片電流密度 (1.0 ~ 5.0 A/m²)
    float raw_coupon_v = readADS1219Channel(4);
    float couponJ_Am2  = raw_coupon_v * 1.5f + 2.5f + (adcNoise * 0.05f); 
    if (couponJ_Am2 > 6.0f) couponJ_Am2 = 6.0f;
    if (couponJ_Am2 < 0.5f) couponJ_Am2 = 1.0f;

    // 3. 陰極極化電位 E_polarized 計算
    float e_polarized = -650.0f - (couponJ_Am2 * 110.0f);

    // 4. IR Drop 計算 (150 ~ 200 mV)
    float ir_drop_val = 150.0f + (couponJ_Am2 * 10.0f);

    // 5. 根據 Relay 狀態算出的實時電位
    float currentPipeMV = 0.0f;

    if (relayIsOn) {
      // 通電時：測得電位 = 純極化電位 + IR Drop + 高頻雜訊
      currentPipeMV = e_polarized - ir_drop_val + adcNoise; 
      lastE_on = currentPipeMV; // 通電時即時更新 E_on 鎖定值
    } else {
      // 斷電時：電流為 0，IR Drop 消失，僅剩純極化電位 + 微小雜訊
      currentPipeMV = e_polarized + (adcNoise * 0.2f); 
    }

    // --- ★ 精確抓取 300ms 瞬間極化電位 E_off ---
    if (!relayIsOn) {
      unsigned long offDuration = 0;
      if (isRemoteCycleActive) offDuration = currentMillis - cyclePhaseStartTime;
      else if (isInstantOffTesting) offDuration = currentMillis - instantOffStartTime;

      // 只要斷電時間滿 300ms 且本輪尚未抓取過，立即鎖定！(解決視窗錯過問題)
      if (offDuration >= 300 && !capturedOffThisCycle) {
        lastE_off            = currentPipeMV;              // 鎖定 300ms 瞬間電位
        lastIrDrop           = fabs(lastE_on - lastE_off); // 算出的 IR Drop
        capturedOffThisCycle = true;                       // 標記本輪已採集
      }
    }

    // --- 將 100ms 數據存入 RAM 緩衝區 ---
    if (sampleIndex < MAX_SAMPLES) {
      cycleBuffer[sampleIndex].timeMs  = (uint16_t)(currentMillis - currentCycleStartMs);
      cycleBuffer[sampleIndex].pipeMV  = currentPipeMV;
      cycleBuffer[sampleIndex].couponJ = relayIsOn ? couponJ_Am2 : 0.0f;
      cycleBuffer[sampleIndex].relayOn = relayIsOn;
      sampleIndex++;
    }

    // 系統狀態判斷
    CpSystemState currentState = STATE_PROTECTED;
    if (couponJ_Am2 > STRAY_CURRENT_LIMIT_J) {
      currentState = STATE_STRAY_ALARM;
    } else if (isRemoteCycleActive) {
      currentState = STATE_CYCLE_4S1S_ACTIVE;
    } else if (isInstantOffTesting) {
      currentState = STATE_INSTANT_OFF_TEST;
    } else {
      if (lastE_off <= CP_THRESHOLD_MV) currentState = STATE_PROTECTED;
      else currentState = STATE_UNDER_PROTECTION;
    }

    // 更新 TFT 畫面顯示
    updateAdcValuesUI(currentPipeMV, couponJ_Am2, currentState, relayIsOn);
  }

  // -----------------------------------------------------------------------
  // D. 本地實體按鈕處理 (GPIO 2: Instant-Off 按鈕)
  // -----------------------------------------------------------------------
  int instantOffReading = digitalRead(INSTANT_OFF_BTN_PIN);
  if (instantOffReading == LOW && lastInstantOffBtnState == HIGH) {
    if ((currentMillis - lastInstantOffDebounce) > debounceDelay) {
      if (!isInstantOffTesting && !isRemoteCycleActive) {
        isInstantOffTesting  = true;
        capturedOffThisCycle = false;
        instantOffStartTime  = currentMillis;
        currentCycleStartMs  = currentMillis;
        sampleIndex          = 0;
        digitalWrite(RELAY_CTRL_PIN, LOW); // 實體按下紅鈕：即刻斷路！
        Serial.println("[實體按鈕] 觸發 300ms Instant-Off 測試！");
      }
      lastInstantOffDebounce = currentMillis;
    }
  }
  lastInstantOffBtnState = instantOffReading;

  // (GPIO 1: 自檢按鈕)
  int selfTestReading = digitalRead(SELF_TEST_BTN_PIN);
  if (selfTestReading == LOW && lastSelfTestBtnState == HIGH) {
    if ((currentMillis - lastSelfTestDebounce) > debounceDelay) {
      runSystemSelfTest();
      lastSelfTestDebounce = currentMillis;
    }
  }
  lastSelfTestBtnState = selfTestReading;

  delay(1);
}