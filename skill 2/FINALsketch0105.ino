#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

// --- 硬件配置 (保持不变) ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define CHIP_SELECT   4    

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TinyGPSPlus gps;
SdFat SD;
SdFile dataFile;

#define STRIP1_PIN 6      
#define STRIP2_PIN 5      
#define TOTAL_LEDS 60    
#define ACTIVE_LEDS 50  
Adafruit_NeoPixel strip1(TOTAL_LEDS, STRIP1_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(TOTAL_LEDS, STRIP2_PIN, NEO_GRB + NEO_KHZ800);

const unsigned char heart_bmp[] PROGMEM = {
  0b00000000, 0b01100110, 0b11111111, 0b11111111,
  0b11111111, 0b01111110, 0b00111100, 0b00011000
};
const int gsrPin = A3, flexPin1 = A1, pulsePin = A2;
const int motorPin = 3, flexLedPin1 = 8;

// --- 传感器核心变量 (保持不变) ---
float smoothF1 = 0;
float filterWeight = 0.1;
float baseline = 512;
bool inBeat = false;
unsigned long lastBeatTime = 0;
int bpm = 0;
int flexHistory[SCREEN_WIDTH];
int writePtr = 0;

// --- 调度计时器 ---
unsigned long lastSDWrite = 0; 
unsigned long lastOLED = 0;
unsigned long lastLED = 0;
unsigned long lastSensorRead = 0;

bool sdActive = false;
long logIndex = 0;
String filename = "Data.csv";
static float ledOffset = 0;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  // --- 【仅在此处修改：增加删除旧文件逻辑】 ---
  if (SD.begin(CHIP_SELECT, SD_SCK_MHZ(4))) {
    sdActive = true;
    
    // 如果发现 SD 卡里已经有 Data.csv，先把它删掉
    if (SD.exists(filename.c_str())) {
      SD.remove(filename.c_str());
      Serial.println(F("Old Data.csv deleted."));
    }

    // 重新创建全新的文件并写入表头
    if (dataFile.open(filename.c_str(), O_RDWR | O_CREAT | O_AT_END)) {
      dataFile.println(F("index,time,lat,lng,flex,gsr,bpm"));
      dataFile.close();
      Serial.println(F("New Data.csv created."));
    }
  }

  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  strip1.begin(); strip2.begin();
  strip1.setBrightness(40); strip2.setBrightness(40);
  strip1.show(); strip2.show();
  
  pinMode(motorPin, OUTPUT);
  pinMode(flexLedPin1, OUTPUT);

  smoothF1 = analogRead(flexPin1);
  for(int i=0; i<SCREEN_WIDTH; i++) flexHistory[i] = 63;
  Serial.println(F("GPS MODIFIED - AUTO-DELETE ENABLED"));
}

void loop() {
  unsigned long now = millis();

  // --- GPS 解析部分：保持在主循环最外层持续运行 ---
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // --- 1. 核心计算 (保持不变) ---
  if (now - lastSensorRead >= 20) {
    lastSensorRead = now;
    int rawF1 = analogRead(flexPin1);
    smoothF1 = (smoothF1 * (1.0 - filterWeight)) + (rawF1 * filterWeight);
    int rawPulse = analogRead(pulsePin);
    baseline = (baseline * 0.985) + (rawPulse * 0.015);
    float acSignal = rawPulse - baseline;
    if (!inBeat && acSignal > 8.0 && (now - lastBeatTime > 450)) {
      inBeat = true;
      int instantBpm = 60000 / (now - lastBeatTime);
      if (instantBpm > 45 && instantBpm < 160) bpm = (bpm == 0) ? instantBpm : (bpm * 0.7 + instantBpm * 0.3);
      lastBeatTime = now;
    }
    if (inBeat && acSignal < 0) inBeat = false;
  }

  // --- 2. 核心 GPS 改动：强制同步记录逻辑 (每1000ms一次快照) ---
  if (now - lastSDWrite >= 1000) { 
    lastSDWrite = now; 

    String timeString;
    // 强制从 GPS 获取最新的快照时间
    if (gps.time.isValid()) {
      int itHour = gps.time.hour() + 1; // 意大利 UTC+1
      if (itHour >= 24) itHour -= 24;
      char tBuf[12];
      sprintf(tBuf, "%02d:%02d:%02d", itHour, gps.time.minute(), gps.time.second());
      timeString = String(tBuf);
    } else {
      timeString = String(now / 1000) + "s"; 
    }

    // 强制捕捉当前的经纬度快照
    double currentLat = gps.location.isValid() ? gps.location.lat() : 0.0;
    double currentLng = gps.location.isValid() ? gps.location.lng() : 0.0;
    int currentGSR = analogRead(gsrPin);
    int currentFlex = (int)smoothF1;

    // A. 写入 SD 卡 (同步且高精度)
    if (sdActive && dataFile.open(filename.c_str(), O_RDWR | O_CREAT | O_AT_END)) {
      dataFile.print(logIndex);    dataFile.print(",");
      dataFile.print(timeString);  dataFile.print(",");
      dataFile.print(currentLat, 6); dataFile.print(","); // 强制 6 位精度
      dataFile.print(currentLng, 6); dataFile.print(","); // 强制 6 位精度
      dataFile.print(currentFlex);  dataFile.print(",");
      dataFile.print(currentGSR);   dataFile.print(",");
      dataFile.println(bpm);
      dataFile.close();
    }

    // B. 串口全息同步显示
    Serial.print(F("[LOG ")); Serial.print(logIndex++); Serial.print(F("] "));
    Serial.print(timeString);
    Serial.print(F(" | LAT:")); Serial.print(currentLat, 6);
    Serial.print(F(" | LNG:")); Serial.print(currentLng, 6);
    Serial.print(F(" | BPM:")); Serial.print(bpm);
    Serial.print(F(" | GSR:")); Serial.print(currentGSR);
    Serial.print(F(" | FLX:")); Serial.print(currentFlex);
    Serial.print(F(" | SAT:")); Serial.println(gps.satellites.value());
  }

  // --- 3. 马达反馈 (保持不变) ---
  if (smoothF1 > 35) {
    if (now % 100 < 85) digitalWrite(motorPin, HIGH);
    else digitalWrite(motorPin, LOW);
    digitalWrite(flexLedPin1, HIGH);
  } else {
    digitalWrite(motorPin, LOW);
    digitalWrite(flexLedPin1, LOW);
  }

  // --- 4. 灯带刷新 (保持不变) ---
  if (now - lastLED > 20) {
    unsigned long delta = now - lastLED;
    lastLED = now;
    float scrollSpeed = (smoothF1 < 30) ? 0.0015 : (smoothF1 < 50 ? 0.0022 : 0.005);
    bool strobe = (smoothF1 >= 50) ? ((now / 70) % 2 == 0) : true;
    ledOffset += scrollSpeed * delta * 10.0;
    if (ledOffset > 6.0) ledOffset -= 6.0;

    for(int i=0; i < ACTIVE_LEDS; i++) {
      if (!strobe) strip1.setPixelColor(i, strip1.Color(2, 2, 2));
      else {
        float pos = fmod((float)i + ledOffset, 6.0);
        strip1.setPixelColor(i, (pos < 2.0) ? strip1.Color(180, 180, 180) : strip1.Color(5, 5, 5));
      }
      strip2.setPixelColor(i, (inBeat || (now - lastBeatTime < 150)) ? strip2.Color(255, 0, 0) : strip2.Color(35, 0, 0));
    }
    strip1.show(); strip2.show();
  }

  // --- 5. OLED 显示 (保持不变) ---
  if (now - lastOLED > 80) {  
    lastOLED = now;
    flexHistory[writePtr] = map(constrain((int)smoothF1, 10, 150), 10, 150, 62, 15);
    writePtr = (writePtr + 1) % SCREEN_WIDTH;
    display.clearDisplay();
    display.drawBitmap(0, 0, heart_bmp, 8, 8, WHITE);
    display.setCursor(12, 1); display.print(F("BPM:")); display.print(bpm);
    display.setCursor(80, 1); display.print(F("GSR:")); display.print(analogRead(gsrPin));
    display.drawFastHLine(0, 11, 128, WHITE);
    for (int x = 0; x < SCREEN_WIDTH - 1; x++) {
      display.drawLine(x, flexHistory[(writePtr+x)%SCREEN_WIDTH], x+1, flexHistory[(writePtr+x+1)%SCREEN_WIDTH], WHITE);
    }
    display.display();
  }
}