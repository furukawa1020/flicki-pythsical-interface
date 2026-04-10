// =============================================================================
// main.cpp  —  物理フリック入力 (ATOM S3)
// =============================================================================
// ハードウェア構成:
//   MPR121 #1 (I2C 0x5A) … 電極 0-3  → パッド 0(あ) 1(か) 2(さ) 3(た) 行
//   MPR121 #2 (I2C 0x5B) … 電極 0-4  → パッド 4(な) 5(は) 6(ま) 7(や) 8(ら) 行
//   MCP23017  (I2C 0x20) … GPA0=↑ GPA1=← GPA2=↓ GPA3=→ (active-LOW)
//   USB HID Keyboard … PC の日本語 IME にローマ字を送信
//
// 動作フロー:
//   1. パッドに触れる        → 行を確定
//   2. 方向ボタンを押す      → 文字を確定して HID 送信
//      (または HOLD_MS 以内にボタンなし → 中央=1文字目を送信)
//   3. パッドを離す          → 次の入力待ち
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPR121.h>
#include <Adafruit_MCP23X17.h>
#include <M5Unified.h>
#include <USB.h>
#include <USBHIDKeyboard.h>

#include "kana_map.h"

// ── I2C アドレス ──────────────────────────────────────────────────────────────
static constexpr uint8_t MPR_ADDR_A   = 0x5A;  // パッド 0-3
static constexpr uint8_t MPR_ADDR_B   = 0x5B;  // パッド 4-8
static constexpr uint8_t MCP_ADDR     = 0x20;  // 方向ボタン

// ── MCP23017 GPA ピン番号 (active-LOW) ───────────────────────────────────────
static constexpr uint8_t PIN_UP    = 0;  // GPA0
static constexpr uint8_t PIN_LEFT  = 1;  // GPA1
static constexpr uint8_t PIN_DOWN  = 2;  // GPA2
static constexpr uint8_t PIN_RIGHT = 3;  // GPA3

// ── タイミング定数 ────────────────────────────────────────────────────────────
static constexpr uint32_t HOLD_MS     = 250;  // 方向入力待ちタイムアウト → 中央
static constexpr uint32_t DEBOUNCE_MS = 50;   // チャタリング防止

// ── オブジェクト ──────────────────────────────────────────────────────────────
static Adafruit_MPR121   mprA, mprB;
static Adafruit_MCP23X17 mcp;
static USBHIDKeyboard    keyboard;

// ── 状態 ──────────────────────────────────────────────────────────────────────
enum State : uint8_t { IDLE, PAD_HELD, WAIT_RELEASE };
static State    gState       = IDLE;
static int8_t   gPad         = -1;
static uint32_t gTouchedAt   = 0;

// =============================================================================
// ヘルパー
// =============================================================================

// 現在タッチされているパッド番号を返す (-1 = 無し)
static int8_t readPad() {
    uint16_t a = mprA.touched();
    uint16_t b = mprB.touched();
    for (uint8_t i = 0; i < 4; i++) if (a & (1 << i)) return (int8_t)i;
    for (uint8_t i = 0; i < 5; i++) if (b & (1 << i)) return (int8_t)(4 + i);
    return -1;
}

// MCP23017 GPA 下位 4bit からフリック方向を読む (active-LOW)
static FlickDir readDir() {
    uint8_t gpa = mcp.readGPIOA();
    if (!(gpa & (1 << PIN_UP)))    return DIR_UP;
    if (!(gpa & (1 << PIN_LEFT)))  return DIR_LEFT;
    if (!(gpa & (1 << PIN_DOWN)))  return DIR_DOWN;
    if (!(gpa & (1 << PIN_RIGHT))) return DIR_RIGHT;
    return DIR_CENTER;
}

// HID: ASCII 文字列を送信
static void sendStr(const char* s) {
    while (*s) {
        keyboard.press(*s);
        delay(6);
        keyboard.release(*s);
        delay(6);
        s++;
    }
}

// LCD: 状態表示
static void showLCD(int8_t pad, FlickDir dir, bool sent) {
    M5.Display.clear();
    M5.Display.setTextSize(2);
    if (pad < 0) {
        M5.Display.setTextColor(TFT_DARKGREY);
        M5.Display.setCursor(4, 8);
        M5.Display.println("Flick Input");
        M5.Display.println("Ready");
    } else {
        M5.Display.setCursor(4, 4);
        M5.Display.setTextColor(TFT_YELLOW);
        M5.Display.printf("%s%s\n", PAD_LABEL[pad], DIR_LABEL[dir]);
        M5.Display.setTextColor(sent ? TFT_GREEN : TFT_WHITE);
        M5.Display.printf("-> %s\n", KANA_MAP[pad][dir]);
    }
}

// =============================================================================
// setup / loop
// =============================================================================

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(0);
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.println("Initializing...");

    // I2C (ATOM S3: SDA=G2 / SCL=G1)
    Wire.begin(2, 1);

    // ── MPR121 #1 (0x5A) ──────────────────────────────────────────────────────
    if (!mprA.begin(MPR_ADDR_A, &Wire)) {
        M5.Display.setTextColor(TFT_RED);
        M5.Display.println("MPR121-A NG!");
        while (true) delay(500);
    }

    // ── MPR121 #2 (0x5B) ──────────────────────────────────────────────────────
    if (!mprB.begin(MPR_ADDR_B, &Wire)) {
        M5.Display.setTextColor(TFT_RED);
        M5.Display.println("MPR121-B NG!");
        while (true) delay(500);
    }

    // ── MCP23017 (0x20) ───────────────────────────────────────────────────────
    if (!mcp.begin_I2C(MCP_ADDR, &Wire)) {
        M5.Display.setTextColor(TFT_RED);
        M5.Display.println("MCP23017 NG!");
        while (true) delay(500);
    }
    // GPA0-GPA3: 入力 + 内部プルアップ (active-LOW スイッチ)
    for (uint8_t i = 0; i < 4; i++) mcp.pinMode(i, INPUT_PULLUP);

    // ── USB HID キーボード ────────────────────────────────────────────────────
    keyboard.begin();
    USB.begin();

    delay(600);
    showLCD(-1, DIR_CENTER, false);
}

void loop() {
    M5.update();

    int8_t   pad = readPad();
    FlickDir dir = readDir();
    uint32_t now = millis();

    switch (gState) {

    // ── IDLE: タッチ待ち ───────────────────────────────────────────────────────
    case IDLE:
        if (pad >= 0) {
            gPad       = pad;
            gTouchedAt = now;
            gState     = PAD_HELD;
            showLCD(pad, DIR_CENTER, false);
        }
        break;

    // ── PAD_HELD: 方向ボタン or タイムアウトを待つ ──────────────────────────────
    case PAD_HELD:
        // パッドが変わったらリセット
        if (pad != gPad) {
            if (pad >= 0) {
                gPad       = pad;
                gTouchedAt = now;
                showLCD(pad, DIR_CENTER, false);
            } else {
                // 離した → 中央 (1文字目) を送信
                const FlickDir d = DIR_CENTER;
                showLCD(gPad, d, true);
                sendStr(KANA_MAP[gPad][d]);
                gPad   = -1;
                gState = WAIT_RELEASE;
            }
            break;
        }

        // 方向ボタン検出 → 即送信
        if (dir != DIR_CENTER) {
            delay(DEBOUNCE_MS);
            showLCD(gPad, dir, true);
            sendStr(KANA_MAP[gPad][dir]);
            // 方向ボタンが離されるまで待機
            while (readDir() != DIR_CENTER) delay(10);
            gPad   = -1;
            gState = WAIT_RELEASE;
            break;
        }

        // タイムアウト → 中央 (1文字目)
        if (now - gTouchedAt >= HOLD_MS) {
            showLCD(gPad, DIR_CENTER, true);
            sendStr(KANA_MAP[gPad][DIR_CENTER]);
            // パッドが離されるまで待機
            while (readPad() == gPad) delay(10);
            gPad   = -1;
            gState = WAIT_RELEASE;
        }
        break;

    // ── WAIT_RELEASE: 送信後チャタリング防止 ────────────────────────────────────
    case WAIT_RELEASE:
        if (pad < 0) {
            delay(DEBOUNCE_MS);
            showLCD(-1, DIR_CENTER, false);
            gState = IDLE;
        }
        break;
    }

    delay(8);
}
