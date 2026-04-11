// =============================================================================
// main.cpp  —  物理フリック入力 (ATOM S3)  v2 — 2ページ対応
// =============================================================================
// ハードウェア構成:
//   MPR121 #1 (I2C 0x5A) … 電極 0-3  → パッド 0-3
//   MPR121 #2 (I2C 0x5B) … 電極 0-4  → パッド 4-8
//   MCP23017  (I2C 0x20) … GPA:
//       GPA0=↑  GPA1=←  GPA2=↓  GPA3=→  (フリック方向 active-LOW)
//       GPA4=BS (バックスペース)
//       GPA5=SP (スペース / 変換候補送り)
//   ATOM S3 内蔵ボタン (BtnA) … ページ切替 PAGE1⇔PAGE2
//   USB HID Keyboard … PC の日本語 IME にローマ字を送信
//
// PAGE1 (通常): あかさたなはまやら
// PAGE2 (BtnA): わが ざだばぱ ん小文字記号
//
// 動作フロー:
//   1. パッドに触れる         → 行を確定 (LCD表示)
//   2. フリックボタン押下      → 文字確定・HID送信
//      タイムアウト(250ms)     → 中央=1文字目を送信
//   3. BS/SP ボタン           → タッチ不要でいつでも送信
//   4. ATOM BtnA             → ページ切替
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
static constexpr uint8_t MPR_ADDR_A = 0x5A;
static constexpr uint8_t MPR_ADDR_B = 0x5B;
static constexpr uint8_t MCP_ADDR   = 0x20;

// ── MCP23017 GPA ピン番号 (active-LOW) ───────────────────────────────────────
static constexpr uint8_t PIN_UP    = 0;  // GPA0
static constexpr uint8_t PIN_LEFT  = 1;  // GPA1
static constexpr uint8_t PIN_DOWN  = 2;  // GPA2
static constexpr uint8_t PIN_RIGHT = 3;  // GPA3
static constexpr uint8_t PIN_BS    = 4;  // GPA4 バックスペース
static constexpr uint8_t PIN_SP    = 5;  // GPA5 スペース

// ── タイミング定数 ────────────────────────────────────────────────────────────
static constexpr uint32_t HOLD_MS       = 250;
static constexpr uint32_t DEBOUNCE_MS   = 50;
static constexpr uint32_t BTN_REPEAT_MS = 300;  // BS長押し連続送信間隔

// ── オブジェクト ──────────────────────────────────────────────────────────────
static Adafruit_MPR121   mprA, mprB;
static Adafruit_MCP23X17 mcp;
static USBHIDKeyboard    keyboard;

// ── 状態 ──────────────────────────────────────────────────────────────────────
enum State : uint8_t { IDLE, PAD_HELD, WAIT_RELEASE };
static State    gState     = IDLE;
static int8_t   gPad       = -1;
static uint32_t gTouchedAt = 0;
static uint8_t  gPage      = 0;     // 0=PAGE1, 1=PAGE2

// LCD 再描画フラグ (ページ切替時に強制リフレッシュ)
static bool gNeedsRedraw = true;

// =============================================================================
// ヘルパー関数
// =============================================================================

// 現在タッチされているパッド番号 (-1=無し)
static int8_t readPad() {
    uint16_t a = mprA.touched();
    uint16_t b = mprB.touched();
    for (uint8_t i = 0; i < 4; i++) if (a & (1u << i)) return (int8_t)i;
    for (uint8_t i = 0; i < 5; i++) if (b & (1u << i)) return (int8_t)(4 + i);
    return -1;
}

// GPA 全 6bit をまとめて読む
static uint8_t readGPA() { return mcp.readGPIOA(); }

// GPA からフリック方向 (GPA0-3)
static FlickDir readDir(uint8_t gpa) {
    if (!(gpa & (1u << PIN_UP)))    return DIR_UP;
    if (!(gpa & (1u << PIN_LEFT)))  return DIR_LEFT;
    if (!(gpa & (1u << PIN_DOWN)))  return DIR_DOWN;
    if (!(gpa & (1u << PIN_RIGHT))) return DIR_RIGHT;
    return DIR_CENTER;
}

// GPA から BS/SP 状態 (active-LOW)
static bool isBsPressed(uint8_t gpa) { return !(gpa & (1u << PIN_BS)); }
static bool isSpPressed(uint8_t gpa) { return !(gpa & (1u << PIN_SP)); }

// HID: 文字列を1文字ずつ送信
static void sendStr(const char* s) {
    while (*s) {
        keyboard.press(*s);
        delay(6);
        keyboard.release(*s);
        delay(6);
        s++;
    }
}

// HID: 特殊キー1発送信
static void sendKey(uint8_t key) {
    keyboard.press(key);
    delay(6);
    keyboard.release(key);
}

// LCD 描画 ─────────────────────────────────────────────────────────────────────
static void showLCD(int8_t pad, FlickDir dir, bool sent) {
    M5.Display.clear();
    M5.Display.setTextSize(1);

    // ページインジケータ (右上)
    M5.Display.setCursor(88, 2);
    M5.Display.setTextColor(gPage == 0 ? TFT_CYAN : TFT_MAGENTA);
    M5.Display.printf("P%d", gPage + 1);

    M5.Display.setTextSize(2);
    if (pad < 0) {
        M5.Display.setTextColor(TFT_DARKGREY);
        M5.Display.setCursor(4, 20);
        M5.Display.println("Flick");
        M5.Display.println("Input");
        // ページラベル
        M5.Display.setTextSize(1);
        M5.Display.setTextColor(gPage == 0 ? TFT_CYAN : TFT_MAGENTA);
        M5.Display.setCursor(4, 80);
        M5.Display.print(gPage == 0 ? "P1: あかさたなはまやら"
                                    : "P2: わがざだばぱんら");
    } else {
        M5.Display.setCursor(4, 10);
        M5.Display.setTextColor(TFT_YELLOW);
        M5.Display.printf("%s%s", padLabel(gPage, (uint8_t)pad), DIR_LABEL[dir]);
        M5.Display.setCursor(4, 50);
        M5.Display.setTextColor(sent ? TFT_GREEN : TFT_WHITE);
        M5.Display.printf("->%s", kanaGet(gPage, (uint8_t)pad, dir));
    }
    gNeedsRedraw = false;
}

// =============================================================================
// setup
// =============================================================================
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(0);
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.println("Initializing...");

    Wire.begin(2, 1);  // ATOM S3: SDA=G2 / SCL=G1

    // MPR121 #1
    if (!mprA.begin(MPR_ADDR_A, &Wire)) {
        M5.Display.setTextColor(TFT_RED);
        M5.Display.println("MPR121-A NG!"); while (true) delay(500);
    }
    // MPR121 #2
    if (!mprB.begin(MPR_ADDR_B, &Wire)) {
        M5.Display.setTextColor(TFT_RED);
        M5.Display.println("MPR121-B NG!"); while (true) delay(500);
    }
    // MCP23017
    if (!mcp.begin_I2C(MCP_ADDR, &Wire)) {
        M5.Display.setTextColor(TFT_RED);
        M5.Display.println("MCP23017 NG!"); while (true) delay(500);
    }
    // GPA0-GPA5: 入力+内部プルアップ (active-LOW)
    for (uint8_t i = 0; i < 6; i++) mcp.pinMode(i, INPUT_PULLUP);

    keyboard.begin();
    USB.begin();

    delay(600);
    showLCD(-1, DIR_CENTER, false);
}

// =============================================================================
// loop
// =============================================================================
void loop() {
    M5.update();

    // ── ATOM S3 内蔵ボタン: ページ切替 ─────────────────────────────────────────
    if (M5.BtnA.wasPressed()) {
        gPage = (gPage + 1) % PAGE_COUNT;
        gPad  = -1;
        gState = IDLE;
        showLCD(-1, DIR_CENTER, false);
        return;
    }

    int8_t   pad = readPad();
    uint8_t  gpa = readGPA();
    FlickDir dir = readDir(gpa);
    uint32_t now = millis();

    // ── BS / SP ボタン (タッチ状態問わず即時処理) ────────────────────────────────
    if (isBsPressed(gpa)) {
        delay(DEBOUNCE_MS);
        sendKey(KEY_BACKSPACE);
        // 長押し連続送信
        uint32_t t0 = millis();
        while (isBsPressed(readGPA())) {
            if (millis() - t0 > BTN_REPEAT_MS) {
                sendKey(KEY_BACKSPACE);
                t0 = millis();
            }
            delay(10);
        }
        showLCD(-1, DIR_CENTER, false);
        gState = IDLE; gPad = -1;
        delay(DEBOUNCE_MS);
        return;
    }
    if (isSpPressed(gpa)) {
        delay(DEBOUNCE_MS);
        sendKey(' ');
        while (isSpPressed(readGPA())) delay(10);
        showLCD(-1, DIR_CENTER, false);
        gState = IDLE; gPad = -1;
        delay(DEBOUNCE_MS);
        return;
    }

    // ── メインステートマシン ──────────────────────────────────────────────────────
    switch (gState) {

    case IDLE:
        if (pad >= 0) {
            gPad       = pad;
            gTouchedAt = now;
            gState     = PAD_HELD;
            showLCD(pad, DIR_CENTER, false);
        }
        break;

    case PAD_HELD:
        // パッドが変わった/離れた
        if (pad != gPad) {
            if (pad >= 0) {
                // 別パッドに移動
                gPad       = pad;
                gTouchedAt = now;
                showLCD(pad, DIR_CENTER, false);
            } else {
                // 離した → 中央(1文字目)
                showLCD(gPad, DIR_CENTER, true);
                sendStr(kanaGet(gPage, (uint8_t)gPad, DIR_CENTER));
                gPad   = -1;
                gState = WAIT_RELEASE;
            }
            break;
        }
        // 方向ボタン検出 → 即送信
        if (dir != DIR_CENTER) {
            delay(DEBOUNCE_MS);
            showLCD(gPad, dir, true);
            sendStr(kanaGet(gPage, (uint8_t)gPad, dir));
            while (readDir(readGPA()) != DIR_CENTER) delay(10);
            gPad   = -1;
            gState = WAIT_RELEASE;
            break;
        }
        // タイムアウト → 中央
        if (now - gTouchedAt >= HOLD_MS) {
            showLCD(gPad, DIR_CENTER, true);
            sendStr(kanaGet(gPage, (uint8_t)gPad, DIR_CENTER));
            while (readPad() == gPad) delay(10);
            gPad   = -1;
            gState = WAIT_RELEASE;
        }
        break;

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
