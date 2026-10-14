#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <ir_Daikin.h>
#include <ir_LG.h>
#include <ir_Gree.h>
#include <ir_Carrier.h>
#include <ir_Midea.h>
#include <ir_Hitachi.h>
#include <ir_Mitsubishi.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

// =================================================================
// HARDWARE PIN DEFINITIONS
// =================================================================
#define PIN_I2C_SDA     4
#define PIN_I2C_SCL     5
#define PIN_ENC_A       6
#define PIN_ENC_B       7
#define PIN_ENC_SW      0
#define PIN_BTN_PWR     2
#define PIN_BUZZER      3
#define PIN_IR_TX       1
#define PIN_IR_RX       10

// =================================================================
// SYSTEM CONSTANTS & STRUCTURES
// =================================================================
#define INACTIVITY_TIMEOUT_MS  30000
#define LONG_PRESS_TIMEOUT_MS  2000
#define SHORT_PRESS_MAX_MS     500
#define DEBOUNCE_DELAY_MS      30
#define LEARN_TIMEOUT_MS       10000
#define RAW_BUFFER_CAPACITY    800
#define NVS_DEBOUNCE_DELAY_MS  1500
#define RENDER_INTERVAL_MS     50      // Max OLED refresh rate (~20 fps)
#define BUZZER_GAP_MS          40      // Silence between double-beep pulses
#define TEMP_MIN               18
#define TEMP_MAX               32
#define NUM_CUSTOM_SLOTS       3       // Custom 1, 2, 3

enum AppState {
    STATE_CONTROL_ROOM,
    STATE_BRAND_MENU,
    STATE_LEARNING,
    STATE_SAVE_SLOT          // New: choose which custom slot to write to
};

const char* BRAND_NAMES[] = {
    "Voltas",
    "Daikin",
    "LG",
    "Carrier",
    "Hitachi",
    "Mitsubishi",
    "Custom 1",
    "Custom 2",
    "Custom 3",
    "[+] Learn Remote"
};
const uint8_t TOTAL_BRANDS      = sizeof(BRAND_NAMES) / sizeof(BRAND_NAMES[0]);
#define INDEX_LEARN_REMOTE        (TOTAL_BRANDS - 1)
#define INDEX_CUSTOM_FIRST        (TOTAL_BRANDS - 1 - NUM_CUSTOM_SLOTS)  // 6

// Forward declarations
void buzzerTrigger(uint16_t durationMs, bool doubleBeep = false);
void requestRender();
void renderCurrentScreen();

// =================================================================
// HARDWARE DRIVERS & STATE VARIABLES
// =================================================================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
Preferences prefs;

IRsend irsend(PIN_IR_TX);
IRrecv irrecv(PIN_IR_RX, RAW_BUFFER_CAPACITY, 120, true); // 120 ms gap timeout
decode_results irResults;

// Specific AC Protocol Handlers
IRDaikinESP     irDaikin(PIN_IR_TX);
IRLgAc          irLG(PIN_IR_TX);
IRGreeAC        irVoltas(PIN_IR_TX);   // Voltas via Gree OEM protocol
IRMideaAC       irMidea(PIN_IR_TX);    // Carrier India = Midea OEM (48-bit)
IRHitachiAc     irHitachi(PIN_IR_TX);
IRMitsubishiAC  irMitsubishi(PIN_IR_TX);

// Operational States
AppState currentState = STATE_CONTROL_ROOM;
uint8_t selectedBrand = 0;
uint8_t currentTemp = 24;
bool isACOn = true;
uint8_t menuCursor = 0;

// Custom Remote Storage (3 independent slots)
uint16_t customRawBuffer[NUM_CUSTOM_SLOTS][RAW_BUFFER_CAPACITY];
uint16_t customRawLength[NUM_CUSTOM_SLOTS] = {0, 0, 0};

// Staging buffer: holds the captured signal until the user picks a slot
uint16_t stagedRawBuffer[RAW_BUFFER_CAPACITY];
uint16_t stagedRawLength = 0;

// Save-slot screen cursor (0–2)
uint8_t saveSlotCursor = 0;

// Deferred Flash Commit (NVS Wear Protection)
bool settingsDirty = false;
unsigned long lastSettingsChange = 0;

// Encoder Gray-Code Table
const int8_t ENCODER_STATES[] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};
uint8_t oldEncoderState = 0;
int8_t positionDelta = 0;

// Button Debounce & Timers
unsigned long encSwPressTime = 0;
bool encSwPressed = false;
bool encSwIgnore = false;
unsigned long pwrBtnPressTime = 0;
bool pwrBtnPressed = false;
bool pwrBtnIgnore = false;

// Non-blocking Buzzer
unsigned long buzzerStartTime = 0;
uint16_t buzzerDuration = 0;
bool buzzerDoubleBeepPending = false;
bool buzzerInGap = false;

// Deferred Rendering
bool needsRender = true;
unsigned long lastRenderTime = 0;

// System Timers
unsigned long lastActivityTime = 0;
unsigned long learnStartTime = 0;

// =================================================================
// AUDIO FEEDBACK SUBSYSTEM
// =================================================================
void buzzerTrigger(uint16_t durationMs, bool doubleBeep) {
    digitalWrite(PIN_BUZZER, HIGH);
    buzzerStartTime = millis();
    buzzerDuration = durationMs;
    buzzerDoubleBeepPending = doubleBeep;
    buzzerInGap = false;
}

void buzzerUpdate() {
    if (buzzerInGap) {
        if (millis() - buzzerStartTime >= BUZZER_GAP_MS) {
            buzzerInGap = false;
            buzzerTrigger(40, false);
        }
        return;
    }

    if (buzzerDuration > 0 && (millis() - buzzerStartTime >= buzzerDuration)) {
        digitalWrite(PIN_BUZZER, LOW);
        buzzerDuration = 0;

        if (buzzerDoubleBeepPending) {
            buzzerDoubleBeepPending = false;
            buzzerInGap = true;
            buzzerStartTime = millis();
        }
    }
}

// =================================================================
// NON-VOLATILE STORAGE (NVS)
// =================================================================

// NVS key helpers: "c_len_0", "c_raw_0", etc.
static void nvsSlotLenKey(uint8_t slot, char* buf) { sprintf(buf, "c_len_%d", slot); }
static void nvsSlotRawKey(uint8_t slot, char* buf) { sprintf(buf, "c_raw_%d", slot); }

void loadSettings() {
    prefs.begin("ac_rem", false);
    selectedBrand   = prefs.getUChar("brand", 0);
    currentTemp     = prefs.getUChar("temp", 24);
    isACOn          = prefs.getBool("pwr", true);

    char lenKey[12], rawKey[12];
    for (uint8_t s = 0; s < NUM_CUSTOM_SLOTS; s++) {
        nvsSlotLenKey(s, lenKey);
        nvsSlotRawKey(s, rawKey);
        uint16_t len = prefs.getUShort(lenKey, 0);
        if (len > 0 && len <= RAW_BUFFER_CAPACITY) {
            customRawLength[s] = len;
            prefs.getBytes(rawKey, customRawBuffer[s], len * sizeof(uint16_t));
        } else {
            customRawLength[s] = 0;
        }
    }
    prefs.end();

    // Sanity clamps
    if (selectedBrand >= TOTAL_BRANDS - 1) selectedBrand = 0; // never land on [+] Learn
    if (currentTemp < TEMP_MIN || currentTemp > TEMP_MAX) currentTemp = 24;
}

void saveSettings() {
    prefs.begin("ac_rem", false);
    prefs.putUChar("brand", selectedBrand);
    prefs.putUChar("temp", currentTemp);
    prefs.putBool("pwr", isACOn);
    prefs.end();
    settingsDirty = false;
}

void saveCustomSlot(uint8_t slot, uint16_t* buffer, uint16_t len) {
    char lenKey[12], rawKey[12];
    nvsSlotLenKey(slot, lenKey);
    nvsSlotRawKey(slot, rawKey);
    prefs.begin("ac_rem", false);
    prefs.putUShort(lenKey, len);
    prefs.putBytes(rawKey, buffer, len * sizeof(uint16_t));
    prefs.end();
}

void markSettingsChanged() {
    settingsDirty = true;
    lastSettingsChange = millis();
}

// =================================================================
// IR TRANSMISSION SUBSYSTEM
// =================================================================
void sendIRCommand() {
    Serial.printf("[IR TX] Brand: %s | Pwr: %d | Temp: %d C\n",
                  BRAND_NAMES[selectedBrand], isACOn, currentTemp);

    // Custom slot replay (brands 6, 7, 8 → slots 0, 1, 2)
    if (selectedBrand >= INDEX_CUSTOM_FIRST && selectedBrand < INDEX_LEARN_REMOTE) {
        uint8_t slot = selectedBrand - INDEX_CUSTOM_FIRST;
        if (customRawLength[slot] > 0) {
            irsend.sendRaw(customRawBuffer[slot], customRawLength[slot], 38);
            buzzerTrigger(60);
        }
        // Silent no-op if slot is empty — by design
        return;
    }

    switch (selectedBrand) {
        case 0: // Voltas (Gree OEM protocol)
            irVoltas.setPower(isACOn);
            irVoltas.setTemp(currentTemp);
            irVoltas.setMode(kGreeCool);
            irVoltas.setFan(kGreeFanAuto);
            irVoltas.send();
            break;

        case 1: // Daikin
            irDaikin.setPower(isACOn);
            irDaikin.setTemp(currentTemp);
            irDaikin.setMode(kDaikinCool);
            irDaikin.setFan(kDaikinFanAuto);
            irDaikin.send();
            break;

        case 2: // LG
            irLG.setPower(isACOn);
            irLG.setTemp(currentTemp);
            irLG.setMode(kLgAcCool);
            irLG.setFan(kLgAcFanAuto);
            irLG.send();
            break;

        case 3: // Carrier (India = Midea OEM, 48-bit protocol)
            irMidea.setPower(isACOn);
            irMidea.setTemp(currentTemp);
            irMidea.setMode(kMideaACCool);
            irMidea.setFan(kMideaACFanAuto);
            irMidea.send();
            break;

        case 4: // Hitachi
            irHitachi.setPower(isACOn);
            irHitachi.setTemp(currentTemp);
            irHitachi.setMode(kHitachiAcCool);
            irHitachi.setFan(kHitachiAcFanAuto);
            irHitachi.send();
            break;

        case 5: // Mitsubishi
            irMitsubishi.setPower(isACOn);
            irMitsubishi.setTemp(currentTemp);
            irMitsubishi.setMode(kMitsubishiAcCool);
            irMitsubishi.setFan(kMitsubishiAcFanAuto);
            irMitsubishi.send();
            break;

        default:
            return;
    }

    buzzerTrigger(60);
}

// =================================================================
// UI RENDERING ENGINE
// =================================================================
void renderControlRoom() {
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 9, BRAND_NAMES[selectedBrand]);

    const char* pwrLabel = isACOn ? "SYS: ON" : "SYS: OFF";
    u8g2.drawStr(126 - u8g2.getStrWidth(pwrLabel), 9, pwrLabel);
    u8g2.drawHLine(0, 12, 128);

    u8g2.setFont(u8g2_font_logisoso24_tr);
    char tempStr[6];
    snprintf(tempStr, sizeof(tempStr), "%02d", currentTemp);
    u8g2.drawStr(22, 44, tempStr);

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(60, 28, "o");
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(68, 42, "C");

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(88, 26, "COOL");
    u8g2.drawStr(88, 38, "AUTO");

    u8g2.drawHLine(0, 52, 128);

    if (encSwPressed && (millis() - encSwPressTime > 200)) {
        unsigned long elapsed = millis() - encSwPressTime;
        int barWidth = map(constrain(elapsed, 0, LONG_PRESS_TIMEOUT_MS),
                           0, LONG_PRESS_TIMEOUT_MS, 0, 124);
        u8g2.drawBox(2, 55, barWidth, 6);
    } else {
        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.drawStr(4, 62, "CLICK:TX  HOLD:MENU");
    }

    u8g2.sendBuffer();
}

void renderBrandMenu() {
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 9, "-- SELECT BRAND --");
    u8g2.drawHLine(0, 11, 128);

    int startIdx = max(0, menuCursor - 1);
    if (startIdx + 4 > TOTAL_BRANDS) startIdx = max(0, TOTAL_BRANDS - 4);

    for (int i = 0; i < 4; i++) {
        int itemIdx = startIdx + i;
        if (itemIdx >= TOTAL_BRANDS) break;

        int yPos = 21 + (i * 10);
        if (itemIdx == menuCursor) {
            u8g2.drawBox(0, yPos - 8, 128, 10);
            u8g2.setDrawColor(0);
            u8g2.drawStr(4, yPos, BRAND_NAMES[itemIdx]);
            u8g2.setDrawColor(1);
        } else {
            u8g2.drawStr(4, yPos, BRAND_NAMES[itemIdx]);
        }
    }

    u8g2.drawHLine(0, 54, 128);
    u8g2.drawStr(2, 63, "PWR: OK | SW: CANCEL");
    u8g2.sendBuffer();
}

void renderLearningScreen() {
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(12, 12, "LEARNING MODE");
    u8g2.drawHLine(0, 15, 128);

    u8g2.drawStr(6, 28, "Point Remote at RX");
    u8g2.drawStr(6, 40, "Press a button...");

    if ((millis() / 400) % 2 == 0) {
        u8g2.drawDisc(112, 34, 4);
    }

    unsigned long elapsed = millis() - learnStartTime;
    unsigned long remaining = (elapsed >= LEARN_TIMEOUT_MS) ? 0 : (LEARN_TIMEOUT_MS - elapsed);
    int remWidth = map(remaining, 0, LEARN_TIMEOUT_MS, 0, 124);
    u8g2.drawFrame(1, 52, 126, 8);
    u8g2.drawBox(2, 53, remWidth, 6);

    u8g2.sendBuffer();
}

// New screen: shown after a successful capture, before saving
void renderSaveSlotScreen() {
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(2, 9, "SAVE TO SLOT");
    u8g2.drawHLine(0, 11, 128);

    // 3 items always fit; no scrolling needed
    const char* slotNames[NUM_CUSTOM_SLOTS] = {"Custom 1", "Custom 2", "Custom 3"};
    for (uint8_t i = 0; i < NUM_CUSTOM_SLOTS; i++) {
        int yPos = 24 + (i * 12);
        if (i == saveSlotCursor) {
            u8g2.drawBox(0, yPos - 8, 128, 10);
            u8g2.setDrawColor(0);
            u8g2.drawStr(4, yPos, slotNames[i]);
            // Show "(saved)" tag if slot already has data
            if (customRawLength[i] > 0) {
                u8g2.drawStr(90, yPos, "(saved)");
            }
            u8g2.setDrawColor(1);
        } else {
            u8g2.drawStr(4, yPos, slotNames[i]);
            if (customRawLength[i] > 0) {
                u8g2.drawStr(90, yPos, "(saved)");
            }
        }
    }

    u8g2.drawHLine(0, 54, 128);
    u8g2.drawStr(2, 63, "PWR: SAVE | SW: BACK");
    u8g2.sendBuffer();
}

void requestRender() {
    needsRender = true;
}

void renderCurrentScreen() {
    switch (currentState) {
        case STATE_CONTROL_ROOM: renderControlRoom();    break;
        case STATE_BRAND_MENU:   renderBrandMenu();      break;
        case STATE_LEARNING:     renderLearningScreen(); break;
        case STATE_SAVE_SLOT:    renderSaveSlotScreen(); break;
    }
    needsRender = false;
    lastRenderTime = millis();
}

// =================================================================
// LIGHT SLEEP & POWER MANAGEMENT
// =================================================================
void enterLightSleep() {
    Serial.println("[PWR] Inactivity timeout reached. Entering light sleep...");
    Serial.flush();

    if (settingsDirty) saveSettings();

    u8g2.setPowerSave(1);
    digitalWrite(PIN_BUZZER, LOW);

    gpio_wakeup_enable((gpio_num_t)PIN_ENC_SW,  GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable((gpio_num_t)PIN_BTN_PWR, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    esp_light_sleep_start();

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    gpio_wakeup_disable((gpio_num_t)PIN_ENC_SW);
    gpio_wakeup_disable((gpio_num_t)PIN_BTN_PWR);
    u8g2.setPowerSave(0);

    encSwIgnore  = true;
    pwrBtnIgnore = true;

    oldEncoderState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
    positionDelta = 0;

    lastActivityTime = millis();
    Serial.println("[PWR] Woke from light sleep.");
}

// =================================================================
// SYSTEM SETUP
// =================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_ENC_A,   INPUT_PULLUP);
    pinMode(PIN_ENC_B,   INPUT_PULLUP);
    pinMode(PIN_ENC_SW,  INPUT_PULLUP);
    pinMode(PIN_BTN_PWR, INPUT_PULLUP);
    pinMode(PIN_BUZZER,  OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    oldEncoderState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    u8g2.setBusClock(400000);
    u8g2.begin();

    irsend.begin();
    irDaikin.begin();
    irLG.begin();
    irVoltas.begin();
    irMidea.begin();
    irHitachi.begin();
    irMitsubishi.begin();

    loadSettings();
    lastActivityTime = millis();
    renderCurrentScreen();
}

// =================================================================
// MAIN RUNTIME LOOP
// =================================================================
void loop() {
    buzzerUpdate();

    if (settingsDirty && (millis() - lastSettingsChange >= NVS_DEBOUNCE_DELAY_MS)) {
        saveSettings();
    }

    // -------------------------------------------------------------
    // 1. ROTARY ENCODER ROTATION
    // -------------------------------------------------------------
    uint8_t newEncoderState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);

    if (newEncoderState != oldEncoderState) {
        uint8_t index = (oldEncoderState << 2) | newEncoderState;
        positionDelta += ENCODER_STATES[index];
        oldEncoderState = newEncoderState;

        if (positionDelta >= 4) {
            lastActivityTime = millis();
            positionDelta = 0;

            if (currentState == STATE_CONTROL_ROOM) {
                if (currentTemp < TEMP_MAX) { currentTemp++; markSettingsChanged(); requestRender(); }
            } else if (currentState == STATE_BRAND_MENU) {
                if (menuCursor < TOTAL_BRANDS - 1) { menuCursor++; requestRender(); }
            } else if (currentState == STATE_SAVE_SLOT) {
                if (saveSlotCursor < NUM_CUSTOM_SLOTS - 1) { saveSlotCursor++; requestRender(); }
            }

        } else if (positionDelta <= -4) {
            lastActivityTime = millis();
            positionDelta = 0;

            if (currentState == STATE_CONTROL_ROOM) {
                if (currentTemp > TEMP_MIN) { currentTemp--; markSettingsChanged(); requestRender(); }
            } else if (currentState == STATE_BRAND_MENU) {
                if (menuCursor > 0) { menuCursor--; requestRender(); }
            } else if (currentState == STATE_SAVE_SLOT) {
                if (saveSlotCursor > 0) { saveSlotCursor--; requestRender(); }
            }
        }
    }

    // -------------------------------------------------------------
    // 2. ENCODER SWITCH (SHORT CLICK VS 2-SECOND LONG PRESS)
    // -------------------------------------------------------------
    int currentEncSw = digitalRead(PIN_ENC_SW);

    if (encSwIgnore) {
        lastActivityTime = millis();
        if (currentEncSw == HIGH) encSwIgnore = false;
    } else {
        if (currentEncSw == LOW && !encSwPressed) {
            delay(DEBOUNCE_DELAY_MS);
            if (digitalRead(PIN_ENC_SW) == LOW) {
                encSwPressed = true;
                encSwPressTime = millis();
                lastActivityTime = millis();
            }
        }

        if (encSwPressed) {
            unsigned long holdTime = millis() - encSwPressTime;

            if (holdTime >= LONG_PRESS_TIMEOUT_MS) {
                encSwPressed = false;
                encSwIgnore = true;
                lastActivityTime = millis();

                if (currentState == STATE_CONTROL_ROOM) {
                    buzzerTrigger(80);
                    currentState = STATE_BRAND_MENU;
                    menuCursor = selectedBrand;
                    renderCurrentScreen();
                }
            } else if (currentState == STATE_CONTROL_ROOM && holdTime > 200) {
                requestRender();
            }
        }

        if (currentEncSw == HIGH && encSwPressed) {
            unsigned long duration = millis() - encSwPressTime;
            encSwPressed = false;
            lastActivityTime = millis();

            if (duration < SHORT_PRESS_MAX_MS) {
                if (currentState == STATE_CONTROL_ROOM) {
                    sendIRCommand();
                } else if (currentState == STATE_BRAND_MENU) {
                    currentState = STATE_CONTROL_ROOM;
                } else if (currentState == STATE_LEARNING) {
                    // Abort capture
                    irrecv.disableIRIn();
                    buzzerTrigger(80);
                    currentState = STATE_BRAND_MENU;
                } else if (currentState == STATE_SAVE_SLOT) {
                    // Back — discard the staged capture
                    stagedRawLength = 0;
                    currentState = STATE_BRAND_MENU;
                }
            }
            requestRender();
        }
    }

    // -------------------------------------------------------------
    // 3. SIDE POWER BUTTON (BTN_PWR)
    // -------------------------------------------------------------
    int currentPwrBtn = digitalRead(PIN_BTN_PWR);

    if (pwrBtnIgnore) {
        lastActivityTime = millis();
        if (currentPwrBtn == HIGH) pwrBtnIgnore = false;
    } else {
        if (currentPwrBtn == LOW && !pwrBtnPressed) {
            delay(DEBOUNCE_DELAY_MS);
            if (digitalRead(PIN_BTN_PWR) == LOW) {
                pwrBtnPressed = true;
                pwrBtnPressTime = millis();
                lastActivityTime = millis();

                if (currentState == STATE_CONTROL_ROOM) {
                    isACOn = !isACOn;
                    saveSettings();
                    sendIRCommand();
                    requestRender();

                } else if (currentState == STATE_BRAND_MENU) {
                    buzzerTrigger(40);
                    if (menuCursor == INDEX_LEARN_REMOTE) {
                        currentState = STATE_LEARNING;
                        learnStartTime = millis();
                        irrecv.enableIRIn();
                    } else {
                        selectedBrand = menuCursor;
                        saveSettings();
                        currentState = STATE_CONTROL_ROOM;
                    }
                    renderCurrentScreen();

                } else if (currentState == STATE_SAVE_SLOT) {
                    // Confirm: write staged capture to the chosen slot
                    uint8_t slot = saveSlotCursor;
                    memcpy(customRawBuffer[slot], stagedRawBuffer,
                           stagedRawLength * sizeof(uint16_t));
                    customRawLength[slot] = stagedRawLength;
                    saveCustomSlot(slot, stagedRawBuffer, stagedRawLength);

                    // Switch the active brand to the slot that was just saved
                    selectedBrand = INDEX_CUSTOM_FIRST + slot;
                    saveSettings();

                    stagedRawLength = 0;
                    buzzerTrigger(60);
                    currentState = STATE_CONTROL_ROOM;
                    lastActivityTime = millis();
                    renderCurrentScreen();
                }
            }
        }

        if (currentPwrBtn == HIGH && pwrBtnPressed) {
            pwrBtnPressed = false;
        }
    }

    // -------------------------------------------------------------
    // 4. LEARNING MODE WATCHDOG & PACKET CAPTURE
    // -------------------------------------------------------------
    if (currentState == STATE_LEARNING) {
        requestRender();

        if (irrecv.decode(&irResults)) {
            uint16_t len = getCorrectedRawLength(&irResults);
            if (len < 30) {
                irrecv.resume(); // Discard noise, keep listening
            } else {
                // Capture into staging buffer
                stagedRawLength = (len > RAW_BUFFER_CAPACITY) ? RAW_BUFFER_CAPACITY : len;
                for (uint16_t i = 0; i < stagedRawLength; i++) {
                    stagedRawBuffer[i] = irResults.rawbuf[i + 1] * kRawTick;
                }
                irrecv.disableIRIn();

                // Play the double-beep confirmation, then move to slot-select
                buzzerTrigger(40, true);
                saveSlotCursor = 0;
                currentState = STATE_SAVE_SLOT;
                lastActivityTime = millis();
                requestRender();
            }
        } else if (millis() - learnStartTime >= LEARN_TIMEOUT_MS) {
            irrecv.disableIRIn();
            buzzerTrigger(150);
            currentState = STATE_BRAND_MENU;
            lastActivityTime = millis();
            requestRender();
        }
    }

    // -------------------------------------------------------------
    // 5. INACTIVITY WATCHDOG & LIGHT SLEEP ENTRY
    // -------------------------------------------------------------
    if (!encSwPressed && !pwrBtnPressed &&
        (millis() - lastActivityTime >= INACTIVITY_TIMEOUT_MS)) {

        // Collapse any open menu back to the control room before sleeping
        if (currentState == STATE_BRAND_MENU || currentState == STATE_SAVE_SLOT) {
            stagedRawLength = 0; // Discard any un-saved staged capture
            currentState = STATE_CONTROL_ROOM;
            renderCurrentScreen();
        }

        if (currentState == STATE_CONTROL_ROOM) {
            enterLightSleep();
            requestRender();
        }
    }

    // -------------------------------------------------------------
    // 6. DEFERRED RENDER PASS
    // -------------------------------------------------------------
    if (needsRender && (millis() - lastRenderTime >= RENDER_INTERVAL_MS)) {
        renderCurrentScreen();
    }
}