#include <Control_Surface.h>
#include <Wire.h>
#include "LedController.hpp"
#include <Adafruit_NeoPixel.h>

// ============================================================================
// Configuration Constants
// ============================================================================

// I2C Configuration
constexpr uint8_t I2C_SDA_PIN = 35;
constexpr uint8_t I2C_SCL_PIN = 36;
constexpr uint32_t I2C_CLOCK_SPEED = 400000;

// I2C Slave Addresses
constexpr uint8_t MOTOR_FADER_SLAVE_ADDR = 8;
constexpr uint8_t ENCODER_SLAVE_ADDR = 0x09;

// Protocol Constants
constexpr uint8_t ENCODER_PACKET_START = 0xAB;
constexpr uint8_t ENCODER_PACKET_END = 0xCD;
constexpr uint8_t ENCODER_COUNT = 4;
constexpr uint8_t FADER_COUNT = 4;
constexpr uint8_t BUTTON_COUNT = 8;

// Timing Constants (milliseconds)
constexpr unsigned long LED_BLINK_INTERVAL = 300;
constexpr unsigned long ENCODER_SYNC_TIMEOUT = 200;
constexpr unsigned long DOUBLE_CLICK_INTERVAL = 400;
constexpr unsigned long LED_OVERRIDE_DURATION = 1000;
constexpr unsigned long BUTTON_DEBOUNCE_MS = 100;

// LED Brightness Levels
constexpr uint8_t BRIGHTNESS_LEVELS[] = {30, 60, 128};
constexpr uint8_t BRIGHTNESS_LEVEL_COUNT = 3;

// Hardware Pins
constexpr uint8_t NEOPIXEL_PIN = 4;
constexpr uint8_t NEOPIXEL_COUNT = 4;
constexpr uint8_t PAGE_ENCODER_PIN_A = 13;
constexpr uint8_t PAGE_ENCODER_PIN_B = 14;
constexpr uint8_t INPUT_BUTTON_PIN = 21;

// MAX7219 Pins
constexpr uint8_t MAX7219_DIN = 15;
constexpr uint8_t MAX7219_CS = 16;
constexpr uint8_t MAX7219_CLK = 17;

// MIDI CC Address Offsets
constexpr uint8_t CC_ENCODER_BASE = 0x30;      // CC 48-63
constexpr uint8_t CC_FADER_BASE = 0x40;        // CC 64-79
constexpr uint8_t CC_TOUCH_BASE = 0x68;        // CC 104-107
constexpr uint8_t CC_BUTTON_BASE = 0x00;       // Dynamic per page

// Page Configuration
constexpr uint8_t PAGE_COUNT = 4;
constexpr uint8_t PAGE_SIZE = 4;
constexpr uint8_t PAGE_CC_OFFSETS[PAGE_COUNT] = {0, 4, 8, 12};

// ============================================================================
// Hardware Instances
// ============================================================================

USBMIDI_Interface midi;

// Page selection encoder
CCAbsoluteEncoder pageEncoder {
    {PAGE_ENCODER_PIN_A, PAGE_ENCODER_PIN_B},
    MCU::V_POT_1,
    8
};

// NeoPixel strip for page/button indication
Adafruit_NeoPixel pixelStrip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// MAX7219 LED matrix controller
auto ledMatrix = LedController<1, 1>();

// Button multiplexer
CD74HC4067 buttonMux(3, AH::Array<pin_t, 4>{18, 8, 38, 46});

// Input button for brightness adjustment
Button configButton(INPUT_BUTTON_PIN);

// ============================================================================
// Page Color Configuration
// ============================================================================

struct PageColor {
    uint8_t r, g, b;
    uint32_t toNeoPixelColor() const {
        return Adafruit_NeoPixel::Color(r, g, b);
    }
};

constexpr PageColor PAGE_COLORS[PAGE_COUNT] = {
    {0xFF, 0x00, 0x00},  // Red - Page 0
    {0x00, 0xFF, 0x00},  // Green - Page 1
    {0x00, 0x00, 0xFF},  // Blue - Page 2
    {0xFF, 0xFF, 0x00},  // Yellow - Page 3
};

constexpr PageColor PAGE_INDICATOR_COLOR = {0xFF, 0xC0, 0xCB};  // Pink

// ============================================================================
// State Structures
// ============================================================================

struct EncoderState {
    uint8_t currentValue;
    uint8_t lastValue;
    uint8_t syncedValue;
    unsigned long lastChangeTime;
    bool isUserControlling;  // true = user turning, false = following MIDI
};

struct FaderState {
    uint16_t position;
    uint16_t targetPosition;
    bool isTouched;
    uint8_t lastMidiValue;
};

struct ButtonState {
    bool currentState;
    bool storedState;
    uint8_t bufferValue;  // 0=off, 1=on, 2=cleared
};

struct PageState {
    uint8_t currentIndex;
    uint8_t previousIndex;
    bool isBlinking;
    unsigned long blinkStartTime;
    bool blinkState;
};

struct BrightnessState {
    uint8_t levelIndex;
    bool overrideActive;
    unsigned long overrideStartTime;
    bool firstClickDetected;
    unsigned long lastClickTime;
    uint8_t toggleCounter;
};

// ============================================================================
// Global State
// ============================================================================

EncoderState encoders[ENCODER_COUNT] = {};
FaderState faders[FADER_COUNT] = {};
ButtonState buttons[BUTTON_COUNT] = {};
PageState page = {0, 0, false, 0, false};
BrightnessState brightness = {0, false, 0, false, 0, 0};

// MIDI CC value receivers for encoder ring LEDs
CCValue encoderCcValues[16] = {
    {0x30}, {0x31}, {0x32}, {0x33},
    {0x34}, {0x35}, {0x36}, {0x37},
    {0x38}, {0x39}, {0x3A}, {0x3B},
    {0x3C}, {0x3D}, {0x3E}, {0x3F}
};

// MIDI CC value receivers for fader motor control
CCValue faderCcValues[16] = {
    {0x40}, {0x41}, {0x42}, {0x43},
    {0x44}, {0x45}, {0x46}, {0x47},
    {0x48}, {0x49}, {0x4A}, {0x4B},
    {0x4C}, {0x4D}, {0x4E}, {0x4F}
};

// CC Buttons
CCButton ccButtons[BUTTON_COUNT] = {
    {buttonMux.pin(0), {0, Channel_1}},
    {buttonMux.pin(1), {1, Channel_1}},
    {buttonMux.pin(2), {2, Channel_1}},
    {buttonMux.pin(3), {3, Channel_1}},
    {buttonMux.pin(4), {4, Channel_1}},
    {buttonMux.pin(5), {5, Channel_1}},
    {buttonMux.pin(6), {6, Channel_1}},
    {buttonMux.pin(7), {7, Channel_1}},
};

// MIDI input note addresses for RGB button feedback
constexpr uint8_t BUTTON_FEEDBACK_CC[16] = {
    0, 1, 2, 3, 8, 9, 10, 11,
    16, 17, 18, 19, 24, 25, 26, 27
};

// LED matrix state
byte ledMatrixRows[8] = {0};

// ============================================================================
// MIDI Callbacks
// ============================================================================

struct MidiCallbacks : FineGrainedMIDI_Callbacks<MidiCallbacks> {
    void onControlChange(Channel channel, uint8_t controller, uint8_t value, Cable cable) {
        (void)channel;
        (void)cable;

        for (uint8_t i = 0; i < 16; i++) {
            if (controller != BUTTON_FEEDBACK_CC[i]) continue;

            bool newState = (value == 127);
            uint8_t pageOffset = i / PAGE_SIZE;
            uint8_t buttonInPage = i % PAGE_SIZE;

            if (newState) {
                buttons[i].storedState = true;
                if (pageOffset == page.currentIndex / PAGE_SIZE) {
                    ccButtons[buttonInPage].flip_noteon();
                } else {
                    buttons[i].bufferValue = 1;
                }
            } else {
                buttons[i].storedState = false;
                if (pageOffset == page.currentIndex / PAGE_SIZE) {
                    ccButtons[buttonInPage].flip_noteoff();
                } else {
                    buttons[i].bufferValue = 0;
                }
            }
        }
    }
} midiCallbacks;

// ============================================================================
// Initialization
// ============================================================================

void setup() {
    delay(1000);

    Serial.begin(115200);

    // Initialize Control Surface
    Control_Surface.begin();
    AH::Button::setDebounceTime(BUTTON_DEBOUNCE_MS);
    midi.setCallbacks(midiCallbacks);

    // Initialize LED Matrix (MAX7219)
    setupLedMatrix();

    // Initialize NeoPixel strip
    setupNeoPixel();

    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_SPEED);

    // Initialize config button
    pinMode(INPUT_BUTTON_PIN, INPUT_PULLUP);
    configButton.begin();
}

void setupLedMatrix() {
    controller_configuration<1, 1> config;
    config.useHardwareSpi = false;
    config.SPI_MOSI = MAX7219_DIN;
    config.SPI_CLK = MAX7219_CLK;
    config.SPI_CS = MAX7219_CS;
    config.spiTransferSpeed = 800000;
    ledMatrix.init(config);
    ledMatrix.clearMatrix();
}

void setupNeoPixel() {
    pixelStrip.begin();
    pixelStrip.show();
    pixelStrip.setBrightness(BRIGHTNESS_LEVELS[brightness.levelIndex]);
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    // Update MIDI input
    Control_Surface.updateMidiInput();

    // Check for brightness override timeout
    updateBrightnessOverride();

    // Read encoder values from I2C slave
    readEncodersFromI2C();

    // Update RGB button LEDs
    updateButtonLeds();

    // Check page encoder and update page index
    updatePageSelection();

    // Process each encoder
    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        updateEncoder(i);
    }

    // Read fader values from I2C slave
    readFadersFromI2C();

    // Send target positions to fader motors
    for (uint8_t i = 0; i < FADER_COUNT; i++) {
        sendFaderTarget(i);
    }

    // Update button MIDI addresses for current page
    updateButtonAddresses();

    // Update page indicator LED
    updatePageIndicator();

    // Handle brightness button
    handleBrightnessButton();
}

// ============================================================================
// Encoder Functions
// ============================================================================

void readEncodersFromI2C() {
    constexpr uint8_t PACKET_SIZE = ENCODER_COUNT + 2;

    Wire.requestFrom(ENCODER_SLAVE_ADDR, PACKET_SIZE);

    uint8_t buffer[PACKET_SIZE];
    uint8_t bytesRead = 0;

    while (Wire.available() && bytesRead < PACKET_SIZE) {
        buffer[bytesRead++] = Wire.read();
    }

    if (bytesRead != PACKET_SIZE) return;
    if (buffer[0] != ENCODER_PACKET_START) return;
    if (buffer[PACKET_SIZE - 1] != ENCODER_PACKET_END) return;

    // Validate and extract encoder values
    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        uint8_t value = buffer[i + 1];
        if (value > 127) return;  // Invalid data

        if (encoders[i].currentValue != value) {
            encoders[i].currentValue = value;
            encoders[i].lastChangeTime = millis();
            encoders[i].isUserControlling = true;
        }
    }
}

void updateEncoder(uint8_t index) {
    // Update LED ring regardless of sync state
    updateEncoderLed(index);

    // Check if encoder should sync to MIDI input
    checkEncoderSync(index);

    // Send MIDI if user is controlling
    if (encoders[index].isUserControlling &&
        encoders[index].currentValue != encoders[index].lastValue) {

        encoders[index].lastValue = encoders[index].currentValue;

        uint8_t ccNumber = CC_ENCODER_BASE + index + page.currentIndex;
        midi.sendControlChange({ccNumber, Channel_1}, encoders[index].currentValue);
    }
}

void checkEncoderSync(uint8_t index) {
    uint8_t globalIndex = index + page.currentIndex;
    uint8_t targetValue = encoderCcValues[globalIndex].getValue();

    // If no change for timeout period, consider syncing to MIDI
    if ((millis() - encoders[index].lastChangeTime) > ENCODER_SYNC_TIMEOUT) {
        if (targetValue != encoders[index].syncedValue) {
            encoders[index].isUserControlling = false;
            encoders[index].syncedValue = targetValue;

            // Send sync value to encoder slave
            uint16_t data = (targetValue & 0x7F) | (index << 12);
            Wire.beginTransmission(ENCODER_SLAVE_ADDR);
            Wire.write(reinterpret_cast<uint8_t*>(&data), 2);
            Wire.endTransmission();
        }
    }
}

void updateEncoderLed(uint8_t index) {
    static const uint8_t LED_MASK[8] = {
        0b10000000, 0b01000000, 0b00100000, 0b00010000,
        0b00001000, 0b00000100, 0b00000010, 0b00000001
    };

    uint8_t globalIndex = index + page.currentIndex;
    uint8_t displayValue;

    if (encoders[index].isUserControlling) {
        displayValue = encoders[index].currentValue >> 3;
    } else {
        displayValue = encoderCcValues[globalIndex].getValue() >> 3;
    }

    uint8_t rowIndex = index * 2 + (displayValue >= 8 ? 1 : 0);
    uint8_t rowValue = LED_MASK[displayValue % 8];

    ledMatrixRows[rowIndex] = rowValue;
    ledMatrix.setRow(0, rowIndex, rowValue);

    ledMatrixRows[rowIndex ^ 1] = 0;
    ledMatrix.setRow(0, rowIndex ^ 1, 0);
}

// ============================================================================
// Fader Functions
// ============================================================================

void readFadersFromI2C() {
    constexpr uint8_t PACKET_SIZE = 1 + FADER_COUNT * 2;

    Wire.requestFrom(MOTOR_FADER_SLAVE_ADDR, PACKET_SIZE);

    uint8_t buffer[PACKET_SIZE];
    uint8_t bytesRead = 0;

    while (Wire.available() && bytesRead < PACKET_SIZE) {
        buffer[bytesRead++] = Wire.read();
    }

    if (bytesRead != PACKET_SIZE) return;

    uint8_t touchFlags = buffer[0];

    for (uint8_t i = 0; i < FADER_COUNT; i++) {
        bool touched = touchFlags & (1 << i);

        if (touched != faders[i].isTouched) {
            faders[i].isTouched = touched;
            sendTouchMidi(i, touched);
        }

        if (faders[i].isTouched && bytesRead >= 1 + (i + 1) * 2) {
            uint16_t position = buffer[1 + i * 2] | (buffer[2 + i * 2] << 8);
            faders[i].position = position;

            uint8_t midiValue = map(position, 0, 1023, 0, 127);
            if (midiValue != faders[i].lastMidiValue) {
                midi.sendControlChange(CC_FADER_BASE + i + page.currentIndex, midiValue);
                faders[i].lastMidiValue = midiValue;
            }
        }
    }
}

void sendTouchMidi(uint8_t index, bool touched) {
    DigitalCCSender sender(0x7F, 0x00);
    uint8_t ccNumber = CC_TOUCH_BASE + index + page.currentIndex;
    MIDIAddress addr(ccNumber, Channel_1);

    if (touched) {
        sender.sendOn(addr);
    } else {
        sender.sendOff(addr);
    }
}

void sendFaderTarget(uint8_t index) {
    uint8_t globalIndex = index + page.currentIndex;

    uint16_t dawTarget = map(faderCcValues[globalIndex].getValue(), 0, 127, 0, 1023);
    uint16_t currentPos = faders[index].position >> 4;

    // Prioritize DAW target, fallback to current position
    uint16_t target = faders[index].targetPosition;

    if (dawTarget != faders[index].targetPosition) {
        target = dawTarget;
        faders[index].targetPosition = dawTarget;
    } else if (currentPos != faders[index].targetPosition) {
        target = currentPos;
        faders[index].targetPosition = currentPos;
    } else {
        return;  // No change
    }

    // Send to motor slave: 4-bit index + 12-bit position
    uint16_t data = target | (index << 12);

    Wire.beginTransmission(MOTOR_FADER_SLAVE_ADDR);
    Wire.write(reinterpret_cast<uint8_t*>(&data), 2);
    Wire.endTransmission();
}

// ============================================================================
// Page Selection
// ============================================================================

void updatePageSelection() {
    pageEncoder.update();
    int value = pageEncoder.getValue();

    uint8_t newPage = page.currentIndex;

    if (value < 32) {
        newPage = 0;
    } else if (value < 64) {
        newPage = 4;
    } else if (value < 96) {
        newPage = 8;
    } else if (value <= 127) {
        newPage = 12;
    }

    if (newPage != page.currentIndex) {
        page.previousIndex = page.currentIndex;
        page.currentIndex = newPage;
        onPageChanged();
    }
}

void onPageChanged() {
    page.isBlinking = true;
    page.blinkStartTime = millis();
    page.blinkState = true;

    // Turn off all button LEDs
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
        pixelStrip.setPixelColor(i, 0);
    }
    pixelStrip.show();

    // Show page indicator
    uint32_t color = pixelStrip.Color(
        PAGE_INDICATOR_COLOR.r,
        PAGE_INDICATOR_COLOR.g,
        PAGE_INDICATOR_COLOR.b
    );
    pixelStrip.setPixelColor(page.currentIndex / PAGE_SIZE, color);
    pixelStrip.show();
}

void updatePageIndicator() {
    if (!page.isBlinking) return;

    unsigned long elapsed = millis() - page.blinkStartTime;

    if (elapsed >= LED_BLINK_INTERVAL) {
        // End blink, restore button states
        page.isBlinking = false;
        page.blinkState = false;

        for (int i = 0; i < NEOPIXEL_COUNT; i++) {
            pixelStrip.setPixelColor(i, 0);
        }
        pixelStrip.show();

        restoreButtonStates();
    }
}

// ============================================================================
// Button Functions
// ============================================================================

void updateButtonAddresses() {
    for (int i = 0; i < BUTTON_COUNT; i++) {
        uint8_t note = i + page.currentIndex * 2;
        ccButtons[i].setAddressUnsafe({note, Channel_1});
        ccButtons[i].update();
    }
}

void updateButtonLeds() {
    if (page.isBlinking || brightness.overrideActive) return;

    for (uint8_t i = 0; i < PAGE_SIZE; i++) {
        // Apply buffered state if present
        applyButtonBuffer(i);

        // Update stored state from button
        uint8_t globalIndex = i + page.currentIndex;
        buttons[globalIndex].currentState = ccButtons[i].getLastMessageIsNoteOn();

        // Set LED color
        if (buttons[globalIndex].currentState) {
            PageColor color = PAGE_COLORS[page.currentIndex / PAGE_SIZE];
            pixelStrip.setPixelColor(i, color.toNeoPixelColor());
        } else {
            pixelStrip.setPixelColor(i, 0);
        }
    }

    pixelStrip.show();
}

void applyButtonBuffer(uint8_t index) {
    uint8_t globalIndex = index + page.currentIndex;
    uint8_t buffer = buttons[globalIndex].bufferValue;

    if (buffer == 1) {
        ccButtons[index].flip_noteon();
        buttons[globalIndex].bufferValue = 2;
    } else if (buffer == 0) {
        ccButtons[index].flip_noteoff();
        buttons[globalIndex].bufferValue = 2;
    }
}

void restoreButtonStates() {
    for (uint8_t i = 0; i < PAGE_SIZE; i++) {
        uint8_t globalIndex = i + page.currentIndex;
        if (buttons[globalIndex].storedState) {
            ccButtons[i].flip_noteon();
        } else {
            ccButtons[i].flip_noteoff();
        }
    }
}

// ============================================================================
// Brightness Control
// ============================================================================

void handleBrightnessButton() {
    Button::State state = configButton.update();

    if (state != Button::Falling) return;

    unsigned long now = millis();

    // Check for double-click
    if (brightness.firstClickDetected &&
        (now - brightness.lastClickTime) < DOUBLE_CLICK_INTERVAL) {

        // Double-click: cycle brightness
        brightness.levelIndex = (brightness.levelIndex + 1) % BRIGHTNESS_LEVEL_COUNT;
        pixelStrip.setBrightness(BRIGHTNESS_LEVELS[brightness.levelIndex]);
        pixelStrip.show();

        brightness.firstClickDetected = false;
    } else {
        // Single click: toggle LED override
        brightness.firstClickDetected = true;
        brightness.lastClickTime = now;
        brightness.overrideActive = true;
        brightness.overrideStartTime = now;
        brightness.toggleCounter++;

        uint8_t pageLed = page.currentIndex / PAGE_SIZE;

        if (brightness.toggleCounter % 2 == 1) {
            uint32_t color = pixelStrip.Color(
                PAGE_INDICATOR_COLOR.r,
                PAGE_INDICATOR_COLOR.g,
                PAGE_INDICATOR_COLOR.b
            );
            pixelStrip.setPixelColor(pageLed, color);
        } else {
            pixelStrip.setPixelColor(pageLed, 0);
        }
        pixelStrip.show();
    }
}

void updateBrightnessOverride() {
    if (!brightness.overrideActive) return;

    if (millis() - brightness.overrideStartTime >= LED_OVERRIDE_DURATION) {
        brightness.overrideActive = false;
        brightness.toggleCounter++;
    }
}
