#include <Control_Surface.h>
#include <Wire.h>
#include "LedController.hpp"
#include <Adafruit_NeoPixel.h>

// ============================================================================
// Configuration Constants
// ============================================================================

// Hardware Pins
constexpr uint8_t I2C_SDA_PIN = 35;
constexpr uint8_t I2C_SCL_PIN = 36;
constexpr uint8_t PAGE_ENCODER_PIN_A = 13;
constexpr uint8_t PAGE_ENCODER_PIN_B = 14;
constexpr uint8_t CONFIG_BUTTON_PIN = 21;
constexpr uint8_t BUTTON_MUX_PIN_Z = 3;
constexpr uint8_t BUTTON_MUX_PINS_S[4] = {18, 8, 38, 39};

// MAX7219 Pins
constexpr uint8_t MAX7219_DIN = 15;
constexpr uint8_t MAX7219_CS = 16;
constexpr uint8_t MAX7219_CLK = 17;
constexpr uint8_t MAX7219_DEVICE_COUNT = 4;  // 4 devices for 16 encoders

// I2C Configuration
constexpr uint8_t ENCODER_SLAVE_ADDRESS = 0x09;
constexpr uint32_t I2C_CLOCK_SPEED = 400000;

// Protocol Constants
constexpr uint8_t PACKET_START_BYTE = 0xAB;
constexpr uint8_t PACKET_END_BYTE = 0xCD;

// Timing Constants
constexpr unsigned long ENCODER_SYNC_TIMEOUT_MS = 200;
constexpr unsigned long DOUBLE_CLICK_INTERVAL_MS = 420;
constexpr unsigned long MODE_CHANGE_INTERVAL_MS = 1000;
constexpr unsigned long BLINK_INTERVAL_MS = 500;
constexpr uint8_t BUTTON_DEBOUNCE_MS = 100;

// Encoder Configuration
constexpr uint8_t ENCODER_COUNT = 16;
constexpr uint8_t ENCODER_VALUES_PER_DEVICE = 4;
constexpr uint8_t LED_ROWS_PER_ENCODER = 2;

// Page Configuration
constexpr uint8_t PAGE_COUNT = 8;
constexpr uint8_t PAGE_SIZE = 16;
constexpr uint8_t PAGE_ENCODER_DIVISOR = 16;

// MIDI Configuration
constexpr uint8_t MIDI_CC_BASE = 0x00;
constexpr MidiChannel ENCODER_MIDI_CHANNEL = Channel_2;
constexpr MidiChannel BUTTON_MIDI_CHANNEL = Channel_1;

// LED Configuration
constexpr uint8_t LED_MASK[8] = {
    0b10000000, 0b01000000, 0b00100000, 0b00010000,
    0b00001000, 0b00000100, 0b00000010, 0b00000001
};

constexpr uint8_t BLINK_SEQUENCE[4] = {3, 7, 11, 15};

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

// MAX7219 LED matrix controllers (4 devices for 16 encoders)
auto ledMatrix = LedController<MAX7219_DEVICE_COUNT, 1>();

// Button multiplexer
CD74HC4067 buttonMux(BUTTON_MUX_PIN_Z, AH::Array<pin_t, 4>{
    BUTTON_MUX_PINS_S[0], BUTTON_MUX_PINS_S[1],
    BUTTON_MUX_PINS_S[2], BUTTON_MUX_PINS_S[3]
});

// Configuration button
Button configButton(CONFIG_BUTTON_PIN);

// ============================================================================
// Operating Modes
// ============================================================================

enum class OperatingMode : uint8_t {
    NORMAL = 0,           // Normal operation with encoder rings
    RELATIVE_ASSIGN = 1,  // Select encoders for relative mode
    // Mode 2 reserved
    PAGE_SELECT = 3       // Page selection mode
};

// ============================================================================
// State Structures
// ============================================================================

struct EncoderState {
    uint8_t currentValue;
    uint8_t lastValue;
    uint8_t syncedValue;
    unsigned long lastChangeTime;
    bool isUserControlling;
};

struct PageState {
    uint8_t currentOffset;
    uint8_t encoderValue;
};

struct ModeState {
    OperatingMode current;
    bool firstLongClick;
    bool firstShortClick;
    bool awaitingSecondClick;
    unsigned long lastClickTime;
};

struct BlinkState {
    uint8_t sequenceIndex;
    unsigned long lastUpdateTime;
    bool toggleState;
};

// ============================================================================
// Global State
// ============================================================================

EncoderState encoders[ENCODER_COUNT] = {};
PageState page = {0, 0};
ModeState mode = {OperatingMode::NORMAL, false, false, false, 0};
BlinkState blink = {0, 0, false};

// LED row buffers (8 rows per MAX7219 device)
byte ledRows[MAX7219_DEVICE_COUNT][8] = {};
uint8_t lastCcValues[ENCODER_COUNT] = {0};

// MIDI CC value receivers for encoder ring sync
CCValue ccReceivers[128];

// Note buttons (16 buttons for encoder selection)
NoteButton noteButtons[ENCODER_COUNT] = {
    {buttonMux.pin(0), {0, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(1), {1, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(2), {2, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(3), {3, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(4), {4, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(5), {5, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(6), {6, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(7), {7, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(8), {8, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(9), {9, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(10), {10, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(11), {11, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(12), {12, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(13), {13, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(14), {14, BUTTON_MIDI_CHANNEL}},
    {buttonMux.pin(15), {15, BUTTON_MIDI_CHANNEL}}
};

// Temporary buffer for encoder values from I2C
uint8_t encoderValues[ENCODER_COUNT] = {0};

// ============================================================================
// Initialization
// ============================================================================

void setup() {
    delay(500);  // Allow I2C peripherals to initialize

    Serial.begin(115200);

    // Initialize Control Surface
    Control_Surface.begin();
    AH::Button::setDebounceTime(BUTTON_DEBOUNCE_MS);

    // Initialize LED matrix controllers
    initializeLedMatrix();

    // Initialize configuration button
    pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
    configButton.begin();

    // Initialize encoder change tracking
    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        encoders[i].isUserControlling = true;
        noteButtons[i].setstatus(0);
    }

    // Initialize I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_SPEED);

    // Initialize CC receivers
    for (uint8_t i = 0; i < 128; i++) {
        ccReceivers[i] = CCValue(i);
    }
}

void initializeLedMatrix() {
    controller_configuration<MAX7219_DEVICE_COUNT, 1> config;
    config.useHardwareSpi = false;
    config.SPI_MOSI = MAX7219_DIN;
    config.SPI_CLK = MAX7219_CLK;
    config.SPI_CS = MAX7219_CS;
    config.spiTransferSpeed = 800000;

    ledMatrix.init(config);
    ledMatrix.clearMatrix();
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    // Update MIDI input
    Control_Surface.updateMidiInput();

    // Read encoder values from I2C slave
    readEncodersFromI2C();

    // Process mode selection and page control
    handleModeSelection();

    // Update all encoders
    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        processEncoder(i);
    }

    // Handle mode-specific behavior
    switch (mode.current) {
        case OperatingMode::NORMAL:
            handleNormalMode();
            break;
        case OperatingMode::RELATIVE_ASSIGN:
            handleRelativeAssignMode();
            break;
        case OperatingMode::PAGE_SELECT:
            handlePageSelectMode();
            break;
        default:
            break;
    }

    // Update button MIDI addresses for current page
    updateButtonAddresses();
}

// ============================================================================
// I2C Communication
// ============================================================================

void readEncodersFromI2C() {
    constexpr uint8_t PACKET_SIZE = ENCODER_COUNT + 2;

    Wire.requestFrom(ENCODER_SLAVE_ADDRESS, static_cast<uint8_t>(PACKET_SIZE));

    uint8_t buffer[PACKET_SIZE];
    uint8_t bytesRead = 0;

    while (Wire.available() && bytesRead < PACKET_SIZE) {
        buffer[bytesRead++] = Wire.read();
    }

    if (bytesRead != PACKET_SIZE) return;
    if (buffer[0] != PACKET_START_BYTE) return;
    if (buffer[PACKET_SIZE - 1] != PACKET_END_BYTE) return;

    // Extract encoder values
    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        uint8_t value = buffer[i + 1];
        if (value > 127) continue;  // Invalid value

        encoderValues[i] = value;

        if (encoders[i].currentValue != value) {
            encoders[i].currentValue = value;
            encoders[i].lastChangeTime = millis();
            encoders[i].isUserControlling = true;
        }
    }
}

void sendEncoderSync(uint8_t index, uint8_t value) {
    uint16_t data = (value & 0x7F) | (index << 12);

    Wire.beginTransmission(ENCODER_SLAVE_ADDRESS);
    Wire.write(reinterpret_cast<uint8_t*>(&data), 2);
    Wire.endTransmission();
}

// ============================================================================
// Encoder Processing
// ============================================================================

void processEncoder(uint8_t index) {
    // Check for MIDI input sync
    checkEncoderSync(index);

    // Send MIDI if user is controlling
    if (encoders[index].isUserControlling &&
        encoders[index].currentValue != encoders[index].lastValue) {

        encoders[index].lastValue = encoders[index].currentValue;

        uint8_t ccNumber = (MIDI_CC_BASE + index + page.currentOffset) % 128;
        midi.sendControlChange(
            {ccNumber, ENCODER_MIDI_CHANNEL},
            encoders[index].currentValue
        );

        ccReceivers[index + page.currentOffset].setValue(encoders[index].currentValue);
    }
}

void checkEncoderSync(uint8_t index) {
    uint8_t globalIndex = index + page.currentOffset;
    uint8_t targetValue = ccReceivers[globalIndex].getValue();

    unsigned long elapsed = millis() - encoders[index].lastChangeTime;

    if (elapsed > ENCODER_SYNC_TIMEOUT_MS && targetValue != encoders[index].syncedValue) {
        encoders[index].isUserControlling = false;
        encoders[index].syncedValue = targetValue;
        sendEncoderSync(index, targetValue);
    }
}

// ============================================================================
// LED Ring Display
// ============================================================================

void updateEncoderLed(uint8_t index) {
    uint8_t deviceNum = index / ENCODER_VALUES_PER_DEVICE;
    uint8_t localIndex = index % ENCODER_VALUES_PER_DEVICE;

    uint8_t globalIndex = index + page.currentOffset;
    uint8_t displayValue;

    if (encoders[index].isUserControlling) {
        displayValue = encoderValues[index] >> 3;
    } else {
        displayValue = ccReceivers[globalIndex].getValue() >> 3;
    }

    uint8_t rowIndex = localIndex * LED_ROWS_PER_ENCODER;
    uint8_t rowValue = LED_MASK[displayValue % 8];

    // Clear both rows
    ledRows[deviceNum][rowIndex] = 0;
    ledRows[deviceNum][rowIndex + 1] = 0;

    // Set appropriate row
    if (displayValue >= 8) {
        ledRows[deviceNum][rowIndex + 1] = rowValue;
    } else {
        ledRows[deviceNum][rowIndex] = rowValue;
    }

    // Update matrix
    ledMatrix.setRow(deviceNum, rowIndex, ledRows[deviceNum][rowIndex]);
    ledMatrix.setRow(deviceNum, rowIndex + 1, ledRows[deviceNum][rowIndex + 1]);
}

void clearEncoderLed(uint8_t index) {
    uint8_t deviceNum = index / ENCODER_VALUES_PER_DEVICE;
    uint8_t localIndex = index % ENCODER_VALUES_PER_DEVICE;
    uint8_t rowIndex = localIndex * LED_ROWS_PER_ENCODER;

    ledRows[deviceNum][rowIndex] = 0;
    ledRows[deviceNum][rowIndex + 1] = 0;

    ledMatrix.setRow(deviceNum, rowIndex, 0);
    ledMatrix.setRow(deviceNum, rowIndex + 1, 0);
}

void blinkEncoderLed(uint8_t index) {
    uint8_t deviceNum = index / ENCODER_VALUES_PER_DEVICE;
    uint8_t localIndex = index % ENCODER_VALUES_PER_DEVICE;

    // Update blink sequence
    unsigned long currentTime = millis();
    if (currentTime - blink.lastUpdateTime >= BLINK_INTERVAL_MS) {
        blink.lastUpdateTime = currentTime;
        blink.sequenceIndex = (blink.sequenceIndex + 1) % 4;
    }

    uint8_t displayValue = BLINK_SEQUENCE[blink.sequenceIndex];
    uint8_t rowIndex = localIndex * LED_ROWS_PER_ENCODER;
    uint8_t rowValue = LED_MASK[displayValue % 8];

    // Clear rows
    ledRows[deviceNum][rowIndex] = 0;
    ledRows[deviceNum][rowIndex + 1] = 0;

    // Set row
    if (displayValue >= 8) {
        ledRows[deviceNum][rowIndex + 1] = rowValue;
    } else {
        ledRows[deviceNum][rowIndex] = rowValue;
    }

    ledMatrix.setRow(deviceNum, rowIndex, ledRows[deviceNum][rowIndex]);
    ledMatrix.setRow(deviceNum, rowIndex + 1, ledRows[deviceNum][rowIndex + 1]);
}

// ============================================================================
// Mode Handlers
// ============================================================================

void handleNormalMode() {
    // Update buttons
    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        noteButtons[i].update();
    }

    // Update all encoder LEDs
    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        updateEncoderLed(i);
    }
}

void handleRelativeAssignMode() {
    // Update button status for encoder binding
    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        noteButtons[i].update_status();

        if (noteButtons[i].getstatus()) {
            updateEncoderLed(i);
        } else {
            blinkEncoderLed(i);
        }
    }

    // Handle relative encoder input
    pageEncoder.relative_update();
    int32_t relativeValue = pageEncoder.get_relative_value();

    if (relativeValue == 0) return;

    int8_t delta = (relativeValue == 2) ? 1 : -1;

    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        if (!noteButtons[i].getstatus()) continue;

        uint8_t globalIndex = i + page.currentOffset;
        uint8_t currentValue = ccReceivers[globalIndex].getValue();
        uint8_t newValue = constrain(currentValue + delta, 0, 127);

        if (newValue != currentValue) {
            uint8_t ccNumber = (MIDI_CC_BASE + globalIndex) % 128;
            midi.sendControlChange({ccNumber, BUTTON_MIDI_CHANNEL}, newValue);
            ccReceivers[globalIndex].setValue(newValue);
        }
    }
}

void handlePageSelectMode() {
    // Update page encoder
    pageEncoder.update();
    page.encoderValue = pageEncoder.getValue();

    // Map encoder to page
    uint8_t newPage = (page.encoderValue / PAGE_ENCODER_DIVISOR) * PAGE_SIZE;
    newPage = constrain(newPage, 0, (PAGE_COUNT - 1) * PAGE_SIZE);
    page.currentOffset = newPage;

    // Blink indicators for current and adjacent pages
    uint8_t normalLed1 = page.currentOffset / 8;
    uint8_t normalLed2 = normalLed1 + 1;

    unsigned long currentMillis = millis();
    if (currentMillis - blink.lastUpdateTime >= BLINK_INTERVAL_MS) {
        blink.lastUpdateTime = currentMillis;
        blink.toggleState = !blink.toggleState;
    }

    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        if (i == normalLed1) {
            if (blink.toggleState) {
                updateEncoderLed(i);
            } else {
                clearEncoderLed(i);
            }
        } else if (i == normalLed2) {
            if (blink.toggleState) {
                clearEncoderLed(i);
            } else {
                updateEncoderLed(i);
            }
        } else {
            updateEncoderLed(i);
        }
    }
}

// ============================================================================
// Mode Selection
// ============================================================================

void handleModeSelection() {
    Button::State buttonState = configButton.update();
    unsigned long currentTime = millis();

    switch (buttonState) {
        case Button::Falling:
            // Button pressed
            mode.lastClickTime = currentTime;
            mode.awaitingSecondClick = true;
            break;

        case Button::Pressed:
            // Check for long press
            if (mode.awaitingSecondClick &&
                (currentTime - mode.lastClickTime > MODE_CHANGE_INTERVAL_MS)) {

                mode.awaitingSecondClick = false;

                if (mode.firstLongClick) {
                    // Second long press - return to normal
                    mode.current = OperatingMode::NORMAL;
                    mode.firstLongClick = false;
                } else {
                    // First long press - enter relative assign mode
                    mode.current = OperatingMode::RELATIVE_ASSIGN;
                    mode.firstLongClick = true;
                }
                mode.firstShortClick = false;
            }
            break;

        case Button::Rising:
            // Button released - check for short press
            if (mode.awaitingSecondClick) {
                if (mode.firstShortClick) {
                    // Second short press - return to normal
                    mode.current = OperatingMode::NORMAL;
                    mode.firstShortClick = false;
                } else {
                    // First short press - enter page select mode
                    mode.current = OperatingMode::PAGE_SELECT;
                    mode.firstShortClick = true;
                }
                mode.firstLongClick = false;
            }
            break;

        default:
            // Timeout check
            if (mode.awaitingSecondClick &&
                (currentTime - mode.lastClickTime > MODE_CHANGE_INTERVAL_MS)) {
                mode.awaitingSecondClick = false;
            }
            break;
    }
}

// ============================================================================
// Button MIDI Address Management
// ============================================================================

void updateButtonAddresses() {
    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        uint8_t note = i + page.currentOffset;
        noteButtons[i].setAddressUnsafe({note, BUTTON_MIDI_CHANNEL});
    }
}
