#include <Control_Surface.h>
#include <Wire.h>
#include <MCP23017.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

// ============================================================================
// Configuration
// ============================================================================

// I2C Configuration
constexpr uint8_t I2C_SLAVE_ADDRESS = 0x09;
constexpr uint8_t I2C_SDA_PIN = 19;
constexpr uint8_t I2C_SCL_PIN = 18;
constexpr uint32_t I2C_SLAVE_CLOCK = 400000;
constexpr uint32_t I2C_MASTER_CLOCK = 1000000;

// Protocol Constants
constexpr uint8_t PACKET_START_BYTE = 0xAB;
constexpr uint8_t PACKET_END_BYTE = 0xCD;

// Encoder Configuration
constexpr uint8_t ENCODER_COUNT = 16;
constexpr uint8_t ENCODER_PER_CHIP = 8;
constexpr uint8_t ENCODER_VALUE_MIN = 0;
constexpr uint8_t ENCODER_VALUE_MAX = 127;
constexpr int ENCODER_RAW_MIN = 0;
constexpr int ENCODER_RAW_MAX = 254;

// MCP23017 I2C Addresses
constexpr uint8_t MCP23017_ADDR_1 = 0x20;
constexpr uint8_t MCP23017_ADDR_2 = 0x21;

// Speed Calculation
constexpr uint8_t SPEED_HISTORY_SIZE = 5;
constexpr uint16_t SPEED_THRESHOLD_VERY_FAST = 100;
constexpr uint16_t SPEED_THRESHOLD_FAST = 150;
constexpr uint16_t SPEED_THRESHOLD_MEDIUM_FAST = 200;
constexpr uint16_t SPEED_THRESHOLD_MEDIUM = 250;

constexpr uint8_t SPEED_MULTIPLIER_VERY_FAST = 5;
constexpr uint8_t SPEED_MULTIPLIER_FAST = 4;
constexpr uint8_t SPEED_MULTIPLIER_MEDIUM_FAST = 3;
constexpr uint8_t SPEED_MULTIPLIER_MEDIUM = 2;
constexpr uint8_t SPEED_MULTIPLIER_SLOW = 1;

// ============================================================================
// Hardware Instances
// ============================================================================

MCP23017 gpioExpander1(MCP23017_ADDR_1);
MCP23017 gpioExpander2(MCP23017_ADDR_2);
TwoWire i2cSlave(1);

// ============================================================================
// Encoder Pin Configuration
// ============================================================================

struct EncoderPinConfig {
    MCP23017* expander;
    uint8_t pinA;
    uint8_t pinB;
};

constexpr EncoderPinConfig ENCODER_PINS[ENCODER_COUNT] = {
    // First MCP23017 (addresses 0x00-0x07)
    {&gpioExpander1, 0, 1},
    {&gpioExpander1, 2, 3},
    {&gpioExpander1, 4, 5},
    {&gpioExpander1, 6, 7},
    {&gpioExpander1, 8, 9},
    {&gpioExpander1, 10, 11},
    {&gpioExpander1, 12, 13},
    {&gpioExpander1, 14, 15},
    // Second MCP23017 (addresses 0x08-0x0F)
    {&gpioExpander2, 0, 1},
    {&gpioExpander2, 2, 3},
    {&gpioExpander2, 4, 5},
    {&gpioExpander2, 6, 7},
    {&gpioExpander2, 8, 9},
    {&gpioExpander2, 10, 11},
    {&gpioExpander2, 12, 13},
    {&gpioExpander2, 14, 15}
};

// ============================================================================
// Rotary Encoder Class
// ============================================================================

class RotaryEncoder {
public:
    RotaryEncoder(MCP23017* expander, uint8_t pinA, uint8_t pinB)
        : expander_(expander)
        , pinA_(pinA)
        , pinB_(pinB)
        , rawPosition_(0)
        , lastEncoded_(0)
        , lastRawPosition_(0)
        , currentSpeed_(SPEED_MULTIPLIER_SLOW)
        , historyIndex_(0)
        , lastUpdateTime_(0)
        , midiSyncPosition_(-1)
        , syncActive_(false)
    {
        for (uint8_t i = 0; i < SPEED_HISTORY_SIZE; i++) {
            waitTimes_[i] = 0;
        }
    }

    void initialize() {
        expander_->pinMode(pinA_, INPUT);
        expander_->digitalWrite(pinA_, HIGH);
        expander_->pinMode(pinB_, INPUT);
        expander_->digitalWrite(pinB_, HIGH);
    }

    void update() {
        // Read encoder state
        int msb = expander_->digitalRead(pinA_);
        int lsb = expander_->digitalRead(pinB_);

        int encoded = (msb << 1) | lsb;
        int sum = (lastEncoded_ << 2) | encoded;

        // Decode rotation (quadrature)
        bool clockwise = (sum == 0b1101 || sum == 0b0100 ||
                         sum == 0b0010 || sum == 0b1011);
        bool counterClockwise = (sum == 0b1110 || sum == 0b0111 ||
                                sum == 0b0001 || sum == 0b1000);

        if (clockwise) {
            rawPosition_ += currentSpeed_;
            syncActive_ = false;
        } else if (counterClockwise) {
            rawPosition_ -= currentSpeed_;
            syncActive_ = false;
        }

        // Constrain to valid range
        rawPosition_ = constrain(rawPosition_, ENCODER_RAW_MIN, ENCODER_RAW_MAX);

        // Update speed calculation on position change
        if (rawPosition_ != lastRawPosition_) {
            updateSpeed();
            lastRawPosition_ = rawPosition_;
        }

        // Apply MIDI sync if active
        if (syncActive_) {
            rawPosition_ = midiSyncPosition_;
        }

        lastEncoded_ = encoded;
    }

    uint8_t getValue() const {
        return map(rawPosition_, ENCODER_RAW_MIN, ENCODER_RAW_MAX,
                   ENCODER_VALUE_MIN, ENCODER_VALUE_MAX);
    }

    void setMidiSyncPosition(uint8_t value) {
        midiSyncPosition_ = value * 2;
        syncActive_ = true;
    }

private:
    void updateSpeed() {
        unsigned long now = millis();
        waitTimes_[historyIndex_] = now - lastUpdateTime_;
        lastUpdateTime_ = now;

        // Calculate average wait time
        unsigned long totalWait = 0;
        for (uint8_t i = 0; i < SPEED_HISTORY_SIZE; i++) {
            totalWait += waitTimes_[i];
        }

        // Determine speed multiplier
        if (totalWait < SPEED_THRESHOLD_VERY_FAST) {
            currentSpeed_ = SPEED_MULTIPLIER_VERY_FAST;
        } else if (totalWait < SPEED_THRESHOLD_FAST) {
            currentSpeed_ = SPEED_MULTIPLIER_FAST;
        } else if (totalWait < SPEED_THRESHOLD_MEDIUM_FAST) {
            currentSpeed_ = SPEED_MULTIPLIER_MEDIUM_FAST;
        } else if (totalWait < SPEED_THRESHOLD_MEDIUM) {
            currentSpeed_ = SPEED_MULTIPLIER_MEDIUM;
        } else {
            currentSpeed_ = SPEED_MULTIPLIER_SLOW;
        }

        historyIndex_ = (historyIndex_ + 1) % SPEED_HISTORY_SIZE;
    }

    MCP23017* expander_;
    uint8_t pinA_;
    uint8_t pinB_;

    int rawPosition_;
    int lastEncoded_;
    int lastRawPosition_;

    uint8_t currentSpeed_;
    uint8_t historyIndex_;
    unsigned long waitTimes_[SPEED_HISTORY_SIZE];
    unsigned long lastUpdateTime_;

    int midiSyncPosition_;
    bool syncActive_;
};

// ============================================================================
// Global Instances
// ============================================================================

RotaryEncoder encoders[ENCODER_COUNT] = {
    RotaryEncoder(ENCODER_PINS[0].expander, ENCODER_PINS[0].pinA, ENCODER_PINS[0].pinB),
    RotaryEncoder(ENCODER_PINS[1].expander, ENCODER_PINS[1].pinA, ENCODER_PINS[1].pinB),
    RotaryEncoder(ENCODER_PINS[2].expander, ENCODER_PINS[2].pinA, ENCODER_PINS[2].pinB),
    RotaryEncoder(ENCODER_PINS[3].expander, ENCODER_PINS[3].pinA, ENCODER_PINS[3].pinB),
    RotaryEncoder(ENCODER_PINS[4].expander, ENCODER_PINS[4].pinA, ENCODER_PINS[4].pinB),
    RotaryEncoder(ENCODER_PINS[5].expander, ENCODER_PINS[5].pinA, ENCODER_PINS[5].pinB),
    RotaryEncoder(ENCODER_PINS[6].expander, ENCODER_PINS[6].pinA, ENCODER_PINS[6].pinB),
    RotaryEncoder(ENCODER_PINS[7].expander, ENCODER_PINS[7].pinA, ENCODER_PINS[7].pinB),
    RotaryEncoder(ENCODER_PINS[8].expander, ENCODER_PINS[8].pinA, ENCODER_PINS[8].pinB),
    RotaryEncoder(ENCODER_PINS[9].expander, ENCODER_PINS[9].pinA, ENCODER_PINS[9].pinB),
    RotaryEncoder(ENCODER_PINS[10].expander, ENCODER_PINS[10].pinA, ENCODER_PINS[10].pinB),
    RotaryEncoder(ENCODER_PINS[11].expander, ENCODER_PINS[11].pinA, ENCODER_PINS[11].pinB),
    RotaryEncoder(ENCODER_PINS[12].expander, ENCODER_PINS[12].pinA, ENCODER_PINS[12].pinB),
    RotaryEncoder(ENCODER_PINS[13].expander, ENCODER_PINS[13].pinA, ENCODER_PINS[13].pinB),
    RotaryEncoder(ENCODER_PINS[14].expander, ENCODER_PINS[14].pinA, ENCODER_PINS[14].pinB),
    RotaryEncoder(ENCODER_PINS[15].expander, ENCODER_PINS[15].pinA, ENCODER_PINS[15].pinB)
};

uint8_t lastEncoderValues[ENCODER_COUNT] = {0};

// ============================================================================
// I2C Communication Handlers
// ============================================================================

void onI2CRequest() {
    i2cSlave.write(PACKET_START_BYTE);

    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        i2cSlave.write(encoders[i].getValue());
    }

    i2cSlave.write(PACKET_END_BYTE);
}

void onI2CReceive(int byteCount) {
    if (byteCount != 2) return;

    uint8_t lowByte = i2cSlave.read();
    uint8_t highByte = i2cSlave.read();
    uint16_t data = (highByte << 8) | lowByte;

    uint8_t encoderIndex = (data >> 12) & 0x0F;
    uint8_t ccValue = data & 0x7F;

    if (encoderIndex < ENCODER_COUNT) {
        encoders[encoderIndex].setMidiSyncPosition(ccValue);
    }
}

// ============================================================================
// Setup and Main Loop
// ============================================================================

void setup() {
    // Initialize I2C slave interface
    i2cSlave.begin(I2C_SLAVE_ADDRESS, I2C_SDA_PIN, I2C_SCL_PIN, I2C_SLAVE_CLOCK);
    i2cSlave.onRequest(onI2CRequest);
    i2cSlave.onReceive(onI2CReceive);

    // Initialize I2C master interface (for future expansion)
    Wire.begin();
    Wire.setClock(I2C_MASTER_CLOCK);

    // Initialize MCP23017 expanders
    gpioExpander1.init();
    gpioExpander2.init();

    // Initialize encoders
    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        encoders[i].initialize();
    }

    // Initialize debug serial
    Serial.begin(115200);
}

void loop() {
    // Update all encoders
    for (uint8_t i = 0; i < ENCODER_COUNT; i++) {
        encoders[i].update();

        // Debug output on value change
        uint8_t currentValue = encoders[i].getValue();
        if (currentValue != lastEncoderValues[i]) {
            Serial.print(currentValue);
            Serial.print('\t');
            Serial.println(i);
            lastEncoderValues[i] = currentValue;
        }
    }
}
