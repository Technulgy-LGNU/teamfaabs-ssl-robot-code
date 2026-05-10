#include <Arduino.h>

// ======================================================
// Protocol Sizes
// ======================================================

constexpr uint8_t REC_MSG_SIZE  = 6;
constexpr uint8_t SEND_MSG_SIZE = 11;

// ======================================================
// Incoming Flags (Rust -> Teensy)
// Matches TeensySendMsg.flags
// ======================================================

namespace SendFlags {
    constexpr uint16_t ERROR     = 1 << 0;
    constexpr uint16_t KICK      = 1 << 1;
    constexpr uint16_t CHIP      = 1 << 2;
    constexpr uint16_t DRIBBLER = 1 << 3;
}

// ======================================================
// Outgoing Flags (Teensy -> Rust)
// Matches TeensyRecMSG.flags
// ======================================================

namespace RecFlags {
    constexpr uint32_t ERROR       = 1 << 0;
    constexpr uint32_t HAS_BALL    = 1 << 1;
    constexpr uint32_t KICK_READY  = 1 << 2;
    constexpr uint32_t CHIP_READY  = 1 << 3;
}

// ======================================================
// Rust -> Teensy Message
// ======================================================

#pragma pack(push, 1)

struct TeensySendMsg {
    uint16_t flags;

    uint8_t state;
    uint8_t kick_pwr;
    uint8_t dribbler_pwr;

    uint16_t dir;
    uint16_t speed;
    uint16_t orient;

    bool hasFlag(uint16_t flag) const {
        return (flags & flag) != 0;
    }
};

static_assert(sizeof(TeensySendMsg) == 11, "TeensySendMsg size mismatch");

// ======================================================
// Teensy -> Rust Message
// ======================================================

struct TeensyRecMSG {
    uint32_t flags;

    uint8_t batt_level;
    uint8_t orientation;

    void setFlag(uint32_t flag) {
        flags |= flag;
    }

    void clearFlag(uint32_t flag) {
        flags &= ~flag;
    }

    bool hasFlag(uint32_t flag) const {
        return (flags & flag) != 0;
    }
};

#pragma pack(pop)

static_assert(sizeof(TeensyRecMSG) == 6, "TeensyRecMSG size mismatch");

// ======================================================
// Decode Rust -> Teensy
// ======================================================

bool decode_msg(const uint8_t* buf, TeensySendMsg& msg) {

    memcpy(&msg, buf, sizeof(TeensySendMsg));

    return true;
}

// ======================================================
// Encode Teensy -> Rust
// ======================================================

void encode_msg(const TeensyRecMSG& msg, uint8_t* buf) {
    memcpy(buf, &msg, sizeof(TeensyRecMSG));
}

// ======================================================
// Globals
// ======================================================

uint8_t recv_buf[64];
uint8_t send_buf[64];

TeensySendMsg cmd;
TeensyRecMSG status;

// ======================================================
// Setup
// ======================================================

void setup() {
    Serial.begin(115200);
}

// ======================================================
// Main Loop
// ======================================================

void loop() {

    // ==========================================
    // Receive packet from Rust
    // ==========================================

    memset(recv_buf, 0, sizeof(recv_buf));

    int n = RawHID.recv(recv_buf, 0);

    if (n > 0) {

        if (n >= SEND_MSG_SIZE) {
            decode_msg(recv_buf, cmd);
        }

        // ------------------------------
        // Example motion data
        // ------------------------------

        Serial.print("DIR: ");
        Serial.println(cmd.dir);

        Serial.print("SPEED: ");
        Serial.println(cmd.speed);

        Serial.print("ORIENT: ");
        Serial.println(cmd.orient);
    }


    // ------------------------------
    // Update status example
    // ------------------------------

    status.setFlag(RecFlags::CHIP_READY);
    status.setFlag(RecFlags::HAS_BALL);
    
    // ==========================================
    // Send packet back to Rust
    // ==========================================
    
    encode_msg(status, send_buf);

    for (int i = 0; i < 6; i++) {
        Serial.print(send_buf[i]);
        Serial.print(" ");
    }
    Serial.println();
    
    RawHID.send(send_buf, 0);

    delay(20);
}
