//
// Created by eb on 5/6/26.
//

//
// Created by Luna on 4/23/26.
//


#include "compass.h"


namespace angle {
    uint16_t offset(uint16_t angle, uint16_t offset) {
        int16_t result = static_cast<int16_t>(angle % 360) - static_cast<int16_t>(offset % 360);
        if (result < 0) {
            result += 360;
        }
        return static_cast<uint16_t>(result);
    }
}

Compass::Compass() {
    Wire.begin();
    bno.begin(OPERATION_MODE_IMUPLUS);
}

void Compass::calibrate(uint16_t actual_angle) {
    uint16_t rawAngle = measureAngle();

    int16_t diff = static_cast<int16_t>(rawAngle) - static_cast<int16_t>(actual_angle);

    if (diff < 0) {
        diff += 360;
    }

    offset = diff;

}

uint16_t Compass::getCurrentAngle() const {
    return angle;
}


uint16_t Compass::measureAngle() {
    sensors_event_t eulerEvent;
    bno.getEvent(&eulerEvent, Adafruit_BNO055::VECTOR_EULER);

    return eulerEvent.orientation.x;
}

void Compass::tick(jetson::State const& state) {
    uint16_t rawAngle = measureAngle();

    uint16_t curAngle = angle::offset(rawAngle, offset);

    angle = curAngle;

    calibrationTimer([this, state]{
        this->calibrate(state.self.orientation);
    }, state.cp_state == jetson::CP_State::CP_STATE_HALT);
}

