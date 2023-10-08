#include "radio_driver/radio_interface.h"

#include <glog/logging.h>

namespace radio_driver {

RadioInterface::RadioInterface() : connected_(false) {
    buffer_ = new char[12];  // TODO: check if 12 is enough
}

RadioInterface::~RadioInterface() {
    if (!disconnect()) {
        LOG(ERROR) << "Failed to cleanly disconnect from radio on shutdown";
    }

    delete[] buffer_;
}

bool RadioInterface::isConnected() const { return connected_; }

bool RadioInterface::sendControl(float throttle, float steering) {
    if (!connected_) {
        LOG(ERROR) << "Attempted to send control to radio while disconnected";
        return false;
    }

    commandToBuffer(throttle, steering);
    // TODO: send buf to serial device
    return true;
}

bool RadioInterface::connect(const std::string& port) {
    // TODO: actually connect to serial device
    connected_ = true;
    return true;
}

bool RadioInterface::disconnect() {
    // TODO: actually disconnect from serial device
    connected_ = false;
    return true;
}

void RadioInterface::commandToBuffer(float throttle, float steering) {
    static float min_val = 1000;
    static float max_val = 2000;

    // TODO: print these to see if they are correct
    uint32_t throttle_pwm = (throttle + 1) * (max_val - min_val) / 2 + min_val;
    uint32_t steering_pwm = (steering + 1) * (max_val - min_val) / 2 + min_val;

    if (throttle_pwm < min_val || throttle_pwm > max_val) {
        LOG(ERROR) << "Throttle command out of range: " << throttle_pwm;
    } else if (steering_pwm < min_val || steering_pwm > max_val) {
        LOG(ERROR) << "Steering command out of range: " << steering_pwm;
    } else {
        std::ostringstream ss;
        ss << throttle_pwm << "," << steering_pwm << "\n";
        ss.str().copy(buffer_, ss.str().size());
    }
}

}  // namespace radio_driver
