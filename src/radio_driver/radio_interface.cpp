#include "radio_driver/radio_interface.h"

#include <glog/logging.h>

namespace radio_driver {

RadioInterface::RadioInterface() : buffer_size_(0), connected_(false) {
    buffer_ = new char[12];  // TODO: check if 12 is enough
}

RadioInterface::~RadioInterface() { delete[] buffer_; }

bool RadioInterface::isConnected() const { return connected_; }

bool RadioInterface::sendControl(float throttle, float steering) {
    if (!connected_) {
        LOG(ERROR) << "Attempted to send control to radio while disconnected";
        return false;
    }

    commandToBuffer(throttle, steering);
    if (buffer_size_) {
        serial_.write(buffer_, buffer_size_);
        return true;
    } else {
        return false;
    }
}

bool RadioInterface::connect(const std::string& port, int baud) {
    connected_ = serial_.open(port.c_str(), baud);
    if (!connected_) {
        LOG(ERROR) << "Failed to connect to radio on serial port " << port;
    }
    return connected_;
}

void RadioInterface::disconnect() {
    serial_.close();
    connected_ = false;
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
        std::string data = ss.str();
        data.copy(buffer_, data.size());
        buffer_size_ = data.size();
    }
}

}  // namespace radio_driver
