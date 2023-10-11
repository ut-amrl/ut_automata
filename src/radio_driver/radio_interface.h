#include <string>

#include "vesc_driver/serial.h"

namespace radio_driver {

class RadioInterface {
   public:
    RadioInterface();

    ~RadioInterface();

    bool isConnected() const;

    // send a command to the car via the radio
    // throttle and steering are in the range [-1, 1]
    bool sendControl(float throttle, float steering);

   private:
    bool connect(const std::string& port, int baud);
    void disconnect();

    // convert throttle and steering commands to ascii bytes in the format arduino expects
    // throttle and steering are in the range [-1, 1]
    void commandToBuffer(float throttle, float steering);

    Serial serial_;
    char* buffer_;
    size_t buffer_size_;
    bool connected_;
};

}  // namespace radio_driver