#pragma once

#include <string>
#include <memory>
#include <vector>
#include <deque>

#include <cppystruct.h>

namespace electrical_protocol
{
    class SerialDevice;
    class Packet
    {
        public:
        friend class SerialDevice;
        Packet(uint8_t classId, uint8_t subClassId);
        ~Packet();

        private:
        std::vector<uint8_t> data_;
        uint16_t id_;
    };
    
}