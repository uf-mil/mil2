#include <electrical_protocol_cpp/packet.h>

#include <vector>
#include <queue>
#include <mutex>
// #include <cstdint>

namespace electrical_protocol
{
    Packet::Packet(uint8_t classId, uint8_t subClassId)
    {
        id_ = (static_cast<uint16_t>(classId) << 8) + subClassId;
    }

    Packet::~Packet()
    {

    }

}
