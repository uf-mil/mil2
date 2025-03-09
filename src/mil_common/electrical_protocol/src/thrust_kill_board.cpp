#include "electrical_protocol/Packet.h"
#include "electrical_protocol/SerialDeviceNode.h"

class HeartbeatSetPacket : public electrical_protocol::Packet<HeartbeatSetPacket, 0x02, 0x00, "">
{
  public:
    constexpr HeartbeatSetPacket()
    {
        pack();
    }
    inline bool operator==(HeartbeatSetPacket const& other) const
    {
        return true;
    }
    inline bool operator!=(HeartbeatSetPacket const& other) const
    {
        return !(*this == other);
    }
    constexpr void unpack_handle(std::array<char, SIZE> const& spliced_data)
    {
    }
    constexpr std::array<char, DATA_LEN> pack_format() const
    {
        return pystruct::pack(PY_STRING(""), );
    }
}

int main()
{
    return 0;
}
