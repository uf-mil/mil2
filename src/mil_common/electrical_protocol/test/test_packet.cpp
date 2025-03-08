#include <gtest/gtest.h>

#include "electrical_protocol/Packet.h"

class TestPacket : public electrical_protocol::Packet<TestPacket, 0x03, 0x07, "i">
{
    int fav_number_ = 0;

  public:
    constexpr TestPacket(int fav_number) : fav_number_(fav_number)
    {
        pack();
    }
    inline bool operator==(TestPacket const& other) const
    {
        return fav_number_ == other.fav_number_;
    }
    inline bool operator!=(TestPacket const& other) const
    {
        return !(*this == other);
    }
    constexpr void unpack_handle(std::array<char, SIZE> const& spliced_data)
    {
        fav_number_ = std::get<0>(pystruct::unpack(PY_STRING("i"), spliced_data));
    }
    constexpr std::array<char, DATA_LEN> pack_format() const
    {
        return pystruct::pack(PY_STRING("i"), fav_number_);
    }
};

TEST(electrical_protocol, test_packet)
{
    constexpr TestPacket packet(3701);
    // Sync characters
    static_assert(packet.data[0] == 0x37);
    static_assert(packet.data[1] == 0x01);
    // Class and subclass IDs
    static_assert(packet.data[2] == 0x03);
    static_assert(packet.data[3] == 0x07);
    // data length
    static_assert(packet.data[4] == 0x04);
    static_assert(packet.data[5] == 0x00);
    // data (int of 3701)
    static_assert(packet.data[6] == 0x75);
    static_assert(packet.data[7] == 0xE);
    static_assert(packet.data[8] == 0x0);
    static_assert(packet.data[9] == 0x0);
    ASSERT_EQ(packet, TestPacket(3701));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
