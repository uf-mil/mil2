#include <gtest/gtest.h>

#include <variant>

#include "electrical_protocol/Packet.h"
#include "electrical_protocol/SerialDeviceNode.h"

class TestPacket : public electrical_protocol::Packet<TestPacket, 0x99, 0x98, "i">
{
    int fav_number_ = 0;

  public:
    constexpr TestPacket() = default;
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
    constexpr auto data_string() const
    {
        return PY_STRING("i");
    }
    constexpr void unpack_handle(std::array<char, DATA_LEN> const& spliced_data)
    {
        fav_number_ = std::get<0>(pystruct::unpack(data_string(), spliced_data));
    }
    constexpr std::array<char, DATA_LEN> pack_format() const
    {
        return pystruct::pack(data_string(), fav_number_);
    }
};

class TestNode : public electrical_protocol::SerialDeviceNode<TestNode, TestPacket>
{
  public:
    TestNode(std::string node_name, std::optional<std::string> port, std::optional<int> baudrate)
      : SerialDeviceNode(node_name, port, baudrate)
    {
    }
    void test_send()
    {
        send_packet(TestPacket(3701));
    }
    void process_packet(std::variant<TestPacket> var_packet)
    {
        throw std::runtime_error("Not implemented");
        std::visit([](auto&& arg) { EXPECT_EQ(arg, TestPacket(3701)); }, var_packet);
    }
};

class InitNodeTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
    }
    void TearDown() override
    {
        rclcpp::shutdown();
    }
};

TEST(InitNodeTest, test_serial)
{
    TestNode node("test_serial_node", "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6614C311B66C338-if00", 9600);
    node.test_send();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
