#include <array>
#include <boost/asio.hpp>
#include <map>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <variant>

#include "cppystruct.h"
#include "electrical_protocol/Packet.h"

namespace electrical_protocol
{

template <typename Derived, typename... RecvPackets>
class SerialDeviceNode : public rclcpp::Node
{
  private:
    std::optional<std::string> port_;
    std::optional<int> baudrate_;
    std::optional<std::thread> io_thread_;
    boost::asio::io_service io_;
    boost::asio::serial_port device_;
    size_t recv_buffer_size_;
    std::array<char, 1024> recv_buffer_;
    // Handler for unpack() method of each packet:
    // constexpr void unpack(std::array<char, SIZE> const& data) const
    using PacketVariant = std::variant<RecvPackets...>;

    constexpr PacketVariant _create_packet(int class_id)
    {
        std::optional<PacketVariant> result;
        bool matched = ((RecvPackets::ClassId == class_id ? (result = RecvPackets{}, true) : false) ||
                        ...);  // Fold expression over logical OR (||)

        if (!matched)
        {
            throw std::invalid_argument("Unknown packet ClassId");
        }

        return *result;
    }

    void _start_async_read_header()
    {
        boost::asio::async_read(device_, boost::asio::buffer(recv_buffer_),
                                [this](boost::system::error_code ec, std::size_t bytes_transferred)
                                {
                                    if (!ec && bytes_transferred >= PacketBase::HEADER_LEN)
                                    {
                                        _handle_header();
                                    }
                                    else
                                    {
                                        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                                              ec.message().c_str());
                                    }
                                });
    }

    void _handle_header()
    {
        std::pair<int, int> ids = std::make_pair(recv_buffer_[2], recv_buffer_[3]);
        std::optional<PacketVariant> packet_variant;
        try
        {
            packet_variant = _create_packet(ids.first);
        }
        catch (std::exception& e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to create packet: %s",
                                  e.what());
            _start_async_read_header();
            return;
        }
        // All packets have ::SIZE and ::DATA_LEN defined
        std::visit(
            [this](auto&& arg)
            {
                boost::asio::async_read(
                    device_, boost::asio::buffer(recv_buffer_.data() + PacketBase::HEADER_LEN, arg.DATA_LEN),
                    [this, &arg](boost::system::error_code ec, size_t bytes_transferred)
                    {
                        if (!ec && bytes_transferred >= arg.SIZE)
                        {
                            arg.unpack_unbounded(recv_buffer_);
                            static_cast<Derived*>(this)->process_packet(arg);
                        }
                        else if (ec)
                        {
                            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, ec.message().c_str());
                        }
                        _start_async_read_header();
                    });
            },
            *packet_variant);
    }

  public:
    SerialDeviceNode(std::string node_name, std::optional<std::string> port, std::optional<int> baudrate)
      : Node(node_name), port_(port), baudrate_(baudrate), io_(), device_(io_), recv_buffer_size_(1024), recv_buffer_()
    {
        // If we provide the port now, let's open up a connection
        if (port)
        {
            boost::system::error_code ec;
            device_.open(*port_, ec);
            if (ec)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", ec.message().c_str());
                throw std::runtime_error("Failed to open port");
                return;
            }
        }
        device_.set_option(boost::asio::serial_port::baud_rate(*baudrate_));
        _start_async_read_header();
        io_thread_ = std::thread([this]() { io_.run(); });
    }

    ~SerialDeviceNode()
    {
        boost::system::error_code ec;
        device_.cancel(ec);
        device_.close(ec);
        io_.stop();
        if ((*io_thread_).joinable())
            (*io_thread_).join();
    }

    template <size_t SIZE>
    void write(std::array<char, SIZE> const& data)
    {
        boost::asio::write(device_, boost::asio::buffer(data));
    }

    void write(std::vector<char> const& data)
    {
        boost::asio::write(device_, boost::asio::buffer(data));
    }

    void write(std::string const& data)
    {
        boost::asio::write(device_, boost::asio::buffer(data));
    }

    // Send packet down the pipeline
    template <typename T>
    void send_packet(T const& packet)
    {
        write(packet.data);
    }
};

}  // namespace electrical_protocol
