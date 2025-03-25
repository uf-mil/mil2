#pragma once

#include <cppystruct.h>

#include <algorithm>
#include <array>
#include <format>
#include <string>
#include <string_view>
#include <type_traits>

namespace electrical_protocol
{

template <size_t N>
struct StringLiteral
{
    constexpr StringLiteral(char const (&str)[N])
    {
        std::copy_n(str, N, value_);
    }
    // value() should return character literal
    constexpr char const* value() const
    {
        return value_;
    }
    char value_[N];
};

// struct BasicString : pystruct::internal::format_string {
//     const char* s;
//     constexpr BasicString(const char* s) : s(s) {}
//       static constexpr decltype(auto) value() { return s; }
//       static constexpr size_t size() { return std::size(value()) - 1; }
//       static constexpr auto at(size_t i) { return value()[i]; };
//     };

template <std::size_t N>
class StaticString
{
    static constexpr StaticString<N> instance()
    {
        return StaticString<N>(value_static_liberal);
    }
    static constexpr char value_static_liberal[N]{};

  public:
    char value_[N];

    constexpr StaticString() = default;
    constexpr StaticString(char const (&str)[N])
    {
        std::copy_n(str, N, value_);
    }
    static constexpr auto value()
    {
        return instance().value_;
    }
    static constexpr size_t size()
    {
        return N;
    }
    static constexpr char at(size_t i)
    {
        return value()[i];
    }
    // operator+
};

template <std::size_t N>
StaticString(char const (&)[N]) -> StaticString<N>;

struct PacketBase
{
    static constexpr uint8_t SYNC_CHAR_1 = 0x37;
    static constexpr uint8_t SYNC_CHAR_2 = 0x01;
    static constexpr uint8_t HEADER_LEN = 8;
};

template <typename Derived, int ClassId_, int SubclassId_, StringLiteral Fmt>
class Packet : public PacketBase
{
  public:
    constexpr Packet()
    {
    }
    // Header length:
    //  * Sync characters: 2 bytes
    //  * Class ID: 1 byte
    //  * Subclass ID: 1 byte
    //  * Data length: 2 bytes
    //  * Checksum: 2 bytes
    static constexpr int ClassId = ClassId_;
    static constexpr int SubclassId = SubclassId_;
    static constexpr uint16_t DATA_LEN = pystruct::calcsize(PY_STRING("i"));
    static constexpr uint16_t SIZE = 12;  // HEADER_LEN + DATA_LEN;
    constexpr auto format_string() const
    {
        return PY_STRING("<BBBBH4sBB");
    }
    std::array<char, SIZE> data{};
    constexpr std::pair<int, int> calculate_checksum() const
    {
        int sum1 = 0, sum2 = 0;
        for (char piece : data)
        {
            sum1 = (sum1 + piece) % 255;
            sum2 = (sum1 + sum2) % 255;
        }
        return { sum1, sum2 };
    }
    constexpr void pack()
    {
        std::array<char, DATA_LEN> spliced_data = static_cast<Derived const*>(this)->pack_format();
        data = pystruct::pack(format_string(), SYNC_CHAR_1, SYNC_CHAR_2, static_cast<uint8_t>(ClassId),
                              static_cast<uint8_t>(SubclassId), DATA_LEN,
                              std::string(spliced_data.begin(), spliced_data.end()), 0, 0);
        std::pair<int, int> checksum = calculate_checksum();
        data[SIZE - 2] = checksum.first;
        data[SIZE - 1] = checksum.second;
    }
    constexpr void unpack(std::array<char, SIZE> const& data)
    {
        std::array<char, DATA_LEN> spliced_data;
        std::copy(data.begin() + HEADER_LEN, data.end() - 2, spliced_data.begin());
        static_cast<Derived*>(this)->unpack_handle(spliced_data);
    }
    template <size_t TEMP_SIZE>
    constexpr void unpack_unbounded(std::array<char, TEMP_SIZE> const& data)
    {
        std::array<char, DATA_LEN> spliced_data;
        std::copy(data.begin() + HEADER_LEN, data.end() - 2, spliced_data.begin());
        static_cast<Derived*>(this)->unpack_handle(spliced_data);
    }
};

}  // namespace electrical_protocol
