#pragma once

#include <string>
#include <memory>
#include <vector>
#include <deque>

#include <cppystruct.h>
// #include <cppystruct/format.h>

namespace electrical_protocol
{
    class SerialDevice;

    class PacketBase
    {
        public:
        friend class SerialDevice;
        PacketBase()
        {

        }
        ~PacketBase()
        {
            
        }

        virtual uint8_t* getData() = 0;
        virtual size_t getDataSize() = 0;

        protected:
        uint16_t id_;

    };

    template<typename Fmt>
    class Packet: private PacketBase
    {
        public:
        Packet(Fmt, uint8_t classId, uint8_t subClassId)
        {
            id_ = (static_cast<uint16_t>(classId) << 8) + subClassId;
        }

        ~Packet()
        {

        }

        uint8_t* getData()
        {
            return data_.data();
        }

        size_t getDataSize()
        {
            return data_.size();
        }
        
        template <typename... Args>
        constexpr void pack(Args&&... args)
        {
            constexpr size_t itemCount = pystruct::countItems(Fmt{});
            pack_(std::make_index_sequence<itemCount>(), std::forward<Args>(args)...);
        }

        constexpr auto unpack()
        {
            return unpack_(std::make_index_sequence<countItems(Fmt{})>());
        }

        private:

        std::array<uint8_t, pystruct::calcsize(Fmt{})> data_;

        template <size_t... Items, typename... Args>
        constexpr void pack_(std::index_sequence<Items...>, Args&&... args)
        {
            static_assert(sizeof...(args) == sizeof...(Items), "pack expected items for packing != sizeof...(args) passed");
            constexpr auto formatMode = pystruct::getFormatMode(Fmt{});

            constexpr pystruct::FormatType formats[] = { pystruct::getTypeOfItem<Items>(Fmt{})... };
            using Types = std::tuple<typename pystruct::RepresentedType<decltype(formatMode), formats[Items].formatChar> ...>;

            // Convert args to a tuple of the represented types
            Types t = std::make_tuple(pystruct::internal::convert<std::tuple_element_t<Items, Types>>(std::forward<Args>(args))...);

            constexpr size_t offsets[] = { pystruct::getBinaryOffset<Items>(Fmt{})... };
            int _[] = { 0, pystruct::internal::packElement(reinterpret_cast<char*>(data_.data()) + offsets[Items],  formatMode.isBigEndian(), formats[Items], std::get<Items>(t))... };
            (void)_; // _ is a dummy for pack expansion
        }

        template <size_t... Items>
        constexpr auto unpack_(std::index_sequence<Items...>)
        {
            constexpr auto formatMode = pystruct::getFormatMode(Fmt{});

            constexpr pystruct::FormatType formats[] = { pystruct::getTypeOfItem<Items>(Fmt{})... };

            using Types = std::tuple<typename pystruct::RepresentedType<decltype(formatMode), formats[Items].formatChar> ...>;

            constexpr size_t offsets[] = { pystruct::getBinaryOffset<Items>(Fmt{})... };
            auto unpacked = std::make_tuple(pystruct::unpackElement<Items, std::tuple_element_t<Items, Types>>(
            reinterpret_cast<char*>(data_.data()) + offsets[Items], formats[Items].size, formatMode.isBigEndian())...);

            return unpacked;
        }

    };
    
}