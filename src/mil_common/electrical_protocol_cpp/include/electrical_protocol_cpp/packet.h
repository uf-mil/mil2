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
    class Packet
    {
        public:
        friend class SerialDevice;
        Packet(uint8_t classId, uint8_t subClassId)
        {
            id_ = (static_cast<uint16_t>(classId) << 8) + subClassId;
        }

        ~Packet()
        {

        }
        
        template <typename Fmt, typename... Args>
        void pack(Fmt, Args&&... args)
        {
            constexpr size_t itemCount = pystruct::countItems(Fmt{});
            pack_<Fmt>(std::make_index_sequence<itemCount>(), std::forward<Args>(args)...);
        }

        template <typename Fmt>
        constexpr auto unpack(Fmt)
        {
            return unpack_<Fmt>(std::make_index_sequence<countItems(Fmt{})>());
        }


        private:
        std::vector<uint8_t> data_;
        uint16_t id_;

        template <typename Fmt, size_t... Items, typename... Args>
        inline void pack_(std::index_sequence<Items...>, Args&&... args)
        {
            static_assert(sizeof...(args) == sizeof...(Items), "pack expected items for packing != sizeof...(args) passed");
            constexpr auto formatMode = pystruct::getFormatMode(Fmt{});
            constexpr auto byteSize = pystruct::calcsize(Fmt{});
            data_.resize(byteSize);

            constexpr pystruct::FormatType formats[] = { pystruct::getTypeOfItem<Items>(Fmt{})... };
            using Types = std::tuple<typename pystruct::RepresentedType<decltype(formatMode), formats[Items].formatChar> ...>;

            // Convert args to a tuple of the represented types
            Types t = std::make_tuple(pystruct::internal::convert<std::tuple_element_t<Items, Types>>(std::forward<Args>(args))...);

            constexpr size_t offsets[] = { pystruct::getBinaryOffset<Items>(Fmt{})... };
            int _[] = { 0, pystruct::internal::packElement(reinterpret_cast<char*>(data_.data()) + offsets[Items],  formatMode.isBigEndian(), formats[Items], std::get<Items>(t))... };
            (void)_; // _ is a dummy for pack expansion
        }

        template <typename Fmt, size_t... Items>
        inline auto unpack_(std::index_sequence<Items...>)
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