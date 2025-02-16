#pragma once

#include <string>
#include <memory>
#include <vector>
#include <deque>
#include <exception>

#include <cppystruct.h>
// #include <cppystruct/format.h>

namespace electrical_protocol
{
    class SerialDevice;

    class Packet
    {
        public:
        friend class SerialDevice;
        Packet()
        {

        }
        
        Packet(uint8_t classId, uint8_t subClassId)
        {
            data_[2] = classId;
            data_[3] = subClassId;
        }

        Packet(const std::pair<uint8_t, uint8_t>& Id)
        {
            data_[2] = Id.first;
            data_[3] = Id.second;
        }

        Packet(Packet&& packet)
        {
            data_ = std::move(packet.data_);
        }

        Packet(Packet& packet)
        {
            data_ = packet.data_;
        }

        Packet& operator= (Packet&& packet)
        {
            std::vector<uint8_t> temp(std::move(data_));
            data_ = std::move(packet.data_);
            packet.data_ = std::move(temp);
            packet.data_.resize(HEADER_LEN + TRAILER_LEN);
            return *this;
        }

        Packet& operator= (Packet& packet)
        {
            data_ = packet.data_;
            return *this;
        }

        ~Packet()
        {

        }

        struct IdHash
        {
            size_t operator()(const std::pair<uint8_t, uint8_t>& p) const
            {
                return static_cast<size_t>((static_cast<uint16_t>(p.first) << 8) + p.second);
            }
        };

        inline void setId(const std::pair<uint8_t, uint8_t>& Id)
        {
            data_[2] = Id.first;
            data_[3] = Id.second;
        }

        inline std::pair<uint8_t, uint8_t> getId() const
        {
            return {data_[2], data_[3]};
        }

        inline size_t size() const
        {
            return data_.size()- HEADER_LEN - TRAILER_LEN;
        }

        inline uint8_t* data()
        {
            return &data_[HEADER_LEN];
        }
        
        template <typename Fmt, typename... Args>
        void pack(Fmt, Args&&... args)
        {
            data_.resize(pystruct::calcsize(Fmt{}) + HEADER_LEN + TRAILER_LEN);
            *reinterpret_cast<uint16_t*>(&data_[4]) = pystruct::calcsize(Fmt{});
            constexpr size_t itemCount = pystruct::countItems(Fmt{});
            pack_<Fmt>(std::make_index_sequence<itemCount>(), std::forward<Args>(args)...);
            calcCheckSum_(&data_[data_.size() - TRAILER_LEN]);
        }

        template<typename Fmt>
        auto unpack(Fmt) const
        {
            constexpr size_t expectSize = pystruct::calcsize(Fmt{}) + HEADER_LEN + TRAILER_LEN;
            if(expectSize > data_.size())
                throw std::out_of_range("No enough data expect " + std::to_string(expectSize)
                                        + " get " + std::to_string(data_.size()));

            uint8_t sum[2];
            calcCheckSum_(sum);
            if(sum[0] != data_[data_.size()-2] || sum[1] != data_[data_.size()-1])
                throw std::runtime_error("Bad checksum");

            return unpack_<Fmt>(std::make_index_sequence<countItems(Fmt{})>());
        }

        private:
        static constexpr uint8_t SYNC_CHAR_1 = 0x37;
        static constexpr uint8_t SYNC_CHAR_2 = 0x01;
        static constexpr size_t HEADER_LEN = 6;
        static constexpr size_t TRAILER_LEN = 2;
        static constexpr size_t SYNC_LEN = 2;
        std::vector<uint8_t> data_ = {SYNC_CHAR_1, SYNC_CHAR_2, 0, 0, 0, 0, 0, 0};
        
        template <typename Fmt, size_t... Items, typename... Args>
        constexpr void pack_(std::index_sequence<Items...>, Args&&... args)
        {
            static_assert(sizeof...(args) == sizeof...(Items), "pack expected items for packing != sizeof...(args) passed");
            constexpr auto formatMode = pystruct::getFormatMode(Fmt{});

            constexpr pystruct::FormatType formats[] = { pystruct::getTypeOfItem<Items>(Fmt{})... };
            using Types = std::tuple<typename pystruct::RepresentedType<decltype(formatMode), formats[Items].formatChar> ...>;

            // Convert args to a tuple of the represented types
            Types t = std::make_tuple(pystruct::internal::convert<std::tuple_element_t<Items, Types>>(std::forward<Args>(args))...);

            constexpr size_t offsets[] = { pystruct::getBinaryOffset<Items>(Fmt{})... };
            int _[] = { 0, pystruct::internal::packElement(reinterpret_cast<char*>(data_.data()) + HEADER_LEN + offsets[Items],  formatMode.isBigEndian(), formats[Items], std::get<Items>(t))... };
            (void)_; // _ is a dummy for pack expansion
        }

        template <typename Fmt, size_t... Items>
        constexpr auto unpack_(std::index_sequence<Items...>) const
        {
            constexpr auto formatMode = pystruct::getFormatMode(Fmt{});

            constexpr pystruct::FormatType formats[] = { pystruct::getTypeOfItem<Items>(Fmt{})... };

            using Types = std::tuple<typename pystruct::RepresentedType<decltype(formatMode), formats[Items].formatChar> ...>;

            constexpr size_t offsets[] = { pystruct::getBinaryOffset<Items>(Fmt{})... };
            auto unpacked = std::make_tuple(pystruct::unpackElement<Items, std::tuple_element_t<Items, Types>>(
            reinterpret_cast<const char*>(data_.data()) + HEADER_LEN + offsets[Items], formats[Items].size, formatMode.isBigEndian())...);

            return unpacked;
        }

        void calcCheckSum_(uint8_t* sum) const
        {
            // Process packet data
            for (auto it = data_.begin() + HEADER_LEN; it < data_.end() - TRAILER_LEN; it++)
            {
                sum[0] = (sum[0] + *it) % 255;
                sum[1] = (sum[1] + sum[0]) % 255;
            }

        }

    };
    
}