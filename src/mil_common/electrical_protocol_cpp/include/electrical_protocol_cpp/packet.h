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
        Packet() = delete;
        ~Packet() noexcept
        {

        }
        
        Packet(uint8_t classId, uint8_t subClassId) noexcept :id_({classId, subClassId})
        {
        }

        explicit Packet(const std::pair<uint8_t, uint8_t>& id) noexcept :id_(id) 
        {
        }

        Packet(Packet&& packet) noexcept : id_(packet.id_)
        {
            data_ = std::move(packet.data_);
        }

        Packet(const Packet& packet) noexcept : id_(packet.id_)
        {
            data_ = packet.data_;
        }

        inline Packet& operator= (Packet&& packet)
        {
            if(id_ != packet.id_)
                throw std::runtime_error("= between two packets with different id is impossiable");
            
            data_ = std::move(packet.data_);
            return *this;
        }

        inline Packet& operator= (Packet& packet)
        {
            if(id_ != packet.id_)
                throw std::runtime_error("= between two packets with different id is impossiable");
            
            data_ = packet.data_;
            
            return *this;
        }

        inline uint8_t& operator[] (size_t index)
        {
            if(index + HEADER_LEN + TRAILER_LEN >= data_.size())
                throw std::out_of_range("No enough data");

            return data_[index + HEADER_LEN];
        }

        inline const uint8_t& operator[] (size_t index) const
        {
            if(index + HEADER_LEN + TRAILER_LEN >= data_.size())
                throw std::out_of_range("No enough data");

            return data_[index + HEADER_LEN];
        }

        struct IdHash
        {
            size_t operator()(const std::pair<uint8_t, uint8_t>& p) const
            {
                return static_cast<size_t>((static_cast<uint16_t>(p.first) << 8) + p.second);
            }
        };

        inline const std::pair<uint8_t, uint8_t>& id() const noexcept
        {
            return id_;
        }

        inline size_t size() const noexcept
        {
            if(data_.size() < (HEADER_LEN + TRAILER_LEN))
                return 0;

            return data_.size() - HEADER_LEN - TRAILER_LEN;
        }

        inline void resize(size_t size) noexcept
        {
            size_t orignalSize = data_.size();
            data_.resize(size + HEADER_LEN + TRAILER_LEN);
            if(orignalSize == 0)
            {
                data_[0] = SYNC_CHAR_1;
                data_[1] = SYNC_CHAR_2;
                data_[2] = id_.first;
                data_[3] = id_.second;
            }
        }
        
        template <typename Fmt, typename... Args>
        void pack(Fmt, Args&&... args)
        {
            resize(pystruct::calcsize(Fmt{}));

            constexpr size_t itemCount = pystruct::countItems(Fmt{});
            pack_<Fmt>(std::make_index_sequence<itemCount>(), std::forward<Args>(args)...);
        }

        template<typename Fmt>
        auto unpack(Fmt) const
        {
            constexpr size_t dataLen = pystruct::calcsize(Fmt{});
            if(dataLen + HEADER_LEN + TRAILER_LEN > data_.size())
                throw std::out_of_range("No enough data: expect " + std::to_string(dataLen)
                                        + " has " + std::to_string(size()));

            return unpack_<Fmt>(std::make_index_sequence<countItems(Fmt{})>());
        }

        private:
        static constexpr uint8_t SYNC_CHAR_1 = 0x37;
        static constexpr uint8_t SYNC_CHAR_2 = 0x01;
        static constexpr size_t HEADER_LEN = 6;
        static constexpr size_t TRAILER_LEN = 2;
        static constexpr size_t SYNC_LEN = 2;
        std::vector<uint8_t> data_;
        const std::pair<uint8_t, uint8_t> id_;
        
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