#pragma once

#include <fcntl.h>
#include <pthread.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>

#include "electrical_protocol/packet.h"

namespace electrical_protocol
{

class SerialDevice
{
  public:
    SerialDevice(std::string const& deviceName, speed_t baudrate = B9600);
    SerialDevice();
    virtual ~SerialDevice();

    int open(std::string const& deviceName, speed_t baudrate);
    void close();
    bool isOpened() const;

    template <typename T>
    void write(T&& packet)
    {
        static_assert(std::is_same_v<std::remove_const_t<std::remove_reference_t<T>>, Packet>, "Only Packet is allowed "
                                                                                               "in "
                                                                                               "SerialDevice::write");

        if (packet.data_.size() == 0)
        {
            Packet cpacket(std::forward<T>(packet));
            onWrite(cpacket, EINVAL);
            return;
        }

        if (!opened_)
        {
            Packet cpacket(std::forward<T>(packet));
            onWrite(cpacket, EACCES);
            return;
        }

        pthread_mutex_lock(&writeMutex_);
        writeQueue_.push(std::forward<T>(packet));
        pthread_cond_signal(&writeCond_);
        pthread_mutex_unlock(&writeMutex_);
    }

    template <typename T>
    void read(T&& packet)
    {
        static_assert(std::is_same_v<std::remove_const_t<std::remove_reference_t<T>>, Packet>, "Only Packet is allowed "
                                                                                               "in "
                                                                                               "SerialDevice::read");

        if (!opened_)
        {
            onRead(packet, EACCES);
            return;
        }

        pthread_mutex_lock(&readMutex_);
        readQueues_[packet.id_].push(std::forward<T>(packet));
        pthread_mutex_unlock(&readMutex_);
    }

    virtual void onWrite(Packet& packet, int errorCode) = 0;
    virtual void onRead(Packet& packet, int errorCode) = 0;

  private:
    enum class ReadState
    {
        SYNC_1,
        SYNC_2,
        DATA,
        CALLBACK,
    };

    static constexpr uint8_t SYNC_CHAR_1 = 0x37;
    static constexpr uint8_t SYNC_CHAR_2 = 0x01;
    static constexpr std::array<uint8_t, 2> header_ = { SYNC_CHAR_1, SYNC_CHAR_2 };

    bool opened_ = false;

    int serialFd_ = -1;
    pthread_t readThread_;
    pthread_t writeThread_;

    pthread_cond_t writeCond_ = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t writeMutex_ = PTHREAD_MUTEX_INITIALIZER;

    pthread_mutex_t readMutex_ = PTHREAD_MUTEX_INITIALIZER;

    std::queue<Packet> writeQueue_;
    std::unordered_map<std::pair<uint8_t, uint8_t>, std::queue<Packet>, Packet::IdHash> readQueues_;

    static void* readThreadFunc_(void* arg);
    static void* writeThreadFunc_(void* arg);
    static void readThreadCleanupFunc_(void* arg);
    static void writeThreadCleanupFunc_(void* arg);

    void close_();
};

}  // namespace electrical_protocol
