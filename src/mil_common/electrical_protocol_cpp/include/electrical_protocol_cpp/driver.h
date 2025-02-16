#pragma once

#include <string>
#include <queue>
#include <array>
#include <unordered_map>
#include <iostream>

#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include <fcntl.h>

#include <electrical_protocol_cpp/packet.h>

namespace electrical_protocol
{

class SerialDevice;
// class SerialTransfer;

class SerialDevice
{
    public:

    SerialDevice(const std::string& deviceName, speed_t baudrate = B9600);
    SerialDevice();
    ~SerialDevice();

    int open(const std::string& deviceName, speed_t baudrate);
    void close();
    bool isOpened() const;
    void write(Packet&& packet);
    void read(Packet&& packet);

    virtual void onWrite(Packet&& packet, int errorCode) = 0;
    virtual void onRead(Packet&& packet, int errorCode) = 0;

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
    static constexpr std::array<uint8_t, 2> header_ = {SYNC_CHAR_1, SYNC_CHAR_2};

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
};

}