#pragma once

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

#include <string>
#include <queue>
#include <array>
#include <unordered_map>
#include <iostream>

#include <unistd.h>
#include <termios.h>
#include <pthread.h>

#include <electrical_protocol_cpp/packet.h>

namespace electrical_protocol
{

class SerialDevice;
// class SerialTransfer;

class SerialDevice
{
    public:

    SerialDevice(const std::string& portName, unsigned baudrate=9600);
    SerialDevice();
    ~SerialDevice();

    int open(const std::string& portName, unsigned baudrate=9600);
    int setBaudrate(unsigned baudrate);
    void close();
    bool isOpened() const;
    void write(std::shared_ptr<PacketBase> packet);
    void read(std::shared_ptr<PacketBase> packet);

    virtual void onWrite(std::shared_ptr<PacketBase> package, int errorCode ,size_t bytesWritten) = 0;
    virtual void onRead(std::shared_ptr<PacketBase> package, int errorCode ,size_t bytesRead) = 0;

    private:

    enum class ReadState
    {
        SYNC_1,
        SYNC_2,
        ID,
        DATA,
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

    std::queue<std::shared_ptr<PacketBase>> writeQueue_;
    std::queue<std::shared_ptr<PacketBase>> readQueue_;
 
    static void* readThreadFunc_(void* arg);
    static void* writeThreadFunc_(void* arg);
    static void readThreadCleanupFunc_(void* arg);
    static void writeThreadCleanupFunc_(void* arg);

    void readPacket_(std::shared_ptr<PacketBase> packet);
    void readPacket_();

    uint16_t calcCheckSum_(std::shared_ptr<PacketBase> data);
    void writeData_(std::shared_ptr<PacketBase> data);
    void readData_(std::shared_ptr<PacketBase> data);
};

// class Subscriber
// {
//     public:
//     friend class SerialTransfer;
//     Subscriber();
//     ~Subscriber();
//     private:
// };

// class Publisher
// {
//     public:
//     friend class SerialTransfer;
//     Publisher();
//     ~Publisher();
//     private:
//     std::queue<std::shared_ptr<std::vector<uint8_t>>> dataQueue_;
// };

// class SerialTransfer: private SerialDevice
// {
//     public:
//     SerialTransfer() = delete;
//     SerialTransfer(std::string& portName, unsigned baudrate=9600);
//     ~SerialTransfer();

//     int advertise(Publisher& publisher, unsigned queueLen);
//     int subscribe(Subscriber& subscriber, unsigned queueLen);

//     private:
//     pthread_t transferThread_;

//     static void* transferThreadFunc_(void* arg);

//     unsigned baudrate_;
//     std::string portName_;
// };

}