#include <array>
#include <deque>

#include <electrical_protocol_cpp/driver.h>

namespace electrical_protocol
{
    SerialDevice::SerialDevice(const std::string& deviceName, speed_t baudrate)
    {
        this->open(deviceName, baudrate);
    }

    SerialDevice::SerialDevice()
    {
        
    }

    SerialDevice::~SerialDevice()
    {
        close();
    }

    int SerialDevice::open(const std::string& deviceName, speed_t baudrate)
    {
        int error = 0;
        if(opened_)
            goto ret;

        serialFd_ = ::open(deviceName.c_str(), O_RDWR | O_NOCTTY);
        if(serialFd_ == -1)
        {
            error = errno;
            goto ret;
        }

        struct termios options;
        if (tcgetattr(serialFd_, &options) != -1)
        {
            cfmakeraw(&options);
            cfsetispeed(&options, baudrate);
            cfsetospeed(&options, baudrate);
            options.c_cc[VMIN] = 0;
            options.c_cc[VTIME] = 1;
            tcsetattr(serialFd_, TCSANOW, &options);
        }


        error = pthread_create(&writeThread_, NULL, writeThreadFunc_, this);
        if(error < 0)
        {
            goto closeFd;
        }

        error = pthread_create(&readThread_, NULL, readThreadFunc_, this);
        if(error < 0)
        {
            goto cancelThread;
        }
        
        opened_ = true;
        goto ret;

    cancelThread:
        pthread_cancel(writeThread_);
        pthread_join(writeThread_, NULL);
    closeFd:
        ::close(serialFd_);
    ret:
        return error;
    }

    void SerialDevice::close()
    {
        if(!opened_)
            return;
        
        if(pthread_cancel(writeThread_) == 0)
        {
            pthread_join(writeThread_, NULL);
        }

        if(pthread_cancel(readThread_) == 0)
        {
            pthread_join(readThread_, NULL);
        }

        ::close(serialFd_);

        opened_ = false;
    }

    bool SerialDevice::isOpened() const
    {
        return opened_;
    }

    void SerialDevice::write(Packet&& packet)
    {   
        pthread_mutex_lock(&writeMutex_);
        writeQueue_.push(std::move(packet));
        pthread_cond_signal(&writeCond_);
        pthread_mutex_unlock(&writeMutex_);
    }

    void SerialDevice::read(Packet&& packet)
    {
        pthread_mutex_lock(&readMutex_);
        readQueues_[packet.getId()].push(std::move(packet));
        pthread_mutex_unlock(&readMutex_);
    }

    void SerialDevice::writeThreadCleanupFunc_(void* arg)
    {
        // SerialDevice* device = reinterpret_cast<SerialDevice*>(arg);
        // pthread_mutex_lock(&device->writeMutex_);

        // while(device->writeQueue_.size() > 0)
        // {
        //     std::shared_ptr<Packet> packet = device->writeQueue_.front();
        //     device->onWrite(packet, EINTR, 0);
        //     device->writeQueue_.pop();
        // }
        // pthread_mutex_unlock(&device->writeMutex_);
    }

    void* SerialDevice::writeThreadFunc_(void* arg)
    {
        SerialDevice* device = reinterpret_cast<SerialDevice*>(arg);
        pthread_cleanup_push(writeThreadCleanupFunc_, device);

        while (1)
        {
            pthread_mutex_lock(&device->writeMutex_);
            while (device->writeQueue_.size() == 0) 
            {
                pthread_cond_wait(&device->writeCond_, &device->writeMutex_);
            }

            Packet packet = std::move(device->writeQueue_.front());
            device->writeQueue_.pop();

            pthread_mutex_unlock(&device->writeMutex_);

            size_t bytesToWrite = packet.data_.size();
            size_t bytesWritten = 0;
            int ret = 0;
            while(bytesWritten < bytesToWrite && ret != -1)
            {
                bytesWritten += ret;
                ret = ::write(device->serialFd_, packet.data_.data() + bytesWritten, bytesToWrite - bytesWritten);
            }
            
            size_t dataLen = bytesToWrite - Packet::HEADER_LEN - Packet::TRAILER_LEN;
            device->onWrite(std::move(packet), errno, bytesWritten);

            pthread_testcancel();
        }
        pthread_cleanup_pop(1);
        return NULL;
    
    }

    void SerialDevice::readThreadCleanupFunc_(void* arg)
    {
        // SerialDevice* device = reinterpret_cast<SerialDevice*>(arg);
        // pthread_mutex_lock(&device->readMutex_);
        // while(device->readQueue_.size() > 0)
        // {
        //     std::shared_ptr<Packet> packet = device->readQueue_.front();
        //     device->onRead(packet, EINTR, 0);
        //     device->readQueue_.pop();
        // }

        // pthread_mutex_unlock(&device->readMutex_);
    }

    void* SerialDevice::readThreadFunc_(void* arg)
    {
        SerialDevice* device = reinterpret_cast<SerialDevice*>(arg);
        pthread_cleanup_push(readThreadCleanupFunc_, device);
        ReadState state = ReadState::SYNC_1;

        size_t bytesRead = 0;
        Packet packet;

        while(1)
        {
            if(state == ReadState::SYNC_1)
            {
                uint8_t header = 0;
                int ret = ::read(device->serialFd_,&header,1);
                if(ret == -1)
                {
                    state = ReadState::CALLBACK;
                }
                else if(ret == 1 && header == SYNC_CHAR_1)
                {
                    bytesRead = 1;
                    state = ReadState::SYNC_2;
                }
            }
            else if(state == ReadState::SYNC_2)
            {
                uint8_t header = 0;
                int ret = ::read(device->serialFd_,&header,1);
                if(ret == -1)
                {
                    state = ReadState::CALLBACK;
                }
                else if(ret == 1)
                {
                    if(header == SYNC_CHAR_2)
                    {
                        bytesRead = 2;
                        state = ReadState::DATA;
                    }
                    else
                    {
                        state = ReadState::SYNC_1;
                    }
                }
            }
            else if(state == ReadState::DATA)
            {
                uint8_t idndataLen[4];

                size_t bytesToRead = Packet::HEADER_LEN;
                int ret = 0;
                while(bytesRead < bytesToRead && ret != -1)
                {
                    bytesRead += ret;
                    ret = ::read(device->serialFd_, idndataLen + (bytesRead - Packet::SYNC_LEN), bytesToRead - bytesRead);
                }

                if(ret == -1)
                {
                    state = ReadState::CALLBACK;
                    continue;
                }

                bool findPacket = false;
                pthread_mutex_lock(&device->readMutex_);
                auto it = device->readQueues_.find({idndataLen[0], idndataLen[1]});
                if(it != device->readQueues_.end() && it->second.size() > 0)
                {
                    packet = std::move(it->second.front());
                    it->second.pop();
                    findPacket = true;
                }
                pthread_mutex_unlock(&device->readMutex_);

                size_t dataLen = *reinterpret_cast<uint16_t*>(&idndataLen[2]);
                bytesToRead += (dataLen + Packet::TRAILER_LEN);

                if(!findPacket)
                {
                    constexpr size_t bufferLen = 100;
                    uint8_t buffer[bufferLen];
                    
                    ret = 0;
                    while(bytesRead < bytesToRead && ret != -1)
                    {
                        bytesRead += ret;
                        ret = ::read(device->serialFd_, buffer + (bytesRead - Packet::HEADER_LEN) % bufferLen, std::min(bufferLen, bytesToRead - bytesRead));
                    }

                    bytesRead = 0;
                    errno = 0;
                    state = ReadState::SYNC_1;
                }
                else
                {
                    packet.data_.resize(Packet::HEADER_LEN + dataLen + Packet::TRAILER_LEN);
                    *reinterpret_cast<uint16_t*>(&packet.data_[4]) = dataLen;

                    ret = 0;
                    while(bytesRead < bytesToRead && ret != -1)
                    {
                        bytesRead += ret;
                        ret = ::read(device->serialFd_, packet.data_.data() + bytesRead, bytesToRead - bytesRead);
                    }

                    state = ReadState::CALLBACK;
                }

            }
            else if(state == ReadState::CALLBACK)
            {
                device->onRead(std::move(packet), errno, bytesRead);
                bytesRead = 0;
                errno = 0;
                state = ReadState::SYNC_1;
            }

            pthread_testcancel();
        }

        pthread_cleanup_pop(1);

        return NULL;
    }

    // void SerialDevice::onRead(std::shared_ptr<Packet> packet, int errorCode ,size_t bytesRead)
    // {
    //     return;
    // }

    // void SerialDevice::onWrite(std::shared_ptr<Packet> packet, int errorCode ,size_t bytesWritten)
    // {
    //     return;
    // }

    // SerialTransfer::SerialTransfer(std::string& portName, unsigned baudrate=9600):
    //     portName_(portName),
    //     baudrate_(baudrate)
    // {
    //     transferThread_;
    //     int ret = pthread_create(&transferThread_, NULL, transferThreadFunc_, this);
    //     if(ret < 0)
    //     {
    //         throw std::runtime_error("Failed to create transfer thread for serial port" + std::string(strerror(ret)));
    //     }
    // }

    // SerialTransfer::~SerialTransfer()
    // {
    //     if(pthread_cancel(transferThread_) == 0)
    //     {
    //         pthread_join(transferThread_, NULL);
    //     }
    // }

    // void* SerialTransfer::transferThreadFunc_(void* arg)
    // {
    //     SerialTransfer* transfer = reinterpret_cast<SerialTransfer*>(arg);
    //     enum class State
    //     {
    //         CLOSED,
    //         OPENED
    //     };

    //     State state = State::CLOSED;
    //     while(1)
    //     {
    //         if(state == State::CLOSED)
    //         {
    //             transfer->open(transfer->portName_, transfer->baudrate_);
    //             if(transfer->isOpened())
    //                 state == State::OPENED;
    //         }
    //         else if(state == State::OPENED)
    //         {

    //         }
    //     }
    // }

} 



