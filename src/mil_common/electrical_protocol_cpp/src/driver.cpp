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
        if(packet.data_.size() == 0)
        {
            onWrite(std::move(packet), EINVAL);
            return;
        }

        if(!opened_)
        {
            onWrite(std::move(packet), EACCES);
            return;
        }
         
        pthread_mutex_lock(&writeMutex_);
        writeQueue_.push(std::move(packet));
        pthread_cond_signal(&writeCond_);
        pthread_mutex_unlock(&writeMutex_);
    }

    void SerialDevice::read(Packet&& packet)
    {
        if(!opened_)
        {
            onRead(std::move(packet), EACCES);
            return;
        }

        pthread_mutex_lock(&readMutex_);
        readQueues_[packet.id_].push(std::move(packet));
        pthread_mutex_unlock(&readMutex_);
    }

    void SerialDevice::writeThreadCleanupFunc_(void* arg)
    {
        SerialDevice* device = reinterpret_cast<SerialDevice*>(arg);
        pthread_mutex_unlock(&device->writeMutex_);

        while(device->writeQueue_.size() > 0)
        {
            Packet packet = std::move(device->writeQueue_.front());
            device->writeQueue_.pop();
            device->onWrite(std::move(packet), EINTR);
        }
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

            *reinterpret_cast<uint16_t*>(&packet.data_[4]) = packet.size();
            packet.calcCheckSum_(&packet.data_[packet.data_.size() - Packet::TRAILER_LEN]);

            size_t bytesToWrite = packet.data_.size();
            size_t bytesWritten = 0;
            int ret = 0;
            while(bytesWritten < bytesToWrite && ret != -1)
            {
                bytesWritten += ret;
                ret = ::write(device->serialFd_, packet.data_.data() + bytesWritten, bytesToWrite - bytesWritten);
            }

            device->onWrite(std::move(packet), errno);

            pthread_testcancel();
        }
        pthread_cleanup_pop(1);
        return NULL;
    
    }

    void SerialDevice::readThreadCleanupFunc_(void* arg)
    {
        SerialDevice* device = reinterpret_cast<SerialDevice*>(arg);
        pthread_mutex_unlock(&device->readMutex_);
        for(auto& pair : device->readQueues_)
        {
            while(pair.second.size() > 0)
            {
                Packet packet = std::move(pair.second.front());
                pair.second.pop();
                device->onRead(std::move(packet), EINTR);
            }
        }
        device->readQueues_.clear();
    }

    void* SerialDevice::readThreadFunc_(void* arg)
    {
        SerialDevice* device = reinterpret_cast<SerialDevice*>(arg);
        pthread_cleanup_push(readThreadCleanupFunc_, device);
        ReadState state = ReadState::SYNC_1;

        size_t bytesRead = 0;

        while(1)
        {
            if(state == ReadState::SYNC_1)
            {
                uint8_t header = 0;
                int ret = ::read(device->serialFd_,&header,1);
                if(ret == -1)
                {
                    device->onRead(Packet(0,0), errno);
                    errno = 0;
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
                    device->onRead(Packet(0,0), errno);
                    errno = 0;
                    bytesRead = 0;
                    state = ReadState::SYNC_1;
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
                    device->onRead(Packet(0,0), errno);
                    errno = 0;
                    state = ReadState::SYNC_1;
                    continue;
                }

                Packet packet(idndataLen[0], idndataLen[1]);
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
                    packet.resize(dataLen);
                    *reinterpret_cast<uint16_t*>(&packet.data_[4]) = dataLen;

                    ret = 0;
                    while(bytesRead < bytesToRead && ret != -1)
                    {
                        bytesRead += ret;
                        ret = ::read(device->serialFd_, packet.data_.data() + bytesRead, bytesToRead - bytesRead);
                    }

                    uint8_t sum[2];
                    packet.calcCheckSum_(sum);
                    if(sum[0] != packet.data_[packet.data_.size()-2] || sum[1] != packet.data_[packet.data_.size()-1])
                        errno = EBADMSG;

                    device->onRead(std::move(packet), errno);
                    bytesRead = 0;
                    errno = 0;
                    state = ReadState::SYNC_1;
                }

            }

            pthread_testcancel();
        }

        pthread_cleanup_pop(1);

        return NULL;
    }

} 



