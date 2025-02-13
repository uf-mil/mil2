#include <array>
#include <deque>

#include <electrical_protocol_cpp/driver.h>

namespace electrical_protocol
{
    SerialDevice::SerialDevice(const std::string& deviceName)
    {
        this->open(deviceName);
    }

    SerialDevice::SerialDevice()
    {
        
    }

    SerialDevice::~SerialDevice()
    {
        close();
    }

    int SerialDevice::setBaudrate(unsigned baudrate) 
    {
        speed_t speed;
        switch (baudrate)
        {
        case 1200: speed = B1200; break;
        case 2400: speed = B2400; break;
        case 4800: speed = B4800; break;
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default: return EINVAL;
        }
        
        struct termios options;
        if (tcgetattr(serialFd_, &options) < 0)
        {
            return errno;
        }
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);
        if (tcsetattr(serialFd_, TCSANOW, &options) < 0) 
        {
            return errno;
        }
        return 0;
    }

    int SerialDevice::open(const std::string& deviceName)
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
            options.c_cflag |= (CLOCAL | CREAD);
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS8;
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            options.c_iflag &= ~(IXON | IXOFF | IXANY);
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            options.c_oflag &= ~OPOST;
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

    void SerialDevice::write(std::shared_ptr<Packet> packet)
    {
        if(packet == nullptr || packet->data_.size() == 0)
            onWrite(packet, EINVAL, 0);
        
        pthread_mutex_lock(&writeMutex_);
        writeQueue_.push(packet);
        pthread_cond_signal(&writeCond_);
        pthread_mutex_unlock(&writeMutex_);
    }

    void SerialDevice::read(std::shared_ptr<Packet> packet)
    {
        if(packet == nullptr || packet->data_.size() == 0)
            onRead(packet, EINVAL, 0);
        
        pthread_mutex_lock(&readMutex_);
        readQueue_.push(packet);
        pthread_mutex_unlock(&readMutex_);
    }

    void SerialDevice::writeThreadCleanupFunc_(void* arg)
    {
        SerialDevice* device = reinterpret_cast<SerialDevice*>(arg);
        pthread_mutex_lock(&device->writeMutex_);

        while(device->writeQueue_.size() > 0)
        {
            std::shared_ptr<Packet> packet = device->writeQueue_.front();
            device->onWrite(packet, EINTR, 0);
            device->writeQueue_.pop();
        }
        pthread_mutex_unlock(&device->writeMutex_);
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
            
            pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
            std::cout << "Write lock acquired" << std::endl;

            std::shared_ptr<Packet> packet = device->writeQueue_.front();
            device->writeQueue_.pop();

            pthread_mutex_unlock(&device->writeMutex_);
            
            std::cout << "Write lock released" << std::endl;
            pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

            uint16_t checkSum = device->calcCheckSum_(packet);
    
            uint16_t dataLen = packet->data_.size();
    
            struct iovec iov[5];
            iov[0].iov_base = const_cast<uint8_t*>(header_.data());
            iov[0].iov_len = header_.size();
    
            iov[1].iov_base = &packet->id_;
            iov[1].iov_len = sizeof(packet->id_);
    
            iov[2].iov_base = &dataLen;
            iov[2].iov_len = sizeof(dataLen);
    
            iov[3].iov_base = packet->data_.data();
            iov[3].iov_len = dataLen;
    
            iov[4].iov_base = &checkSum;
            iov[4].iov_len = sizeof(checkSum);
    
            ssize_t bytesWritten = writev(device->serialFd_, iov, 3);
            if(bytesWritten < 0)
                bytesWritten = 0;
            device->onWrite(packet, errno, bytesWritten);

            pthread_testcancel();
        }
        pthread_cleanup_pop(1);
        return NULL;
    
    }

    void SerialDevice::readPacket_(std::shared_ptr<Packet> packet)
    {
        ReadState state = ReadState::SYNC_1;
        int error = 0;
        size_t bytesRead = 0;

        while(1)
        {
            if(state == ReadState::SYNC_1)
            {
                uint8_t header = 0;
                if(::read(serialFd_,&header,1) != -1)
                {
                    error = errno;
                    break;
                }
                
                if(header == SYNC_CHAR_1)
                {
                    state = ReadState::SYNC_2;
                }
            }
            else if(state == ReadState::SYNC_2)
            {
                uint8_t header = 0;
                if(::read(serialFd_,&header,1) == -1)
                {
                    error = errno;
                    break;
                }

                if(header == SYNC_CHAR_2)
                {
                    state = ReadState::DATA;
                }
                else
                {
                    state = ReadState::SYNC_1;
                }
            }
            else if(state == ReadState::DATA)
            {
                uint16_t dataLen = 0;

                struct iovec iov[2];
                iov[0].iov_base = &packet->id_;
                iov[0].iov_len = sizeof(packet->id_);
                iov[1].iov_base = &dataLen;
                iov[1].iov_len = sizeof(dataLen);

                if(::readv(serialFd_, iov, 2) == -1)
                {
                    error = errno;
                    break;
                }

                packet->data_.resize(dataLen);

                uint16_t checkSum;
                iov[0].iov_base = packet->data_.data();
                iov[0].iov_len = dataLen;
                iov[1].iov_base = &checkSum;
                iov[1].iov_len = sizeof(checkSum);

                int ret = readv(serialFd_, iov, 2);
                if(ret == -1)
                {
                    error = errno;
                    break;
                }

                bytesRead = ret;

                if(checkSum != calcCheckSum_(packet))
                {
                    error = EBADMSG;
                }

                break;
            }
        }
        
        onRead(packet, error, bytesRead);
    }

    void SerialDevice::readPacket_()
    {
        ReadState state = ReadState::SYNC_1;

        while(1)
        {
            if(state == ReadState::SYNC_1)
            {
                uint8_t header = 0;
                if(::read(serialFd_,&header,1) != -1)
                {
                    break;
                }
                
                if(header == SYNC_CHAR_1)
                {
                    state = ReadState::SYNC_2;
                }
            }
            else if(state == ReadState::SYNC_2)
            {
                uint8_t header = 0;
                if(::read(serialFd_,&header,1) == -1)
                {
                    break;
                }

                if(header == SYNC_CHAR_2)
                {
                    state = ReadState::DATA;
                }
                else
                {
                    state = ReadState::SYNC_1;
                }
            }
            else if(state == ReadState::DATA)
            {
                uint16_t dataLen = 0;
                uint16_t id = 0;
                struct iovec iov[2];
                iov[0].iov_base = &id;
                iov[0].iov_len = sizeof(id);
                iov[1].iov_base = &dataLen;
                iov[1].iov_len = sizeof(dataLen);

                if(::readv(serialFd_, iov, 2) == -1)
                {
                    break;
                }

                std::vector<uint8_t> data(dataLen);

                uint16_t checkSum;
                iov[0].iov_base = data.data();
                iov[0].iov_len = dataLen;
                iov[1].iov_base = &checkSum;
                iov[1].iov_len = sizeof(checkSum);

                int ret = readv(serialFd_, iov, 2);
                if(ret == -1)
                {
                    break;
                }

                break;
            }
        }
    }

    void SerialDevice::readThreadCleanupFunc_(void* arg)
    {
        SerialDevice* device = reinterpret_cast<SerialDevice*>(arg);
        pthread_mutex_lock(&device->readMutex_);
        while(device->readQueue_.size() > 0)
        {
            std::shared_ptr<Packet> packet = device->readQueue_.front();
            device->onRead(packet, EINTR, 0);
            device->readQueue_.pop();
        }

        pthread_mutex_unlock(&device->readMutex_);
    }

    void* SerialDevice::readThreadFunc_(void* arg)
    {
        SerialDevice* device = reinterpret_cast<SerialDevice*>(arg);
        pthread_cleanup_push(readThreadCleanupFunc_, device);

        while(1)
        {
            std::shared_ptr<Packet> packet;
            pthread_mutex_lock(&device->readMutex_);

            if(device->readQueue_.size() > 0)
            {
                packet = device->readQueue_.front();
                device->readQueue_.pop();
            }
            
            pthread_mutex_unlock(&device->readMutex_);
            
            if(packet == nullptr)
            {
                device->readPacket_();
            }
            else
            {
                device->readPacket_(packet);
            }

            pthread_testcancel();
        }

        pthread_cleanup_pop(1);

        return NULL;
    }

    uint16_t SerialDevice::calcCheckSum_(std::shared_ptr<Packet> packet)
    {
        uint8_t sum[2] = {0, 0};

        // Process packet ID (uint16_t, little-endian)
        sum[0] = (sum[0] + (packet->id_ & 0xFF)) % 255;
        sum[1] = (sum[1] + sum[0]) % 255;
        sum[0] = (sum[0] + ((packet->id_ >> 8) & 0xFF)) % 255;
        sum[1] = (sum[1] + sum[0]) % 255;

        // Process packet size (uint16_t, little-endian)
        uint16_t size = packet->data_.size();
        sum[0] = (sum[0] + (size & 0xFF)) % 255;
        sum[1] = (sum[1] + sum[0]) % 255;
        sum[0] = (sum[0] + ((size >> 8) & 0xFF)) % 255;
        sum[1] = (sum[1] + sum[0]) % 255;

        // Process packet data
        for (auto byte: packet->data_)
        {
            sum[0] = (sum[0] + byte) % 255;
            sum[1] = (sum[1] + sum[0]) % 255;
        }

        // Return the checksum as a uint16_t (little-endian)
        return (sum[1] << 8) | sum[0];
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



