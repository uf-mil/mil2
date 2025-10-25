#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <unistd.h>

#include <iostream>
#include <string>

#include <mil_tools/ringBuffer.hpp>

namespace mil_tools
{
template <size_t BufferSize = 128>
class PairedSerial
{
  public:
    PairedSerial()
    {
    }
    ~PairedSerial()
    {
        close();
    }

    int open(std::string& slave1Name, std::string& slave2Name)
    {
        char* slave1;
        char* slave2;
        int slave1Fd;
        int slave2Fd;

        master1Fd_ = posix_openpt(O_RDWR | O_NOCTTY);
        if (master1Fd_ == -1)
            goto ret;

        if (grantpt(master1Fd_) < 0 || unlockpt(master1Fd_) < 0)
            goto close1;

        slave1 = ptsname(master1Fd_);
        if (slave1 == nullptr)
            goto close1;

        slave1Fd = ::open(slave1, O_RDWR | O_NOCTTY, 0620);
        if (slave1Fd == -1)
            goto close1;

        ::close(slave1Fd);

        slave1Name.assign(slave1);

        master2Fd_ = posix_openpt(O_RDWR | O_NOCTTY);
        if (master2Fd_ == -1)
            goto close1;

        if (grantpt(master2Fd_) < 0 || unlockpt(master2Fd_) < 0)
            goto close2;

        slave2 = ptsname(master2Fd_);
        if (slave2 == nullptr)
            goto close2;

        slave2Fd = ::open(slave2, O_RDWR | O_NOCTTY, 0620);
        if (slave2Fd == -1)
            goto close2;

        ::close(slave2Fd);

        slave2Name.assign(slave2);

        if (pthread_create(&workThread1_, NULL, workThreadFunc1_, this) < 0)
            goto close2;

        if (pthread_create(&workThread2_, NULL, workThreadFunc2_, this) < 0)
            goto cancelThread;

        std::cout << "Slave pty 1 name: " << slave1Name << std::endl;
        std::cout << "Slave pty 2 name: " << slave2Name << std::endl;
        goto ret;

    cancelThread:
        pthread_cancel(workThread1_);
        pthread_join(workThread1_, NULL);
    close2:
        ::close(master2Fd_);
    close1:
        ::close(master1Fd_);
    ret:
        return errno;
    }

    void close()
    {
        pthread_cancel(workThread1_);
        pthread_join(workThread1_, NULL);

        pthread_cancel(workThread2_);
        pthread_join(workThread2_, NULL);

        ::close(master1Fd_);
        ::close(master2Fd_);
    }

  private:
    int master1Fd_;
    int master2Fd_;
    pthread_t workThread1_;
    pthread_t workThread2_;

    static void* workThreadFunc1_(void* arg);
    static void* workThreadFunc2_(void* arg);

    int doTransfer(RingBuffer<BufferSize>& buffer, int inFd, int outFd)
    {
        int bytesRead = buffer.readFrom(inFd, -1);
        if (bytesRead <= 0)
            return bytesRead;

        return buffer.writeTo(outFd, bytesRead);
    }

    int waitConnection(int masterFd)
    {
        while (1)
        {
            struct pollfd pFd;
            pFd.fd = masterFd;
            pFd.events = POLLHUP;
            if (poll(&pFd, 1, 0) < 0)
            {
                return -1;
            }
            if (!(pFd.revents & POLLHUP))
            {
                break;
            }

            usleep(10000);
        }

        return 0;
    }
};

template <size_t BufferSize>
void* PairedSerial<BufferSize>::workThreadFunc1_(void* arg)
{
    PairedSerial* serial = reinterpret_cast<PairedSerial*>(arg);
    RingBuffer<BufferSize> buffer;

    while (serial->waitConnection(serial->master1Fd_) == -1)
        ;

    std::cout << "Slave pty 1 connected" << std::endl;

    while (1)
    {
        int bytesWritten = serial->doTransfer(buffer, serial->master1Fd_, serial->master2Fd_);
        if (bytesWritten > 0)
            std::cout << "Transferred " << bytesWritten << " bytes from serial 1 to serial 2" << std::endl;
        pthread_testcancel();
    }
    return 0;
}

template <size_t BufferSize>
void* PairedSerial<BufferSize>::workThreadFunc2_(void* arg)
{
    PairedSerial* serial = reinterpret_cast<PairedSerial*>(arg);
    RingBuffer<BufferSize> buffer;

    while (serial->waitConnection(serial->master2Fd_) == -1)
        ;

    std::cout << "Slave pty 2 connected" << std::endl;

    while (1)
    {
        int bytesWritten = serial->doTransfer(buffer, serial->master2Fd_, serial->master1Fd_);
        if (bytesWritten > 0)
            std::cout << "Transferred " << bytesWritten << " bytes from serial 2 to serial 1" << std::endl;
        pthread_testcancel();
    }
    return 0;
}
}  // namespace mil_tools
