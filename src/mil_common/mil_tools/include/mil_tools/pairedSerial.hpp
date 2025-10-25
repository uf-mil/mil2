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

    int open(std::string& s1Name, std::string& s2Name)
    {
        char* s1;
        char* s2;
        int s1Fd;
        int s2Fd;

        m1Fd_ = posix_openpt(O_RDWR | O_NOCTTY);
        if (m1Fd_ == -1)
            goto ret;

        if (grantpt(m1Fd_) < 0 || unlockpt(m1Fd_) < 0)
            goto close1;

        s1 = ptsname(m1Fd_);
        if (s1 == nullptr)
            goto close1;

        s1Fd = ::open(s1, O_RDWR | O_NOCTTY, 0620);
        if (s1Fd == -1)
            goto close1;

        ::close(s1Fd);

        s1Name.assign(s1);

        m2Fd_ = posix_openpt(O_RDWR | O_NOCTTY);
        if (m2Fd_ == -1)
            goto close1;

        if (grantpt(m2Fd_) < 0 || unlockpt(m2Fd_) < 0)
            goto close2;

        s2 = ptsname(m2Fd_);
        if (s2 == nullptr)
            goto close2;

        s2Fd = ::open(s2, O_RDWR | O_NOCTTY, 0620);
        if (s2Fd == -1)
            goto close2;

        ::close(s2Fd);

        s2Name.assign(s2);

        if (pthread_create(&workThread1_, NULL, workThreadFunc1_, this) < 0)
            goto close2;

        if (pthread_create(&workThread2_, NULL, workThreadFunc2_, this) < 0)
            goto cancelThread;

        std::cout << "S pty 1 name: " << s1Name << std::endl;
        std::cout << "S pty 2 name: " << s2Name << std::endl;
        goto ret;

    cancelThread:
        pthread_cancel(workThread1_);
        pthread_join(workThread1_, NULL);
    close2:
        ::close(m2Fd_);
    close1:
        ::close(m1Fd_);
    ret:
        return errno;
    }

    void close()
    {
        pthread_cancel(workThread1_);
        pthread_join(workThread1_, NULL);

        pthread_cancel(workThread2_);
        pthread_join(workThread2_, NULL);

        ::close(m1Fd_);
        ::close(m2Fd_);
    }

  private:
    int m1Fd_;
    int m2Fd_;
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

    while (serial->waitConnection(serial->m1Fd_) == -1)
        ;

    std::cout << "S pty 1 connected" << std::endl;

    while (1)
    {
        int bytesWritten = serial->doTransfer(buffer, serial->m1Fd_, serial->m2Fd_);
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

    while (serial->waitConnection(serial->m2Fd_) == -1)
        ;

    std::cout << "S pty 2 connected" << std::endl;

    while (1)
    {
        int bytesWritten = serial->doTransfer(buffer, serial->m2Fd_, serial->m1Fd_);
        if (bytesWritten > 0)
            std::cout << "Transferred " << bytesWritten << " bytes from serial 2 to serial 1" << std::endl;
        pthread_testcancel();
    }
    return 0;
}
}  // namespace mil_tools
