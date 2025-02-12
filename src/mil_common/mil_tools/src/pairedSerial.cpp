#include <mil_tools/pairedSerial.hpp>

#include <poll.h>
#include <unistd.h>
#include <sys/uio.h>
#include <fcntl.h>

#include <iostream>

namespace mil_tools
{
    PairedSerial::PairedSerial()
    {
    }

    PairedSerial::~PairedSerial()
    {

    }
    
    int PairedSerial::open(std::string& slave1Name, std::string& slave2Name)
    {
        char* slave1;
        char* slave2;

        master1Fd_ = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        if(master1Fd_ == -1)
            goto ret;

        if (grantpt(master1Fd_) < 0 || unlockpt(master1Fd_) < 0) 
            goto close1;

        slave1 = ptsname(master1Fd_);
        if(slave1 == nullptr)
            goto close1;


        master2Fd_ = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        if(master2Fd_ == -1)
            goto close1;

        if (grantpt(master2Fd_) < 0 || unlockpt(master2Fd_) < 0) 
            goto close2;

        slave2 = ptsname(master2Fd_);
        if(slave2 == nullptr)
            goto close2;
        
        if(pthread_create(&workThread_, NULL, workThreadFunc_,this) < 0)
            goto close2;
        
        slave1Name.assign(slave1);
        slave2Name.assign(slave2);
        
        goto ret;

    close2:
        ::close(master2Fd_);
    close1:
        ::close(master1Fd_);
    ret:
        return errno;
    }

    void PairedSerial::close()
    {
        if(pthread_cancel(workThread_) == 0)
            pthread_join(workThread_, NULL);

        ::close(master1Fd_);
        ::close(master2Fd_);
    }

    void* PairedSerial::workThreadFunc_(void* arg)
    {
        PairedSerial* serial = reinterpret_cast<PairedSerial*>(arg);
        
        RingBuffer<64> buffer1;
        RingBuffer<64> buffer2;

        struct pollfd fds[2];
        fds[0].fd = serial->master1Fd_;
        fds[0].events = POLLIN | POLLOUT;
        fds[1].fd = serial->master2Fd_; 
        fds[1].events = POLLIN | POLLOUT;

        while(1)
        {
            int ret = poll(fds, 2, 100);
            if(ret == -1 || ret == 0)
            {
                continue;
            }
            
            if(fds[0].revents & POLLIN)
            {
                buffer1.readFrom(fds[0].fd, -1);
            }
            if(fds[1].revents & POLLIN)
            {
                buffer2.readFrom(fds[1].fd, -1);
            }

            if(fds[0].revents & POLLOUT)
            {
                buffer2.writeTo(fds[0].fd, -1);
            }
            if(fds[1].revents & POLLOUT)
            {
                buffer1.writeTo(fds[1].fd, -1);
            }
            
            pthread_testcancel();
        }
        return 0;
    }
}
