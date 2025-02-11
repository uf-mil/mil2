#include <electrical_protocol_cpp/driver.h>

#include <gtest/gtest.h>
#include <pthread.h>

class DualSerial
{
    public:
    DualSerial()
    {

    }
    ~DualSerial()
    {
        close();
    }

    int open(std::string& slave1Name, std::string& slave2Name)
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
        
        if(pthread_create(&readThread_, NULL, readThreadFunc_,this) < 0)
            goto close2;

        if(pthread_create(&writeThread_, NULL, writeThreadFunc_, this) < 0)
            goto delThread;
        
        slave1Name.assign(slave1);
        slave2Name.assign(slave2);
        
        goto ret;
    delThread:
        pthread_cancel(readThread_);
        pthread_join(readThread_, NULL);
    close2:
        ::close(master2Fd_);
    close1:
        ::close(master1Fd_);
    ret:
        return errno;
    }

    void close()
    {
        if(pthread_cancel(readThread_) == 0)
            pthread_join(readThread_, NULL);

        if(pthread_cancel(writeThread_) == 0)
            pthread_join(writeThread_, NULL);

        ::close(master1Fd_);
        ::close(master2Fd_);
    }

    private:

    int master1Fd_;
    int master2Fd_;
    pthread_t readThread_;
    pthread_t writeThread_;

    static void doTransfer(int readFd, int writeFd)
    {
        uint8_t buffer[128];
        struct pollfd pFds[2];
        pFds[0].fd = ;
        pFds[0].fd = ;
        pFds[1].fd = ;

        struct iovec readVec[2];
        struct iovec writeVec[2];
        while(1)
        {
            int bytesRead = read(fd1, buffer, 12);
            if(bytesRead < 0)
            {
                //Handle the error
                pthread_exit((void*)-1);
            }

            int bytesToWrite = bytesRead; 
            do
            {
                int bytesWritten = write(fd2, );
                if(bytesWritten == -1)
                {
                    pthread_exit((void*)-1);
                }
                bytesToWrite -= bytesWritten;
            }while(bytesToWrite > 0);
            

        }
    }

    static void* readThreadFunc_(void* arg)
    {
        DualSerial* serial = reinterpret_cast<DualSerial*>(arg);
        serial->doTransfer(serial->master1Fd_, serial->master2Fd_);
    }

    static void* writeThreadFunc_(void* arg)
    {
        DualSerial* serial = reinterpret_cast<DualSerial*>(arg);
        serial->doTransfer(serial->master2Fd_, serial->master1Fd_);
    }
};


int main(int argc, char **argv)
{

    


    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

