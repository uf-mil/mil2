#include <unistd.h>
#include <sys/uio.h>
#include <errno.h>

#include <array>
#include <cstdint>

namespace mil_tools
{
template<size_t bufferSize>
class RingBuffer
{
    public:
    RingBuffer()
    {

    }
    ~RingBuffer()
    {

    }

    inline size_t getFreeSpace()
    {
        size_t realReadPtr = readPtr_ % bufferSize;
        size_t realWritePtr = writePtr_ % bufferSize;

        size_t freeSpace = (realWritePtr - realReadPtr + bufferSize) % bufferSize;

        // If realReadPtr - realWritePtr, the buffer may be empty or full.
        if(freeSpace == 0)
        {
            if((readPtr_ < bufferSize && writePtr_ < bufferSize) || (readPtr_ >= bufferSize && writePtr_ >= bufferSize))
            {
                freeSpace = bufferSize;
            }
        }

        return freeSpace;
    }

    // inline size_t getFreeWriteSpace()
    // {
    //     size_t realReadPtr = readPtr_ % bufferSize;
    //     size_t realWritePtr = writePtr_ % bufferSize;

    //     size_t freeWriteSpace = (realReadPtr - realWritePtr + bufferSize) % bufferSize;

    //     // If realReadPtr - realWritePtr, the buffer may be empty or full.
    //     if(freeWriteSpace == 0)
    //     {
    //         if((readPtr_ < bufferSize && writePtr_ >= bufferSize) || (readPtr_ >= bufferSize && writePtr_ < bufferSize))
    //         {
    //             freeWriteSpace = bufferSize;
    //         }
    //     }

    //     return freeWriteSpace;
    // }

    int readFrom(int readFd, int bytesToRead)
    {
        size_t realReadPtr = readPtr_ % bufferSize;
        size_t realWritePtr = writePtr_ % bufferSize;

        size_t totalCanRead = getFreeSpace();

        if(totalCanRead == 0)
        {
            errno = ENOBUFS;
            return -1;
        }

        size_t readRegionEnd;

        if(bytesToRead == -1 || bytesToRead >= static_cast<int>(totalCanRead))
        {
            readRegionEnd = realWritePtr;
        }
        else
        {
            readRegionEnd = (realReadPtr + bytesToRead) % bufferSize;
        }

        struct iovec iov[2];

        if(readRegionEnd > realReadPtr)
        {
            iov[0].iov_base = &buffer_[realReadPtr];
            iov[0].iov_len = readRegionEnd - realReadPtr;
            iov[1].iov_base = nullptr;
            iov[1].iov_len = 0;
        }
        else
        {
            iov[0].iov_base = &buffer_[realReadPtr];
            iov[0].iov_len = bufferSize - realReadPtr;
            iov[1].iov_base = &buffer_[0];
            iov[1].iov_len = readRegionEnd;
        }

        int bytesRead = readv(readFd, iov, 2);

        if(bytesRead == -1)
            goto ret;
        
        readPtr_ = (readPtr_ + bytesRead) % (2 * bufferSize);

    ret:

        return bytesRead;
    }

    int writeTo(int writeFd, int bytesToWrite)
    {
        if(bytesToWrite == 0)
        {
            return 0;
        }

        size_t realWritePtr = writePtr_ % bufferSize;
        size_t realReadPtr = readPtr_ % bufferSize;

        size_t totalToWrite = bufferSize - getFreeSpace();

        if(totalToWrite == 0)
        {
            return 0;
        }

        size_t writeRegionEnd;

        if(bytesToWrite == -1 || bytesToWrite >= static_cast<int>(totalToWrite))
        {
            writeRegionEnd = realReadPtr;
        }
        else
        {
            writeRegionEnd = (realWritePtr + bytesToWrite) % bufferSize;
        }

        struct iovec iov[2];

        if(writeRegionEnd > realWritePtr)
        {
            iov[0].iov_base = &buffer_[realWritePtr];
            iov[0].iov_len = writeRegionEnd - realWritePtr;
            iov[1].iov_base = nullptr;
            iov[1].iov_len = 0;
        }
        else
        {
            iov[0].iov_base = &buffer_[realWritePtr];
            iov[0].iov_len = bufferSize - realWritePtr;
            iov[1].iov_base = &buffer_[0];
            iov[1].iov_len = writeRegionEnd;
        }

        int bytesWritten = writev(writeFd, iov, 2);

        if(bytesWritten == -1)
            goto ret;
        
        writePtr_ = (writePtr_ + bytesWritten) % (2 * bufferSize);

    ret:

        return bytesWritten;
    }

    private:
    std::array<uint8_t, bufferSize> buffer_;
    size_t readPtr_ = 0;
    size_t writePtr_ = 0;
};


} // namespace mil_tools

