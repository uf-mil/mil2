#include <mil_tools/ringBuffer.hpp>
#include <gtest/gtest.h>

#include <unistd.h>
#include <fcntl.h>

TEST(ringBuffer, fullRW)
{
    mil_tools::RingBuffer<128> buffer;

    int readFd = open("/dev/random", O_RDONLY);
    EXPECT_NE(readFd, -1);

    int writeFd = open("/dev/null", O_WRONLY);
    EXPECT_NE(writeFd, -1);
    
    int bytesRead = buffer.readFrom(readFd, -1);
    EXPECT_EQ(bytesRead, 128);

    int bytesWritten = buffer.writeTo(writeFd, -1);
    EXPECT_EQ(bytesWritten, bytesRead);  

    close(readFd);
    close(writeFd);
}

TEST(ringBuffer, overflowRW)
{
    mil_tools::RingBuffer<128> buffer;

    int readFd = open("/dev/random", O_RDONLY);
    EXPECT_NE(readFd, -1);

    int writeFd = open("/dev/null", O_WRONLY);
    EXPECT_NE(writeFd, -1);
    
    int bytesRead = buffer.readFrom(readFd, 200);
    EXPECT_EQ(bytesRead, 128);

    int bytesWritten = buffer.writeTo(writeFd, 200);
    EXPECT_EQ(bytesWritten, bytesRead);  

    close(readFd);
    close(writeFd);
}

TEST(ringBuffer, multiRW)
{
    mil_tools::RingBuffer<128> buffer;

    int readFd = open("/dev/random", O_RDONLY);
    EXPECT_NE(readFd, -1);

    int writeFd = open("/dev/null", O_WRONLY);
    EXPECT_NE(writeFd, -1);
    
    int bytesRead = buffer.readFrom(readFd, 80);
    EXPECT_EQ(bytesRead, 80);

    int bytesWritten = buffer.writeTo(writeFd, 60);
    EXPECT_EQ(bytesWritten, 60);
    
    bytesRead = buffer.readFrom(readFd, 100);
    EXPECT_EQ(bytesRead, 100);

    bytesWritten = buffer.writeTo(writeFd, 100);
    EXPECT_EQ(bytesWritten, 100);

    bytesWritten = buffer.writeTo(writeFd, 20);
    EXPECT_EQ(bytesWritten, 20);

    close(readFd);
    close(writeFd);
}
