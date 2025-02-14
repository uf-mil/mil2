#include <mil_tools/pairedSerial.hpp>

#include <gtest/gtest.h>

#include <unistd.h>
#include <fcntl.h>
#include <termio.h>

void writeBytes(int fd, const std::string& bytes)
{
    int bytesToWrite = bytes.size();
    int bytesWritten = 0;
    do
    {
        int ret = write(fd, bytes.data() + bytesWritten, bytesToWrite - bytesWritten);
        EXPECT_NE(ret, -1);
        bytesWritten += ret;
    } while (bytesWritten < bytesToWrite);
    
}

void readBytes(int fd, std::string& bytes)
{
    int bytesToRead = bytes.size();
    int bytesRead = 0;
    do
    {
        int ret = read(fd, bytes.data() + bytesRead, bytesToRead - bytesRead);
        EXPECT_NE(ret, -1);
        bytesRead += ret;
    }
    while(bytesRead < bytesToRead);
}

TEST(pairedSerial, test)
{
    mil_tools::PairedSerial serial;
    std::string slave1;
    std::string slave2;
    EXPECT_EQ(serial.open(slave1, slave2), 0);

    int slave1Fd_ = open(slave1.c_str(), O_RDWR | O_NOCTTY);
    EXPECT_NE(slave1Fd_, -1);

    struct termios term;
    tcgetattr(slave1Fd_, &term);
    cfmakeraw(&term);
    tcsetattr(slave1Fd_, TCSANOW, &term);

    int slave2Fd_ = open(slave2.c_str(), O_RDWR | O_NOCTTY);
    EXPECT_NE(slave2Fd_, -1);

    tcgetattr(slave2Fd_, &term);
    cfmakeraw(&term);
    tcsetattr(slave2Fd_, TCSANOW, &term);

    std::string in("This is a test string to test the PairedSerial class."
                    "This PairedSerial class will create two serial devices."
                    "Then connect them together to form a pipeline."
                    "The class create a work thread to do the data transfer jobs.");
    std::string out(in.size(), '\0');

    writeBytes(slave1Fd_, in);
    readBytes(slave2Fd_, out);
    EXPECT_EQ(in, out);

    out.assign(in.size(), '\0');
    writeBytes(slave2Fd_, in);
    readBytes(slave1Fd_, out);
    EXPECT_EQ(in, out);
    
    close(slave1Fd_);
    close(slave2Fd_);
    serial.close();
}
