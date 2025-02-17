#include <electrical_protocol_cpp/driver.h>
#include <mil_tools/pairedSerial.hpp>

#include <gtest/gtest.h>
#include <pthread.h>
#include <semaphore.h>

std::string testString("The Machine Intelligence Laboratory (MIL) "
                        "provides a synergistic environment dedicated "
                        "to the study and development of intelligent, autonomous robots. "
                        "The faculty and students associated with the laboratory "
                        "conduct research in the theory and realization of "
                        "machine intelligence covering topics such as machine learning, "
                        "real-time computer vision, statistical modeling, robot kinematics, "
                        "autonomous vehicles, teleoperation and human interfaces, "
                        "robot and nonlinear control, computational intelligence, "
                        "neural networks, and general robotics. "
                        "Applications of MIL research include autonomous underwater vehicles (AUVs), "
                        "autonomous water surface vehicles (ASVs), autonomous land vehicles, "
                        "autonomous air vehicles (AAVs including quadcopters and micro air vehicles, MAVs) , "
                        "swarm robots, humanoid robots, and autonomous household robots.");


class SerialTest: public electrical_protocol::SerialDevice
{
    public:
    SerialTest(const std::string& portName):SerialDevice(portName)
    {
        if(sem_init(&readSem, 0, 0) == -1)
        {
            throw std::runtime_error("Failed to create read sem");
        }
        if(sem_init(&writeSem, 0, 0) == -1)
        {
            throw std::runtime_error("Failed to create write sem");
        }
    }
    ~SerialTest()
    {
        sem_destroy(&readSem);
        sem_destroy(&writeSem);
    }

    int readSync(electrical_protocol::Packet& packet)
    {
        read(std::move(packet));
        if(sem_wait(&readSem) == -1)
            return errno;
        
        packet = std::move(readQueue_.front());
        readQueue_.pop();
        return readErrno;
    }

    int writeSync(electrical_protocol::Packet& packet)
    {
        write(std::move(packet));
        if(sem_wait(&writeSem) == -1)
            return errno;

        packet = std::move(writeQueue_.front());
        writeQueue_.pop();
        return writeErrno;
    }

    void onWrite(electrical_protocol::Packet& packet, int errorCode)
    {
        writeErrno = errorCode;
        writeQueue_.push(std::move(packet));
        sem_post(&writeSem);
    }

    void onRead(electrical_protocol::Packet& packet, int errorCode)
    {
        readErrno = errorCode;
        readQueue_.push(std::move(packet));
        sem_post(&readSem);
    }

    private:
    int readErrno = 0;
    int writeErrno = 0;
    sem_t readSem;
    sem_t writeSem;
    std::queue<electrical_protocol::Packet> readQueue_;
    std::queue<electrical_protocol::Packet> writeQueue_;
};

void* writeThreadFunc(void* arg)
{
    std::string* serialName = reinterpret_cast<std::string*>(arg);
    SerialTest test(*serialName);
    electrical_protocol::Packet packet(1,1);
    packet.pack(PY_STRING("831s"), testString);
    while(1)
    {
        EXPECT_EQ(test.writeSync(packet), 0);
        usleep(100000);
        pthread_testcancel();
    }
    
    return 0;
}

void * readThreadFunc(void* arg)
{
    std::string* serialName = reinterpret_cast<std::string*>(arg);
    SerialTest test(*serialName);
    
    electrical_protocol::Packet packet(1,1);
    test.readSync(packet);
    auto [readString] = packet.unpack(PY_STRING("831s"));
    EXPECT_EQ(readString, testString);
    return 0;
}

TEST(driver, dataTransfer)
{
    mil_tools::PairedSerial pairedSerial;

    std::string serial1Name;
    std::string serial2Name;
    pairedSerial.open(serial1Name, serial2Name);

    pthread_t readThread;
    pthread_t writeThread;

    EXPECT_NE(pthread_create(&readThread, NULL, readThreadFunc, &serial1Name), -1);
    EXPECT_NE(pthread_create(&writeThread, NULL, writeThreadFunc, &serial2Name), -1);

    pthread_join(readThread, NULL);

    pthread_cancel(writeThread);
    pthread_join(writeThread, NULL);
}

class SerialFWTest: public electrical_protocol::SerialDevice
{
    public:
    SerialFWTest(const std::string& portName):SerialDevice(portName)
    {
       
    }
    ~SerialFWTest()
    {

    }

    void onWrite([[maybe_unused]]electrical_protocol::Packet& packet, [[maybe_unused]]int errorCode)
    {
        
    }

    void onRead([[maybe_unused]]electrical_protocol::Packet& packet, [[maybe_unused]]int errorCode)
    {
        
    }

    private:
};

TEST(driver, forward)
{
    electrical_protocol::Packet packet1(1,1);
    packet1.pack(PY_STRING("831s"), testString);
    electrical_protocol::Packet packet2(2,2);
    packet2.pack(PY_STRING("831s"), testString);
    const electrical_protocol::Packet packet3(packet2);

    mil_tools::PairedSerial pairedSerial;

    std::string serial1Name;
    std::string serial2Name;
    pairedSerial.open(serial1Name, serial2Name);
    SerialFWTest serialtest1(serial1Name);
    SerialFWTest serialtest2(serial2Name);

    electrical_protocol::Packet packet4(1,1);
    serialtest1.write(packet1);
    serialtest2.read(packet4);
    EXPECT_NO_THROW(packet1.unpack(PY_STRING("831s")));

    serialtest1.write(std::move(packet2));
    serialtest2.read(electrical_protocol::Packet(2,2));
    EXPECT_THROW(packet2.unpack(PY_STRING("831s")), std::out_of_range);

    serialtest1.write(packet3);
    serialtest2.read(electrical_protocol::Packet(2,2));

    serialtest1.close();
    serialtest2.close();
}
