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
        EXPECT_NE(sem_init(&readSem, 0, 0), -1);
        EXPECT_NE(sem_init(&writeSem, 0, 0), -1);
        auto packet = std::make_shared<electrical_protocol::Packet>(1,1);
        packet->pack(PY_STRING("832s"), testString);
        write(packet);
    }
    ~SerialTest()
    {
        sem_destroy(&readSem);
        sem_destroy(&writeSem);
    }

    void wait()
    {
        EXPECT_NE(sem_wait(&writeSem), -1);
        EXPECT_NE(sem_wait(&readSem), -1);
    }

    void onWrite(std::shared_ptr<electrical_protocol::Packet> packet, int errorCode ,size_t bytesWritten)
    {
        EXPECT_EQ(errorCode, 0);
        EXPECT_EQ(bytesWritten, testString.size());
        EXPECT_NE(sem_post(&writeSem), -1);
    }

    void onRead(std::shared_ptr<electrical_protocol::Packet> packet, int errorCode ,size_t bytesRead)
    {
        EXPECT_EQ(errorCode, 0);
        EXPECT_EQ(bytesRead, testString.size());
        auto [string] = packet->unpack(PY_STRING("832s"));
        EXPECT_EQ(testString, string);
        EXPECT_NE(sem_post(&readSem), -1);
    }

    private:
    sem_t readSem;
    sem_t writeSem;

};

TEST(driver, Test)
{
    mil_tools::PairedSerial pairedSerial;

    std::string serial1Name;
    std::string serial2Name;
    pairedSerial.open(serial1Name, serial2Name);

    SerialTest serialTest1(serial1Name);
    SerialTest serialTest2(serial2Name);

    serialTest1.wait();
    serialTest2.wait();
}
