#include <electrical_protocol_cpp/packet.h>

#include <gtest/gtest.h>

TEST(Packet, common)
{
    electrical_protocol::Packet packet(1,1);

    unsigned ta = 42;
    float tb = 3.14;
    double tc = 2.7182818;
    int td = 7;
    char te = 'A';
    std::string tf = "Hello";
    bool tg = true;
    uint64_t th = 987654321;

    packet.pack(PY_STRING("Ifdhc5s?Q"), ta, tb, tc, td, te, tf, tg, th);
    auto [ra, rb, rc, rd, re, rf, rg, rh] = packet.unpack(PY_STRING("Ifdhc5s?Q"));
    
    EXPECT_EQ(ta, ra);
    EXPECT_EQ(tb, rb);
    EXPECT_EQ(tc, rc);
    EXPECT_EQ(td, rd);
    EXPECT_EQ(te, re);
    EXPECT_EQ(tf, rf);
    EXPECT_EQ(tg, rg);
    EXPECT_EQ(th, rh);

}

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

TEST(Packet, size)
{
    electrical_protocol::Packet packet(1,1);
    packet.pack(PY_STRING("831s"), testString);
    EXPECT_THROW(packet.unpack(PY_STRING("832s")), std::out_of_range);
}




