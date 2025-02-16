#include <electrical_protocol_cpp/packet.h>

#include <gtest/gtest.h>


TEST(packet, packunpack)
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

    EXPECT_THROW(packet.unpack(PY_STRING("Ifdhc5s?Q")), std::out_of_range);

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

    EXPECT_THROW(packet.unpack(PY_STRING("Ifdhc5s?QQ")), std::out_of_range);

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

TEST(packet, size)
{
    electrical_protocol::Packet packet(1,1);
    packet.pack(PY_STRING("831s"), testString);
    EXPECT_EQ(packet.size(), 831);
    EXPECT_THROW(packet.unpack(PY_STRING("832s")), std::out_of_range);
}

TEST(packet, id)
{
    electrical_protocol::Packet packet(1,1);
    std::pair<uint8_t, uint8_t> id = packet.getId();
    EXPECT_EQ(id.first, 1);
    EXPECT_EQ(id.second, 1);
    packet.setId({2,2});
    id = packet.getId();
    EXPECT_EQ(id.first, 2);
    EXPECT_EQ(id.second, 2);
}

TEST(packet, move)
{
    electrical_protocol::Packet packet1(1,1);
    packet1.pack(PY_STRING("831s"), testString);

    electrical_protocol::Packet packet2 = std::move(packet1);
    EXPECT_EQ(packet2.size(), 831);
    EXPECT_EQ(packet1.size(), 0);

    packet1 = std::move(packet2);
    EXPECT_EQ(packet1.size(), 831);
    EXPECT_EQ(packet2.size(), 0);
}

TEST(packet, copy)
{
    electrical_protocol::Packet packet1(1,1);
    packet1.pack(PY_STRING("831s"), testString);

    electrical_protocol::Packet packet2 = packet1;
    EXPECT_EQ(packet1.size(), 831);
    EXPECT_EQ(packet2.size(), 831);
}



