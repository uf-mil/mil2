#include <electrical_protocol_cpp/packet.h>

#include <gtest/gtest.h>

TEST(Packet, Test)
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




