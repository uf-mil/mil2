#include <gtest/gtest.h>

#include "select_face_symbol_logic.hpp"

using namespace select_face_symbol;

TEST(SymbolFor, SurveyRepairByCount)
{
    EXPECT_EQ(symbol_for("survey_repair", 1), "compass");
    EXPECT_EQ(symbol_for("survey_repair", 2), "hammer_pick");
    EXPECT_EQ(symbol_for("survey_repair", 5), "hammer_pick");
}

TEST(SymbolFor, SearchRescueByCount)
{
    EXPECT_EQ(symbol_for("search_rescue", 1), "life_ring");
    EXPECT_EQ(symbol_for("search_rescue", 2), "sos");
    EXPECT_EQ(symbol_for("search_rescue", 3), "sos");
}

TEST(SymbolFor, ZeroItemsHasNoCorrectSymbol)
{
    EXPECT_EQ(symbol_for("survey_repair", 0), "");
    EXPECT_EQ(symbol_for("search_rescue", 0), "");
    EXPECT_EQ(symbol_for("survey_repair", -1), "");
}

TEST(SymbolFor, UnknownRoleIsEmpty)
{
    EXPECT_EQ(symbol_for("", 1), "");
    EXPECT_EQ(symbol_for("bogus", 2), "");
}

TEST(AllSymbols, ContainsFourClassesCsv)
{
    EXPECT_EQ(all_symbols(), "compass,hammer_pick,life_ring,sos");
}
