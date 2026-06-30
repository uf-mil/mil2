#pragma once
#include <string>

namespace select_face_symbol
{

// Correct octagon-wall symbol to face, per the RoboSub Task 5 rules:
//   role + item count -> symbol. 0 items has no "correct" symbol (face-any only).
//   survey_repair: 1 item -> compass, 2+ -> hammer_pick
//   search_rescue: 1 item -> life_ring, 2+ -> sos
// Unknown role or items<=0 -> "" (caller falls back to face-any).
inline std::string symbol_for(std::string const& role, int items_collected)
{
    if (items_collected <= 0)
        return "";
    bool const many = items_collected >= 2;
    if (role == "survey_repair")
        return many ? "hammer_pick" : "compass";
    if (role == "search_rescue")
        return many ? "sos" : "life_ring";
    return "";
}

// The full four-symbol set as a CSV, for HoneBearing's multi-label `labels` port
// (the face-any fallback). Order is fixed so tests can assert on it.
inline std::string all_symbols()
{
    return "compass,hammer_pick,life_ring,sos";
}

}  // namespace select_face_symbol
