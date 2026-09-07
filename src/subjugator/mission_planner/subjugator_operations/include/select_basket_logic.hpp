#pragma once
#include <string>

namespace select_basket
{

// Role -> its drop-off basket marker class. Mirror of
// select_target::labels_for_role, kept in its own header so the basket and
// target decisions stay decoupled. Unknown/empty role -> empty (caller waits).
inline std::string marker_for_role(std::string const& role, std::string const& survey_marker,
                                   std::string const& rescue_marker)
{
    if (role == "survey_repair")
        return survey_marker;  // caller provides e.g. "warning"
    if (role == "search_rescue")
        return rescue_marker;  // caller provides e.g. "red_cross"
    return {};
}

}  // namespace select_basket
