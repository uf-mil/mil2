#pragma once
#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace select_target
{

// Minimal candidate view, decoupled from yolo_msgs so the logic is unit-testable
// without ROS message types.
struct Candidate
{
    std::string class_name;
    double score;
};

// Split "a, b ,c" -> ["a","b","c"], trimming spaces/tabs and dropping empties.
inline std::vector<std::string> parse_labels(std::string const& csv)
{
    std::vector<std::string> out;
    std::string cur;
    auto flush = [&]()
    {
        size_t b = cur.find_first_not_of(" \t");
        size_t e = cur.find_last_not_of(" \t");
        if (b != std::string::npos)
            out.push_back(cur.substr(b, e - b + 1));
        cur.clear();
    };
    for (char c : csv)
    {
        if (c == ',')
            flush();
        else
            cur.push_back(c);
    }
    flush();
    return out;
}

// Role -> its target labels. Unknown role -> empty (caller treats as "wait").
inline std::vector<std::string> labels_for_role(std::string const& role, std::string const& survey_csv,
                                                std::string const& rescue_csv)
{
    if (role == "survey_repair")
        return parse_labels(survey_csv);
    if (role == "search_rescue")
        return parse_labels(rescue_csv);
    return {};
}

// Highest-score candidate whose class is in `targets`, score >= min_conf, and
// class != exclude. nullopt if none qualify.
inline std::optional<std::string> pick_best(std::vector<Candidate> const& cands,
                                            std::vector<std::string> const& targets, double min_conf,
                                            std::string const& exclude)
{
    std::optional<std::string> best;
    double best_score = 0.0;
    for (auto const& c : cands)
    {
        if (c.score < min_conf)
            continue;
        if (!exclude.empty() && c.class_name == exclude)
            continue;
        if (std::find(targets.begin(), targets.end(), c.class_name) == targets.end())
            continue;
        if (!best || c.score > best_score)
        {
            best = c.class_name;
            best_score = c.score;
        }
    }
    return best;
}

}  // namespace select_target
