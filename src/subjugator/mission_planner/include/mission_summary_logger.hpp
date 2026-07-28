#pragma once

#include <behaviortree_cpp/loggers/abstract_logger.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

class MissionSummaryLogger : public BT::StatusChangeLogger
{
  public:
    MissionSummaryLogger(const BT::Tree& tree) : BT::StatusChangeLogger(tree.rootNode())
    {
    }

    ~MissionSummaryLogger() override
    {
        std::vector<std::pair<uint16_t, NodeRecord>> rows(records_.begin(), records_.end());
        std::sort(rows.begin(), rows.end(),
                  [](auto const& a, auto const& b) { return a.second.start < b.second.start; });

        std::ostringstream table;
        table << "=== MISSION SUMMARY ===\n"
              << std::left << std::setw(6) << "UID" << std::setw(26) << "Operation" << std::setw(10) << "Result"
              << "Duration\n"
              << std::string(54, '-') << "\n";

        for (auto const& [uid, record] : rows)
        {
            table << std::left << std::setw(6) << uid << std::setw(26) << record.name << std::setw(10)
                  << BT::toStr(record.final_status) << std::fixed << std::setprecision(2) << record.elapsed.count()
                  << " s\n";
        }
        std::cout << table.str();
        std::time_t now = std::time(nullptr);
        char fname[64];
        std::strftime(fname, sizeof(fname), "mission_summary_%Y%m%d_%H%M%S.txt", std::localtime(&now));
        std::ofstream file(fname);
        if (file)
        {
            file << table.str();
            std::cout << "Summary written to " << fname << "\n";
        }
    }

    void callback(BT::Duration timestamp, const BT::TreeNode& node, BT::NodeStatus prev_status,
                  BT::NodeStatus status) override
    {
        if (status == BT::NodeStatus::RUNNING)
        {
            records_[node.UID()].name = node.name();
            records_[node.UID()].start = timestamp;
        }
        else if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE)
        {
            auto it = records_.find(node.UID());
            if (it == records_.end())
            {
                return;
            }
            auto& record = it->second;
            record.final_status = status;
            record.elapsed = std::chrono::duration<double>(timestamp - record.start);
        }
    }

    void flush() override
    {
    }

  private:
    struct NodeRecord
    {
        std::string name;
        BT::Duration start{};
        std::chrono::duration<double> elapsed{};
        BT::NodeStatus final_status = BT::NodeStatus::IDLE;
    };

    std::unordered_map<uint16_t, NodeRecord> records_;
};
