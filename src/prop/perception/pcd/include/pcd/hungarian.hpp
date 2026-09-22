/*

Hungarian Algorithm Implementation for Assignment Problem

used for track association instead of the greedy nearest-neighbour assignment

standard O(n^3) primal-dual (successive shortest paths with reduced costs / potentials) formulation

To respect a maximum-association-distance gate, the caller should
replace any cost[i][j] that exceeds the gate with a very large sentinel (see kGateRejectCost below) before calling
Solve().


*/

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

namespace pcd
{

class Hungarian
{
  public:
    Hungarian();
    ~Hungarian();
    static constexpr double kGateRejectCost = 1.0e9;

    // Method to solve the assignment problem
    // Rows can be tracks
    // Columns can be new detection
    // cost matrix is basically the euclidean distance between the predicted track and the new detection

    static double solve(std::vector<std::vector<double>> const &cost, std::vector<int> &assignment)
    {
        // Handle edge cases; any without any zero cost, will just return 0 as well

        // subtract row mimum

        // subtract column minimum

        // Cover all zeros with a minimum number of lines

        //  Find the smallest value (denoted k) that is not covered by any line in Step 3. Subtract k from all uncovered
        //  elements, and add k to all elements that are covered twice.

        // return total cost
    }

  private:
    std::vector<int> assignment_;
    double totalCost_;

    // Helper methods
    void initialize(std::vector<std::vector<double>> const &costMatrix);
    void optimize();
};

}  // namespace pcd
