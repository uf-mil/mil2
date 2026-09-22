/*

Hungarian Algorithm Implementation for Assignment Problem

used for track association instead of the greedy nearest-neighbour assignment

*/

#include <vector>

namespace pcd
{

class Hungarian
{
  public:
    Hungarian();
    ~Hungarian();

    // Method to solve the assignment problem
    void solve(std::vector<std::vector<double>> const& costMatrix);

    // Method to get the assignment result
    std::vector<int> const& getAssignment() const;

  private:
    std::vector<int> assignment_;
    double totalCost_;

    // Helper methods
    void initialize(std::vector<std::vector<double>> const& costMatrix);
    void optimize();
};

}  // namespace pcd
