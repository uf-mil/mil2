/*

Hungarian Algorithm Implementation for Assignment Problem

used for track association instead of the greedy nearest-neighbour assignment

standard O(n^3) primal-dual (successive shortest paths with reduced costs / potentials) formulation

To respect a maximum-association-distance gate, the caller should
replace any cost[i][j] that exceeds the gate with a very large sentinel (see kGateRejectCost below) before calling
Solve().


*/

#include <algorithm>
#include <cmath>
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
    static constexpr double kEps = 1e-9;
    static constexpr double kGateRejectCost = 1.0e9;

    // Method to solve the assignment problem
    // Rows can be tracks
    // Columns can be new detection
    // cost matrix is basically the euclidean distance between the predicted track and the new detection

    static double solve(std::vector<std::vector<double>> const &cost_in, std::vector<int> &assignment)
    {
        // Handle edge cases; any without any zero cost, will just return 0 as well

        std::size_t const nRows = cost_in.size();

        if (nRows == 0)
        {
            assignment.clear();
            return 0.0;
        }

        std::size_t const nCols = cost_in[0].size();

        if (nCols == 0)
        {
            assignment.assign(nRows, -1);
            return 0.0;
        }

        // Need to make it a square matrix first, so if it is mismatched, then add dummy columns/rows

        std::size_t const N = std::max(nRows, nCols);
        std::vector<std::vector<double>> a(N, std::vector<double>(N, 0.0));

        for (std::size_t i = 0; i < nRows; ++i)
            for (std::size_t j = 0; j < nCols; ++j)
                a[i][j] = cost_in[i][j];

        // subtract row mimum

        for (std::size_t i = 0; i < N; ++i)
        {
            double const rowMin = *std::min_element(a[i].begin(), a[i].end());

            for (std::size_t j = 0; j < N; ++j)

                a[i][j] -= rowMin;
        }

        // subtract column minimum

        for (std::size_t j = 0; j < N; ++j)
        {
            double colMin = std::numeric_limits<double>::max();

            for (std::size_t i = 0; i < N; ++j)
            {
                if (a[i][j] < colMin)
                {
                    colMin = a[i][j];
                }
            }

            for (std::size_t i = 0; i < N; ++j)
            {
                a[i][j] -= colMin;
            }
        }

        // mask: 0 = plain, 1 = starred zero (tentative match), 2 = primed zero
        std::vector<std::vector<int>> mask(N, std::vector<int>(N, 0));
        std::vector<bool> rowCover(N, false), colCover(N, false);

        auto isZero = [&](double v) { return std::fabs(v) < kEps; };

        // Initial starring: one starred zero per row/col, greedily.

        for (std::size_t i = 0; i < N; ++i)

            for (std::size_t j = 0; j < N; ++j)

                if (isZero(a[i][j]) && !rowCover[i] && !colCover[j])
                {
                    mask[i][j] = 1;
                    rowCover[i] = true;
                    colCover[j] = true;
                }

        std::fill(rowCover.begin(), rowCover.end(), false);

        std::fill(colCover.begin(), colCover.end(), false);

        auto coverStarredColumns = [&]()
        {
            std::fill(colCover.begin(), colCover.end(), false);

            for (std::size_t i = 0; i < N; ++i)

                for (std::size_t j = 0; j < N; ++j)

                    if (mask[i][j] == 1)

                        colCover[j] = true;
        };

        auto countCoveredCols = [&]()
        { return static_cast<std::size_t>(std::count(colCover.begin(), colCover.end(), true)); };

        coverStarredColumns();

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
