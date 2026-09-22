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

    // assidgment vector changed as pass-in-by parameter

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

        // star-prime submarking

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
        // repeat until all columns are covered (i.e. we have a complete assignment)

        while (countCoveredCols() < N)
        {
            bool augmented = false;
            while (!augmented)
            {
                // Trying to find a non-covered zero and prime it. If there is no starred zero in the row, we can
                // augment the path and break out of this loop.

                int zc = -1;
                int zr = -1;

                for (std::size_t i = 0; i < N && zr < 0; ++i)
                {
                    if (rowCover[i])
                        continue;
                    for (std::size_t j = 0; j < N; ++j)
                    {
                        if (!colCover[j] && isZero(a[i][j]))
                        {
                            zr = static_cast<int>(i);

                            zc = static_cast<int>(j);

                            break;
                        }
                    }
                }

                if (zr < 0)
                {
                    // No uncovered zero left: find the smallest uncovered value k,

                    // element covered twice (covered elements covered once are
                    // left unchanged). This creates a new uncovered zero.

                    //  Find the smallest value (denoted k) that is not covered by any line in Step 3. Subtract k from
                    //  all uncovered elements, and add k to all elements that are covered twice.

                    double k = std::numeric_limits<double>::max();

                    for (std::size_t i = 0; i < N; ++i)

                        if (!rowCover[i])

                            for (std::size_t j = 0; j < N; ++j)

                                if (!colCover[j])

                                    k = std::min(k, a[i][j]);

                    for (std::size_t i = 0; i < N; ++i)

                        for (std::size_t j = 0; j < N; ++j)
                        {
                            if (rowCover[i] && colCover[j])
                            {
                                a[i][j] += k;
                            }

                            else if (!rowCover[i] && !colCover[j])
                            {
                                a[i][j] -= k;
                            }
                        }

                    continue;  // retry: an uncovered zero now exists
                }

                mask[zr][zc] = 2;  // prime it

                // Is there a starred zero in this row?
                int starCol = -1;
                for (std::size_t j = 0; j < N; ++j)
                    if (mask[zr][j] == 1)
                    {
                        starCol = static_cast<int>(j);
                        break;
                    }

                if (starCol >= 0)
                {
                    // Cover this row and uncover the column of the starred zero, then continue to find another
                    // uncovered zero.
                    rowCover[zr] = true;
                    colCover[starCol] = false;
                }
                else
                {
                    // No: augment along the alternating path starting at (zr, zc).
                    std::vector<std::pair<int, int>> path;

                    path.push_back({ zr, zc });

                    while (true)
                    {
                        int const c = path.back().second;

                        int r = -1;

                        for (std::size_t i = 0; i < N; ++i)

                            if (mask[i][c] == 1)
                            {
                                r = static_cast<int>(i);

                                break;
                            }

                        if (r < 0)

                            break;  // no starred zero in this column: path is done
                        path.push_back({ r, c });

                        int c2 = -1;
                        for (std::size_t j = 0; j < N; ++j)
                            if (mask[r][j] == 2)
                            {
                                c2 = static_cast<int>(j);
                                break;
                            }
                        path.push_back({ r, c2 });
                    }

                    // Toggle: unstar every starred zero on the path, star every
                    // primed zero on the path -> one more starred zero overall.

                    for (auto const &rc : path)
                    {
                        if (mask[rc.first][rc.second] == 1)

                            mask[rc.first][rc.second] = 0;

                        else if (mask[rc.first][rc.second] == 2)

                            mask[rc.first][rc.second] = 1;
                    }

                    std::fill(rowCover.begin(), rowCover.end(), false);

                    for (std::size_t i = 0; i < N; ++i)

                        for (std::size_t j = 0; j < N; ++j)
                            if (mask[i][j] == 2)
                                mask[i][j] = 0;

                    coverStarredColumns();
                    augmented = true;
                }
            }
        }

        // return total cost
        assignment.assign(nRows, -1);

        double total = 0.0;

        for (std::size_t i = 0; i < nRows; ++i)

            for (std::size_t j = 0; j < nCols; ++j)

                if (mask[i][j] == 1)
                {
                    assignment[i] = static_cast<int>(j);
                    total += cost_in[i][j];
                }
        return total;
    }
};

}  // namespace pcd
