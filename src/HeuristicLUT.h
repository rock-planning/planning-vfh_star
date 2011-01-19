#ifndef VFHSTAR_HEURISTIC_LUT_HPP
#define VFHSTAR_HEURISTIC_LUT_HPP

#include <vector>
#include <map>
#include <base/pose.h>
#include <boost/noncopyable.hpp>

namespace vfh_star
{
    class TreeSearchConf;
    class TreeSearch;
    class TreeNode;
    class HeuristicLUTSearch;

    struct HeuristicLUT
    {
        int size;
        std::vector<float> values;
        double step;

        int getCellIndex(base::Vector2d const& p) const;
        int getCellIndex(int x, int y) const;

        double getHeuristic(base::Pose const& current, base::Position const& target) const;
    };

    struct HeuristicLUTBuilder : public HeuristicLUT, boost::noncopyable
    {
        HeuristicLUTBuilder(TreeSearch const& actual_search);
        ~HeuristicLUTBuilder();
        HeuristicLUTSearch* heuristic_search;
        void compute(float horizon, float step, double trim_limit);
        std::pair<double, bool> computeHeuristic(base::Vector2d const& p);
        void openNext(std::multimap<float, base::Vector2d>& targets, std::vector<bool> const& computed, base::Vector2d const& current, double trim);
    };
}

#endif

