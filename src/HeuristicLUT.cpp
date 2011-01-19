#include <vfh_star/HeuristicLUT.h>
#include <vfh_star/TreeSearch.h>
#include <map>
#include <boost/tuple/tuple.hpp>
#include <base/pose.h>

using namespace vfh_star;
using namespace std;
using boost::tie;

struct vfh_star::HeuristicLUTSearch : public TreeSearch
{
    TreeSearch const& actual_search;
    HeuristicLUT&  heuristic_lut;
    AngleIntervals possible_directions;
    base::Position goal;
    double step;

    HeuristicLUTSearch(TreeSearch const& actual_search, HeuristicLUT& heuristic_lut)
        : actual_search(actual_search)
        , heuristic_lut(heuristic_lut)
    {
        possible_directions.push_back( make_pair(0, 2*M_PI) );
    }

    pair<double, bool> compute(base::Vector2d const& p, double step)
    {
        goal = base::Vector3d(p.x() * step, p.y() * step, 0);
        this->step = step;
        setSearchConf(actual_search.getSearchConf());

        base::Pose start_pose;
        start_pose.position = base::Position(0, 0, 0);
        start_pose.orientation = base::Orientation::Identity();
        TreeNode const* node = TreeSearch::compute(start_pose);
        if (node)
            return make_pair(node->getCost(), true);
        return make_pair(0, false);
    }

    double getHeuristic(TreeNode const& node) const
    { return heuristic_lut.getHeuristic(node.getPose(), goal); }

    double getCostForNode(base::Pose const& pose, double direction,
            TreeNode const& parentNode) const
    { return actual_search.getCostForNode(pose, direction, parentNode); }

    std::pair<base::Pose, bool> getProjectedPose(const vfh_star::TreeNode& curNode,
            double heading, double distance) const
    { return actual_search.getProjectedPose(curNode, heading, distance); }

    AngleIntervals getNextPossibleDirections(
            vfh_star::TreeNode const&,
            double, double) const
    { return possible_directions; }

    bool isTerminalNode(TreeNode const& node) const
    {
        if (node.isRoot())
            return false;

        base::Position from = node.getParent()->getPose().position;
        base::Position to   = node.getPose().position;

        double projection_t = (goal - from).dot(to - from);

        base::Vector3d unit = (to - from).normalized();
        base::Position projection = from + unit * projection_t;

        return (projection.x() > goal.x() && projection.x() < goal.x() + step
                && projection.y() > goal.y() && projection.y() < goal.y() + step);
    }
};

void HeuristicLUTBuilder::openNext(std::multimap<float, base::Vector2d >& targets, std::vector<bool> const& computed, base::Vector2d const& current, double trim)
{
    for (int dx = -1; dx < 2; ++dx)
    {
        for (int dy = -1; dy < 2; ++dy)
        {
            double x = current.x() + dx;
            double y = current.y() + dy;
            if (x < 0 || x > size / 2)
                continue;
            if (y < 0 || y > size)
                continue;

            int coord = getCellIndex(x, y);
            if (computed[coord])
                continue;

            std::cerr << "adding trim=" << trim << " " << current.x() + dx << " " << current.y() + dy << std::endl;
            targets.insert( make_pair(trim, base::Vector2d(current.x() + dx, current.y() + dy)) );
        }
    }
}
int HeuristicLUT::getCellIndex(base::Vector2d const& p) const
{ return getCellIndex(p.x(), p.y()); }

int HeuristicLUT::getCellIndex(int x, int y) const
{
    return abs(x) * (size + 2) + (size / 2 + 1 + y);
}

double HeuristicLUT::getHeuristic(base::Pose const& current, base::Position const& target) const
{
    base::Vector3d v = (target - current.position);
    // correct the start orientation
    v = current.orientation.inverse() * v;

    int x = v.x(), y = v.y();

    float result = values[getCellIndex(x - 1, y - 1)];
    for (int dx = 1; dx < 3; ++dx)
        for (int dy = 1; dy < 3; ++dy)
            result = std::min(result, values[getCellIndex(x + dx, y + dy)]);

    return result;
}


HeuristicLUTBuilder::HeuristicLUTBuilder(TreeSearch const& actual_search)
    : heuristic_search(new HeuristicLUTSearch(actual_search, *this))
{
}

HeuristicLUTBuilder::~HeuristicLUTBuilder()
{ delete heuristic_search; }

pair<double, bool> HeuristicLUTBuilder::computeHeuristic(base::Vector2d const& p)
{
    double result; bool found;
    int coord = getCellIndex(p);

    tie(result, found) = heuristic_search->compute(p, step);
    if (found)
    {
        double trim = values[coord] / result;
        values[coord] = result;
        std::cerr << "found value for " << p.x() << " " << p.y() << ": " << result << " (" << trim << ")" << std::endl;
        return make_pair(trim, true);
    }
    return make_pair(0, false);
}

void HeuristicLUTBuilder::compute(float horizon, float step, double trim_limit)
{
    this->size = 2 * ceil(horizon / step);
    this->step = step;

    // Initialize the LUT using euclidian distance
    //
    // We build only a half LUT in the Y axis as we use the symmetry along the
    // forward axis of the robot
    values.resize((2 + size / 2) * (size + 2));
    std::vector<bool>  computed;
    computed.resize((2 + size / 2) * (size + 2), false);

    double step2 = step * step;
    values[getCellIndex(0, 0)] = 0;
    for (int x = 0; x < size / 2; ++x)
    {
        for (int y = 0; y < size / 2; ++y)
        {
            double dist = sqrt(y * y * step2 + x * x * step2);
            values[getCellIndex(x, y)] = dist;
            values[getCellIndex(x, -y)] = dist;
        }
    }
    for (int x = -1; x < size / 2 + 1; ++x)
    {
        values[getCellIndex(x, -1)]   = std::numeric_limits<float>::infinity();
        values[getCellIndex(x, size)] = std::numeric_limits<float>::infinity();
        std::cerr << x << " -1 " << getCellIndex(x, -1) << " " << values.size() << std::endl;
        std::cerr << x << " " << size << " " << getCellIndex(x, size) << " " << values.size() << std::endl;
    }
    for (int y = -1; y < size + 1; ++y)
    {
        values[getCellIndex(size/2, y)] = std::numeric_limits<float>::infinity();
    }

    computed[getCellIndex(0, 0)] = true;

    std::multimap<float, base::Vector2d> targets;
    openNext(targets, computed, base::Vector2d(0, 0), 0);
    while (!targets.empty())
    {
        double est_trim;
        base::Vector2d current;
        tie(est_trim, current) = *targets.begin();
        targets.erase(targets.begin());

        int coord = getCellIndex(current);
        if (computed[coord])
            continue;

        bool found;
        double actual_trim;
        std::cerr << "computing heuristic value for trim=" << est_trim << " " << current.x() << " " << current.y() << std::endl;
        tie(actual_trim, found) = computeHeuristic(current);
        computed[coord] = true;

        if (found)
        {
            if (actual_trim > trim_limit)
                return;

            openNext(targets, computed, current, actual_trim);
        }
    }
}

