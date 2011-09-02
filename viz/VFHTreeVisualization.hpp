#ifndef vfh_star_VFHTreeVisualization_H
#define vfh_star_VFHTreeVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/VizPlugin.hpp>
#include <vfh_star/TreeSearch.h>

namespace osg
{
    class Geometry;
}

namespace vizkit
{
    class VFHTreeVisualization
        : public vizkit::VizPlugin<vfh_star::Tree>
	, public vizkit::VizPluginAddType< std::vector<base::Vector3d> >
        , boost::noncopyable
    {
    public:
        VFHTreeVisualization();
        ~VFHTreeVisualization();

        enum COST_MODE
        {
            SHOW_COST, SHOW_HEURISTICS, SHOW_BOTH
        };

        void setCostMode(COST_MODE mode);
        void removeLeaves(bool enable);
        void setMaxNodeCount(int count);

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(vfh_star::Tree const& plan);
        virtual void updateDataIntern(std::vector<base::Vector3d> const& segment);

        osg::Geometry* createTreeNode(std::set<vfh_star::TreeNode const*> const& nodes, double color_a, double color_b);
        osg::Geometry* createSolutionNode(vfh_star::TreeNode const* node, double color_a, double color_b);

        std::pair<double, double> computeColorMapping(std::set<vfh_star::TreeNode const*> const& nodes) const;
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
