#ifndef vfh_star_VFHTreeVisualization_H
#define vfh_star_VFHTreeVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <vfh_star/TreeSearch.h>

namespace osg
{
    class Geometry;
}

namespace vizkit3d
{
    class VFHTreeVisualization
        : public vizkit3d::Vizkit3DPlugin<vfh_star::Tree>
	, public vizkit3d::VizPluginAddType< std::vector<base::Vector3d> >
        , boost::noncopyable
    {
        Q_OBJECT
    public:
        VFHTreeVisualization();
        ~VFHTreeVisualization();

        enum COST_MODE
        {
            SHOW_COST, SHOW_HEURISTICS, SHOW_BOTH
        };
        
        Q_PROPERTY(COST_MODE CostMode READ getCostMode WRITE setCostMode)
        Q_PROPERTY(int MaxNodeCount READ getMaxNodeCount WRITE setMaxNodeCount)
        Q_PROPERTY(bool RemoveLeaves READ areLeavesRemoved WRITE removeLeaves)

        Q_INVOKABLE void updateData(vfh_star::Tree const& plan)
        { Vizkit3DPlugin<vfh_star::Tree>::updateData(plan); }
        Q_INVOKABLE void updateTree(vfh_star::Tree const& plan)
        { Vizkit3DPlugin<vfh_star::Tree>::updateData(plan); }

        Q_INVOKABLE void updateData(std::vector<base::Vector3d> const& segment)
        { Vizkit3DPlugin<vfh_star::Tree>::updateData(segment); }
        Q_INVOKABLE void updateSegment(std::vector<base::Vector3d> const& segment)
        { Vizkit3DPlugin<vfh_star::Tree>::updateData(segment); }

        void setCostMode(COST_MODE mode);
        COST_MODE getCostMode();
        
        void setMaxNodeCount(int count);
        int getMaxNodeCount();

        void removeLeaves(bool enable);
        bool areLeavesRemoved();

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(vfh_star::Tree const& plan);
        virtual void updateDataIntern(std::vector<base::Vector3d> const& segment);

        osg::Geometry* createTreeNode(const std::multimap< double, const vfh_star::TreeNode* >& sorted_nodes, double color_a, double color_b);
        osg::Geometry* createSolutionNode(vfh_star::TreeNode const* node, double color_a, double color_b);

        std::pair<double, double> computeColorMapping(const std::multimap< double, const vfh_star::TreeNode* >& sorted_nodes) const;
	void addRecursive(std::multimap<double, vfh_star::TreeNode const*> &sorted_nodes, vfh_star::TreeNode const* curNode) const;
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
