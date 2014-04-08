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
        : public vizkit3d::Vizkit3DPlugin< vfh_star::DebugTree>
        , public vizkit3d::VizPluginAddType< vfh_star::HorizonPlannerDebugData >
        , boost::noncopyable
    {
        Q_OBJECT
    public:
        VFHTreeVisualization();
        ~VFHTreeVisualization();

        Q_PROPERTY(int MaxNodeCount READ getMaxNodeCount WRITE setMaxNodeCount)
        Q_PROPERTY(bool RemoveLeaves READ areLeavesRemoved WRITE removeLeaves)

        Q_INVOKABLE void updateData(vfh_star::DebugTree const& treeDebug)
        { Vizkit3DPlugin<vfh_star::DebugTree>::updateData(treeDebug); }
        Q_INVOKABLE void updateTree(vfh_star::DebugTree const& treeDebug)
        { Vizkit3DPlugin<vfh_star::DebugTree>::updateData(treeDebug); }

        Q_INVOKABLE void updateData(vfh_star::HorizonPlannerDebugData const &data)
        { Vizkit3DPlugin<vfh_star::DebugTree>::updateData(data); }
        Q_INVOKABLE void updateDebugData(vfh_star::HorizonPlannerDebugData const &data)
        { Vizkit3DPlugin<vfh_star::DebugTree>::updateData(data); }

        void setMaxNodeCount(int count);
        int getMaxNodeCount();

        void removeLeaves(bool enable);
        bool areLeavesRemoved();

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        
        virtual void updateDataIntern(const vfh_star::DebugTree& data);
        virtual void updateDataIntern(const vfh_star::HorizonPlannerDebugData& data);

        osg::Geometry* createTreeNode(const vfh_star::DebugTree& tree, double color_a, double color_b);
        osg::Geometry* createSolutionNode(const vfh_star::DebugTree& tree, double color_a, double color_b);

        std::pair<double, double> computeColorMapping(const vfh_star::DebugTree& tree) const;
// 	void addRecursive(std::multimap<double, vfh_star::TreeNode const*> &sorted_nodes, vfh_star::TreeNode const* curNode) const;
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
