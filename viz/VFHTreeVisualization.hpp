#ifndef vfh_star_VFHTreeVisualization_H
#define vfh_star_VFHTreeVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/VizPlugin.hpp>
#include <vfh_star/TreeSearch.h>

namespace vizkit
{
    class VFHTreeVisualization
        : public vizkit::VizPlugin<vfh_star::Tree>
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

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(vfh_star::Tree const& plan);

        std::pair<double, double> computeColorMapping() const;
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
