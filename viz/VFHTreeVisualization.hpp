#ifndef vfh_star_VFHTreeVisualization_H
#define vfh_star_VFHTreeVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/VizPlugin.hpp>
#include <vfh_star/TreeSearch.h>
#include <QObject>

class QWidget;
class QSpinBox;

namespace osg
{
    class Geometry;
}

namespace vizkit
{    
    typedef std::pair< base::Vector3d, base::Vector3d > VFHTreeVisualizationSegment;
    class VFHTreeVisualization
        : public vizkit::VizPlugin<vfh_star::Tree>
	, public vizkit::VizPluginAddType< VFHTreeVisualizationSegment >
        , boost::noncopyable
    {
    public:
	friend class QTAdapter;
        typedef VFHTreeVisualizationSegment Segment;

        VFHTreeVisualization();
        ~VFHTreeVisualization();

        enum COST_MODE
        {
            SHOW_COST, SHOW_HEURISTICS, SHOW_BOTH
        };

        void setCostMode(COST_MODE mode);
        void removeLeaves(bool enable);
        void setMaxNodeCount(int count);

	QWidget *getSideWidget();
	
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(vfh_star::Tree const& plan);
        virtual void updateDataIntern(Segment const& segment);

        osg::Group* createTreeNode(const std::set< const vfh_star::TreeNode* >& nodes, double color_a, double color_b);
        osg::Geometry* createSolutionNode(vfh_star::TreeNode const* node, double color_a, double color_b);

        std::pair<double, double> computeColorMapping(std::set<vfh_star::TreeNode const*> const& nodes) const;
        
    private:
        struct Data;
        Data* p;
    };
    
    //Note, needs to be in header because of QT
    class QTAdapter: public QObject
    {
	Q_OBJECT
    public:
	
	QTAdapter();
	virtual ~QTAdapter();
	VFHTreeVisualization *viz;
	QSpinBox *spinBox;
	
    public slots:
	void numberChanged(int x);
    

    };

}
#endif
