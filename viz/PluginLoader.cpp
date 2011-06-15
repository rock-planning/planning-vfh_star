#include <vizkit/VizPlugin.hpp>
#include "TraversabilityMapGeneratorVisualization.h"
#include "VFHTreeVisualization.hpp"

namespace vizkit {
    class QtPluginVFHStar : public vizkit::VizkitQtPluginBase {
    public:
        virtual QStringList getAvailablePlugins() const
        {
            QStringList result;
            result.push_back("TraversabilityMapGeneratorVisualization");
            result.push_back("VFHTreeVisualization");
            return result;
        }
        virtual vizkit::VizPluginBase* createPlugin(QString const& name)
        {
            if (name == "TraversabilityMapGeneratorVisualization")
                return new TraversabilityMapGeneratorVisualization;
            else if (name == "VFHTreeVisualization")
                return new VFHTreeVisualization;
            else return 0;
        };
    };
    Q_EXPORT_PLUGIN2(QtPluginVFHStar, QtPluginVFHStar)
}
