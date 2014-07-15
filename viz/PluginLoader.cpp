#include <vizkit3d/Vizkit3DPlugin.hpp>
#include "VFHTreeVisualization.hpp"

namespace vizkit3d {
    class QtPluginVFHStar : public vizkit3d::VizkitPluginFactory {
    public:
        virtual QStringList* getAvailablePlugins() const
        {
            QStringList* result = new QStringList;
            result->push_back("VFHTreeVisualization");
            return result;
        }
        virtual QObject* createPlugin(QString const& name)
        {
            if (name == "VFHTreeVisualization")
                return new VFHTreeVisualization;
            else return 0;
        };
    };
    Q_EXPORT_PLUGIN2(QtPluginVFHStar, QtPluginVFHStar)
}
