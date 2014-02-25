Vizkit::UiLoader.register_3d_plugin('VFHTreeViewer', 'vfh_star', 'VFHTree')
Vizkit::UiLoader.register_3d_plugin_for('VFHTreeViewer', "/wrappers/vfh_star/Tree", :updateTree)
Vizkit::UiLoader.register_3d_plugin_for('VFHTreeViewer', "/vfh_star/HorizonPlannerDebugData_m", :updateDebugData)

Vizkit::UiLoader.register_3d_plugin('VFHLocalMapViewer', 'vfh_star', 'TraversabilityMapGenerator')
Vizkit::UiLoader.register_3d_plugin_for('VFHLocalMapViewer', "/vfh_star/GridDump", :updateGridDump)

