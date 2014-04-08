Vizkit::UiLoader.register_3d_plugin('VFHTreeViewer', 'vfh_star', 'VFHTree')
Vizkit::UiLoader.register_3d_plugin_for('VFHTreeViewer', "/vfh_star/DebugTree_m", :updateTree)
Vizkit::UiLoader.register_3d_plugin_for('VFHTreeViewer', "/vfh_star/HorizonPlannerDebugData_m", :updateDebugData)

Vizkit::UiLoader.register_3d_plugin('VFHLocalMapViewer', 'vfh_star', 'TraversabilityMapGenerator')

