Vizkit::UiLoader.register_3d_plugin('VFHTreeViewer', 'vfh_star', 'VFHTree')
Vizkit::UiLoader.register_3d_plugin_for('VFHTreeViewer', "/wrappers/vfh_star/Tree", :updateTree)
Vizkit::UiLoader.register_3d_plugin_for('VFHTreeViewer', "/corridor_navigation/FollowingDebug_m") do |plugin, data, port|
    plugin.updateTree(data.tree)
    plugin.updateSegment(data.horizon)
end
Vizkit::UiLoader.register_3d_plugin_for('VFHTreeViewer', "/corridor_navigation/VFHStarDebugData_m") do |plugin, data, port|
    segment = []
    segment << data.horizonOrigin + data.horizonVector * 0.5
    segment << data.horizonOrigin - data.horizonVector * 0.5
    plugin.updateSegment(segment)
end

Vizkit::UiLoader.register_3d_plugin('VFHLocalMapViewer', 'vfh_star', 'TraversabilityMapGenerator')
Vizkit::UiLoader.register_3d_plugin_for('VFHLocalMapViewer', "/vfh_star/GridDump", :updateGridDump)

