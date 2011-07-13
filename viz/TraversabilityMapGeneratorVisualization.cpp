#include "TraversabilityMapGeneratorVisualization.h"
#include <osg/Vec3>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>

#include <iostream>

namespace vizkit {

TraversabilityMapGeneratorVisualization::TraversabilityMapGeneratorVisualization()
{
    VizPluginRubyAdapter(TraversabilityMapGeneratorVisualization, vfh_star::GridDump, GridDump);
    width = 0;
    height = 0;
}

osg::ref_ptr< osg::Node > TraversabilityMapGeneratorVisualization::createMainNode()
{
    return new osg::PositionAttitudeTransform();
}

void TraversabilityMapGeneratorVisualization::updateMainNode(osg::Node* node)
{
    std::cout << "TraversabilityMapGeneratorViz: Drawing Grid" << std::endl;

    osg::PositionAttitudeTransform *group = dynamic_cast<osg::PositionAttitudeTransform *>(node);
    assert(group);
    
    osg::Vec3d toMiddle(-width / 2 *gridEntrySize, -height / 2 * gridEntrySize, 0);
    
    group->setPosition(gridPos + toMiddle);
    
    //remove old drawable
    group->removeChildren(0, group->getNumChildren());
    
    // Create an object to store geometry in.
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    
    // Create an array of four vertices.
    osg::ref_ptr<osg::Vec3Array> v = new osg::Vec3Array;
    geom->setVertexArray( v.get() );

    //sorrage for color per vertex
    osg::ref_ptr<osg::Vec4Array> colorArray = new osg::Vec4Array;

    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    
    for(int x = 0; x < width; x++) {
	for(int y = 0; y < height; y++) {
	    //up left
	    v->push_back(osg::Vec3(x * gridEntrySize, y*gridEntrySize, grid[x][y]));
	    colorArray->push_back(gridColor[x][y]);
	    //down left
	    v->push_back(osg::Vec3(x * gridEntrySize, (y+1)*gridEntrySize, grid[x][y]));
	    colorArray->push_back(gridColor[x][y]);
	    //down right
	    v->push_back(osg::Vec3((x +1)* gridEntrySize, (y+1)*gridEntrySize, grid[x][y]));
	    colorArray->push_back(gridColor[x][y]);
	    //up right
	    v->push_back(osg::Vec3((x+1) * gridEntrySize, y*gridEntrySize, grid[x][y]));
	    colorArray->push_back(gridColor[x][y]);
	    
	}
    }
    
    geom->setColorArray( colorArray.get() );
    geom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

    // Draw a four-vertex quad from the stored data.
    geom->addPrimitiveSet(
	    new osg::DrawArrays( osg::PrimitiveSet::QUADS, 0, v->size() ) );

    normals->push_back(osg::Vec3(0,0,1));
    geom->setNormalArray(normals);
    geom->setNormalBinding( osg::Geometry::BIND_OVERALL );    
    
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geom.get());

    group->addChild(geode);
    
    osg::ref_ptr<osg::Sphere> sp = new osg::Sphere(robotPos, 0.1);
    osg::ref_ptr<osg::ShapeDrawable> spd = new osg::ShapeDrawable(sp);
    osg::ref_ptr<osg::Geode> spg = new osg::Geode();
    spg->addDrawable(spd);
    
    group->addChild(spg.get());
    
}

void TraversabilityMapGeneratorVisualization::updateDataIntern(const vfh_star::GridDump& data)
{
    vfh_star::ElevationGrid egrid;
    vfh_star::TraversabilityGrid trgrid;
    for(int x = 0; x < egrid.getWidth(); x++) {
	for(int y = 0; y < egrid.getHeight(); y++) {
	    
	    if(data.height[x*egrid.getWidth() + y] != std::numeric_limits< double >::infinity()) {
		if(data.interpolated[x*egrid.getWidth() + y]) {
		    egrid.getEntry(x,y).setInterpolatedMeasurement(data.height[x*egrid.getWidth() + y]);
		} else
		    egrid.getEntry(x,y).addHeightMeasurement(data.height[x*egrid.getWidth() + y]);
	    }
	    
	    egrid.getEntry(x, y).setMaximumHeight(data.max[x*egrid.getWidth() + y]);
	    
	    trgrid.getEntry(x, y) = (vfh_star::Traversability) data.traversability[x*egrid.getWidth() + y];	
	}
    }
    Eigen::Vector3d pos(data.gridPositionX, data.gridPositionY, data.gridPositionZ);
    std::cout << "Grid Pos " << pos.transpose() << std::endl;
    egrid.setGridPosition(pos);
    trgrid.setGridPosition(pos);

    if(width != egrid.getWidth() || height != egrid.getHeight()) {
	width = egrid.getWidth(); 
	height = egrid.getHeight();

	//TODO free memory
    
	grid = new double *[width];
	for(int i = 0; i < width; i++)
	    grid[i] = new double[height];
	
	gridColor = new osg::Vec4 *[width];
	for(int i = 0; i < width; i++)
	    gridColor[i] = new osg::Vec4[height];
    }
    
    
    for(int x = 0; x < egrid.getWidth(); x++) {
	for(int y=0; y < egrid.getHeight(); y++) {
	    if(egrid.getEntry(x, y).getMeasurementCount() || (egrid.getEntry(x, y).getMaximum() == -std::numeric_limits< double >::max()))
		grid[x][y] = egrid.getEntry(x, y).getMedian();
	    else 
	    {
		
		grid[x][y] = egrid.getEntry(x, y).getMaximum();
	    }
	    
	    double intensity = 1.0;
	    if(egrid.getEntry(x, y).getMeasurementCount() || egrid.getEntry(x, y).isInterpolated()) {
		intensity = std::min(fabs(grid[x][y] / 2), 1.0);
		gridColor[x][y] = osg::Vec4(0,intensity,0,1);
		if(egrid.getEntry(x, y).isInterpolated())
		    gridColor[x][y] = osg::Vec4(0,0,1,1);
	    } else {
		gridColor[x][y] = osg::Vec4(1,1,0,1);		
	    }
		
//  	    std::cout << "Intensity " << intensity << " median " << grid[x][y] << std::endl;
	    //color
	    switch(trgrid.getEntry(x,y)) {
		case vfh_star::OBSTACLE:
		    gridColor[x][y] = osg::Vec4(1.0,0,0,1);
		    break;
		case vfh_star::UNKNOWN_OBSTACLE:
		    gridColor[x][y] = osg::Vec4(1,1,0,1);
		    break;
		case vfh_star::TRAVERSABLE:
		    gridColor[x][y] = osg::Vec4(0,intensity,0,1);
		    break;
		case vfh_star::UNCLASSIFIED:
		    gridColor[x][y] = osg::Vec4(0,0,1,1);
		    break;
		default:
		    break;
	    }
	}
    }
    
    width = egrid.getWidth();
    height =egrid.getHeight();
    gridEntrySize = egrid.getGridEntrySize();
    
    Eigen::Vector3d rPos(0,0,0);// = data.getRelativeRobotPosition();
    robotPos.x() = rPos.x();
    robotPos.y() = rPos.y();
    robotPos.z() = rPos.z();
    
    gridPos.x() = egrid.getGridPosition().x();
    gridPos.y() = egrid.getGridPosition().y();
    gridPos.z() = egrid.getGridPosition().z();
}

    
}

