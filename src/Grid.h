#ifndef GRID_H
#define GRID_H

#include <Eigen/Core>

namespace vfh_star
{
template <class T, int size, int resolution>
class Grid
{
    public:
	Grid():gridOrigin(gridOriginX, gridOriginY),  gridPosition(0,0,0) {
	    
	}
	
	bool inGrid(const Eigen::Vector2i &p) const {
	    return p.x() > 0 && p.x() < gridWidth && p.y() > 0 && p.y() < gridHeight;
	}

	bool inGrid(const int x, const int y) const {
	    return x > 0 && x < gridWidth && y > 0 && y < gridHeight;
	}

	/**
	* This function returns a 2d point in the grid coresponding to the given 
	* 3d point in world coordinates. 
	* If the given world point is outside of the grid, the method returns false;
	**/
	bool getGridPoint(const Eigen::Vector3d& p_world, Eigen::Vector2i& p_grid) const {
	    Eigen::Vector3d p3_grid = p_world - gridPosition;
	    int posx = p3_grid.x() / gridResolution + gridOriginX;
	    int posy = p3_grid.y() / gridResolution + gridOriginY;    
	    
	    p_grid.x() = posx;
	    p_grid.y() = posy;

	    //check if point is in grid
	    if(!inGrid(posx, posy))
		return false;	    

	    return true;
	}

	
	/**
	* Moves the grid to the new position
	*
	* The given newPosition is a absolut value, not a relative one.
	* note that the grid will only move by gridresolution steps.
	*/
	void moveGrid(const Eigen::Vector3d& newPosition) {
	    T tempGrid[size / resolution][size / resolution];
	    
	    Eigen::Vector3d diff = newPosition - gridPosition; 
    
	    int diffx = diff.x() / gridResolution;
	    int diffy = diff.y() / gridResolution;    

	    //copy grid to tempgrid
	    for(int x = 0; x < gridWidth; x++) {
		for(int y = 0; y < gridHeight; y++) {
		    int nx = x + diffx;
		    int ny = y + diffy;
		    
		    if(nx > 0 && nx < gridWidth && ny > 0 && ny < gridHeight) {
			tempGrid[x][y] = grid[nx][ny];
		    }
		}
	    }

	    //set new position
	    gridPosition.x() += diffx * gridResolution;
	    gridPosition.y() += diffy * gridResolution;

	    //copy back to real grid
	    for(int x = 0; x < gridWidth; x++) {
		for(int y = 0; y < gridHeight; y++) {
		    grid[x][y] = tempGrid[x][y];		    
		}
	    }
	};
	
	void clear() {
	    for(int x = 0; x < gridWidth; x++) {
		for(int y = 0; y < gridHeight; y++) {
		    grid[x][y] = T();		    
		}
	    }
	};

	const T &getEntry(const Eigen::Vector2i &p) const {
	    return (grid)[p.x()][p.y()];
	}

	T &getEntry(const Eigen::Vector2i &p) {
	    return (grid)[p.x()][p.y()];
	}

	const T &getEntry(int x, int y) const {
	    return (grid)[x][y];
	}

	T &getEntry(int x, int y) {
	    return (grid)[x][y];
	}

	int getWidth() const {
	    return gridWidth;
	}
	
	int getHeight() const {
	    return gridHeight;
	}
	
	double getGridEntrySize() const {
	    return gridResolution;
	}
	
	const Eigen::Vector3d &getPosition() const {
	    return gridPosition;
	}
	
	double getGridSize() const {
	    return gridSize;
	}
	
	double getGridResolution() const {
	    return gridResolution;
	}
	
	const Eigen::Vector2i &getGridOrigin() const {
	    return gridOrigin;
	}

	Eigen::Vector2d getUpLeftCorner() const {
	    return Eigen::Vector2d(-gridSize / 2.0, gridSize / 2.0);
	}

	Eigen::Vector2d getUpRightCorner() const {
	    return Eigen::Vector2d(gridSize / 2.0, gridSize / 2.0);
	}

	Eigen::Vector2d getDownLeftCorner() const {
	    return Eigen::Vector2d(-gridSize / 2.0, -gridSize / 2.0);
	}

	Eigen::Vector2d getDownRightCorner() const {
	    return Eigen::Vector2d(gridSize / 2.0, -gridSize / 2.0);
	}

	const Eigen::Vector3d &getGridPosition() const {
	    return gridPosition;
	}
	
	void setGridPosition(const Eigen::Vector3d &pos) {
	    gridPosition = pos;
	}
	
    private:
	static const double gridSize = size / 100.0;
	static const double gridResolution = resolution / 100.0;
	static const int gridWidth = size / resolution;
	static const int gridHeight = size / resolution;
	
	//the origin of the grid is in its middle meaning 
	static const int gridOriginX = size / resolution / 2;
	static const int gridOriginY = size / resolution / 2;
	const Eigen::Vector2i gridOrigin;
	
	T grid[size / resolution][size / resolution];
	
	Eigen::Vector3d gridPosition;
};
}

#endif // GRID_H
