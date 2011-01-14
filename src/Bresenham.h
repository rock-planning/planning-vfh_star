#ifndef BRESENHAM_H
#define BRESENHAM_H

#include <Eigen/Geometry>


namespace vfh_star {

class Bresenham {
    public:
        Bresenham(const Eigen::Vector2i &p1, const Eigen::Vector2i &p2) {
	    int x0 = p1.x();
	    int x1 = p2.x();
	    int y0 = p1.y();
	    int y1 = p2.y();
	    
	    steep = abs(y1 - y0) > abs(x1 - x0);
	    if( steep) { 
		std::swap(x0, y0);
		std::swap(x1, y1);
	    }
		
	    if(x0 > x1) {
		std::swap(x0, x1);
		std::swap(y0, y1);
	    }
	    
	    deltax = x1 - x0;
	    deltay = abs(y1 - y0);
	    error = deltax / 2;
	    y = y0;
	    
	    if (y0 < y1) {
		ystep = 1;
	    } else {
		ystep = -1;
	    }
		
	    xend = x1;
	    x = x0;
	}
	
	bool getNextPoint(Eigen::Vector2i &p) {
	    if(x <= xend) {
		if(steep) {
		    p.x() = y;
		    p.y() = x;
		} else { 
		    p.x() = x;
		    p.y() = y;
		}
		
		error = error - deltay;
		if (error < 0) {
		    y = y + ystep;
		    error = error + deltax;  
		}
		
		x++;
		
		return true;
	    }
	    return false;
	}
    private:
	bool steep;
	int deltax;
	int deltay;
	int error;
	int ystep;
	int y;
	int x;
	int xend;

};

}

#endif
