#ifndef CONVEXHULL_HPP
#define CONVEXHULL_HPP

#include <vector>
#include <stack>
#include <iostream>

template<typename T>
class DoublyLinkedEdgeList {
	struct Face;

	struct HalfEdge {
		T* origin;
		HalfEdge* twin;
		Face* incidentFace;
		HalfEdge* next;
		HalfEdge* prev;
	};

	struct Face {
		HalfEdge* outerComponent;
		std::vector<HalfEdge*> innerComponents;
	};
};

template<typename T>
using ConvexHull = DoublyLinkedEdgeList<T>;

namespace Convex {

	template<class Point>
	bool collinear(const Point& p1, const Point& p2, const Point& p3) {
		return p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) +  p3[0] * (p1[1] - p2[1]) == 0;
	}

	template<class Point>
	bool coplanar(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
		Point a = p3 - p1;
		Point b = p2 - p1;
		Point c = p4 - p3;

		Point d(b[1]*c[2] - c[1]*b[2], b[2]*c[0] - c[2]*b[0], b[0]*c[1] - c[0]*b[1]);
		return 0 == (a[0]*d[0] + a[1]*d[1] + a[2]*d[2]);
	}

	template<class Point>
	ConvexHull<Point> convexHull(const std::vector<Point>& points) {
		//Try to find four non-coplanar points, and remove them from the points to be processed
		std::stack<Point> remaining;
		Point p1 = points[0];
		Point p2 = points[1];
		Point p3, p4;

		unsigned int i = 2;
		//First find the first non-collinear point to p1 and p2
		for(; i < points.size(); i++) {
			Point p = points[i];
			if(!collinear(p1, p2, p)) {
				p3 = p;
				break;
			}
			remaining.push(p);
		}

		//No more points left :( should output something better TODO
		if(points.size() - i < 2) std::cout << "--ERROR: POINTS LIE ON A PLANE" << std::endl;

		//Now, go on to find the other point of the tetrahedron
		for(; i < points.size(); i++) {
			Point p = points[i];
			if(!coplanar(p1, p2, p3, p)) {
				p4 = p;
				break;
			}
			remaining.push(p);
		}

		ConvexHull<Point> CH;
	}
}
#endif
