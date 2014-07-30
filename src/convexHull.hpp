#ifndef CONVEXHULL_HPP
#define CONVEXHULL_HPP

#include <Eigen/Core>
#include <vector>

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
	template<typename T>
	ConvexHull<T> convexHull(std::vector<T> points);
}
#endif
