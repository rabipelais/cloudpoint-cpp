#ifndef CONVEXHULL_HPP
#define CONVEXHULL_HPP

#include "point.hpp"

#include <vector>
#include <iostream>
#include <cassert>


template<class T>
class DoublyLinkedEdgeList {
public:

	struct Face;

	struct HalfEdge {
		T* origin;
		HalfEdge* twin;
		Face* incidentFace;
		HalfEdge* next;
		HalfEdge* prev;
	};

	struct Face {
		HalfEdge* outerComponent; //The face lies on the left of the edge
		std::vector<HalfEdge*> innerComponents;
		std::vector<T*> visiblePoints;
		T* lastVisitedBy;
	};

	~DoublyLinkedEdgeList() {
		for(HalfEdge* e : mHalfEdges) {
			delete e;
		}
		for(Face* f : mFaces) {
			delete f;
		}
	}

	std::vector<Face *> faces() {return mFaces;}

	/*
	 * Check if the face f is visible from point p
	 */
	static bool visible(const Face* f, const T& p) {
		HalfEdge* a = f->outerComponent;
		HalfEdge* b = a->next;
		HalfEdge* c = b->next;

		T x = sub(*b->origin, *a->origin);
		T y = sub(*c->origin, *a->origin);
		T n = cross(x, y);

		//The triangle is only visible if the product of the
		//normal and P - A is positive, i.e. smaller than 90 degs.
		return dot(n, sub(p, *(a->origin))) >= 0;
	}

	/*
	 * (Positive) distance from a point to a face
	 */
	static double distance(const Face* f, const T& p) {
		//TODO STUB
		return norm(sub(*(f->outerComponent->origin), p));
	}

     /*
     * Add a CCW triangular face, where e is the base and p the
     * opposite point.
     */
    Face* addFace(HalfEdge* e, T* p) {
        Face* f = new Face();
        f->outerComponent = e;
        HalfEdge* ep = new HalfEdge();
        HalfEdge* pe = new HalfEdge();
        ep->origin = e->twin->origin;
        pe->origin = p;
        ep->incidentFace = pe->incidentFace = e->incidentFace = f;
        e->next = ep; ep->next = pe; pe->next = e;
        e->prev = pe; pe->prev = ep; ep->prev = e;

        //Note that here we don't use the `twin->origin` tests
        assert(e->prev->next == e);
        assert(e->next->prev == e);
        assert(e->twin->twin == e);
        assert(e->prev->incidentFace == e->incidentFace);
        assert(e->next->incidentFace == e->incidentFace);

        assert(ep->prev->next == ep);
        assert(ep->next->prev == ep);
        assert(ep->prev->incidentFace == ep->incidentFace);
        assert(ep->next->incidentFace == ep->incidentFace);

        assert(pe->prev->next == pe);
        assert(pe->next->prev == pe);
        assert(pe->prev->incidentFace == pe->incidentFace);
        assert(pe->next->incidentFace == pe->incidentFace);

        assert(f->outerComponent->incidentFace == f);
        return f;
    }

	/*
	 * Assume the first three vertices are already in CCW fashion
	 * when looking at the base from the bottom, i.e. the border
	 * of the base is a->b->c->a.
	 */
	void addTetrahedron(T& a, T& b, T& c, T& d) {
		//This code is going to be sooo buggy.....

		//Create the base
		Face* base = new Face();
		HalfEdge* ab = new HalfEdge(); //Base edge from a and twin
		HalfEdge* ba = new HalfEdge();
		ab->twin = ba; ba->twin = ab;
		ab->origin = &a; ba->origin = &b;
		ab->incidentFace = base;
		mHalfEdges.push_back(ab);
		mHalfEdges.push_back(ba);

		HalfEdge* bc = new HalfEdge(); //Base edge from b and twin
		HalfEdge* cb = new HalfEdge();
		bc->twin = cb; cb->twin = bc;
		bc->origin = &b; cb->origin = &c;
		bc->incidentFace = base;
		mHalfEdges.push_back(bc);
		mHalfEdges.push_back(cb);

		HalfEdge* ca = new HalfEdge(); //Base edge from c and twin
		HalfEdge* ac = new HalfEdge();
		ca->twin = ac; ac->twin = ca;
		ca->origin = &c; ac->origin = &a;
		ca->incidentFace = base;
		mHalfEdges.push_back(ca);
		mHalfEdges.push_back(ac);

		base->outerComponent = ab;

		//Link the edges
		ab->next = bc; bc->next = ca; ca->next = ab;
		ab->prev = ca; bc->prev = ab; ca->prev = bc;

		mFaces.push_back(base);

		assert(!DoublyLinkedEdgeList<T>::visible(base, d));

		//Create the edges from the point
		HalfEdge* bd = new HalfEdge();
		HalfEdge* db = new HalfEdge();
		bd->twin = db; db->twin = bd;
		bd->origin = &b; db->origin = &d;
		mHalfEdges.push_back(bd);
		mHalfEdges.push_back(db);

		HalfEdge* da = new HalfEdge();
		HalfEdge* ad = new HalfEdge();
		da->twin = ad; ad->twin = da;
		da->origin = &d; ad->origin = &a;
		mHalfEdges.push_back(da);
		mHalfEdges.push_back(ad);

		HalfEdge* dc = new HalfEdge();
		HalfEdge* cd = new HalfEdge();
		dc->twin = cd; cd->twin = dc;
		dc->origin = &d; cd->origin = &c;
		mHalfEdges.push_back(dc);
		mHalfEdges.push_back(cd);

		//Now the other faces
		Face* bad = new Face();
		ba->incidentFace = db->incidentFace = ad->incidentFace = bad;
		ba->next = ad; ad->next = db; db->next = ba;
		ba->prev = db; db->prev = ad; ad->prev = ba;
		bad->outerComponent = ba;
		mFaces.push_back(bad);

		Face* acd = new Face();
		ac->incidentFace = cd->incidentFace = da->incidentFace = acd;
		ac->next = cd; cd->next = da; da->next = ac;
		ac->prev = da; da->prev = cd; cd->prev = ac;
		acd->outerComponent = ac;
		mFaces.push_back(acd);

		Face* cbd = new Face();
		cb->incidentFace = bd->incidentFace = dc->incidentFace = cbd;
		cb->next = bd; bd->next = dc; dc->next = cb;
		cb->prev = dc; dc->prev = bd; bd->prev = cb;
		cbd->outerComponent = cb;
		mFaces.push_back(cbd);

		//Now try to make sense of this mess
		for(Face* f : mFaces) {
			assert(f->outerComponent->incidentFace == f);
		}
		int i = 0;
		for(HalfEdge* e : mHalfEdges) {
			assert(e->prev->next == e);
			assert(e->next->prev == e);
			assert(e->twin->twin == e);
			assert(e->twin->next->origin == e->origin);
			assert(e->prev->twin->origin == e->origin);
			assert(e->prev->incidentFace == e->incidentFace);
			assert(e->next->incidentFace == e->incidentFace);
		}
	}

private:
	std::vector<HalfEdge*> mHalfEdges;
	std::vector<Face*> mFaces;
};

template<typename T>
using ConvexHull = DoublyLinkedEdgeList<T>;

namespace Convex {

	/*
	 * Implementation based on the QuickHull algorithm. The idea is to assign to each face of
	 * the CH the points from which it is visible. If this list is non-empty, this face should
	 * not be on the CH, and has to be processed. The faces are put on a stack. For each face on
	 * the stack, a cone is built from the furthest point and its horizon edges. The points
	 * visible from the old faces are reassigned to the new faces.
	 */
	template<class Point>
	ConvexHull<Point> convexHull(std::vector<Point>& points) {
		const double eps = 0.0001;

		//Try to find four non-coplanar points, and remove them from the points to be processed
		std::vector<Point*> remaining;
		Point p1 = points[0];
		Point p2 = points[1];
		Point p3 = p1;
		Point p4 = p1;

		unsigned int i = 2;
		//First find the first non-collinear point to p1 and p2
		for(; i < points.size(); i++) {
			Point p = points[i];
			if(!collinear(p1, p2, p)) {
				p3 = p;
				break;
			}
			remaining.push_back(&p);
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
			remaining.push_back(&p);
		}

		ConvexHull<Point> CH;
		//Check if the tetrahedron would be properly oriented
		if(!inFront(p1, p2, p3, p4)) {
			CH.addTetrahedron(p1, p2, p3, p4);
		} else {
			//Change the orientation of the base
			CH.addTetrahedron(p2, p1, p3, p4);
		}

		auto facesStack = CH.faces();
		assert(facesStack.size() == 4);

		//Assign points to the face that is closest
		for(int p = 0; p < points.size(); p++) {
			bool visible = false;
			double d = std::numeric_limits<double>::max();
			typename ConvexHull<Point>::Face* closestFace = NULL;
			//Find closest face
			for(auto f : facesStack) {
				double distance = ConvexHull<Point>::distance(f, points.at(p));
				if(ConvexHull<Point>::visible(f, points.at(p)) && distance > eps) {
					if(distance < d) {
						d = distance;
						visible = true;
						closestFace = f;
					}
				}
			}
			if(visible) {
				closestFace->visiblePoints.push_back(&(points.at(p)));
			}
		}

		//Process the stack of facets
		while(!facesStack.empty()) {
			std::cout << "Processing face" << std::endl;
			auto currentFace = facesStack.back();
			facesStack.pop_back();

			//Find point farthest away
			Point* point;
			double d = std::numeric_limits<double>::min();
			for(Point *p : currentFace->visiblePoints) {
				double distance = ConvexHull<Point>::distance(currentFace, *p);
				if(distance >= d) {
					d = distance;
					point = p;
				}
			}

			//Find the horizon as seen from that point
			typename ConvexHull<Point>::HalfEdge* horizonStart = NULL;
			//First find all visible faces
			std::vector<typename ConvexHull<Point>::Face*> visibleFaces;
			bool newFaceAdded = false;
			currentFace->lastVisitedBy = point;
			visibleFaces.push_back(currentFace);

			//Spread from the current face to the adjacent ones until no new
			//face can be added
			do {
				newFaceAdded = false;
				currentFace = visibleFaces.back();
				auto e = currentFace->outerComponent;

				//Go through the adjacent faces
				do {
					auto adjFace = e->twin->incidentFace;
					if(adjFace->lastVisitedBy != point && ConvexHull<Point>::visible(adjFace, *point)) {
						std::cout << "-- Processing adjacent face" << std::endl;
						adjFace->lastVisitedBy = point;
						visibleFaces.push_back(adjFace);
						newFaceAdded = true;
					}

					//If the adjacent face is not visible, this edge lies on the horizon
					//TODO pull into the othe if-branch
					if(horizonStart == NULL && !ConvexHull<Point>::visible(adjFace, *point)) {
						horizonStart = e;
					}
					e = e->next;
				} while(e != currentFace->outerComponent);
			} while(newFaceAdded);

			assert(horizonStart != NULL);

			//The horizon should be convex when 2D-projected from the point
			std::vector<typename ConvexHull<Point>::HalfEdge*> horizon;
			auto currentHorizon = horizonStart;

			//Build the horizon step by step until the loop is closed
			do {
				horizon.push_back(currentHorizon);
				//Find adjacent edge that is on the horizon
				auto nextEdge = currentHorizon->next->twin->next;
				while(ConvexHull<Point>::visible(nextEdge->twin->incidentFace, *point)) {
					nextEdge = nextEdge->next->twin->next;
				}
				currentHorizon = nextEdge;
			} while(currentHorizon != horizonStart);

			//Now iterate over the horizon and build the new faces

			//Also reassign the points of the old visible faces to the new faces

			//Remember to delete the old visible faces (which are no longer in the CH)
		}

		return CH;
	}
}
#endif
