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
		Face() : outerComponent(NULL), lastVisitedBy(NULL), toDelete(false) {}
		HalfEdge* outerComponent; //The face lies on the left of the edge
		std::vector<HalfEdge*> innerComponents;
		std::vector<T*> visiblePoints;
		T* lastVisitedBy;
		bool toDelete;
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
		auto e = f->outerComponent;
		T* a = e->origin;
		T* b = e->next->origin;
		T* c = e->next->next->origin;

		return distanceToTriangle(*a, *b, *c, p);
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
        mFaces.push_back(f);
        return f;
    }

	double area() {
		return 0;
	}

	double volume() {
		return 0;
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

	void cleanup() {
		mFaces.erase(remove_if(mFaces.begin(),
		                       mFaces.end(),
		                       [](Face* f){return f->toDelete;}),
		             mFaces.end());
	}
private:
	std::vector<HalfEdge*> mHalfEdges;
	std::vector<Face*> mFaces;
};

template<typename T>
using ConvexHull = DoublyLinkedEdgeList<T>;

namespace Convex {
	template<class Point>
	std::vector<Point*> extremePoints(std::vector<Point>& points) {
		std::vector<Point*> tetraPoints;
		Point *a, *b, *c, *d;
		double dmax, dcur;
		dmax = dcur = 0.0;
		Point* tmp[6] = {&points[0], &points[0], //x min, max
		                 &points[0], &points[0], //y min, max
		                 &points[0], &points[0]};//z min, max
		for(int p = 0; p < points.size(); p++) {
			if(points.at(p)[0] < (*tmp[0])[0]) tmp[0] = &points.at(p);
			if(points.at(p)[0] > (*tmp[1])[0]) tmp[1] = &points.at(p);
			if(points.at(p)[1] < (*tmp[2])[1]) tmp[2] = &points.at(p);
			if(points.at(p)[1] > (*tmp[3])[1]) tmp[3] = &points.at(p);
			if(points.at(p)[2] < (*tmp[4])[2]) tmp[4] = &points.at(p);
			if(points.at(p)[2] > (*tmp[5])[2]) tmp[5] = &points.at(p);
		}

		//Find the two most distant points
		for(int i = 0; i < 6; i++) {
			for(int j = i + 1; j < 6; j++) {
				dcur = distance(*tmp[i], *tmp[j]);
				if(dmax < dcur) {
					dmax = dcur;
					a = tmp[i];
					b = tmp[j];
				}
			}
		}

		//Find the most distant point to the line
		dmax = 0.0;
		for(int i = 0; i < 6; i++) {
			dcur = distanceToLine(*a, *b, *tmp[i]);
			if(dmax < dcur) {
				dmax = dcur;
				c = tmp[i];
			}
		}

		//Find the most distant point to the plane (from the whole point list)
		dmax = 0.0;
		for(int i = 0; i < points.size(); i++) {
			dcur = distanceToTriangle(*a, *b, *c, points.at(i));
			if(dmax < dcur) {
				dmax = dcur;
				d = &points.at(i);
			}
		}

		if(inFront(*a, *b, *c, *d)) {
			tetraPoints.push_back(b);
			tetraPoints.push_back(a);
		} else {
			tetraPoints.push_back(a);
			tetraPoints.push_back(b);
		}

		tetraPoints.push_back(c);
		tetraPoints.push_back(d);

		return tetraPoints;
	}

	/*
	 * Implementation based on the QuickHull algorithm. The idea is to assign to each face of
	 * the CH the points from which it is visible. If this list is non-empty, this face should
	 * not be on the CH, and has to be processed. The faces are put on a stack. For each face on
	 * the stack, a cone is built from the furthest point and its horizon edges. The points
	 * visible from the old faces are reassigned to the new faces.
	 */
	template<class Point>
	ConvexHull<Point> convexHull(std::vector<Point>& points) {
		const double eps = 0.001;

		std::cout << "STARTING POINTS: " << points.size() << std::endl;

		//Try to find four non-coplanar points
		auto tetraPoints = extremePoints<Point>(points);

		assert(tetraPoints.size() == 4);

		for(auto t : tetraPoints) {
			std::cout << *t << "\n" << std::endl;
		}

		ConvexHull<Point> CH;

		CH.addTetrahedron(*tetraPoints[0], *tetraPoints[1], *tetraPoints[2], *tetraPoints[3]);

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

		int tPoints = 0;
		for(auto f : facesStack) {
			tPoints += f->visiblePoints.size();
			std::cout << f->visiblePoints.size() << std::endl;;
		}
		std::cout << "-- POINTS IN TETRA: " << tPoints << std::endl;

		//Process the stack of facets
		while(!facesStack.empty()) {
			auto currentFace = facesStack.back();
			std::cout << std::endl;
			std::cout << "Processing face: " << currentFace << std::endl;
			facesStack.pop_back();

			std::cout << "-- POINTS VISIBLE FROM CURRENT FACE: " << currentFace->visiblePoints.size() << std::endl;
			if(currentFace->visiblePoints.size() == 0) continue;
			//std::cout << *currentFace->outerComponent->origin << "\n" << std::endl;
			//std::cout << *currentFace->outerComponent->next->origin << "\n" << std::endl;
			//std::cout << *currentFace->outerComponent->next->next->origin << "\n" << std::endl;

			//Find point farthest away
			Point* point = NULL;
			double d = std::numeric_limits<double>::min();
			for(Point *p : currentFace->visiblePoints) {
				double distance = ConvexHull<Point>::distance(currentFace, *p);
				if(distance >= d) {
					d = distance;
					point = p;
				}
			}

			// std::cout << "distance: " << d << std::endl;
			// std::cout << ConvexHull<Point>::distance(currentFace, *point) << std::endl;
			// std::cout << distanceToTriangle(*currentFace->outerComponent->origin, *currentFace->outerComponent->next->origin, *currentFace->outerComponent->next->next->origin, *point) << std::endl;
			// std::cout << distanceToTriangle(Point(2.5437, -86.7204, 16.7802), Point(-28.3979, -67.5083, 2.7127), Point(-26.2986, -69.5868, 2.0645), Point(-26.2986, -69.5868, 2.0645)) << std::endl;
			// std::cout << std::endl;

			// std::cout << *point << "\n" << std::endl;

			//Find the horizon as seen from that point
			typename ConvexHull<Point>::HalfEdge* horizonStart = NULL;
			//First find all visible faces
			std::vector<typename ConvexHull<Point>::Face*> visibleFaces;
			std::vector<typename ConvexHull<Point>::Face*> toVisit;
			currentFace->lastVisitedBy = point;
			visibleFaces.push_back(currentFace);
			toVisit.push_back(currentFace);

			//Spread from the current face to the adjacent ones until no new
			//face can be added
			while(!toVisit.empty()) {
				currentFace = toVisit.back();
				toVisit.pop_back();
				auto e = currentFace->outerComponent;
				//Go through the adjacent faces
				do {
					auto adjFace = e->twin->incidentFace;
					if(adjFace->lastVisitedBy != point && ConvexHull<Point>::visible(adjFace, *point)) {
						adjFace->lastVisitedBy = point;
						visibleFaces.push_back(adjFace);
						toVisit.push_back(adjFace);
					}

					//If the adjacent face is not visible, this edge lies on the horizon
					//TODO pull into the othe if-branch
					if(horizonStart == NULL && !ConvexHull<Point>::visible(adjFace, *point)) {
						horizonStart = e;
					}
					e = e->next;
				} while(e != currentFace->outerComponent);
			}
			std::cout << "-- VISIBLE FACES: " << visibleFaces.size() << std::endl;
			assert(horizonStart != NULL);
			//Mark visible faces for deletion later on
			for(auto v : visibleFaces) {
				v->toDelete = true;
			}

			//The horizon should be convex when 2D-projected from the point
			std::vector<typename ConvexHull<Point>::HalfEdge*> horizon;
			auto currentHorizon = horizonStart;

			//Build the horizon step by step until the loop is closed
			do {
				horizon.push_back(currentHorizon);
				//Find adjacent edge that is on the horizon
				auto nextEdge = currentHorizon->next;
				while(ConvexHull<Point>::visible(nextEdge->twin->incidentFace, *point)) {
					nextEdge = nextEdge->twin->next;
				}
				currentHorizon = nextEdge;
			} while(currentHorizon != horizonStart);

			std::cout << "-- LENGTH OF HORIZON: " << horizon.size() << std::endl;

			//Now iterate over the horizon and build the new faces
			//Save the last one so that we can go around the horizon
			std::vector<typename ConvexHull<Point>::Face*> newFaces;
			auto prev = horizon.back();
			newFaces.push_back(CH.addFace(prev, point));
			for(auto e : horizon) {
				if(e != horizon.back()) {
					//For each one create the new triangular facet to the point
					auto f = CH.addFace(e, point);
					newFaces.push_back(f);

					//Assume you are going in CCW order?
					assert(prev->twin->origin == e->origin);

					//Link to the prev face
					prev->next->twin = e->prev;
					e->prev->twin = prev->next;

					prev = e;
					assert(e->prev->twin->twin == e->prev);
					assert(e->prev->twin->origin == e->origin);
				} else {
					//Went through the whole horizon, join the start and the end,
					//but don't create a new face
					prev->next->twin = e->prev;
					e->prev->twin = prev->next;

					assert(e->prev->twin->twin == e->prev);
					assert(e->prev->twin->origin == e->origin);
				}
			}
			std::cout << "-- NEW FACES: " << newFaces.size() << std::endl;

			int visiblePoints = 0;
			int assignedPoints = 0;
			//Also reassign the points of the old visible faces to the new faces
			for(auto v : visibleFaces) {
				for(auto p : v->visiblePoints) {
					visiblePoints++;
					bool visible = false;
					double d = std::numeric_limits<double>::max();
					typename ConvexHull<Point>::Face* closestFace = NULL;
					//Find closest face
					for(auto f : newFaces) {
						double distance = ConvexHull<Point>::distance(f, *p);
						if(ConvexHull<Point>::visible(f, *p) && distance > eps) {
							if(distance < d) {
								d = distance;
								visible = true;
								closestFace = f;
							}
						}
					}
					if(visible) {
						closestFace->visiblePoints.push_back(p);
						assignedPoints++;
					}
				}
				v->visiblePoints.clear();
			}
			std::cout << "-- VISIBLE POINTS: " << visiblePoints << std::endl;
			std::cout << "-- NEW ASSIGNED POINTS: " << assignedPoints << std::endl;
			//Push the new faces into the faces stack
			for(auto f : newFaces) {
				std::cout << "*";
				if(!f->visiblePoints.empty()) {
					std::cout << "=";

// std::cout << *f->outerComponent->origin << "\n" << std::endl;
					// std::cout << *f->outerComponent->next->origin << "\n" << std::endl;
					// std::cout << *f->outerComponent->next->next->origin << "\n" << std::endl;

					facesStack.push_back(f);
				}
			}
			std::cout << std::endl;
			std::cout << "-- NEW STACK SIZE: " << facesStack.size() << std::endl;
			int remainingPoints = 0;
			for(auto f : facesStack) {
				remainingPoints += f->visiblePoints.size();
			}
			std::cout << "-- REMAINING POINTS IN STACK: " << remainingPoints << std::endl;


			//Remember to delete the old visible faces (which are no longer in the CH)
		}

		CH.cleanup();
		std::cout << "** TOTAL FACES IN CH: " << CH.faces().size() << std::endl;
		return CH;
	}
}
#endif
