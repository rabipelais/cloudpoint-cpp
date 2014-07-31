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
		mFaces.push_back(base);

		//Link the edges
		ab->next = bc; bc->next = ca; ca->next = ab;
		ab->prev = ca; bc->prev = ab; ca->prev = bc;

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
		ba->next = ad; ad->next = db->next = ba;
		ba->prev = db; db->prev = ad->prev = ba;
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
			assert(f->outerComponent->incidentFace = f);
		}
		for(HalfEdge* e : mHalfEdges) {
			assert(e->prev->next = e);
			assert(e->next->prev = e);
			assert(e->twin->twin = e);
			assert(e->twin->next->origin = e->origin);
			assert(e->prev->twin->origin = e->origin);
			assert(e->prev->incidentFace == e->incidentFace);
			assert(e->next->incidentFace == e->incidentFace);
		}
	}

private:
	std::vector<HalfEdge*> mHalfEdges;
	std::vector<Face*> mFaces;
};


template<typename P, typename F>
class ConflictGraph {
public:
	struct FNode;
	struct PNode {
		P* val;
		std::vector<FNode*> flist;
	};
	struct FNode {
		F* val;
		std::vector<PNode*> plist;
	};

	PNode* addPNode(P* p, FNode* fn) {
		PNode *pn = new PNode;
		pn->val = p;
		pn->flist.push_back(fn);
		fn->plist.push_back(pn);
		pnodes.push_back(pn);
		return pn;
	}

	FNode* addFNode(F* f) {
		FNode* fn = new FNode;
		fn->val = f;
		fnodes.push_back(fn);
		return fn;
	}

	~ConflictGraph() {
		for(auto f : fnodes) {
			delete f;
		}
		for(auto p : pnodes) {
			delete p;
		}
	}

private:
	std::vector<FNode*> fnodes;
	std::vector<PNode*> pnodes;
};


template<typename T>
using ConvexHull = DoublyLinkedEdgeList<T>;

namespace Convex {

	template<class Point>
	ConvexHull<Point> convexHull(std::vector<Point>& points) {
		//Try to find four non-coplanar points, and remove them from the points to be processed
		std::vector<Point*> remaining;
		Point& p1 = points[0];
		Point& p2 = points[1];
		Point& p3 = p1;
		Point& p4 = p1;

		unsigned int i = 2;
		//First find the first non-collinear point to p1 and p2
		for(; i < points.size(); i++) {
			Point& p = points[i];
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
			Point& p = points[i];
			if(!coplanar(p1, p2, p3, p)) {
				p4 = p;
				break;
			}
			remaining.push_back(&p);
		}

		ConvexHull<Point> CH;
		CH.addTetrahedron(p1, p2, p3, p4);

		auto faces = CH.faces();

		//Create conflict graph
		//std::vector<std::vector<typename DoublyLinkedEdgeList<Point>::Face*> > conflicts;
		ConflictGraph<Point, typename DoublyLinkedEdgeList<Point>::Face> conflicts;
		for(auto f : faces) {
			auto fn = conflicts.addFNode(f);
			for(auto p : remaining) {
				//Push the faces that ARE visible from each point
				if(DoublyLinkedEdgeList<Point>::visible(f, *p)) {
					conflicts.addPNode(p, fn);
				}
			}
		}

		while(!remaining.empty()) {
			Point* p = remaining.back();
			remaining.pop_back();
		}

		return CH;
	}
}
#endif
