#ifndef POINT_HPP
#define POINT_HPP

#include <algorithm>
#include <limits>
#include <stdlib.h>
#include <iostream>
#include <cmath>

template<class Point>
Point cross(const Point& x, const Point& y) {
	Point z;
	z[0] = x[1]*y[2] - y[1]*x[2];
	z[1] = x[2]*y[0] - y[2]*x[0];
	z[2] = x[0]*y[1] - y[0]*x[1];
	return z;
}

template<class Point>
Point sub(const Point& x, const Point& y) {
	Point z;
	z[0] = x[0] - y[0];
	z[1] = x[1] - y[1];
	z[2] = x[2] - y[2];
	return z;
}

template<class Point>
double dot(const Point& x, const Point& y) {
	return x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
}

template<class Point>
bool collinear(const Point& p1, const Point& p2, const Point& p3) {
	//x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)
	return p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) +  p3[0] * (p1[1] - p2[1]) == 0;
}

template<class Point>
bool coplanar(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
	Point a = sub(p3, p1);
	Point b = sub(p2, p1);
	Point c = sub(p4, p3);

	Point d = cross(b, c);
	return 0 == dot(a, d);
}

template<class Point>
bool inFront(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
	Point x = sub(p2, p1);
	Point y = sub(p3, p1);
	Point n = cross(x, y);

	return dot(n, sub(p4, p1)) >= 0;
}


template<class Point>
double distance(const Point& a, const Point& b) {
	return sqrt(std::pow(a[0] - b[0], 2) + std::pow(a[1] - b[1], 2) + std::pow(a[2] - b[2], 2));
}

template<class Point>
double norm(const Point& a) {
	return sqrt(std::pow(a[0], 2) + std::pow(a[1], 2) + std::pow(a[2], 2));
}

template<class Point>
double angle(const Point& a, const Point& b, const Point& c) {
	Point v1 = sub(a, b);
	Point v2 = sub(c, b);
	double d1 = norm(v1);
	double d2 = norm(v2);
	double prod = dot(v1, v2);

	return std::max(0.0, acos(prod / (d1 * d2)));
}

#endif
