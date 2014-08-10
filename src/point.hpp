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

/*
 * Returns the distance from the point p to the line defined by xy.
 */
template<class Point>
double distanceToLine(const Point & x, const Point & y, const Point & p) {
	// d^2 = (|x - p|^2 * |y - x|^2 - ((x - p) * (y - x))^2) / |y - x|^2
	Point px = sub(x, p);
	Point xy = sub(y, x);
	double nom = pow(norm(px), 2) * pow(norm(xy), 2) - pow(dot(px, xy), 2);
	double d = nom / pow(norm(xy), 2);
	return sqrt(d);
}

inline double clamp(double x, double a, double b) {
	return (x < a) ? a : ((x > b) ? b : x);
}

/*
 * Returns the distance from the point p to the CCW triangle defined by xyz.
 */
template<class Point>
double distanceToTriangle(const Point & x, const Point & y,
                          const Point & z, const Point & p)
{
	Point diff = sub(x, p);
	Point edge0 = sub(y, x);
	Point edge1 = sub(z, x);

	double a = dot(edge0, edge0);
	double b = dot(edge0, edge1);
	double c = dot(edge1, edge1);
	double d = dot(edge0, diff);
	double e = dot(edge1, diff);
	double f = dot(diff, diff);

	double det = a * c - b * b;
	double s = b * e - c * d;
	double t = b * d - a * e;

	if ( s + t < det )
	{
		if ( s < 0.0 )
		{
			if ( t < 0.0 )
			{
				if ( d < 0.0 )
				{
					s = clamp( -d/a, 0.0, 1.0 );
					t = 0.0;
				}
				else
				{
					s = 0.0;
					t = clamp( -e/c, 0.0, 1.0 );
				}
			}
			else
			{
				s = 0.0;
				t = clamp( -e/c, 0.0, 1.0 );
			}
		}
		else if ( t < 0.0 )
		{
			s = clamp( -d/a, 0.0, 1.0 );
			t = 0.0;
		}
		else
		{
			float invDet = 1.0 / det;
			s *= invDet;
			t *= invDet;
		}
	}
	else
	{
		if ( s < 0.0 )
		{
			float tmp0 = b+d;
			float tmp1 = c+e;
			if ( tmp1 > tmp0 )
			{
				float numer = tmp1 - tmp0;
				float denom = a-2*b+c;
				s = clamp( numer/denom, 0.0, 1.0 );
				t = 1-s;
			}
			else
			{
				t = clamp( -e/c, 0.0, 1.0 );
				s = 0.0;
			}
		}
		else if ( t < 0.0 )
		{
			if ( a+d > b+e )
			{
				float numer = c+e-b-d;
				float denom = a-2*b+c;
				s = clamp( numer/denom, 0.0, 1.0 );
				t = 1-s;
			}
			else
			{
				s = clamp( -e/c, 0.0, 1.0 );
				t = 0.0;
			}
		}
		else
		{
			float numer = c+e-b-d;
			float denom = a-2*b+c;
			s = clamp( numer/denom, 0.0, 1.0 );
			t = 1.0 - s;
        }
    }

	double q = a*s*s + 2*b*s*t + c*t*t + 2*d*s + 2*e*t + f;
	return sqrt(q);
}

#endif
