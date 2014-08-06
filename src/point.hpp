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
	double a00 = pow(norm(edge0), 2);
	double a01 = dot(edge0, edge1);
	double a11 = pow(norm(edge1), 2);
	double b0 = dot(diff, edge0);
	double b1 = dot(diff, edge1);
	double c = pow(norm(diff), 2);
	double det = abs(a00 * a11 - a01 * a01);
	double s = a01 * b1 - a11 * b0;
	double t = a01 * b0 - a00 * b1;
	double sqrDistance;

	if (s + t <= det) {
		if (s < 0) {
			if (t < 0) {	//region 4
				if (b0 < 0) {
					t = 0;
					if (-b0 >= a00) {
						s = 1;
						sqrDistance = a00 + (2) * b0 + c;
					} else {
						s = -b0 / a00;
						sqrDistance = b0 * s + c;
					}
				} else {
					s = 0;
					if (b1 >= 0) {
						t = 0;
						sqrDistance = c;
					} else if (-b1 >= a11) {
						t = 1;
						sqrDistance = a11 + (2) * b1 + c;
					} else {
						t = -b1 / a11;
						sqrDistance = b1 * t + c;
					}
				}
			} else {		//region 3
				s = 0;
				if (b1 >= 0) {
					t = 0;
					sqrDistance = c;
				} else if (-b1 >= a11) {
					t = 1;
					sqrDistance = a11 + (2) * b1 + c;
				} else {
					t = -b1 / a11;
					sqrDistance = b1 * t + c;
				}
			}
		} else if (t < 0) {	// region 5
			t = 0;
			if (b0 >= 0) {
				s = 0;
				sqrDistance = c;
			} else if (-b0 >= a00) {
				s = 1;
				sqrDistance = a00 + (2) * b0 + c;
			} else {
				s = -b0 / a00;
				sqrDistance = b0 * s + c;
			}
		} else {		//region 0
			// minimum at interior point
			double invDet = (1) / det;
			s *= invDet;
			t *= invDet;
			sqrDistance = s * (a00 * s + a01 * t + (2) * b0) +
				t * (a01 * s + a11 * t + (2) * b1) + c;
		}
	} else {
		double tmp0, tmp1, numer, denom;
		if (s < 0) {			//region 2
			tmp0 = a01 + b0;
			tmp1 = a11 + b1;
			if (tmp1 > tmp0) {
				numer = tmp1 - tmp0;
				denom = a00 - (2) * a01 + a11;
				if (numer >= denom) {
					s = 1;
					t = 0;
					sqrDistance = a00 + (2) * b0 + c;
				} else {
					s = numer / denom;
					t = 1 - s;
					sqrDistance = s * (a00 * s + a01 * t + (2) * b0) +
						t * (a01 * s + a11 * t + (2) * b1) + c;
				}
			} else {
				s = 0;
				if (tmp1 <= 0) {
					t = 1;
					sqrDistance = a11 + (2) * b1 + c;
				} else if (b1 >= 0) {
					t = 0;
					sqrDistance = c;
				} else {
					t = -b1 / a11;
					sqrDistance = b1 * t + c;
				}
			}
		} else if (t < 0)		// region 6
		{
			tmp0 = a01 + b1;
			tmp1 = a00 + b0;
			if (tmp1 > tmp0) {
				numer = tmp1 - tmp0;
				denom = a00 - (2) * a01 + a11;
				if (numer >= denom) {
					t = 1;
					s = 0;
					sqrDistance = a11 + (2) * b1 + c;
				} else {
					t = numer / denom;
					s = 1 - t;
					sqrDistance = s * (a00 * s + a01 * t + (2) * b0) +
						t * (a01 * s + a11 * t + (2) * b1) + c;
				}
			}

			else {
				t = 0;
				if (tmp1 <= 0) {
					s = 1;
					sqrDistance = a00 + (2) * b0 + c;
				} else if (b0 >= 0) {
					s = 0;
					sqrDistance = c;
				} else {
					s = -b0 / a00;
					sqrDistance = b0 * s + c;
				}
			}
		} else				// region 1
		{
			numer = a11 + b1 - a01 - b0;
			if (numer <= 0) {
				s = 0;
				t = 1;
				sqrDistance = a11 + (2) * b1 + c;
			} else {
				denom = a00 - (2) * a01 + a11;
				if (numer >= denom) {
					s = 1;
					t = 0;
					sqrDistance = a00 + (2) * b0 + c;
				} else {
					s = numer / denom;
					t = 1 - s;
					sqrDistance = s * (a00 * s + a01 * t + (2) * b0) +
						t * (a01 * s + a11 * t + (2) * b1) + c;
				}
			}
		}

	}

	// Account for numerical round-off error.
	if (sqrDistance < 0) {
		sqrDistance = 0;
	}

	return sqrt(sqrDistance);
}

#endif
