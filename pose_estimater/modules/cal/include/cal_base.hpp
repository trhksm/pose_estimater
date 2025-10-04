#ifndef CAL_BASE_HPP
#define CAL_BASE_HPP

#include <array>
#include <cmath>
#include <stdexcept>
#include <cstdio>
#include <iostream>
#include <vector>

using Vec3 = std::array<double,3>;

struct Qua {
    double a, b, c, d;

    Qua();
    Qua(double aa, double bb, double cc, double dd);
    void show() const;
    Qua operator+(const Qua& other) const;
    Qua bar() const;
    Qua operator*(const Qua& other) const;
    double norm() const;
    Qua normalize() const;
    static Qua getq(double rad, const Qua& u);
};

void rot0(const Vec3& p0, const Vec3& axis, double rad, Vec3& p1);
void rot(const Vec3& p0, const Vec3& axis, const Vec3& ofs, double rad, Vec3& p1);

void v3add(const Vec3& a, const Vec3& b, Vec3& out);
void v3sub(const Vec3& a, const Vec3& b, Vec3& out);
void v3mul(double c, const Vec3& a, Vec3& out);
void v3cpy(const Vec3& a, Vec3& out);
double v3len(const Vec3& a);
void v3nrm(const Vec3& a, Vec3& out);
double v3dot(const Vec3& a, const Vec3& b);
void v3crs(const Vec3& a, const Vec3& b, Vec3& out);

double distance_surface(const Vec3& oa, const Vec3& n, const Vec3& op);
void foot_perpendicular(const Vec3& oa, const Vec3& n, const Vec3& op, Vec3& oq);
void line_plane_intersection(const Vec3& oa, const Vec3& n, const Vec3& op, const Vec3& l, Vec3& ox);
#endif //CAL_HPP
