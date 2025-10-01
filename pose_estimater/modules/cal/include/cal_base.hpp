#ifndef CAL_BASE_HPP
#define CAL_BASE_HPP

#include <array>
#include <cmath>
#include <stdexcept>
#include <cstdio>
#include <iostream>

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

#endif //CAL_HPP
