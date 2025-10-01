#include "../include/cal_base.hpp"

Qua::Qua()
    : a(0), b(0), c(0), d(0) {}

Qua::Qua(double aa, double bb, double cc, double dd)
    : a(aa), b(bb), c(cc), d(dd) {}

void Qua::show() const {
    std::cout << a << b << c << d << std::endl;
}

Qua Qua::operator+(const Qua& other) const {
    return Qua(a + other.a, b + other.b, c + other.c, d + other.d);
}

Qua Qua::bar() const {
    return Qua(a, -b, -c, -d);
}

Qua Qua::operator*(const Qua& other) const {
    double a1 = a      , b1 = b      , c1 = c      , d1 = d;
    double a2 = other.a, b2 = other.b, c2 = other.c, d2 = other.d;
    return Qua(
            a1*a2 - b1*b2 - c1*c2 - d1*d2,
	    b1*a2 + a1*b2 - d1*c2 + c1*d2,
	    c1*a2 + d1*b2 + a1*c2 - b1*d2,
	    d1*a2 - c1*b2 + b1*c2 + a1*d2);
}

double Qua::norm() const {
    return std::sqrt(a*a + b*b + c*c + d*d);
}

Qua Qua::normalize() const {
    double n = norm();
    if ( n < 1e-10) throw std::runtime_error("error: normalize() norm is almost 0");
    double inv = 1.0 / n;
    return Qua(a*inv, b*inv, c*inv, d*inv);
}

Qua Qua::getq(double rad, const Qua& u) {
    Qua un = u.normalize();
    double sin = std::sin(rad/2.0);
    double cos = std::cos(rad/2.0);
    return Qua(cos, sin*u.b, sin*u.c, sin*u.d);
}

void rot0(const Vec3& p0, const Vec3& axis, double rad, Vec3& p1) {
    Qua u,src;
    u.a   = 0, u.b = axis[0], u.c = axis[1], u.d = axis[2];
    src.a = 0, src.b = p0[0], src.c = p0[1], src.d = p0[2];
    u = u.normalize();
    Qua q   = Qua::getq(rad, u);
    Qua qa  = q * src;
    Qua dst = qa * q.bar();
    p1[0] = dst.b;
    p1[1] = dst.c;
    p1[2] = dst.d;
}

void rot(const Vec3& p0, const Vec3& axis, const Vec3& ofs, double rad, Vec3& p1) {
	Vec3 org;
	org[0]  = p0[0];  org[1]  = p0[1];  org[2]  = p0[2];
	org[0] -= ofs[0]; org[1] -= ofs[1]; org[2] -= ofs[2]; 
	rot0(org, axis, rad, p1);
	p1[0]  += ofs[0]; p1[1]  += ofs[1]; p1[2]  += ofs[2];
}
