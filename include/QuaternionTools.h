#pragma once

#include <Eigen/Dense>
#include <cmath>
using namespace std;
constexpr double PI = 3.14159265358979323846; // since M_PI in cmath does not work for some reason

class QuaternionTools {
public:
    inline static Eigen::Vector4d referenceQuat{1.0, 0.0, 0.0, 0.0};

    static void setReference(const Eigen::Vector4d& refQuat);

    static Eigen::Vector4d toRelative(const Eigen::Vector4d& worldQuat);

    static Eigen::Vector4d toWorld(const Eigen::Vector4d& relativeQuat);

    static void normalize(Eigen::Vector4d& q);

    static Eigen::Vector4d multiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);

    static Eigen::Vector4d derivative(const Eigen::Vector4d& q, const Eigen::Vector3d& omega);

    static Eigen::Vector4d inverse(const Eigen::Vector4d& q);

    static Eigen::Vector3d rotateVector(const Eigen::Vector4d& q, const Eigen::Vector3d& v);

    static Eigen::Vector4d rotateQuat(const Eigen::Vector4d& q, const char axis, const double degrees);

    static Eigen::Vector3d toEulerAngles(const Eigen::Vector4d& quat);
};
