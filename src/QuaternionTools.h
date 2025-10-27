#pragma once

#include <Eigen/Dense>
#include <cmath>
using namespace std;
constexpr double PI = 3.14159265358979323846;

class QuaternionTools {
public:
    // used to set and correct the "zero" angle reference
    inline static double pitchOffset;
    inline static double yawOffset;
    inline static double rollOffset;

    static void setOffsets(double pitchDeg, double yawDeg, double rollDeg) {
        pitchOffset = pitchDeg * PI / 180.0;
        yawOffset   = yawDeg   * PI / 180.0;
        rollOffset  = rollDeg  * PI / 180.0;
    }

    static void normalize(Eigen::Vector4d& q) {
        q /= q.norm();
    }

    static Eigen::Vector4d multiply(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
        double w1 = q1(0), x1 = q1(1), y1 = q1(2), z1 = q1(3);
        double w2 = q2(0), x2 = q2(1), y2 = q2(2), z2 = q2(3);
        Eigen::Vector4d result;
        result(0) = w1*w2 - x1*x2 - y1*y2 - z1*z2;
        result(1) = w1*x2 + x1*w2 + y1*z2 - z1*y2;
        result(2) = w1*y2 - x1*z2 + y1*w2 + z1*x2;
        result(3) = w1*z2 + x1*y2 - y1*x2 + z1*w2;
        return result;
    }

    static Eigen::Vector4d derivative(const Eigen::Vector4d& q, const Eigen::Vector3d& omega) {
        Eigen::Vector4d omegaQuat(0.0, omega(0), omega(1), omega(2));
        Eigen::Vector4d qDot = multiply(q, omegaQuat);
        return 0.5 * qDot;
    }

    static Eigen::Vector4d inverse(const Eigen::Vector4d& q) {
        Eigen::Vector4d qInv;
        qInv(0) = q(0);
        qInv(1) = -q(1);
        qInv(2) = -q(2);
        qInv(3) = -q(3);
        return qInv;
    }

    static Eigen::Vector3d rotateVector(const Eigen::Vector4d& q, const Eigen::Vector3d& v) {
        Eigen::Vector4d vQuat(0.0, v(0), v(1), v(2));
        Eigen::Vector4d qInv = inverse(q);
        Eigen::Vector4d rotatedQuat = multiply(multiply(q, vQuat), qInv);
        return Eigen::Vector3d(rotatedQuat(1), rotatedQuat(2), rotatedQuat(3));
    }

    static Eigen::Vector4d rotateQuat(const Eigen::Vector4d& q, char axis, double degrees){
        
        Eigen::Vector4d rotationQuat;
        Eigen::Vector4d newQuat;
        Eigen::Vector3d unitAxis(0.0, 0.0, 0.0);

        double angle = degrees * (PI/180); // convert to rad
        
        if (axis == 'x'){
            unitAxis(0) = 1.0;
        }else if (axis == 'y'){
            unitAxis(1) = 1.0;
        }else if (axis == 'z'){
            unitAxis(2) = 1.0;
        }else{
            return q; // send quat unchanged if invalid axis is given
        }

        rotationQuat(0) = cos(angle/2.0);              
        rotationQuat(1) = unitAxis(0) * sin(angle/2.0);                  
        rotationQuat(2) = unitAxis(1) * sin(angle/2.0);                  
        rotationQuat(3) = unitAxis(2) * sin(angle/2.0);    

        newQuat = multiply(rotationQuat, q);
        normalize(newQuat);

        return newQuat;
    }

    static Eigen::Vector3d toEulerAngles(const Eigen::Vector4d& quat){
        // using Pitch-Yaw-Roll (YZX) convention
        
        double w = quat(0), x = quat(1), y = quat(2), z = quat(3);
        Eigen::Vector3d eulerAngles;

        // pitch
        eulerAngles(0) = atan2(2.0*(w*y - x*z), 1.0 - 2.0*(y*y + z*z));
        eulerAngles(0) -= pitchOffset;

        // yaw
        double sin_yaw = 2.0 * (w*z + x*y);
        if (abs(sin_yaw) >= 1){
            eulerAngles(1) = copysign(PI/2, sin_yaw); 
        }else{
            eulerAngles(1) = asin(sin_yaw);
        }
        eulerAngles(1) -= yawOffset;

        // roll
        eulerAngles(2) = atan2(2.0*(w*x - y*z), 1.0 - 2.0*(x*x + z*z));
        eulerAngles(2) -= rollOffset;

        return eulerAngles * (180.0/PI); // return in degrees (pitch, yaw, roll)
    }
};
