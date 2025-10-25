#pragma once

#include <Eigen/Dense>
#include <cmath>
using namespace std;

class QuaternionTools {
public:
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

    static Eigen::Vector3d toEulerAngles(const Eigen::Vector4d& quat){
        // using the Yaw-Pitch-Roll convention for transformation
        // returns orientation in euler angles (in degrees)

        double w = quat(0), x = quat(1), y = quat(2), z = quat(3);
        Eigen::Vector3d eulerAngles;

        // for roll:
        eulerAngles(0) = atan2( 2*(w*x + y*z), 1 - 2*(x*x + y*y));

        // for pitch:
        double sin_pitch = 2 * (w*y - z*x);
        if (abs(sin_pitch) >= 1)
            eulerAngles(1) = copysign(M_PI/2, sin_pitch); 
        else
            eulerAngles(1) = asin(sin_pitch);
        
        // for yaw:
        eulerAngles(2) = atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) );
        

        return eulerAngles * (180/M_PI); // return in degrees (roll, pitch, yaw)
    }

    static Eigen::Vector4d RotationMatrixToQuat(const Eigen::Matrix3d& R) {
        // converting rotation matrix to a quaternion

        Eigen::Vector4d q;
        double trace = R.trace();
        
        if (trace > 0) {
            double s = 0.5 / sqrt(trace + 1.0);
            q(0) = 0.25 / s;                   
            q(1) = (R(2,1) - R(1,2)) * s;      
            q(2) = (R(0,2) - R(2,0)) * s;      
            q(3) = (R(1,0) - R(0,1)) * s;      
        } else if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
            double s = 2.0 * sqrt(1.0 + R(0,0) - R(1,1) - R(2,2));
            q(0) = (R(2,1) - R(1,2)) / s;      
            q(1) = 0.25 * s;                   
            q(2) = (R(0,1) + R(1,0)) / s;      
            q(3) = (R(0,2) + R(2,0)) / s;      
        } else if (R(1,1) > R(2,2)) {
            double s = 2.0 * sqrt(1.0 + R(1,1) - R(0,0) - R(2,2));
            q(0) = (R(0,2) - R(2,0)) / s;      
            q(1) = (R(0,1) + R(1,0)) / s;      
            q(2) = 0.25 * s;                   
            q(3) = (R(1,2) + R(2,1)) / s;      
        } else {
            double s = 2.0 * sqrt(1.0 + R(2,2) - R(0,0) - R(1,1));
            q(0) = (R(1,0) - R(0,1)) / s;      
            q(1) = (R(0,2) + R(2,0)) / s;      
            q(2) = (R(1,2) + R(2,1)) / s;      
            q(3) = 0.25 * s;            
        }
        
        return q;
    }

    static Eigen::Vector4d AxisAngleToQuat(const Eigen::Vector3d& axis, double angle) {
        // converting axis-angle to quaternion
        
        Eigen::Vector4d quat;
        quat(0) = cos(angle/2);              
        quat(1) = axis(0) * sin(angle/2);                  
        quat(2) = axis(1) * sin(angle/2);                  
        quat(3) = axis(2) * sin(angle/2);                  
        
        return quat;
    }
};
