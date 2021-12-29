#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    std::cout << j << std::endl;
    // matrix add i + j
    std::cout << i + j << std::endl;
    // matrix scalar multiply i * 2.0
    std::cout << i * 2.0 << std::endl;
    // matrix multiply i * j
    std::cout << i * j << std::endl;
    // matrix multiply vector i * v
    std::cout << i * v << std::endl;

    // others
    std::cout << "Example of others \n";
    std::cout << v.dot(w) << std::endl;
    std::cout << w.dot(v) << std::endl;

    v.normalize();

    std::cout << v.cross(w) << std::endl;
    std::cout << w.cross(v) << std::endl;

    Eigen::Matrix3f mat = Eigen::Matrix3f::Identity();
    std::cout << mat << std::endl;

    Eigen::AngleAxisf r(M_PI / 2, Eigen::Vector3f(1.0, 0.0, 0.0));
    std::cout << r.toRotationMatrix() << std::endl;
    // std::cout << r.inverse().matrix() << std::endl;
    // std::cout << r.matrix() << std::endl;

    // Eigen::Quaternionf q0 = Eigen::Quaternionf::Identity();
    // std::cout << q0.coeffs() << std::endl;
    Eigen::Quaternionf q(r);
    // std::cout << q.coeffs() << std::endl;
    std::cout << q.toRotationMatrix() << std::endl;
    std::cout << q.norm() << std::endl;
    // std::cout << q.matrix().inverse() << std::endl;
    // std::cout << q.inverse().coeffs() << std::endl;
    std::cout << q.conjugate().matrix() << std::endl;

    Eigen::Matrix4f m1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f m2;
    m2 << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    m1.block<3, 3>(0, 0) = m2;
    std::cout << m1 << std::endl;

    return 0;
}