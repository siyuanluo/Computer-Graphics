#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){
    Eigen::Vector3f point(2.0f,1.0f,1.0f);  // Point(2,1)
    Eigen::Matrix3f i;
    i << cos(1.0/4 *acos(-1)), -sin(1.0/4 *acos(-1)), 1.0f, sin(1.0/4 *acos(-1)), cos(1.0/4 *acos(-1)), 2.0f, 0, 0, 1.0f;
    std::cout << (i * point) << std::endl;
    return 0;   
}