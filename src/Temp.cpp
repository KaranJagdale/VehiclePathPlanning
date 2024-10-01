#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <functional>
#include "../inc/spline.h"

int main()
{
    std::vector<double> X, Y;
    X = {1,2};

    Y = {1, 4};

    tk::spline s(X,Y);
    Eigen::Vector2f vec1(-1,1);
    Eigen::Vector2f vec2(1,0);
    float angle = acos(vec1.dot(vec2) / sqrt(vec1.dot(vec1)) / sqrt(vec2.dot(vec2)));
    std::cout << angle * 180 / M_PI << std::endl;

    return 0;

}
    
