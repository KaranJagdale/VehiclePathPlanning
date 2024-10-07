#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <functional>
#include "../inc/spline.h"
#include "../inc/solver.h"

int main()
{
    Eigen::Vector3f vec1(3.4, 7.4, 0);
    Eigen::Vector3f vec2(3.4, 6.4, 0);
    std::cout << eulerDist(vec1(0), vec1(1), vec2(0), vec2(1)) << std::endl;

    return 0;

}
    
