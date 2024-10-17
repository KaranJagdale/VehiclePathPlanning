#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <functional>
#include "../inc/spline.h"
#include "../inc/solver.h"

int main()
{
    Eigen::Matrix2f mat{{1,2},{3,4}};
    Eigen::Vector2f vec{1, 0};

    float a = 1;
    if(true)
    {
        a = 2;
    }
    std::cout << a << std::endl;

    return 0;

}
    
