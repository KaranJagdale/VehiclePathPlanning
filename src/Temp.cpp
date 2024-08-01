#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

int main()
{

    double a = 2.3;
    float b = 2.3;

    if (b == (float) 2.3)
    {
        cout << "equal" << "\n";
    }
    else
    {
        cout << "not equal" << "\n";
    }
    return 0;
}