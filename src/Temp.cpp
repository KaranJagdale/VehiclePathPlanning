#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

int main()
{

    Vector3f a = {1,2,3};

    cout << a(1) << endl;
    return 0;
}