#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main()
{
    Vector3f a = {1.2, 3.0, 4.0}; 
    // a.setOnes();
    // a = a * 2;
    Vector3f b = a * 2.2;
    //a = a * b;
    cout <<a(2) << endl;


    return 0;
}