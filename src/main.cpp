//#include "../inc/vehicle.h"
#include "../inc/solver.h"

#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

bool EulerDist(float x1, float y1, float x2, float y2){

    //returs true if the vehicle is in 1 cm radius of the target
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2,2));

}

// vector<float> getNextPoint(float xLoc, float yLoc, float steerAng, float time){

    
// }

int main(){

    SimEnv simEnv;

    simEnv.boundary = {0, 0, 20, 20};

    simEnv.objects = {{5, 5, 8, 8},
                      {10, 15, 13, 19},
                      {15, 5, 16, 18}};
    
    //vehicle physical parameters
    float vehWheelBase = 1.5; 
    float vehMass = 1500;

    //target location for the vehicle
    float xTar = 20;
    float yTar = 20;

    Vehicle vehicle (vehWheelBase, vehMass);

    //vehicle initial configuration
    vehicle.state = {1, 1, 0};

    Vector2f input = {60 * M_PI/180, 5};

    int maxItr = 100;
    int itr = 0;

    Vector3f nextState, nextState1, nextState2;
    ODESolver odeSolver;

    nextState1 = odeSolver.updateState(vehicle, 0, 2, input, 0.05, "RK4");
    nextState2 = odeSolver.updateState(vehicle, 0, 2, input, 0.05, "euler");

    cout << "RK4" << nextState1 << endl;
    cout << "euler" << nextState2 << endl;




    // while (!isReached(car.xLoc, car.yLoc, xTar, yTar) && itr < maxItr)
    // {

    // }


    return 0;   


}