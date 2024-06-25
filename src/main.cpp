#include "../inc/vehicle.h"
#include <iostream>
#include <cmath>

bool isReached(float vehx, float vehy, float tarx, float tary){

    //returs true if the vehicle is in 1 cm radius of the target
    return (sqrt(pow(vehx - tarx, 2) + pow(vehy - tary,2)) < 0.01);

}

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

    Vehicle car (vehWheelBase, vehMass);

    //vehicle initial configuration
    car.xLoc = 1;
    car.yLoc = 1;
    car.heading = 90; 

    int maxItr = 100;
    int itr = 0;

    while (!isReached(car.xLoc, car.yLoc, xTar, yTar) && itr < maxItr)
    {

    }


    return 0;   


}