// This file contains vehicle class. As of now it will implement simple bicycle model
using namespace std;
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include<math.h>

using namespace Eigen;
using namespace std;

class Vehicle{
private:

    //physical properties of vehicle
    float wheel_base;
    float mass;
public:

    //state contains ocation and orientation of the vehicle
    //in the format, (xLoc, yLoc, heading)
    //location is in meters and heading is in degrees from x-axis
    
    Vector3f state;

    Vehicle(float vehicle_wheel_base, float vehicle_mass)
    {
        wheel_base = vehicle_wheel_base; 
        mass = vehicle_mass;
    }

    Vector3f kinematics(Vector2f input){
        
        //returns dot(state)
        //inputs are steering angle and velocity resp

        Vector3f ret;

        float headRad = state(3) * M_PI / 180;

        ret << cos(headRad) * input(2), sin(headRad) * input(2), input(2) * tan(input(1)) / wheel_base;

        return ret;
    }

    Vector3f kinematics_(Vector2f input, Vector3f vehState){
        
        //returns dot(state)
        //inputs are steering angle (radian) and velocity resp 
        //state is separately provided, it helps computing RK4 without modifying the class

        Vector3f ret;

        float headRad = vehState(2);

        ret << cos(headRad) * input(1), sin(headRad) * input(1), input(1) * tan(input(0)) / wheel_base;

        return ret;
    }


};

class Boundary{
public:
    float* pos; 
    float* size;

    Boundary();

    Boundary(float* obj_pos, float* obj_size){
        //cout<<"created a boundary"<<endl;
        pos = obj_pos;
        size = obj_size;
    }
};

class SimEnv{
public:

    //the structure of the boundary is (x1, y1, x2, y2)
    //(x1, y1) and (x2, y2) are diagonally opposite vertices
    vector<float> boundary;

    //objects are written with same structure as boundary
    vector<vector<float>> objects;

    //SimEnv();
};

