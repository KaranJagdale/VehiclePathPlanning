// This file contains vehicle class. As of now it will implement simple bicycle model
using namespace std;
#include <iostream>
#include <vector>

class Vehicle{
private:

    //physical properties of vehicle
    float wheel_base;
    float mass;
public:

    //Location and orientation of the vehicle
    //location is in meters and heading is in degrees from x-axis
    float xLoc;
    float yLoc;
    float heading;

    Vehicle(float vehicle_wheel_base, float vehicle_mass)
    {
        wheel_base = vehicle_wheel_base; 
        mass = vehicle_mass;
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

