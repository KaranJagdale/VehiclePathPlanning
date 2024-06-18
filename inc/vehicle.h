// This file contains vehicle class. As of now it will implement simple bicycle model
using namespace std;
#include <iostream>

class Vehicle{
private:
    float wheel_base;
    float mass;
public:
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
    Boundary(float* obj_pos, float* obj_size){
        //cout<<"created a boundary"<<endl;
        pos = obj_pos;
        size = obj_size;
    }

};