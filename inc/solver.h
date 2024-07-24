// This file contains vehicle class. As of now it will implement simple bicycle model
using namespace std;
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "../inc/vehicle.h"

using namespace Eigen;

class ODESolver{
public:
    //ODESolver();
    stateWithDistance updateStateWithDist(Vehicle vehicle,float t0, float t1, Vector2f input,
                         float timeStep, string method)
    {
        //use the below function to compute the integral from t0 to t1
        //structWithDistance have two datamembers state and distance
        stateWithDistance ret;
        
        float distance = 0;

        Vector3f smallUpdate;

        float t = t0;

        while (t < t1)
        {
            if (method == "RK4")
            {
                smallUpdate =  RK4Update(vehicle, input, timeStep);
            }
            else if (method == "euler")
            {
                smallUpdate = EulerUpdate(vehicle, input, timeStep);
            }
            else
            {
                cout << "Please input a valid integration method" << endl;
            }

            vehicle.state += smallUpdate;

            distance += sqrt(pow(smallUpdate(0),2) + pow(smallUpdate(1), 2));

            // Wraping the heading angle between (0, 2pi]
            if (vehicle.state(2) > 2*M_PI)
            {
                vehicle.state(2) -= 2*M_PI;
            } 
            else if(vehicle.state(2) < 0)
            {
                vehicle.state(2) += 2*M_PI;
            }

            t += timeStep;

        }

        //distance = sqrt(distance);

        ret.state = vehicle.state;

        ret.distance = distance;

        return ret;
    }
    //write a function to compute the area under the curve for small time delta

    Vector3f RK4Update(Vehicle vehicle, Vector2f input, float timeStep)
    {
        //Integral update with RK4 for a time-step

        //parameters of RK4
        Vector3f k1, k2, k3, k4, y1, y2, y3, res;

        k1 = vehicle.kinematics_(input, vehicle.state);
        y1 = vehicle.state + (k1 * timeStep/2.0);

        k2 = vehicle.kinematics_(input, y1);
        y2 = vehicle.state + (k2 * timeStep/2.0);

        k3 = vehicle.kinematics_(input,y2);
        y3 = vehicle.state + k3 * timeStep;

        k4 = vehicle.kinematics_(input,y3);

        res = (k1 + 2*k2 + 2*k3 + k4)*timeStep/6;   
        
        return res;
    }

    Vector3f EulerUpdate(Vehicle vehicle, Vector2f input, float timeStep)
    {
        //Integral update with Euler for a time-step
        
        Vector3f res;
        res = vehicle.kinematics_(input, vehicle.state) * timeStep;
        return res;

    }
    
};