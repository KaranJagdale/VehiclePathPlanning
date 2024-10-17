#include "../inc/solver.h"
#include "../inc/spline.h"
#include <bits/stdc++.h>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <queue>
#include <vector>
#include <unordered_map>
#include <functional>
#include <utility>
#include <string>
#include <fstream>

using namespace std;
using namespace Eigen;

template<typename A> void printQueue(A pq)
{
	while (!pq.empty())
		{
			cout << pq.top() << endl;
			pq.pop();
		}
}

hash<float> hashFloat;

size_t vectorToKey(Vector3f const& vec) 
 {

  size_t seed = vec.size();
  for(auto& i : vec) 
  {
    seed ^= hashFloat(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  return seed;
}

Vector3f constToDiscretezGrid(Vector3f state, SimEnv simEnv, float xRes, 
                                        float yRes, float headRes)
{
    //converts continuous state to the discrete point in the grid
    //xRes, yRes denotes the resolution of the grid in the respective direction

    state(2) *= 180 / M_PI;  //converting to degree

    state(0) = roundf(state(0) / xRes) * xRes;
    state(1) = roundf(state(1) / yRes) * yRes;
    state(2) = roundf(state(2) / headRes) * headRes;

    state(2) *= M_PI / 180; //converting back to radians

    return state;
}

bool pathGridObstacleOverlap(Vector3f currentNode, Vector3f neighborNode, SimEnv simEnv, float xRes, float yRes)
{
    //boundary is stored as x1, y1, x2, y2, where the later coordinates are diagonal coordinate of the first
    vector<float> gridBoundary = {neighborNode(0) - xRes/2, neighborNode(1) - yRes/2,
                                            neighborNode(0) + xRes/2, neighborNode(1) + yRes/2};
    bool intersect = false;
    for(unsigned int i = 0; i < size(simEnv.objects); i+=1)
    {
        vector<float> object = simEnv.objects[i];

        // first we check of the grid of the final node intersects any of the objects
        intersect = ((gridBoundary[0] > object[0] && gridBoundary[0] < object[2]) ||
                    (gridBoundary[2] > object[0] && gridBoundary[2] < object[2])) &&
                    ((gridBoundary[1] > object[1] && gridBoundary[1] < object[3]) ||
                    (gridBoundary[3] > object[1] && gridBoundary[3] < object[3]));
        if (intersect)
        {
            return true;
        }

        // now we check whether the agent's path is intersecting any of the objects
        // finding the closest edges of the obstacles from the agent
        float xClosest;
        if (abs(currentNode[0] - object[0]) < abs(currentNode[0] - object[2]))
        {
            xClosest = object[0];
        }
        else
        {
            xClosest = object[2];
        }

        float yClosest;
        if (abs(currentNode[1] - object[1]) < abs(currentNode[1] - object[3]))
        {
            yClosest = object[1];
        }
        else
        {
            yClosest = object[3];
        }


        // calculate the intersection of the line passing through currentNode -> neighborNode with the closest edges
        float slope = (neighborNode[1] - currentNode[1]) / (neighborNode[0] - currentNode[0]);
        float yIntersect = currentNode[1] + slope * (xClosest - currentNode[0]);

        if (yIntersect >= object[1] && yIntersect <= object[3] && 
                (eulerDist(currentNode[0], currentNode[1], xClosest, yIntersect) < 1))
        {
            return true;
        }

        float xIntersect = (yClosest - currentNode[1]) / slope + currentNode[0];

        if (xIntersect >= object[0] && xIntersect <= object[2] &&
                (eulerDist(currentNode[0], currentNode[1], xIntersect, yClosest) < 1))
        {
            return true;
        }
    }

    
    return false;
}

// Todo: convert VehicleTrajectory to unordered map
VehicleTrajectory generateSmoothTrajectory(vector<Vector3f> vehiclePath, float averageSpeed, float resolution){
    
    VehicleTrajectory vehicleTrajectory;
    // finding the indices where direction of motion is reversed
    vector<unsigned int> reverseIndex;

    for(unsigned int i = 1; i < size(vehiclePath); i++)
    {
        Vector2f vec1(vehiclePath[i + 1](0) - vehiclePath[i](0), vehiclePath[i + 1](1) - vehiclePath[i](1));
        Vector2f vec2(vehiclePath[i](0) - vehiclePath[i - 1](0), vehiclePath[i](1) - vehiclePath[i - 1](1));
        float angle = acos(vec1.dot(vec2) / sqrt(vec1.dot(vec1)) / sqrt(vec2.dot(vec2)));
        if (angle > M_PI / 2)
        {
            reverseIndex.push_back(i);
        }
    }

    // inserting start and stop indices
    reverseIndex.insert(reverseIndex.begin(), 0);
    reverseIndex.push_back(size(vehiclePath) - 1);

    // creating the time vector considering the averageSpeed for spline interpolation
    vector<double> vehicleTimeStamps;
    vehicleTimeStamps.push_back(0);
    for (int i = 1; i < size(vehiclePath); i++)
    {
        // Note: (i / averageSpeed) is a float as i is int and averageSpeed is float
        float dist = eulerDist(vehiclePath[i](0), vehiclePath[i](1), vehiclePath[i-1](0), vehiclePath[i-1](1));
        if (dist == 0)
        {
            cout << "Warning: zero distance found and the vertices are:" << endl;
            printVector(vehiclePath[i]);
            printVector(vehiclePath[i-1]);
        }
        vehicleTimeStamps.push_back(vehicleTimeStamps[i-1] + dist / averageSpeed);
    }

    // adding first element manually in the trajectory
    vehicleTrajectory.time.push_back(vehicleTimeStamps.front());
    vehicleTrajectory.state.push_back(vehiclePath.front());
    
    for(int i = 0; i < size(reverseIndex) - 1; i++)
    {
        vector<double> timeSubVector(vehicleTimeStamps.begin() + reverseIndex[i],
                                        vehicleTimeStamps.begin() + reverseIndex[i + 1] + 1);

        // creating vector for each state element from vector<Vector3f>
        vector<double> xSubVector, ySubVector, thetaSubVector;

        for (int j = reverseIndex[i]; j <= reverseIndex[i + 1]; j++)
        {
            xSubVector.push_back((double)vehiclePath[j][0]);
            ySubVector.push_back((double)vehiclePath[j][1]);
            thetaSubVector.push_back((double)vehiclePath[j][2]);
        }

        

        //checking if timeSubVector has sufficient (>= 3) elements for generating spline
        if (size(timeSubVector) >= 3)
        {
            // creating splines for all the elements of the state
            tk::spline xSpline(timeSubVector, xSubVector);
            tk::spline ySpline(timeSubVector, ySubVector);
            tk::spline thetaSpline(timeSubVector, thetaSubVector); 

            // computing the spline at the distinct points defined by the resolution. Excluding first element since it's been already added   
            for (float t = timeSubVector.front() + resolution; t <= timeSubVector.back(); t += resolution)
            {
                vehicleTrajectory.time.push_back(t);

                // todo: instead of creating stateFromSpline variable, use Vector3f literal in the push_back

                Vector3f stateFromSpline((float)xSpline(t), (float)ySpline(t), (float)thetaSpline(t));
                vehicleTrajectory.state.push_back(stateFromSpline);   
            }
        }
        else
        {
            for (float t = timeSubVector.front() + resolution; t <= timeSubVector.back(); t += resolution)
            {
                vehicleTrajectory.time.push_back(t);
                
                // if we only have two elements, we use linear interpolation
                vector<Vector3f> stateSubVector(vehiclePath.begin() + reverseIndex[i],
                                        vehiclePath.begin() + reverseIndex[i + 1] + 1); 
                
                Vector3f stateInterp = stateSubVector[0] + (stateSubVector[1] - stateSubVector[0])/(timeSubVector[1] - timeSubVector[0]) * (t - timeSubVector[0]);

                vehicleTrajectory.state.push_back(stateInterp);
            }
        }
    }
    return vehicleTrajectory;
}

int main(){

    SimEnv simEnv;

    simEnv.boundary = {0, 0, 20, 20};

    simEnv.objects = {{5.5, 5, 8, 8},
                      {10, 15, 13, 19},
                      {10, 0, 14, 12},
                      {1, 10, 6, 12}
                      };

    // assigning a small number to handle values near zero
    const float boundaryCorrLength = 0.001; 

    //Grid resolution
    const float xRes{0.2}, yRes{0.2}, headRes{5};

    //vehicle physical parameters
    float vehWheelBase = 1.3;
    float vehMass = 1500;

    //target location for the vehicle
    float xTar = 19;
    float yTar = 4;
 
    unordered_map<size_t, Vector3f> cameFrom;
    unordered_map<size_t, float> costTillNow;

    Vehicle vehicle (vehWheelBase, vehMass);

    //vehicle initial configuration

    Vector3f startState = {2, 6, 0};

    Vector3f startNode = constToDiscretezGrid(startState, simEnv, xRes, yRes, headRes);

    vehicle.state = startNode;

    // parameters for trajectory simulation for cell opening
    float PathGenSteerRes = 30 * M_PI / 180;
    float pathGenSteeringMax = 30 * M_PI / 180;

    float pathGenVel = 1; 
    float simDt = 0.01;

    //parameters for computing thee heuristic cost
    float turnWeightPar = 1.2;
    float reverseWeightPar = 2;

    //parameters for trajectory generation
    float averageSpeed = 10;
    float trajectoryResolution = 0.01; // this the resolution of time

    int maxItr = 47552;
    int itr = 0;

    ODESolver odeSolver;

    // //Define queue to store the nodes that are being explored

    // Defining PQElement type as a pair of cost and state
    // Defining this pair is required to implement the priority queue
    typedef pair<float, Vector3f> PQElement; 

    //Below comparator helps implementing priority queue
    struct compPriorityQueue 
    {
    constexpr bool operator()(
        pair<float, Vector3f> const& a,
        pair<float, Vector3f> const& b)
        const noexcept
    {
        return a.first > b.first;
    }
    };

    priority_queue<PQElement, vector<PQElement>, compPriorityQueue> seq;

    seq.emplace(0, startNode);

    float minDistToTar = 100; //some large value
    Vector3f closestState;
    float closestItr = 0;

    size_t myKey;

    cout << "while loop starting" << endl;
    while (!seq.empty())
    {
        float distToTar = eulerDist(vehicle.state(0), vehicle.state(1), xTar, yTar);

        if (distToTar < minDistToTar)
        {
            minDistToTar = distToTar;
            closestState = vehicle.state;
            closestItr = itr;
        }

        if (distToTar < 0.5)
        {
            cout << "state when ending loop" << "\n" << vehicle.state << endl;
            cout << "key when ending loop - " << vectorToKey(vehicle.state) << endl;
            break;
        }

        itr += 1;

        //converting the continuous state to the descrete node
        Vector3f currentNode = constToDiscretezGrid(vehicle.state, simEnv, xRes, yRes, headRes);

        seq.pop();


        // to iterate over forward and reverse motion
        for (float vel = pathGenVel; vel >= -pathGenVel; vel -= 2*pathGenVel)
        {
            //to iterate over the possible steering angles
            for(float steer = pathGenSteeringMax; steer >= -pathGenSteeringMax; steer -= PathGenSteerRes)
            {
                // cout << vel << ", " << steer << endl;

                //input to compute the next position
                Vector2f input = {steer, vel};

                stateWithDistance update = odeSolver.updateStateWithDist(vehicle, 0, 1, input, simDt, "RK4");

                Vector3f neighbor = update.state;

                float distToNeighbor = update.distance;

                // if (itr == 31433) 
                // {
                //     cout << "neighbor" << endl;
                //     cout << neighbor << endl;
                // }

                //converting the continous position to the decrete grid node
                
                Vector3f neighborNode = constToDiscretezGrid(neighbor, simEnv, xRes, yRes, headRes);

                if ((neighborNode(0) > 18.1) && (neighborNode(0) < 18.3) && (neighborNode(1) > 4.9) && (neighborNode(1) < 5.1 ))
                {
                    cout << neighborNode << endl;
                    cout << "requared itr - " << itr << endl;
                    //break;
                }

                //check if the grid boundary overlaps with obstacle
                                
                bool isNodeIntersect = pathGridObstacleOverlap(currentNode, neighborNode, simEnv, xRes, yRes);

                //cout << "isNodeIntersect - " << isNodeIntersect << endl; 
                
                bool isNodeInBoundary = neighborNode(0) >= (simEnv.boundary[0] + boundaryCorrLength)
                                        && neighborNode(0) <= (simEnv.boundary[2] - boundaryCorrLength)
                                        && neighborNode(1) >= (simEnv.boundary[1] + boundaryCorrLength)
                                        && neighborNode(1) <= (simEnv.boundary[3] - boundaryCorrLength);
     
                if (!isNodeIntersect && isNodeInBoundary)
                {
                    
                    //compute the weights to penalize turns and reversing
                    float turnWeight = (steer != 0) ? turnWeightPar : 1.0f;
                    float reverseWeight = (vel < 0) ? reverseWeightPar : 1.0f;

                    //getting the key for the nodes
                    size_t neighborNodeKey = vectorToKey(neighborNode);
                    size_t currentNodeKey = vectorToKey(currentNode); 

                    
                    float newCost = distToNeighbor * turnWeight * reverseWeight + costTillNow[currentNodeKey] ;
                    float costHeuristic = newCost + eulerDist(neighbor(0), neighbor(1), xTar, yTar);
                 
                    if ((cameFrom.find(neighborNodeKey) == cameFrom.end()) || (newCost < costTillNow[neighborNodeKey]))
                    {
                        //cout << "adding node to the queue" << endl;
                        if (itr == 41326 || itr == 41611)
                        {
                            cout << "inside the if for writting cameFrom" << endl;
                        }
                        
                        costTillNow[neighborNodeKey] = newCost;
                        seq.emplace(costHeuristic, neighborNode);
                        cameFrom[neighborNodeKey] = currentNode;

                        if (itr == 41326 && vel == pathGenVel && steer == 0)
                        {
                            myKey = neighborNodeKey;
                            cout << "did we write successfully ? " << (cameFrom.count(neighborNodeKey) != 0) <<  "\n\n";
                            cout << "key from innermost : " << myKey<< endl;
                        }
                    }
                }
            }
        }

        //updating the vehicle state
        vehicle.state = seq.top().second;

        if (itr >= maxItr)
        {     
            break;
        }
        
        
    }
    cout << vehicle.state << endl;
    cout << "iteration " << itr << endl;

    cout << "path of the agent is -" << endl;
    Vector3f current = closestState;
    unsigned int maxPrintItr = 100;
    unsigned int printItr = 0;

    // storing the vehicle path
    vector<Vector3f> vehiclePath; 
    
    //writing the agent path in a csv
    // first row is the target node and subsequent nodes are the path of the agent
    ofstream myFile("../scripts/res.csv");
    
    myFile << xTar << "," << yTar << "\n";
    while (vectorToKey(current) != vectorToKey(startNode))
    {
        if (printItr > maxPrintItr)
        {
            break;
        }
        
        cout << current << "\n";

        // adding the node to vehiclePath
        vehiclePath.insert(vehiclePath.begin(), current);

        myFile << current(0) << "," << current(1) << "," << current(2) << "\n";

        if ( (cameFrom.count(vectorToKey(current)) != 0))
        {
            current = cameFrom[vectorToKey(current)];
            printItr++;
        }
        else
        {
            cout << "key does not exist" << endl;
            cout << costTillNow.count(vectorToKey(current)) << " " << cameFrom.count(vectorToKey(current)) <<  endl;
            break;
        }
        
    }

    // writing the starting node
    myFile << startNode(0) << "," << startNode(1)<< "," << startNode(2) << "\n";

    vehiclePath.insert(vehiclePath.begin(), startNode);

    myFile.close();

    // creating list of indices whe the vehicls is reversed

    //writing the environment detail in a csv
    ofstream myFileEnv("../scripts/env.csv");
    // the format is: first line - boundary of the env
    // from second line we start listing the boundaries of each obstacle

    //boundary
    for (int i = 0; i < 3; i++)
    {
        myFileEnv << simEnv.boundary[i] << ",";
    }
    myFileEnv << simEnv.boundary[3] << "\n";

    //obstacles
    cout << "number of objects : " << size(simEnv.objects) << "\n";
    for (int i = 0; i < size(simEnv.objects); i++)
    {
        cout << "i" << i << "\n";
        for (int j = 0; j < 3; j++)
        {
            myFileEnv << simEnv.objects[i][j] << ",";
        }
        myFileEnv << simEnv.objects[i][3] << "\n";
    }

    

    cout << "closesDistToTar" << minDistToTar << "\n\n";
    cout << "closestState" << "\n" << closestState << "\n\n";
    cout << "closestItr" << "\n" << closestItr << "\n\n";

    VehicleTrajectory vehicleTrajectory = generateSmoothTrajectory(vehiclePath, averageSpeed, trajectoryResolution);

    // wrirtting smoothened path in csv
    // format is - time, x, y, heading
    ofstream myFileSmoothPath("../scripts/smoothPath.csv");

    for(int i = 0; i < size(vehicleTrajectory.time); i++)
    {
        // writting time
        myFileSmoothPath << vehicleTrajectory.time[i] << ",";
        //writting state
        myFileSmoothPath << vehicleTrajectory.state[i](0) << ",";
        myFileSmoothPath << vehicleTrajectory.state[i](1) << ",";
        myFileSmoothPath << vehicleTrajectory.state[i](2) << "\n";
    }

    // simulating vehicle motion with control commands to follow the generated trajectory

    //creating the bicycle object
    Vehicle bicycle (vehWheelBase, vehMass);

    // assume that the bicycle is at the start of the generated trajectory

    bicycle.state = vehicleTrajectory.state.front();

    float maxSimTime = 10;

    unsigned int simItr = 0;

    unsigned int planItr = 0; //this will progress slower than simItr

    // implementation sample time is lowest compared to the high/low level planning sample time
    float controlSampTime = 0.005;

    unsigned int closestItrOld = 0; // this is itr with respect to the current simulation time "t"
    // this will be used to search the time-stamp just smaller than the simulation time "t" 

    // the referece trajectory will be converted to continuous using zero-order hold
    // this is essential as the simulation time will not necessarily match with the time stamps in the trajectory

    //defining PID controller
    PIDController PID (5, 0, 0.5);
    float yError = 0;
    float yErrorPrev = 0;
    float yErrorDot = 0;
    float yErrorInt = 0;
    float simHeading = bicycle.state(2); //initial heading is obtained by the smoothened trajectory

    // vector to store the simulated trajectory
    VehicleTrajectory simResult; 
    simResult.state.push_back(bicycle.state);
    simResult.time.push_back(0);

    cout << "starting simulation..." << endl;
    for (float t = controlSampTime; t <= maxSimTime; t += controlSampTime)
    {
        // finding the closest time-stamp smaller than or equal to the current time "t"
        float minDist = 100000000; // some large value
        unsigned int closestItr = closestItrOld; 
        for (int i = closestItr; i < closestItr + 3; i ++)
        {
            if ((t - vehicleTrajectory.time[i]) > 0 && (t - vehicleTrajectory.time[i] < minDist))
            {
                minDist = t - vehicleTrajectory.time[i];
                closestItr = i;
            }
        }
        // decide between forward and reverse velocity based on heading and the target state

        // rotation matrix to take from cartessian to the target points frame (x-axis along the target heading)
        
        //Rotation matrix angle
        if (closestItr > 0)
        {
            simHeading = atan2(vehicleTrajectory.state[closestItr](1) - vehicleTrajectory.state[closestItr-1](1), 
                                vehicleTrajectory.state[closestItr](0) - vehicleTrajectory.state[closestItr-1](0));
        }
        Matrix2f rotMat {
            {cos(simHeading), sin(simHeading)},
            {-sin(simHeading), cos(simHeading)},
        };

        // obtain the rotated vector of current and target location and compute the target velocity and the steering input.
        Vector2f state2D = projectionOn2DPlane(bicycle.state);
        Vector2f target2D = projectionOn2DPlane(vehicleTrajectory.state[closestItr]);

        // obtain the 2D vectors in the rotated coordinates

        Vector2f state2DRot = rotMat * state2D;
        Vector2f target2DRot = rotMat * target2D;
        float targetAverageSpeed = (target2DRot(0) > state2DRot(0)) ? averageSpeed : -averageSpeed;
       
        yError = target2DRot(1) - state2DRot(1);
        yErrorDot = (yError - yErrorPrev) / controlSampTime;
        yErrorInt += yError * controlSampTime;

        float targetSteering = PID.action(yError, yErrorDot, yErrorInt);

        // saturating the steering input
        
        if (targetSteering > pathGenSteeringMax)
        {
            targetSteering = pathGenSteeringMax;
        }
        else if(targetSteering < -pathGenSteeringMax)
        {
            targetSteering = -pathGenSteeringMax;
        }

        Vector2f targetInput = {targetSteering, targetAverageSpeed};

        if (t > 0.83 && t < 1)
        {
            cout << "t - " << t << endl;  
            cout << "bicycle state\n" << bicycle.state << endl;
            cout << "target state\n" << vehicleTrajectory.state[closestItr] << endl;
            cout << "state2DRot\n" << state2DRot << "\n";
            cout << "target2DRot\n" << target2DRot << "\n"; 
            cout << yError << "," << yErrorInt << "," << yErrorDot << endl;
            cout << targetInput << "\n\n";
        }

        stateWithDistance update = odeSolver.updateStateWithDist(bicycle, t, t + controlSampTime, targetInput, controlSampTime / 10, "RK4");

        bicycle.state = update.state;

        // pushing new state to the result vector

        simResult.state.push_back(bicycle.state);
        simResult.time.push_back(t);

        // compute the control input 
        yErrorPrev = yError;
        closestItrOld = closestItr;

        // stopping condition
        float distToTar = eulerDist(bicycle.state[0], bicycle.state[1], xTar, yTar);
        if (distToTar < 0.5)
        {
            cout << "reached at the target" << endl;
            break;
        }
    }
    cout << "done with simulation." << endl;

    ofstream myFileSimulationRes("../scripts/simulationRes.csv");

    for(int i = 0; i < size(simResult.time); i++)
    {
        // writting time
        myFileSimulationRes << simResult.time[i] << ",";
        //writting state
        myFileSimulationRes << simResult.state[i](0) << ",";
        myFileSimulationRes << simResult.state[i](1) << ",";
        myFileSimulationRes << simResult.state[i](2) << "\n";
    }


    return 0;   
}