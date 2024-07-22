#include "../inc/solver.h"

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <queue>
#include <vector>
#include <unordered_map>


using namespace std;
using namespace Eigen;

float vectorToKey(Vector3f vec)
{
    //converts vector to key (1,1,1) is converted to float 111
    //this function works independant of the size of the vector

    float key = 0;

    for(int i = 0; i < size(vec); i++)
    {
        key += vec[i] * pow(10, i);
    }

    return key;
}


Vector3f constToDiscretezGrid(Vector3f state, SimEnv simEnv, unsigned int xRes, 
                                        unsigned int yRes, unsigned int headRes)
{
    //converts continuous state to the discrete point in the grid
    //xRes, yRes denotes the resolution of the grid in the respective direction
    Vector3f res;

    res(0) = (float) round(state(0) / xRes) * xRes;
    res(1) = (float) round(state(1) / yRes) * yRes;
    res(2) = (float) round(state(2) / headRes) * headRes;

    return res;
}

bool gridObstacleOverlap(vector<float> gridBoundary, SimEnv simEnv)
{
    //boundary is stored as x1, y1, x2, y2, where the later coordinates are diagonal coordinate of the first

    bool intersect = false;
    for(unsigned int i = 0; i < size(simEnv.objects); i+=1)
    {
        vector<float> object = simEnv.objects[i];
        intersect = (gridBoundary[0] > object[0] && gridBoundary[0] < object[1]) ||
                    (gridBoundary[2] > object[0] && gridBoundary[2] < object[1]) &&
                    (gridBoundary[1] > object[1] && gridBoundary[1] < object[3]) ||
                    (gridBoundary[3] > object[1] && gridBoundary[3] < object[3]);
        if (intersect)
        {
            return true;
        }
    }
    return false;
}

int main(){

    SimEnv simEnv;

    simEnv.boundary = {0, 0, 20, 20};

    simEnv.objects = {{5, 5, 8, 8},
                      {10, 15, 13, 19},
                      {15, 0, 16, 12},
                      };
    //Grid resolution

    float xRes{1}, yRes{1}, headRes{2};
    //vehicle physical parameters
    float vehWheelBase = 1.3; 
    float vehMass = 1500;

    //target location for the vehicle
    float xTar = 20;
    float yTar = 0;

    unordered_map<float, Vector3f> cameFrom;
    unordered_map<float, float> costTillNow;

    Vehicle vehicle (vehWheelBase, vehMass);

    //vehicle initial configuration

    Vector3f startState = {1, 1, 0};
    

    // parameters for trajectory simulation for cell opening
    float PathGenSteerRes = 30;
    float pathGenSteeringMax = 30;

    float pathGenVel = 1.5; 
    float simDt = 0.01;

    //parameters for computing thee heuristic cost
    float turnWeightPar = 1.2;
    float reverseWeightPar = 1.5;

    int maxItr = 2000;
    int itr = 0;

    ODESolver odeSolver;

    // //Define queue to store the nodes that are being explored
    queue<Vector3f> seq;
    seq.push(constToDiscretezGrid(startState, simEnv, xRes, yRes, headRes));
    cout << "while loop starting" << endl;
    while (!seq.empty())
    {
        itr++;

        vehicle.state = seq.front(); //this is redundunt in the first loop

        // cout << "vehicle state in while" << endl;
        // cout << vehicle.state << endl;
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
                // cout << "neighbor - " << endl;
                // cout << neighbor << endl;

                float distToNeighbor = update.distance;

                //converting the continous position to the decrete grid node
                Vector3f currentNode = constToDiscretezGrid(vehicle.state, simEnv, xRes, yRes, headRes);
                Vector3f neighborNode = constToDiscretezGrid(neighbor, simEnv, xRes, yRes, headRes);

                // cout << neighbor << endl;
                // cout<< neighborNode << endl;

                //check if the grid boundary overlaps with obstacle
                vector<float> gridBoundary = {neighborNode(0) - xRes/2, neighborNode(1) - yRes/2,
                                            neighborNode(0) + xRes/2, neighborNode(1) + yRes/2};
                
                bool isNodeIntersect = gridObstacleOverlap(gridBoundary, simEnv);
                bool isNodeInBoundary = neighborNode(0) >= simEnv.boundary[0] && neighborNode[0] <= simEnv.boundary[2]
                                        && neighborNode(1) >= simEnv.boundary[1] && neighborNode[1] <= simEnv.boundary[3];

                //node does not intersect with an obstacle and is inside the simulation boundary
                if (~isNodeIntersect && isNodeInBoundary)
                {
                    //compute the weights to penalize turns and reversing
                    float turnWeight = (steer != 0) ? turnWeightPar : 1.0f;
                    float reverseWeight = (vel < 0) ? reverseWeightPar : 1.0f;

                    //getting the key for the nodes
                    float neighborNodeKey = vectorToKey(neighborNode);
                    float currentNodeKey = vectorToKey(currentNode);

                    float costHeuristic = distToNeighbor * turnWeight * reverseWeight + costTillNow[currentNodeKey] 
                                    + eulerDist(neighbor(0), neighbor(1), xTar, yTar);
                    if (itr == 1711)
                    {
                        cout << neighborNode << endl;
                        cout << (cameFrom.find(neighborNodeKey) == cameFrom.end()) << endl;
                    }
                    if (cameFrom.find(neighborNodeKey) == cameFrom.end() || costHeuristic < costTillNow[neighborNodeKey])
                    {
                        //cout << "adding node to the queue" << endl;
                        costTillNow[neighborNodeKey] = costHeuristic;
                        seq.push(neighborNode);
                        cameFrom[neighborNodeKey] = currentNode;
                    }
                }
            }
        }
        //breaking the loop if the solution is not converging
        if (itr > maxItr)
        {
            
            break;
        }
        
    }
    cout << vehicle.state << endl;
    cout << "iteration " << itr << endl;

    cout << "path of the agent is -" << endl;
    Vector3f current = vehicle.state;

    while (vectorToKey(current) != vectorToKey(startState))
    {
        cout << current << endl;
        current = cameFrom[vectorToKey(current)];
    }
    
    return 0;   
}