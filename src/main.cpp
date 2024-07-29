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


Vector3f constToDiscretezGrid(Vector3f state, SimEnv simEnv, float xRes, 
                                        float yRes, float headRes)
{
    //converts continuous state to the discrete point in the grid
    //xRes, yRes denotes the resolution of the grid in the respective direction

    state(2) *= 180 / M_PI;  //converting to degree

    state(0) = (float) round(state(0) / xRes) * xRes;
    state(1) = (float) round(state(1) / yRes) * yRes;
    state(2) = (float) round(state(2) / headRes) * headRes;

    state(2) *= M_PI / 180; //converting back to radians

    return state;
}

bool gridObstacleOverlap(vector<float> gridBoundary, SimEnv simEnv)
{
    //boundary is stored as x1, y1, x2, y2, where the later coordinates are diagonal coordinate of the first

    bool intersect = false;
    for(unsigned int i = 0; i < size(simEnv.objects); i+=1)
    {
        vector<float> object = simEnv.objects[i];
        intersect = ((gridBoundary[0] > object[0] && gridBoundary[0] < object[2]) ||
                    (gridBoundary[2] > object[0] && gridBoundary[2] < object[2])) &&
                    ((gridBoundary[1] > object[1] && gridBoundary[1] < object[3]) ||
                    (gridBoundary[3] > object[1] && gridBoundary[3] < object[3]));
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

    simEnv.objects = {//{5, 5, 8, 8},
                      //{10, 15, 13, 19},
                      {10, 0, 14, 12},
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
 
    unordered_map<float, Vector3f> cameFrom;
    unordered_map<float, float> costTillNow;

    Vehicle vehicle (vehWheelBase, vehMass);

    //vehicle initial configuration

    Vector3f startState = {5, 5, 0};

    Vector3f startNode = constToDiscretezGrid(startState, simEnv, xRes, yRes, headRes);

    // parameters for trajectory simulation for cell opening
    float PathGenSteerRes = 30 * M_PI / 180;
    float pathGenSteeringMax = 30 * M_PI / 180;

    float pathGenVel = 1; 
    float simDt = 0.01;

    //parameters for computing thee heuristic cost
    float turnWeightPar = 1.1;
    float reverseWeightPar = 1.1;

    int maxItr = 100;
    int itr = 0;

    ODESolver odeSolver;

    // //Define queue to store the nodes that are being explored

    // Defining PQElement type as a pair of cost and state
    // Defining this pair is required to implement the priority queue
    typedef pair<int, Vector3f> PQElement; 

    //Below comparator helps implementing priority queue
    struct myComp 
    {
    constexpr bool operator()(
        pair<float, Vector3f> const& a,
        pair<float, Vector3f> const& b)
        const noexcept
    {
        return a.first > b.first;
    }
    };

    priority_queue<PQElement, vector<PQElement>, myComp> seq;

    seq.emplace(0, startNode);

    cout << "while loop starting" << endl;
    while (!seq.empty())
    {
        float distToTar = eulerDist(vehicle.state(0), vehicle.state(1), xTar, yTar);

        if (distToTar < 1)
        {
            break;
        }

        itr += 1;

        if (itr > 1  && itr < 20)
        {
            for(int i = 0; i < 3; i++)
             {   
                cout << vehicle.state(i) << "  " << seq.top().second(i) << endl;
             }  

        }

        vehicle.state = seq.top().second; //this is redundunt in the first loop

        // if (true)
        // {
        //     cout << "itr : " << itr << endl;
        //     cout << "length of queue : " << seq.size() << endl;
        // }


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

                //converting the continous position to the decrete grid node
                Vector3f currentNode = constToDiscretezGrid(vehicle.state, simEnv, xRes, yRes, headRes);
                Vector3f neighborNode = constToDiscretezGrid(neighbor, simEnv, xRes, yRes, headRes);

                //check if the grid boundary overlaps with obstacle
                vector<float> gridBoundary = {neighborNode(0) - xRes/2, neighborNode(1) - yRes/2,
                                            neighborNode(0) + xRes/2, neighborNode(1) + yRes/2};
                
                bool isNodeIntersect = gridObstacleOverlap(gridBoundary, simEnv);
                

                bool isNodeInBoundary = neighborNode(0) >= (simEnv.boundary[0] + boundaryCorrLength)
                                        && neighborNode(0) <= (simEnv.boundary[2] - boundaryCorrLength)
                                        && neighborNode(1) >= (simEnv.boundary[1] + boundaryCorrLength)
                                        && neighborNode(1) <= (simEnv.boundary[3] - boundaryCorrLength);
                // if (true)
                // {
                //     cout << "vel - " << vel << "  steer - " << steer << "  distance - " << distToNeighbor << endl;  
                //     for(int i = 0; i < 3; i++)
                //     {
                //         cout << vehicle.state(i) << "  " << neighborNode(i) << "  " << neighbor(i) << endl;
                //     }
                //     cout << "isNodeIntersect - " << isNodeIntersect << "\n\n";
                // }

                //node does not intersect with an obstacle and is inside the simulation boundary
                if (!isNodeIntersect && isNodeInBoundary)
                {
                    
                    //compute the weights to penalize turns and reversing
                    float turnWeight = (steer != 0) ? turnWeightPar : 1.0f;
                    float reverseWeight = (vel < 0) ? reverseWeightPar : 1.0f;

                    //getting the key for the nodes
                    float neighborNodeKey = vectorToKey(neighborNode);
                    float currentNodeKey = vectorToKey(currentNode);

                    float costHeuristic = distToNeighbor * turnWeight * reverseWeight + costTillNow[currentNodeKey] 
                                    + eulerDist(neighbor(0), neighbor(1), xTar, yTar);
                    // if (itr == 1711)
                    // {
                    //     cout << neighborNode << endl;
                    //     cout << (cameFrom.find(neighborNodeKey) == cameFrom.end()) << endl;
                    // }
                    if (cameFrom.find(neighborNodeKey) == cameFrom.end() || (costHeuristic < costTillNow[neighborNodeKey]))
                    {
                        //cout << "adding node to the queue" << endl;
                        costTillNow[neighborNodeKey] = costHeuristic;
                        seq.emplace(costHeuristic, neighborNode);
                        cameFrom[neighborNodeKey] = currentNode;
                    }
                }
            }
        }
        //breaking the loop if the solution is not converging
        if (itr >= maxItr)
        {     
            break;
        }
        
    }
    cout << vehicle.state << endl;
    cout << "iteration " << itr << endl;

    // vector<float> temp = {13, 0, 14, 2};

    // bool res = gridObstacleOverlap(temp, simEnv);

    // cout << res << endl;

    //counting the number of nodes visited
    int notVisistedCount = 0;
    for (float x = 0; x <= simEnv.boundary[2]; x += xRes)
    {
        for(float y = 0; y <= simEnv.boundary[3]; y += yRes)
        {
            for (float head = 0; head <= 360; head += headRes)
            {
                Vector3f tempNode = {x, y, head* M_PI /180};

                float tempNodeKey = vectorToKey(tempNode);

                if (cameFrom.find(tempNodeKey) == cameFrom.end())
                {
                    notVisistedCount++;
                }
            }
        }
    }

    // cout << "No. of un-visited nodes - " << notVisistedCount << "\n\n";

    cout << "path of the agent is -" << endl;
    Vector3f current = vehicle.state;
    unsigned int maxPrintItr = 100;
    unsigned int printItr = 0;
    while (vectorToKey(current) != vectorToKey(startState))
    {
        if (printItr > maxPrintItr)
        {
            break;
        }
        cout << current << "\n\n";
        current = cameFrom[vectorToKey(current)];
        printItr++;
    }
    
    return 0;   
}