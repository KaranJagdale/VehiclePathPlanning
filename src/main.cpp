#include "../inc/solver.h"

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <queue>
#include <vector>
#include <unordered_map>
#include <functional>

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
 
    unordered_map<size_t, Vector3f> cameFrom;
    unordered_map<size_t, float> costTillNow;

    Vehicle vehicle (vehWheelBase, vehMass);

    //vehicle initial configuration

    Vector3f startState = {5, 5, 0};

    Vector3f startNode = constToDiscretezGrid(startState, simEnv, xRes, yRes, headRes);

    vehicle.state = startNode;

    // parameters for trajectory simulation for cell opening
    float PathGenSteerRes = 30 * M_PI / 180;
    float pathGenSteeringMax = 30 * M_PI / 180;

    float pathGenVel = 1; 
    float simDt = 0.01;

    //parameters for computing thee heuristic cost
    float turnWeightPar = 1.1;
    float reverseWeightPar = 1.1;

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

        if (distToTar < 2)
        {
            cout << "state when ending loop" << "\n" << vehicle.state << endl;
            cout << "key when ending loop - " << vectorToKey(vehicle.state) << endl;
            break;
        }

        itr += 1;

        // if ((vehicle.state(0) > 18.1) && (vehicle.state(0) < 18.3) && (vehicle.state(1) > 4.9) && (vehicle.state(1) < 5.1 ))
        // {
        //     cout << vehicle.state << endl;
        //     cout << "requared itr - " << itr << endl;
        //     //break;
        // }


        //converting the continuous state to the descrete node
        Vector3f currentNode = constToDiscretezGrid(vehicle.state, simEnv, xRes, yRes, headRes);
        // if (true)
        // {
        //     cout << "itr : " << itr << endl;
        //     cout << "length of queue : " << seq.size() << endl;
        // }


        seq.pop();

        // if (itr == 1)
        // {
        //     cout << "vehicle state" << endl;
        //     cout << vehicle.state << endl;
        // }

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
                vector<float> gridBoundary = {neighborNode(0) - xRes/2, neighborNode(1) - yRes/2,
                                            neighborNode(0) + xRes/2, neighborNode(1) + yRes/2};
                
                bool isNodeIntersect = gridObstacleOverlap(gridBoundary, simEnv);
                

                bool isNodeInBoundary = neighborNode(0) >= (simEnv.boundary[0] + boundaryCorrLength)
                                        && neighborNode(0) <= (simEnv.boundary[2] - boundaryCorrLength)
                                        && neighborNode(1) >= (simEnv.boundary[1] + boundaryCorrLength)
                                        && neighborNode(1) <= (simEnv.boundary[3] - boundaryCorrLength);
                //if ((currentNode(0) >= 8.2 && currentNode(0) <= 8.6) && (currentNode(1) >= 10 && currentNode(1) <= 10.4)) //&& currentNode(2) == 1.48353)
                // if ((currentNode(0) == (float) 8.4) && (currentNode(1) == (float) 10.2))
                if (itr == 41326) 
                {
                    cout << "itr" << itr << endl;
                    cout << "vel - " << vel << "  steer - " << steer << "  distance - " << distToNeighbor << endl;  
                    for(int i = 0; i < 3; i++)
                    {
                        cout << vehicle.state(i) << "  " << neighborNode(i) << "  " << neighbor(i) << endl;
                    }
                    cout << "isNodeIntersect - " << isNodeIntersect << "\n";     
                    cout << "neighborNodeKey - " << vectorToKey(neighborNode) << endl;          
                }

                //node does not intersect with an obstacle and is inside the simulation boundary
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
                    if (itr == 41326)
                    {
                    // cout << "first cost (distToneighbor) : " << distToNeighbor * turnWeight * reverseWeight << "first cost (costTillNow) : " << costTillNow[currentNodeKey] << "\n";
                    // cout << " second cost : " << eulerDist(neighbor(0), neighbor(1), xTar, yTar);
                    // cout << " cost heuristic : " << costHeuristic << "\n";
                    cout << "Is this a new node? 1 " << (cameFrom.find(neighborNodeKey) == cameFrom.end()) << "\n";
                    cout << "neighborNodeKey : " << neighborNodeKey << endl;
                    //cout << "origin node : " << cameFrom[neighborNodeKey] << endl; 
                    cout << "newCost : " << newCost << "costTillNow : " << costTillNow[neighborNodeKey] << "\n";
                    cout << "currentNode : " << currentNode << "\n";
                    cout << "if condition : " << ((cameFrom.find(neighborNodeKey) == cameFrom.end()) || (newCost < costTillNow[neighborNodeKey])) << "\n";
                    cout << "if first : " << (cameFrom.find(neighborNodeKey) == cameFrom.end()) << " if second : " <<  (newCost < costTillNow[neighborNodeKey]) << "\n\n";
                    }

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

        // cout << "below is the priority queue: \n"; 
        // while (!seq.empty())
        // {
        //     cout << seq.top().first <<  "\n" << seq.top().second << "\n\n\n";
        //     seq.pop();
        // }

        //cout << "priority queue printed" << "\n\n";

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
    // int notVisistedCount = 0;
    // for (float x = 0; x <= simEnv.boundary[2]; x += xRes)
    // {
    //     for(float y = 0; y <= simEnv.boundary[3]; y += yRes)
    //     {
    //         for (float head = 0; head <= 360; head += headRes)
    //         {
    //             Vector3f tempNode = {x, y, head* M_PI /180};

    //             float tempNodeKey = vectorToKey(tempNode);

    //             if (cameFrom.find(tempNodeKey) == cameFrom.end())
    //             {
    //                 notVisistedCount++;
    //             }
    //         }
    //     }
    // }

    // cout << "No. of un-visited nodes - " << notVisistedCount << "\n\n";

    cout << "path of the agent is -" << endl;
    Vector3f current = closestState;
    unsigned int maxPrintItr = 100;
    unsigned int printItr = 0;
    while (vectorToKey(current) != vectorToKey(startState))
    {
        if (printItr > maxPrintItr)
        {
            break;
        }
        
        cout << current << "\n";
        cout << "key" << vectorToKey(current) << "\n\n";
        cout << "myKey" << myKey << "\n";
        cout << "is this the same key" << (vectorToKey(current) == myKey) << endl;
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


    cout << "closesDistToTar" << minDistToTar << "\n\n";
    cout << "closestState" << "\n" << closestState << "\n\n";
    cout << "closestItr" << "\n" << closestItr << "\n\n";

    // cout << "testing cameFrom..." << "\n";
    // for (int i = 0; i < 10; i++)
    // {
    //     cout << i << "cameFrom(i) " << "\n" << cameFrom[i*10 + 0.1] << "\n\n";
    // }
    return 0;   
}