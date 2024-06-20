#include "../inc/vehicle.h"
#include "../inc/algoSupport.h"
#include <iostream>
#include<variant>
#include<vector>
#include<queue>

using namespace std;

//void printVec(vector<char)

int main(){
    
    //implementing Breadth-First-Search
    // Declaring the graph with unit edge-weight

    // SimpleGraph sG {{
    //     {'A', {'B', 'C', 'D'}},
    //     {'B', {'A', 'C'}},
    //     {'C', {'A', 'B'}},
    //     {'D', {'A', 'H', 'E'}},
    //     {'E', {'D', 'G', 'F'}},
    //     {'F', {'E', 'G', 'C'}},
    //     {'G', {'E', 'F'}},
    //     {'H', {'D'}}
    // }};

    // //Define start and goal nodes
    // char start {'A'};
    // char goal {'G'};

    // //char current = start;

    // unordered_map<char, char> cameFrom;

    // //queue containing the sequence of the nodes for exploration
    // queue<char> seq;
    // seq.push(start);

    // while(!seq.empty()){
    //     char current = seq.front();
    //     seq.pop();

    //     vector<char> neighbours {sG.structure.at(current)};

    //     for (auto i = 0; i < neighbours.size(); i++){
    //         if (cameFrom.find(neighbours[i]) == cameFrom.end()) {
    //             seq.push(neighbours[i]);
    //             cameFrom[neighbours[i]] = current;
    //         }
            
    //     }
    // }

    // char temp {goal};

    // while (temp != start){
    //     cout<<cameFrom[temp]<<endl;
    //     temp = cameFrom[temp];
    // }

    //Now implementing Dijkstra's algorithm
    
    WeightedGraph wG {{
        {'A', {make_pair('B', 2), make_pair('C', 4), make_pair('D', 1)}},
        {'B', {make_pair('A', 2), make_pair('C', 2)}},
        {'C', {make_pair('A', 4), make_pair('B', 2), make_pair('F', 4)}},
        {'D', {make_pair('A', 1), make_pair('H', 3), make_pair('E', 8)}},
        {'E', {make_pair('D', 8), make_pair('G', 4), make_pair('F', 2)}},
        {'F', {make_pair('E', 2), make_pair('G', 3), make_pair('C', 4)}},
        {'G', {make_pair('E', 4), make_pair('F', 3)}},
        {'H', {make_pair('D', 3)}}
    }};
    
    
    char start {'A'};
    char goal {'G'};

    //char current = start;

    unordered_map<char, char> cameFrom;
    unordered_map<char, int> costTIllNow;

    costTIllNow[start] = 0; 
    cameFrom[start] = start;

    //queue containing the sequence of the nodes for exploration
    queue<char> seq;
    seq.push(start);

    while(!seq.empty()){
        char current = seq.front();
        seq.pop();

        vector<char> neighbours = wG.neighbours(current);

        for (auto i = 0; i < neighbours.size(); i++){

            //computing new cost of neighbour[i]
            int currNextCost = wG.getEdgeWeight(current, neighbours[i]);
            int costOfNeighbour = currNextCost + costTIllNow[current]; 

            //debugging here
            cout<<"current: "<<current<<", next: "<<neighbours[i]<<endl;
            cout<<"costTillNow: "<<costTIllNow[neighbours[i]]<<", costOfNeighbour: "<<costOfNeighbour<<endl;

            if (cameFrom.find(neighbours[i]) == cameFrom.end() || costOfNeighbour < costTIllNow[neighbours[i]]) {
                seq.push(neighbours[i]);
                cameFrom[neighbours[i]] = current;
                costTIllNow[neighbours[i]] = costOfNeighbour;
            }
            
        }
    }

    char temp {goal};

    while (temp != start){
        cout<<cameFrom[temp]<<endl;
        temp = cameFrom[temp];
    }

    cout<<costTIllNow['C']<<endl;
    return 0;

}