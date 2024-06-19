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

    SimpleGraph sG {{
        {'A', {'B', 'C', 'D'}},
        {'B', {'A', 'C'}},
        {'C', {'A', 'B'}},
        {'D', {'A', 'H', 'E'}},
        {'E', {'D', 'G', 'F'}},
        {'F', {'E', 'G', 'C'}},
        {'G', {'E', 'F'}},
        {'H', {'D'}}
    }};

    //Define start and goal nodes
    char start {'A'};
    char goal {'G'};

    //char current = start;

    unordered_map<char, char> cameFrom;

    //queue containing the sequence of the nodes for exploration
    queue<char> seq;
    seq.push(start);

    while(!seq.empty()){
        char current = seq.front();
        seq.pop();

        vector<char> neighbours {sG.structure.at(current)};

        for (auto i = 0; i < neighbours.size(); i++){
            if (cameFrom.find(neighbours[i]) == cameFrom.end()) {
                seq.push(neighbours[i]);
                cameFrom[neighbours[i]] = current;
            }
            
        }
    }

    char temp {goal};

    while (temp != start){
        cout<<cameFrom[temp]<<endl;
        temp = cameFrom[temp];
    }





    
    // WeightedGraph wG {{
    // {'A', {make_pair('B',2), make_pair('D',3), make_pair('F', 10)}},
    // }};
    // wG.structure['C'].push_back(make_pair('B', 2));

    // cout<<"will print now "<< wG.neighbours('A').size()<<endl;
    
    // for (int i = 0; i < wG.neighbours('A').size(); i++){
    //     cout<<wG.neighbours('A')[i]<<"here"<<endl;
    // }
    return 0;

}