#include "../inc/vehicle.h"
#include "../inc/algoSupport.h"
#include <iostream>
#include<variant>
#include<vector>

using namespace std;

//void printVec(vector<char)

int main(){
    
  
    WeightedGraph_ wG {{
    {'A', {('B',2), {'D',3}}},
  }};
    //wG.structure['C'].push_back({('B', 2)});

    cout<<"will print now"<<endl;
    
    for (int i = 0; i < wG.neighbours('C').size(); i++){
        cout<<wG.neighbours('C')[i]<<endl;
    }
    


}