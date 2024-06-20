// This file contains vehicle class. As of now it will implement simple bicycle model
using namespace std;
#include <iostream>
#include<unordered_map>
#include<vector>
#include<utility>

struct SimpleGraph{
    unordered_map<char, vector<char>> structure;

    vector<char> neighbours(char node){
        return structure[node];       
    }

};


struct WeightedGraph{
    //using weighted adjacency matrix
    unordered_map<char,vector<pair<char, int>>> structure;

    vector<char> neighbours(char node){
        vector<char> ret;
        for (int i = 0; i < structure[node].size(); i++){
            ret.push_back(structure[node][i].first);
        }
        return ret;       
    }

    int getEdgeWeight(char node1, char node2){
        for (int i = 0; i < structure[node1].size(); i++){
            if (structure[node1][i].first == node2){
                return structure[node1][i].second;
            }
        }
        cout<<"invalid nodes were entered"<<endl;
        return 0;
    }
};