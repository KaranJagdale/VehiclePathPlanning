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
};