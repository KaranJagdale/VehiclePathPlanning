// This file contains vehicle class. As of now it will implement simple bicycle model
using namespace std;
#include <iostream>
#include<unordered_map>
#include<vector>

struct SimpleGraph{
    unordered_map<char, vector<char>> structure;

    char neighbour(node){
        return structure[node][0]       
    }

}